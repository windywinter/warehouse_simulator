'''
Created on 2013-12-9

@author: windywinter
'''
import math
import cmath
import roslib; roslib.load_manifest('warehouse_simulator')
import rospy
import tf
import sensor_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import warehouse_simulator.msg
import shapely.geometry
import shapely.affinity
import shapely.ops
from Object import Object, Vessel
import Utils

class Battery(object):
    def __init__(self, initial_quantity, max_quantity, recharge_rate):
        self.quantity = initial_quantity
        self.max_quantity = max_quantity
        self.recharge_rate = recharge_rate
        
    def recharge(self):
        self.quantity += self.recharge_rate
        self.quantity = min(self.quantity, self.max_quantity)

class Agent(object):
    def __init__(self, warehouse, uid, action_topic):
        self.warehouse = warehouse
        self.uid = uid
        self.command_flags = {}
        rospy.Subscriber(action_topic, warehouse_simulator.msg.Action, self.action_handler)
    
    def action_handler(self, data):
        pass
    
    def reset_command_flags(self):
        for command in self.command_flags.keys():
            self.command_flags[command] = False
            
    def add_command(self, command):
        self.command_flags[command] = False

class Operator(Agent):
    def __init__(self, warehouse, uid, action_topic, info_topic, order_topic):
        Agent.__init__(self, warehouse, uid, action_topic)
        self.info_pub = rospy.Publisher(info_topic, warehouse_simulator.msg.OperatorInfo)
        self.info_data = warehouse_simulator.msg.OperatorInfo()

        self.add_command('deliver')
        self.add_command('gate')
        
    def publish_info(self):
        self.info_data.header.seq += 1
        self.info_data.header.stamp = rospy.Time.now()
        self.info_data.gate_open = [gate.open for gate in self.warehouse.gates]
        self.info_data.charge_field_occupied = [bool(field.carry) for field in self.warehouse.recharge_fields]
        self.info_pub.publish(self.info_data)
    
    def action_hander(self, data):
        if data.action == 'gate':
            uid = int(data.param)
            self.warehouse.gates[uid] = not self.warehouse.gates[uid]
        elif data.action == 'deliver':
            order = self.warehouse.orderbook.get(int(data.header.seq))
            if order and (order.item in self.warehouse.items) and (order.item.pos.within(order.field)):
                self.warehouse.evaluate(order)
                self.warehouse.items.remove(order.item)
                del self.warehouse.orderbook[int(data.header.seq)]

class Robot(Agent, Object, Vessel):
    def __init__(self, warehouse, uid, action_topic, cmd_vel_topic, scan_topic, odom_topic, amcl_topic, vision_topic, tf_prefix, robot_description):
        Agent.__init__(self, warehouse, uid, action_topic)
        Object.__init__(self, uid, shapely.geometry.Point(tuple(robot_description['pos'])), shapely.geometry.Polygon([tuple(v) for v in robot_description['footprint']]))
        Vessel.__init__(self, uid)
        self.orientation = 0 #radian
        rospy.Subscriber(cmd_vel_topic, geometry_msgs.msg.Twist, self.cmd_vel_handler)

        self.robot_description = robot_description
        
        if scan_topic:
            self.scan_pub = rospy.Publisher(scan_topic, sensor_msgs.msg.LaserScan)
            self.scan_data = sensor_msgs.msg.LaserScan()
            self.scan_data.header.frame_id = tf_prefix + '/base_link'
            self.scan_data.angle_min = self.robot_description['laser_angle_min']
            self.scan_data.angle_max = self.robot_description['laser_angle_max']
            self.scan_data.angle_increment = self.robot_description['laser_angle_increment']
            self.scan_data.range_min = self.robot_description['laser_range_min']
            self.scan_data.range_max = self.robot_description['laser_range_max']
        else:
            self.scan_pub = None
        if odom_topic:
            self.odom_pub = rospy.Publisher(odom_topic, nav_msgs.msg.Odometry)
            self.odom_broadcaster = tf.TransformBroadcaster()
            self.odom_data = nav_msgs.msg.Odometry()
            self.odom_data.header.frame_id = tf_prefix + '/odom'
            self.odom_data.child_frame_id = tf_prefix + '/base_link'
            self.init_pos = self.pos
            self.init_orientation = self.orientation
            self.init_time = rospy.Time.now()
        else:
            self.odom_pub = None
        if amcl_topic:
            self.amcl_pub = rospy.Publisher(amcl_topic, geometry_msgs.msg.PoseWithCovarianceStamped)
            self.amcl_data = geometry_msgs.msg.PoseWithCovarianceStamped()
            self.amcl_data.header.frame_id = '/map'
        else:
            self.amcl_pub = None
        self.vision_pub = rospy.Publisher(vision_topic, warehouse_simulator.msg.AbstractVision)
        
        for command in self.robot_description['actions'].keys():
            self.add_command(command)
        self.add_command('vel')
        
        self.battery = Battery(self.robot_description['battery_max_quantity'], self.robot_description['battery_max_quantity'], self.robot_description['battery_recharge_rate'])
        self.vel = geometry_msgs.msg.Twist()
        self.vel_odom = geometry_msgs.msg.Twist()
        self.vel_target = geometry_msgs.msg.Twist()
        
        self.broken = False
        
    def recharge(self):
        self.battery.recharge()
        
    def run(self):
        linear_accel = min(max(self.robot_description['linear_accel_min']/self.warehouse.simulator_step, self.vel_target.linear.x - self.vel_odom.linear.x), self.robot_description['linear_accel_max']/self.warehouse.simulator_step)
        self.vel_odom.linear.x += linear_accel
        angular_accel = min(max(-self.robot_description['angular_accel_max']/self.warehouse.simulator_step, self.vel_target.angular.z - self.vel_odom.angular.z), self.robot_description['angular_accel_max']/self.warehouse.simulator_step)
        self.vel_odom.angular.z += angular_accel
        
        self.orientation += self.vel_odom.angular.z/self.warehouse.simulator_step
        self.battery.quantity -= abs(self.vel_odom.angular.z / self.robot_description['angular_speed_max']) * self.robot_description['battery_cost_vel_angular'] / self.warehouse.simulator_step
        self.vel.linear.x = self.vel_odom.linear.x*math.cos(self.orientation)
        self.vel.linear.y = self.vel_odom.linear.x*math.sin(self.orientation)
        self.vel.angular.z = self.vel_odom.angular.z
        self.footprint = shapely.affinity.rotate(self.footprint, self.vel.angular.z/self.warehouse.simulator_step, use_radians=True)
        
        if not Utils.approx_eq(self.vel_odom.linear.x, 0):
            self.battery.quantity -= abs(self.vel_odom.linear.x / self.robot_description['linear_speed_max']) * self.robot_description['battery_cost_vel_linear'] / self.warehouse.simulator_step
            new_ft = shapely.ops.transform(lambda x, y, z=None: (x+self.vel.linear.x/self.warehouse.simulator_step, y+self.vel.linear.y/self.warehouse.simulator_step), self.footprint)
            trail = self.footprint.union(new_ft).convex_hull
            if self.warehouse.floor.collide_with_obstacle(trail) :
                self.broken = True
            for robot in self.warehouse.robots:
                if robot is self:
                    continue
                else:
                    if robot.footprint.intersects(trail):
                        robot.broken = True
                        self.broken = True
            for item in self.warehouse.items:
                if item.footprint.intersects(trail):
                    self.broken = True
            for shelf in self.warehouse.shelves:
                if shelf.footprint.intersects(trail):
                    self.broken = True
            for gate in self.warehouse.gates:
                if (not gate.open) and gate.intersects(trail):
                    self.broken = True
            if not self.broken:
                self.pos = shapely.ops.transform(lambda x, y, z=None: (x+self.vel.linear.x/self.warehouse.simulator_step, y+self.vel_odom.linear.y/self.warehouse.simulator_step), self.pos)
                self.footprint = new_ft
        
    def cmd_vel_handler(self, data):
        if not self.command_flags['vel']:
            self.command_flags['vel'] = True
            self.vel_target.linear.x = min(max(-self.robot_description['linear_speed_max'], data.linear.x), self.robot_description['linear_speed_max'])
            self.vel_target.angular.z = min(max(-self.robot_description['angular_speed_max'], data.angular.z), self.robot_description['angular_speed_max'])
    
    def action_handler(self, data):
        if not self.command_flags[data.action]:
            self.command_flags[data.action] = True
            if data.action == 'lift':
                if not self.carry:
                    operation_area = shapely.geometry.Polygon([[0.05, 0], [0.05, self.robot_description['lift_range']], [-0.05, self.robot_description['lift_range']], [-0.05, 0]])
                    operation_area = shapely.ops.transform(lambda x, y, z=None: (x+self.pos.x, y+self.pos.y), operation_area)
                    operation_area = shapely.affinity.rotate(operation_area, self.orientation, use_radians=True)
                    for item in self.warehouse.items:
                        if Utils.approx_eq(item.orientation, self.orientation, 5.0*math.pi/180.0) or Utils.approx_eq(math.pi-item.orientation, self.orientation, 5.0*math.pi/180.0):
                            if item.pos.within(operation_area):
                                self.carry = item
                                self.warehouse.items.remove(item)
                                item.footprint = shapely.ops.transform(lambda x, y, z=None: (x-item.pos.x, y-item.pos.y), item.footprint)
                                item.footprint = shapely.affinity.rotate(item.footprint, -item.orientation, use_radians=True)
                                item.orientation = 0
                                item.pos = shapely.geometry.Point(0, 0)
                                break
                    if not self.carry:
                        height = float(data.param)
                        for shelf in self.warehouse.shelves:
                            if Utils.approx_eq(shelf.orientation, self.orientation, 5.0*math.pi/180.0) or Utils.approx_eq(math.pi-shelf.orientation, self.orientation, 5.0*math.pi/180.0):
                                if shelf.pos.within(operation_area):
                                    grid = height * shelf.pile / shelf.height
                                    self.carry = shelf.carry[grid]
                                    shelf.carry[grid] = None
                                    break
            elif data.action == 'put':
                if self.carry:
                    operation_area = shapely.geometry.Polygon([[0.05, 0], [0.05, self.robot_description['lift_range']], [-0.05, self.robot_description['lift_range']], [-0.05, 0]])
                    operation_area = shapely.ops.transform(lambda x, y, z=None: (x+self.pos.x, y+self.pos.y), operation_area)
                    operation_area = shapely.affinity.rotate(operation_area, self.orientation, use_radians=True)
                    height = float(data.param)
                    for shelf in self.warehouse.shelves:
                        if shelf.pos.within(operation_area):
                            if Utils.approx_eq(shelf.orientation, self.orientation, 5.0*math.pi/180.0) or Utils.approx_eq(math.pi-shelf.orientation, self.orientation, 5.0*math.pi/180.0):
                                grid = height * shelf.pile / shelf.height
                                if not shelf.carry[grid]:
                                    shelf.carry[grid] = self.carry
                                    self.carry = None
                                else:
                                    #TODO
                                    pass
                            else:
                                #TODO
                                pass
                            break
                    if self.carry:
                        item = self.carry
                        item.orientation = self.orientation
                        item.pos = shapely.ops.transform(lambda x, y, z=None: (x+self.robot_description['lift_range']*math.cos(self.orientation), y+self.robot_description['lift_range']*math.sin(self.orientation)), self.pos)
                        item.footprint = shapely.ops.transform(lambda x, y, z=None: (x+item.pos.x, y+item.pos.y), item.footprint)
                        item.footprint = shapely.affinity.rotate(item.footprint, item.orientation, use_radians=True)
                        if not self.warehouse.scene.intersect(item.footprint):
                            self.warehouse.items.add(item)
                            self.carry = None
                        else:
                            #TODO
                            pass
    
    def publish_scan(self):
        if self.scan_pub: 
            self.scan_data.header.seq += 1
            self.scan_data.header.stamp = rospy.Time.now()
            self.scan_data.ranges = [self.scan_data.range_max + 1.0] * int((self.scan_data.angle_max-self.scan_data.angle_min)/self.scan_data.angle_increment + 1)
            scan_lines = []
            angle = self.scan_data.angle_min
            while angle <= self.scan_data.angle_max:
                scan_dir = self.orientation + angle
                scan_line = shapely.geometry.LineString([(self.pos.x+self.scan_data.range_min*math.cos(scan_dir), self.pos.y+self.scan_data.range_min*math.sin(scan_dir)),
                                                         (self.pos.x+self.scan_data.range_max*math.cos(scan_dir), self.pos.y+self.scan_data.range_max*math.sin(scan_dir))])
                scan_lines.append(scan_line)
                angle += self.scan_data.angle_increment
            scan_sector = shapely.geometry.MultiLineString(scan_lines)
            effective_scene = self.warehouse.scene.intersection(self.pos.buffer(self.scan_data.range_max*1.27, 6))
            intersection = scan_sector.intersection(effective_scene)
            if isinstance(intersection, shapely.geometry.Point) or isinstance(intersection, shapely.geometry.LineString):
                intersection = [intersection]
            for geom in intersection:
                if isinstance(geom, shapely.geometry.Point):
                    i = int((cmath.phase(complex(geom.x, geom.y)) - self.orientation + self.scan_data.angle_min)/self.scan_data.angle_increment)
                    if geom.distance(self.pos) < self.scan_data.ranges[i]:
                        self.scan_data.ranges[i] = geom.distance(self.pos)
                elif isinstance(geom, shapely.geometry.LineString):
                    for point in shapely.geometry.MultiPoint(list(geom.coords)):
                        i = int((cmath.phase(complex(point.x, point.y)) - self.orientation + self.scan_data.angle_min)/self.scan_data.angle_increment)
                        if point.distance(self.pos) < self.scan_data.ranges[i]:
                            self.scan_data.ranges[i] = point.distance(self.pos)
            self.scan_pub.publish(self.scan_data)
#            self.scan_data.ranges = []
#            effective_scene = self.warehouse.scene.intersection(self.pos.buffer(self.scan_data.range_max*1.27, 6))
#            angle = self.scan_data.angle_min
#            while angle <= self.scan_data.angle_max:
#                scan_dir = self.orientation + angle
#                scan_line = shapely.geometry.LineString([(self.pos.x+self.scan_data.range_min*math.cos(scan_dir), self.pos.y+self.scan_data.range_min*math.sin(scan_dir)),
#                                                         (self.pos.x+self.scan_data.range_max*math.cos(scan_dir), self.pos.y+self.scan_data.range_max*math.sin(scan_dir))])
#                intersection = scan_line.intersection(effective_scene)
#                if isinstance(intersection, shapely.geometry.Point) or isinstance(intersection, shapely.geometry.LineString):
#                    intersection = [intersection]
#                min_dist = self.scan_data.range_max + 1.0
#                for geom in intersection:
#                    if isinstance(geom, shapely.geometry.Point):
#                        if geom.distance(self.pos) < min_dist:
#                            min_dist = geom.distance(self.pos)
#                    elif isinstance(geom, shapely.geometry.LineString):
#                        for point in shapely.geometry.MultiPoint(list(geom.coords)):
#                            if point.distance(self.pos) < min_dist:
#                                min_dist = point.distance(self.pos)
#                self.scan_data.ranges.append(min_dist)
#                angle += self.scan_data.angle_increment
#            self.scan_pub.publish(self.scan_data)
        
    def publish_odom(self):
        if self.odom_pub:
            self.odom_data.header.seq += 1
            self.odom_data.header.stamp = rospy.Time.now()
            self.odom_data.pose.pose.position.x = self.pos.x - self.init_pos.x
            self.odom_data.pose.pose.position.y = self.pos.y - self.init_pos.y
            orientation = tf.transformations.quaternion_from_euler(0, 0, self.orientation)
            self.odom_data.pose.pose.orientation.x = orientation[0]
            self.odom_data.pose.pose.orientation.y = orientation[1]
            self.odom_data.pose.pose.orientation.z = orientation[2]
            self.odom_data.pose.pose.orientation.w = orientation[3]
            self.odom_data.twist.twist = self.vel_odom
            self.odom_broadcaster.sendTransform((self.odom_data.pose.pose.position.x, self.odom_data.pose.pose.position.y, 0),
                                                orientation, self.odom_data.header.stamp, self.odom_data.child_frame_id, self.odom_data.header.frame_id)
            self.odom_pub.publish(self.odom_data)
        
    def publish_amcl(self):
        if self.amcl_pub:
            self.amcl_data.header.seq += 1
            self.amcl_data.header.stamp = rospy.Time.now()
            self.amcl_data.pose.pose.position.x = self.pos.x
            self.amcl_data.pose.pose.position.y = self.pos.y
            orientation = tf.transformations.quaternion_from_euler(0, 0, self.orientation)
            self.amcl_data.pose.pose.orientation.x = orientation[0]
            self.amcl_data.pose.pose.orientation.y = orientation[1]
            self.amcl_data.pose.pose.orientation.z = orientation[2]
            self.amcl_data.pose.pose.orientation.w = orientation[3]
            self.amcl_pub.publish(self.amcl_data)
    
    def publish_vision(self):
        if self.vision_pub:
            vision = warehouse_simulator.msg.AbstractVision()
            vision.header.stamp = rospy.Time.now()
            for gate in self.warehouse.gates:
                pos_rel = shapely.affinity.rotate(gate.centroid, -self.orientation, self.pos, use_radians=True)
                z = complex(pos_rel.x - self.pos.x, pos_rel.y - self.pos.y)
                if abs(cmath.phase(z)) <= self.robot_description['vision_angle'] and abs(z) <= self.robot_description['vision_range']:
                    vision.direction.append(cmath.phase(z))
                    vision.dist.append(abs(z))
                    vision.type.append(0)
                    vision.orientation.append(gate.orientation - self.orientation)
                    vision.uid.append(gate.uid)
            for robot in self.warehouse.robots:
                #TODO exclude self
                pos_rel = shapely.affinity.rotate(robot.pos, -self.orientation, self.pos, use_radians=True)
                z = complex(pos_rel.x - self.pos.x, pos_rel.y - self.pos.y)
                if abs(cmath.phase(z)) <= self.robot_description['vision_angle'] and abs(z) <= self.robot_description['vision_range']:
                    vision.direction.append(cmath.phase(z))
                    vision.dist.append(abs(z))
                    vision.type.append(1)
                    vision.orientation.append(robot.orientation - self.orientation)
                    vision.uid.append(robot.uid)
                    if robot.carry:
                        vision.direction.append(cmath.phase(z))
                        vision.dist.append(abs(z))
                        vision.type.append(2)
                        vision.orientation.append(robot.orientation - self.orientation)
                        vision.uid.append(robot.carry.uid)
            for item in self.warehouse.items:
                pos_rel = shapely.affinity.rotate(item.pos, -self.orientation, self.pos, use_radians=True)
                z = complex(pos_rel.x - self.pos.x, pos_rel.y - self.pos.y)
                if abs(cmath.phase(z)) <= self.robot_description['vision_angle'] and abs(z) <= self.robot_description['vision_range']:
                    vision.direction.append(cmath.phase(z))
                    vision.dist.append(abs(z))
                    vision.type.append(2)
                    vision.orientation.append(item.orientation - self.orientation)
                    vision.uid.append(item.uid)
            for shelf in self.warehouse.shelves:
                pos_rel = shapely.affinity.rotate(shelf.pos, -self.orientation, self.pos, use_radians=True)
                z = complex(pos_rel.x - self.pos.x, pos_rel.y - self.pos.y)
                if abs(cmath.phase(z)) <= self.robot_description['vision_angle'] and abs(z) <= self.robot_description['vision_range']:
                    vision.direction.append(cmath.phase(z))
                    vision.dist.append(abs(z))
                    vision.type.append(3)
                    vision.orientation.append(shelf.orientation - self.orientation)
                    vision.uid.append(shelf.uid)
                    for item in shelf.carry:
                        if item:
                            vision.direction.append(cmath.phase(z))
                            vision.dist.append(abs(z))
                            vision.type.append(2)
                            vision.orientation.append(shelf.orientation - self.orientation)
                            vision.uid.append(item.uid)
            self.vision_pub.publish(vision)