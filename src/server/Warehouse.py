'''
Created on 2013-12-5

@author: windywinter
'''

import roslib; roslib.load_manifest('warehouse_simulator')
import rospy
import shapely.ops
import shapely.speedups
import warehouse_simulator.msg
from Floor import Floor, Field, Gate 
from Object import Item, Shelf
from Agent import Operator, Robot

class Order(object):
    def __init__(self):
        self.stamp = rospy.Time.now()
        self.item = None
        self.field = None
        self.urgency = 0

class Warehouse(object):
    def __init__(self):
        self.autonomous_localization = rospy.get_param('~autonomous_localization', False)
        self.simulator_step = rospy.get_param('~simulator_step', 10)
        
        self.floor = Floor()
        self.import_fields = [Field(field['uid'], field['area']) for field in rospy.get_param('~import_fields', [])]
        self.import_fields.sort(key=lambda x: x.uid)
        self.export_fields = [Field(field['uid'], field['area']) for field in rospy.get_param('~export_fields', [])]
        self.export_fields.sort(key=lambda x: x.uid)
        self.recharge_fields = [Field(field['uid'], field['area']) for field in rospy.get_param('~recharge_fields', [])]
        self.recharge_fields.sort(key=lambda x: x.uid)
        self.recharge_fields_geom = shapely.ops.cascaded_union(self.recharge_fields) if self.recharge_fields else None
        self.gates = [Gate(gate['uid'], gate['left'], gate['right']) for gate in rospy.get_param('~gates', [])]
        self.gates.sort(key=lambda x: x.uid)

        self.items = set()
        self.shelves = set()

        self.scene = self.floor.obstacles.union(self.gates) if self.gates else self.floor.obstacles
        
        operator = rospy.get_param('~operator', {})
        self.operator = Operator(self, operator['uid'], operator['action_topic'], operator['info_topic'], operator['order_topic'])
        
        self.robots = [Robot(self, robot['uid'], robot['action_topic'], robot['cmd_vel_topic'], robot['scan_topic'], robot['odom_topic'], robot['amcl_topic'], robot['vision_topic'], robot['tf_prefix'], robot['robot_description']) for robot in rospy.get_param('~robots', [])]
        rospy.loginfo("%d robots found", len(self.robots))
        
        self.arrivals = 0
        self.orderbook = {}
        self.evaluation = 0.0
        self.stock_order_pub = rospy.Publisher(rospy.get_param('~stock_order_topic', 'order/stock'), warehouse_simulator.msg.StockOrder)
        self.pickup_order_pub = rospy.Publisher(rospy.get_param('~pickup_order_topic', 'order/pickup'), warehouse_simulator.msg.PickupOrder)

    def send_disp(self):
        pass
    
    def step(self):
        for field in self.recharge_fields:
            field.carry.clear()
        for robot in self.robots:
            if not robot.broken:
                robot.run()
            if self.recharge_fields_geom and robot.pos.within(self.recharge_fields_geom) :
                for field in self.recharge_fields:
                    if robot.within(field):
                        field.carry.add(robot)
                        break
                robot.recharge()

        self.operator.reset_command_flags()
        for robot in self.robots:
            robot.reset_command_flags()

        self.operator.publish_info()
        if self.autonomous_localization:
            self.scene = self.floor.obstacles.union(self.gates) if self.gates else self.floor.obstacles
            self.scene = self.scene.union(shapely.ops.cascaded_union([robot.footprint for robot in self.robots])) if self.robots else self.scene
            self.scene = self.scene.union(shapely.ops.cascaded_union([item.footprint for item in self.items])) if self.items else self.scene
            self.scene = self.scene.union(shapely.ops.cascaded_union([shelf.footprint for shelf in self.shelves])) if self.shelves else self.scene
            self.scene = self.scene.simplify(self.floor.map.info.resolution / 2.0, preserve_topology=False)
            for robot in self.robots:
                robot.publish_scan()
                robot.publish_odom()
        else:
            for robot in self.robots:
                robot.publish_amcl()
        for robot in self.robots:
            robot.publish_vision()
        
        self.send_disp()
    
    def evaluate(self, order):
        #TODO
        self.evaluation += (rospy.Time.now() - order.stamp).to_sec() * order.urgency
    
    def mainloop(self):
        if shapely.speedups.available:
            shapely.speedups.enable()
        r = rospy.Rate(self.simulator_step)
        while not rospy.is_shutdown():
            self.step()
            r.sleep()
    
    def orderloop(self):
        pass