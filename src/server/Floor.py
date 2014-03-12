'''
Created on 2013-12-5

@author: windywinter
'''
import roslib; roslib.load_manifest('warehouse_simulator')
import rospy
import nav_msgs.srv
import shapely.geometry
import shapely.ops
from Object import Vessel

class Floor(object):
    def __init__(self):
        '''load map'''
        rospy.wait_for_service('static_map')
        try:
            static_map = rospy.ServiceProxy('static_map', nav_msgs.srv.GetMap)
            resp = static_map()
            self.map = resp.map
        except:
            pass
        obstacles = []
        for grid, prob in enumerate(self.map.data):
            x = grid % self.map.info.width
            y = grid / self.map.info.width
            if (prob == 100) or (prob == -1) :
                obstacles.append(shapely.geometry.box(x * self.map.info.resolution, y * self.map.info.resolution, (x+1) * self.map.info.resolution, (y+1) * self.map.info.resolution))
        self.obstacles = shapely.ops.cascaded_union(obstacles)
        self.obstacles = self.obstacles.simplify(self.map.info.resolution / 2.0, preserve_topology=False)
        
    def is_obstacle(self, p):
        '''p: Point'''
        x_c = int(p.x/self.map.info.resolution)
        y_c = int(p.y/self.map.info.resolution)
        if x_c < self.map.info.width and y_c < self.map.info.height:
            prob = self.map.data[y_c*self.map.info.width+x_c]
        else:
            prob = -1
        return (prob == 100) or (prob == -1)
    
    def collide_with_obstacle(self, o):
        '''o: Geometry object'''
        return self.obstacles.intersects(o)
    
class Field(shapely.geometry.Polygon, Vessel):
    def __init__(self, uid, area):
        '''area: [[x1, y1], [x2, y2]]'''
        shapely.geometry.Polygon.__init__(self, [tuple(v) for v in area])
        Vessel.__init__(self, uid)
        self.carry = set()

class Gate(shapely.geometry.LineString):
    def __init__(self, uid, left, right):
        '''left: [x, y]; right: [x, y]'''
        shapely.geometry.LineString.__init__(self, [tuple(left), tuple(right)])
        self.uid = uid
        self.open = False