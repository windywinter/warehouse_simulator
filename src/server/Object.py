'''
Created on 2013-12-7

@author: windywinter
'''

import shapely.geometry
import shapely.affinity
import shapely.ops

class Object(object):
    def __init__(self, uid, pos, footprint):
        '''pos: Point, footprint: Polygon'''
        self.uid = uid
        self.pos = pos
        self.footprint = shapely.ops.transform(lambda x, y, z=None: (x+pos.x, y+pos.y), footprint)
        
class OperableObject(Object):
    def __init__(self, uid, pos, footprint, orientation):
        Object.__init__(self, uid, pos, footprint)
        self.orientation = orientation #radian
        self.footprint = shapely.affinity.rotate(self.footprint, orientation, use_radians=True)
        self.carrier = None

class Vessel(object):
    def __init__(self, uid):
        self.uid = uid
        self.carry = None

class Item(OperableObject):
    def __init__(self, uid, pos, length, width, orientation, weight):
        footprint = shapely.geometry.box(-length*0.5, -width*0.5, length*0.5, width*0.5)
        OperableObject.__init__(self, uid, pos, footprint, orientation)
        self.weight = weight
        self.broken = False
        
class Shelf(OperableObject, Vessel):
    def __init__(self, uid, pos, length, width, orientation, height, pile):
        footprint = shapely.geometry.box(-length*0.5, -width*0.5, length*0.5, width*0.5)
        OperableObject.__init__(self, uid, pos, footprint, orientation)
        Vessel.__init__(self, uid)
        self.height = height
        self.pile = pile
        self.carry = [None] * pile