__author__ = 'xiezj'
import pickle
from Painter import *
import math

class item:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.direction = 0

class WarehouseMonitor(object):
    def __init__(self):
        self.map = self.loadMap()
        self.ex = None

    def loadMap(self):
        f = open("map","r")
        m = pickle.load(f)
        f.close();
        return m

    def readItems(self):
        door = item()
        door.x = 144
        door.y = 163
        door.direction = math.pi/2

        self.ex.doors = []
        self.ex.doors.append(door)


        shelf = item()
        shelf.x = 100
        shelf.y = 400

        self.ex.shelfs = []
        self.ex.shelfs.append(shelf)

        robot = item()
        robot.x = 100
        robot.y = 100
        self.ex.rotots = []
        self.ex.robots.append(robot)

        box = item()
        box.x = 100
        box.y = 100
        self.ex.boxes = []
        self.ex.boxes.append(box)

        box = item()
        box.x = 100
        box.y = 400
        self.ex.boxes.append(box)

    def repaint(self):
        self.readItems()
        self.ex.repaint();

    def mainloop(self):
        app = QtGui.QApplication(sys.argv)
        self.ex = Painter(self.map)
        self.repaint()
        sys.exit(app.exec_())


if __name__ == '__main__':
    monitor = WarehouseMonitor()
    monitor.mainloop()