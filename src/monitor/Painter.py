__author__ = 'xiezj'

import sys, random
from PyQt4 import QtGui, QtCore
import math


class Painter(QtGui.QWidget):
    def __init__(self, map):
        super(Painter, self).__init__()
        self.map = map
        self.shelfs = []
        self.doors = []
        self.boxes = []
        self.robots = []
        self.qp = None
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 500, 500)
        self.setWindowTitle('WareHouse')
        self.show()

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        self.drawMap(qp, self.map)
        for door in self.doors:
            self.translate(qp,door.x,door.y,door.direction,self.drawDoor)
        for shelf in self.shelfs:
            self.translate(qp,shelf.x,shelf.y,shelf.direction,self.drawShelf)
        for robot in self.robots:
            self.translate(qp,robot.x,robot.y,robot.direction,self.drawRobot)
        for box in self.boxes:
            self.translate(qp,box.x,box.y,box.direction,self.drawBox)
        qp.end()

    def translate(self,qp,x,y,direction,func):
        qp.translate(x,y)
        qp.rotate(math.degrees(direction))
        func(qp)
        qp.rotate(-math.degrees(direction))
        qp.translate(-x,-y)

    def main(self):
        pass

    def drawMap(self, qp, map):
        qp.setPen(QtCore.Qt.black)
        size = self.size()
        for i in range(500):
            for j in range(500):
                if (map[i * 500 + j] > 0):
                    qp.drawPoint(i, j)

    def drawDoor(self, qp):
        color = QtGui.QColor(0,0,0);
        qp.setPen(color)
        len = 60
        qp.drawLine(-len/2,0,len/2,0)

    def drawShelf(self,qp):
        color = QtGui.QColor(0,0,0);
        qp.setPen(color)
        width = 100
        height = 100
        qp.drawRect(-width/2,-height/2,width,height)

    def drawRobot(self,qp):
        color = QtGui.QColor(122,122,122);
        qp.setPen(color)
        qp.setBrush(QtGui.QColor(122,122,122))
        width = 100
        height = 61
        qp.drawRect(-width/2,-height/2,width,height)

    def drawBox(self,qp):
        color = QtGui.QColor(60,60,60);
        qp.setPen(color)
        qp.setBrush(QtGui.QColor(60,60,60))
        width = 50
        height = 50
        qp.drawRect(-width/2,-height/2,width,height)


