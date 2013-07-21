#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
from textcad import *
from magpie import *
import numpy
import copy

class XCarriage(element.Primitive):
    def __init__(self, linearBallBearing="LM8UU", screw="M3", rodSpacing=50, zipTieWidth = 5, zipTieHeight=2):
        element.Primitive.__init__(self, name="xcarriage")
        #Elements used in the design
        self.lb = bearing.LinearBallBearing(size=linearBallBearing)
        self.screw = hardware.CapScrew(size=screw, length=0)
        self.nut = hardware.Nut(size=screw)
        self.lockNut = hardware.LockNut(size=screw)
        self.rodSpacing = rodSpacing
        self.zipTieWidth = zipTieWidth
        self.lbHolder = LinearBearingHolder(linearBallBearing=self.lb.size, zipTieWidth=self.zipTieWidth)
        #Derived parameters
        self.topLength = self.lb.length * 2 + self.lbHolder.wall * 3
        self.bottomLength = self.lb.length + self.lbHolder.wall * 2
        self.plateThick = self.nut.height * 2
        self.beltClampHoleSpacing = self.rodSpacing - self.lbHolder.width - self.nut.width *2
        self.beltClampSpacing = self.lbHolder.length/2
        self.mountingTabRadius = self.lbHolder.length/4-self.zipTieWidth/4
        self.mountingHoles = [
                             [self.mountingTabRadius, -self.nut.width/2, 0],
                             [self.topLength/2, -self.nut.width/2, 0],
                             [self.topLength-self.mountingTabRadius, -self.nut.width/2, 0],
                             [self.topLength/2-self.mountingTabRadius-self.zipTieWidth/2, self.nut.width/2+self.lbHolder.width+self.rodSpacing, 0],
                             [self.topLength/2+self.mountingTabRadius+self.zipTieWidth/2, self.nut.width/2+self.lbHolder.width+self.rodSpacing, 0],
                             ]
        #element properties
        self.location = [0, 0, 0]
        self.construction = self._construction()
    def _construction(self):
        #Construct a plate between the linear bearing holders
        top = element.Cube()
        top.size = [self.topLength, 1, self.plateThick]
        top.location = [0, self.lbHolder.width, 0]
        bottom = element.Cube()
        bottom.size = [self.bottomLength, 1, self.plateThick]
        bottom.location = [self.topLength/2-self.bottomLength/2, self.rodSpacing-1, 0]
        plate = operation.Hull([top,bottom])

        #left top bearing holder
        ltBearingHolder = copy.deepcopy(self.lbHolder)
        ltBearingHolder.location = [self.lb.length+ltBearingHolder.wall, 0, 0]

        #right top bearing holder
        rtBearingHolder = copy.deepcopy(self.lbHolder)

        #bottom center bearing holder
        bcBearingHolder = copy.deepcopy(self.lbHolder)
        bcBearingHolder.location = [self.topLength/2-self.bottomLength/2, self.rodSpacing, 0]

        #bring linear bearings together
        asm = operation.Union([rtBearingHolder, ltBearingHolder ,bcBearingHolder])
        #add plate
        asm += plate

        #subtract nuttraps and holes for belt clamps
        nuts = [copy.deepcopy(nutTrap(self.nut)) for x in range(0,4)]
        nuts[0].location = [self.topLength/2-self.beltClampSpacing/2,
                            self.lbHolder.width + self.nut.width, 0]
        nuts[1].location = [nuts[0].location[0]+self.beltClampSpacing,
                            nuts[0].location[1], 0]
        nuts[2].location = copy.copy(nuts[0].location)
        nuts[2].location[1] += self.beltClampHoleSpacing
        nuts[3].location = copy.copy(nuts[1].location)
        nuts[3].location[1] += self.beltClampHoleSpacing
        holes = [copy.deepcopy(element.Hole()) for x in range(0,4)]
        for idx, nut in enumerate(nuts):
            holes[idx].location = nut.location
            holes[idx].radius = self.nut.diameter/2
            holes[idx].height = self.plateThick + 0.3
        for objs in nuts:
            objs.location[2] -= 0.1 #shift everything down for clean rendering
            asm -= objs
        for objs in holes:
            objs.location[2] -= 0.1 #shift everything down for clean rendering
            asm -= objs

        #add tabs for mounting stuff
        tabs = []
        for i in range(0,5):
            tabs.append(dShape(radius=self.mountingTabRadius,
                                     length=self.plateThick,
                                     extension=self.mountingTabRadius))
        #tabNut = nutTrap(self.nut)
        #tabNut.location = [0, 0, self.plateThick]
       # tabNut.highlight = True
        #for i in range(4,5):
            #
        for idx,tab in enumerate(tabs):
            if idx in [3,4]:
                tab = operation.Rotate(axis=[0, 0, 1], angle=180, elements=[tab])
            hole = element.Hole(radius=self.screw.outerDiameter/2, height=self.plateThick+0.2)
            hole.location = [0, 0, -0.1]
            trap = nutTrap(self.nut)
            trap.location = [0,0,self.plateThick-self.nut.height]
            tab -= (hole+trap)
            tab.location = self.mountingHoles[idx]
            #tab -= copy.deepcopy(tabNut)
            asm += tab
        return asm




class YCarriage(element.Primitive):
    def __init__(self, linearBallBearing="LM8UU",
                 screw="M3",
                 rodSpacing=50,
                 zipTieWidth = 5
                 ):
        element.Primitive.__init__(self, name="xcarriage")
        self.lb = bearing.LinearBallBearing(size=linearBallBearing)
        self.screw = hardware.CapScrew(size=screw, length=0)
        self.nut = hardware.Nut(size=screw)
        self.lockNut = hardware.LockNut(size=screw)
        self.rodSpacing = rodSpacing
        self.zipTieWidth = zipTieWidth
        self.lbHolder = LinearBearingHolder(linearBallBearing=self.lb.size, zipTieWidth=self.zipTieWidth)
        self.zipTieWidth = 5
        self.location = [0, 0, 0]
        self.construction = self._construction()

    def _construction(self):
        return asm



class MotorMount(element.Primitive):
    def __init__(self, stepper="GenericNEMA17", screw="M3", rodDiameter=8, mountLength=38):
        element.Primitive.__init__(self, name="motormount")
        #Elements used in the design
        self.stepper = motor.Stepper(size=stepper)
        self.screw = hardware.CapScrew(size=screw, length=0)
        self.nut = hardware.Nut(size=screw)
        #Derived parameters
        #element properties
        self.location = [0, 0, 0]
        self.construction = self._construction()
    def _construction(self):
        pass

class YRodMount(component.Element):
    def __init__(self, rodDiameter=8, mountLength=38, stepper="GenericNEMA17", belt="GT2", beltSeperation=16, beltWidth=6, holeDiameter=3.5):
        component.Element.__init__(self, name="yrodmount")
        #Used Elements
        self.stepper = motor.Stepper(size=stepper)
        self.belt = belt.TimingBelt(size=belt)
        

class LinearBearingHolder(component.Element):

    def __init__(self, linearBallBearing="LM8UU", zipTieWidth = 5, zipTieHeight=2):
        component.Element.__init__(self, name="linearbearingholder")
        self.lb = bearing.LinearBallBearing(size=linearBallBearing, negative=True)
        self.zipTieWidth = zipTieWidth
        self.location = [0, 0, 0]
        self.wall = self.lb.outerDiameter*0.25
        self.length = self.lb.length + self.wall * 2
        self.width = self.lb.outerDiameter + self.wall * 2
        self.bearingCenter = self.width/2 + zipTieHeight
        self.height = self.lb.outerDiameter * 0.60 + self.wall + zipTieHeight
        self.construction = self._construction()

    def _construction(self):
        core = element.Cylinder(radius=self.width/2, height=self.length)
        core.rotation.axis = [0, 1, 0]
        core.rotation.angle = 90
        core.location = [0, self.width/2, self.bearingCenter]

        squareBase1 = element.Cube(size=[(self.length-self.zipTieWidth)/2, self.width, self.bearingCenter])
        squareBase2 = copy.deepcopy(squareBase1)

        squareBase2.location = [(self.length+self.zipTieWidth)/2, 0, 0]

        rodClearance = dShapeNeg(radius = self.lb.innerDiameter/2+0.5,
                                 extension=self.lb.outerDiameter,
                                 length=self.length+0.2)
        rodClearance = operation.Rotate(axis=[0, 0, 1], angle=90, elements=[rodClearance])
        rodClearance = operation.Rotate(axis=[0, 1, 0], angle=90, elements=[rodClearance])
        rodClearance.location = [-0.1, self.width/2, self.bearingCenter]
        lb = self.lb
        lb.rotation.axis = [0, 1, 0]
        lb.rotation.angle = 90
        lb.location = [self.wall, self.width/2, self.bearingCenter]
        keep = element.Cube([self.length, self.width, self.height])
        asm = core + squareBase1 + squareBase2 - rodClearance - lb
        asm = operation.Intersection([asm, keep])
        return asm

def dShapeNeg(radius, extension, length):
    """Makes a D shape to subtract from linear bearing holders"""
    a = element.Hole(radius=radius, height=length)
    b = element.Cube(size=[radius*2, extension, length])
    b.location = [-radius, 0, 0]
    return a + b

def dShape(radius, extension, length):
    """Makes a D shape to subtract from linear bearing holders"""
    a = element.Cylinder(radius=radius, height=length)
    b = element.Cube(size=[radius*2, extension, length])
    b.location = [-radius, 0, 0]
    return a + b

def nutTrap(nut):
    return element.Ntube(apothem=nut.width/2, sides=6, height=nut.height+0.1)

if __name__=="__main__":
    #Config
    stepper = "GenericNEMA17"
    linearBearing = "LM8UU"
    screw = "M3"
    rodSpacing = 45
    zipTieWidth = 5
    zipTieHeight = 2
    #ytest = 
    xcar = XCarriage(linearBallBearing=linearBearing,
                     screw=screw,
                     rodSpacing=rodSpacing,
                     zipTieWidth=zipTieWidth,
                     zipTieHeight=zipTieHeight
                    )
    utility.export(xcar, "./json/xcar.json")
