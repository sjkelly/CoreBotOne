#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
from textcad import component, element, operation, utility
from magpie import hardware, bearing, motor, belt, shape
import subprocess
import copy
import math

class CoreBotConfig():
    def __init__(self,
                 buildVolume = [150, 150, 200],
                 stepper = ["Generic NEMA17", "Generic NEMA17", "Generic NEMA17"],
                 linear = ["LM8UU", "LM8UU", "LM8UU"],
                 bearing = ["605zz", "624zz", None],
                 ):
        self.buildVolume = buildVolume
        self.stepper = stepper
        self.linear = linear
        self.bearing = bearing

class CoreBotVitamins():
    def __init__(self,
                 coreBotConfig = None):
        self.stepper = [motor.Stepper(size=coreBotConfig.stepper[i]) for i in 
                        coreBotConfig.stepper]

class XCarriage(element.Primitive):
    def __init__(self,
                 linearBallBearing="LM8UU",
                 screw="M5",
                 beltSize="GT2",
                 beltWidth=6,
                 tolerance=0.05
                 ):
        element.Primitive.__init__(self, name="xcarriage")
        #Elements used in the design
        self.lb = bearing.LinearBallBearing(size=linearBallBearing,
                                            negative=True,
                                            radiusTolerance=tolerance)
        self.screw = hardware.CapScrew(size=screw, length=0)
        self.nut = hardware.Nut(size=screw)
        self.belt = belt.TimingBelt(size=beltSize, width=beltWidth)
        self.lbHolder = LinearBearingHolder(linearBallBearing=self.lb.size,
                                            useZipTie=False,
                                            radiusTolerance=tolerance)
        self.rodSpacing = self.lbHolder.width+beltWidth+self.nut.width*2
        self.tolerance = tolerance
        #Derived parameters
        self.topLength = self.lb.length * 2 + self.lbHolder.wall * 3
        self.bottomLength = self.lb.length + self.lbHolder.wall * 2
        self.nutDiameter = self.nut.width / math.cos(math.pi/6)
        #plateThick is the diameteric length of a nut
        self.plateThick = self.nutDiameter
        self.beltClampHoleSpacing = beltWidth + self.nut.width
        self.beltClampSpacing = self.lbHolder.length - self.nutDiameter
        self.slotLength = self.beltClampSpacing - self.nut.width*1.5
        self.tabRadius = self.lbHolder.length/6
        self.mountingHoles = [
                              [self.tabRadius, -self.nut.width/2, 0],
                              [self.topLength/2, -self.nut.width/2, 0],
                              [self.topLength-self.tabRadius, -self.nut.width/2, 0],
                              [self.topLength/2-self.lbHolder.length/2+self.tabRadius,
                               self.nut.width/2+self.lbHolder.width+self.rodSpacing,
                               0],
                              [self.topLength/2+self.lbHolder.length/2-self.tabRadius,
                               self.nut.width/2+self.lbHolder.width+self.rodSpacing,
                               0],
                              [self.topLength/2-self.lbHolder.length/2-self.nut.width/2,
                               self.rodSpacing, 0],
                              [self.topLength/2+self.lbHolder.length/2+self.nut.width/2, self.rodSpacing, 0]
                             ]
        #element properties
        self.location = [0, 0, 0]
        self.construction = self._construction()

    def _construction(self):
        #left top bearing holder
        ltBearingHolder = copy.deepcopy(self.lbHolder)
        ltBearingHolder.location = [self.lb.length+ltBearingHolder.wall, 0, ]

        #right top bearing holder
        rtBearingHolder = copy.deepcopy(self.lbHolder)

        #bottom center bearing holder
        bcBearingHolder = copy.deepcopy(self.lbHolder)
        bcBearingHolder.location = [self.topLength/2-self.bottomLength/2,
                                    self.rodSpacing,
                                    0]

        #bring linear bearings together
        asm = operation.Union([rtBearingHolder,
                               ltBearingHolder,
                               bcBearingHolder])

        #Construct a plate between the linear bearing holders, to be hulled later
        top = element.Cube()
        top.size = [self.topLength, 1, self.plateThick]
        top.location = [0, self.lbHolder.width, 0]

        #add tabs for mounting stuff
        tabs = []
        for i in range(0, 7):
            tabs.append(shape.D(radius=self.tabRadius,
                               length=self.plateThick,
                               extension=self.tabRadius+self.lbHolder.wall/2))
        for idx, tab in enumerate(tabs):
            if idx in [3, 4]:
                tab.rotation.axis = [0, 0, 1]
                tab.rotation.angle = 180
            hole = element.Hole(radius=self.screw.outerDiameter/2,
                                height=self.plateThick+0.2,
                                tolerance=self.tolerance)
            trap = NutTrap(self.nut)
            tab.location = self.mountingHoles[idx]
            trap.location = [self.mountingHoles[idx][0],
                             self.mountingHoles[idx][1],
                             self.plateThick-self.nut.height]
            hole.location = [self.mountingHoles[idx][0],
                             self.mountingHoles[idx][1],
                             -0.1]
            if idx == 1:
                trap.location[2] = -0.1
            if idx == 5:
                tab.rotation.axis = [0, 0, 1]
                tab.rotation.angle = -90
                trap.rotation.axis = [0, 0, 1]
                trap.rotation.angle = 30
                trap.location[2] = -0.1
            if idx == 6:
                tab.rotation.axis = [0, 0, 1]
                tab.rotation.angle = 90
                trap.rotation.axis = [0, 0, 1]
                trap.rotation.angle = 30
                trap.location[2] = -0.1
            if idx in [3,4]:
                tab = operation.Hull([tabs[idx+2], tab])
            if idx in [5,6]:
                tab = operation.Hull([top, tab])
            asm = (tab + asm) - (hole + trap)
        # Clean the hack hull stuff above
        self.lb.location = [self.topLength/2-self.lb.length/2, 
                            self.rodSpacing+self.lbHolder.width/2,
                            self.lbHolder.bearingCenter]
        self.lb.rotation.angle = 90
        self.lb.rotation.axis = [0, 1, 0]
        asm -= self.lb
        # Hole for tenioning
        tensionHole = element.Hole(radius=self.screw.outerDiameter/2,
                                   height=self.topLength,
                                   tolerance=self.tolerance)
        tensionHole.rotation.angle = 90
        tensionHole.rotation.axis = [0, 1, 0]
        tensionHole.location = [0,
                                self.rodSpacing/2 + self.lbHolder.width/2,
                                self.plateThick/2]
        asm -= tensionHole
        # Belt slot
        beltSlot = element.Cube()
        beltSlot.size = [self.slotLength, self.belt.width, self.plateThick+0.2]
        beltSlot.location = [self.topLength/2-self.slotLength/2,
                             self.lbHolder.width+self.nut.width,
                             -0.1]
        asm -= beltSlot
        #subtract nuttraps and holes for belt clamps
        nuts = [copy.deepcopy(NutTrap(self.nut)) for x in range(0, 4)]
        nuts[0].location = [self.topLength/2-self.beltClampSpacing/2,
                            self.lbHolder.width + self.nut.width/2, 0]
        nuts[1].location = [nuts[0].location[0]+self.beltClampSpacing,
                            nuts[0].location[1], 0]
        nuts[2].location = copy.copy(nuts[0].location)
        nuts[2].location[1] += self.beltClampHoleSpacing
        nuts[3].location = copy.copy(nuts[1].location)
        nuts[3].location[1] += self.beltClampHoleSpacing
        holes = [copy.deepcopy(element.Hole()) for x in range(0, 4)]
        for idx, nut in enumerate(nuts):
            holes[idx].tolerance = self.tolerance
            holes[idx].location = copy.deepcopy(nut.location)
            nut.location[2] = self.plateThick - self.nut.height +0.1
            holes[idx].radius = self.nut.diameter/2
            holes[idx].height = self.plateThick + 0.3
        for objs in nuts:
            # shift everything down for clean rendering
            objs.location[2] -= 0.1
            asm -= objs
        for objs in holes:
            # shift everything down for clean rendering
            objs.location[2] -= 0.1
            asm -= objs
        #subtract tensioning nutslots
        nutSlot1 = NutSlot(nut=self.nut, extension=self.plateThick)
        nutSlot1.rotation.angle = 90
        nutSlot1.rotation.axis = [0, 1, 0]
        nutSlot1.location = [self.topLength/2-self.beltClampSpacing/2-self.nut.height/2,
                             self.lbHolder.width/2+self.rodSpacing/2,
                             self.plateThick/2]
        nutSlot2 = NutSlot(nut=self.nut, extension=self.plateThick)
        nutSlot2.rotation.angle = 90
        nutSlot2.rotation.axis = [0, 1, 0]
        nutSlot2.location = [self.topLength/2+self.beltClampSpacing/2-self.nut.height/2,
                             self.lbHolder.width/2+self.rodSpacing/2,
                             self.plateThick/2]
        asm -= nutSlot1 + nutSlot2
        return asm


class YCarriage(element.Primitive):
    def __init__(self, 
                 linearBallBearing="LM8UU",
                 plateScrew="M3",
                 bearingScrew="M4",
                 ballBearing="624zz",
                 beltSize="GT2",
                 xCarriage=None,
                 bearingMount=None,
                 tolerance=0.05,
                 endstop=True,
                 endstopHoleSpacing=10,
                 endstopDepth=9
                 ):
        element.Primitive.__init__(self, name="ycarriage")
        self.lb = bearing.LinearBallBearing(size=linearBallBearing)
        self.bearingScrew = hardware.CapScrew(size=bearingScrew, length=0)
        self.bearingNut = hardware.Nut(size=bearingScrew)
        self.plateScrew = hardware.CapScrew(size=plateScrew, length=0)
        self.plateNut = hardware.LockNut(size=plateScrew)
        self.xcar = xCarriage
        self.belt = belt.TimingBelt(size=beltSize)
        self.bearing = bearing.BallBearing(size=ballBearing)
        self.bearingMount = bearingMount
        self.tolerance = 0.05
        self.endstop = endstop
        self.endstopHoleSpacing=endstopHoleSpacing
        self.endstopDepth = endstopDepth
        self.lbHolder = LinearBearingHolder(linearBallBearing=self.lb.size,
                                            useZipTie=False)
        self.lbHolderZ = self.bearing.outerDiameter+self.belt.height-self.lbHolder.bearingCenter+self.bearingMount.beltSeperation/2
        self.bearingSpacing = self.bearing.outerDiameter+self.xcar.lbHolder.bearingCenter*2-self.xcar.plateThick*2
        self.rodSpacing = self.xcar.rodSpacing
        self.height = self.lbHolderZ+self.lbHolder.height
        self.rodDiameter = self.xcar.lb.innerDiameter
        self.rodEncasementDiameter = self.lb.outerDiameter
        self.rodDepth = (self.lbHolderZ+self.lbHolder.height)*0.75
        self.construction = self._construction()

    def _construction(self):
        lbHold1 = copy.deepcopy(self.lbHolder)
        lbHold1.location=[-lbHold1.wall/2, -lbHold1.width/2, self.lbHolderZ]
        lbHold2 = copy.deepcopy(self.lbHolder)
        lbHold2.location=[-lbHold2.length+lbHold2.wall/2, -lbHold2.width/2, self.lbHolderZ]
        bearingBase = element.Cube([lbHold1.length*2-lbHold1.wall, lbHold1.width, self.lbHolderZ])
        bearingBase.location = [-lbHold2.length+lbHold2.wall/2, -lbHold2.width/2, 0]
        asm = lbHold1 + lbHold2 + bearingBase
        #rod construction
        rodEn1 = shape.D(radius=self.rodEncasementDiameter/2,
                        extension=(self.rodSpacing-self.lbHolder.width+self.lbHolder.wall)/2,
                        length=self.height)
        rodEn1.rotation.angle = 180
        rodEn1.rotation.axis = [0, 0, 1]
        rodEn2 = shape.D(radius=self.rodEncasementDiameter/2,
                        extension=(self.rodSpacing-self.lbHolder.width+self.lbHolder.wall)/2,
                        length=self.height)
        rodEn1.location = [0, self.rodSpacing/2, 0]
        rodEn2.location = [0, -self.rodSpacing/2, 0]
        asm += rodEn1 + rodEn2
        # endstop
        if self.endstop:
            endstopBack = element.Cube()
            endstopBack.size = [self.rodEncasementDiameter,
                                self.rodEncasementDiameter/2 + self.rodSpacing/2,
                                self.height/2]
            endstopBack.location = [-self.rodEncasementDiameter/2, 0, 0]
            asm += endstopBack
            endstopHole1 = element.Hole(radius=1.2, height=self.rodSpacing/2)
            endstopHole1.rotation.angle = -90
            endstopHole1.rotation.axis = [1, 0, 0]
            endstopHole2 = copy.deepcopy(endstopHole1)
            endstopHole1.location = [-self.endstopHoleSpacing/2,
                                     self.rodSpacing/4,
                                     self.endstopDepth]
            endstopHole2.location = [self.endstopHoleSpacing/2,
                                     self.rodSpacing/4,
                                     self.endstopDepth]
            asm -= endstopHole1 + endstopHole2
        # bearing holes, captured nut, and caphead clearance
        hole1 = element.Hole(radius=self.bearing.innerDiameter/2,
                            height=lbHold1.width+0.2,
                            tolerance=self.tolerance)
        hole1.rotation.angle = 90
        hole1.rotation.axis = [1,0,0]
        hole1.location = [self.bearingSpacing/2,
                          lbHold1.width/2+0.1,
                          self.bearing.outerDiameter/2]
        hole2 = copy.deepcopy(hole1)
        hole2.location = [-self.bearingSpacing/2,
                          lbHold1.width/2+0.1,
                          self.bearing.outerDiameter/2]
        cap1 = element.Hole(radius=self.bearingScrew.headDiameter/2,
                            height=self.rodSpacing,
                            tolerance=self.tolerance)
        cap1.rotation.angle = -90
        cap1.rotation.axis = [1,0,0]
        cap1.location = [self.bearingSpacing/2, lbHold1.width/2, self.bearing.outerDiameter/2]
        cap2 = copy.deepcopy(cap1)
        cap2.rotation.angle = -90
        cap2.rotation.axis = [1,0,0]
        cap2.location = [-self.bearingSpacing/2, lbHold1.width/2, self.bearing.outerDiameter/2]
        capNut1 = NutTrap(nut=self.bearingNut)
        capNut1.rotation.angle = -90
        capNut1.rotation.axis = [1,0,0]
        capNut1.location = [self.bearingSpacing/2, -lbHold1.width/2-0.1, self.bearing.outerDiameter/2]
        capNut2 = NutTrap(nut=self.bearingNut)
        capNut2.rotation.angle = -90
        capNut2.rotation.axis = [1,0,0]
        capNut2.location = [-self.bearingSpacing/2, -lbHold1.width/2-0.1, self.bearing.outerDiameter/2]
        asm -= hole1 + hole2 + cap1 + cap2 + capNut1 + capNut2
        # bearing clearance
        scoop1 = nShape(radius=self.bearing.outerDiameter/2+self.belt.height*1.75,
                        extension=self.lbHolder.length,
                        length=self.bearing.width*1.75)
        scoop1.rotation.angle = 90
        scoop1.rotation.axis = [1,0,0]
        lScoop = operation.Rotate(angle=90, axis=[0, 1, 0], elements=[scoop1])
        lScoop.location = [self.bearingSpacing/2, scoop1.length/2, self.bearing.outerDiameter/2]
        scoop2 = nShape(radius=self.bearing.outerDiameter/2+self.belt.height*1.75,
                        extension=self.lbHolder.length,
                        length=self.bearing.width*1.75)
        scoop2.rotation.angle = 270
        scoop2.rotation.axis = [1,0,0]
        rScoop = operation.Rotate(angle=90, axis=[0, 1, 0], elements=[scoop2])
        rScoop.location = [-self.bearingSpacing/2, -scoop2.length/2, self.bearing.outerDiameter/2]
        asm -= lScoop + rScoop
        # rod Holes
        rodHole1 = element.Hole(radius=self.rodDiameter/2,
                                height=self.rodDepth+0.1,
                                tolerance=self.tolerance)
        rodHole2 = copy.deepcopy(rodHole1)
        rodHole1.location = [0, self.rodSpacing/2, -0.1]
        rodHole2.location = [0, -self.rodSpacing/2, -0.1]
        asm -= rodHole1 + rodHole2
        # Captured nuts for Y carriage
        plateNut1 = NutTrap(self.plateNut)
        plateNut1.location = [0, self.lb.outerDiameter/2+self.plateNut.diameter/2, -0.1]
        plateNut2 = NutTrap(self.plateNut)
        plateNut2.location = [0, -self.lb.outerDiameter/2-self.plateNut.diameter/2, -0.1]
        plateHole1 = element.Hole(radius=self.plateNut.diameter/2, height=self.lbHolderZ+self.lbHolder.height+0.2)
        plateHole1.location = plateNut1.location
        plateHole2 = copy.deepcopy(plateHole1)
        plateHole2.location = plateNut2.location
        asm -= plateNut1 + plateNut2 + plateHole1 + plateHole2
        return asm


class YCarriagePlate(element.Primitive):
    def __init__(self, yCarriage=None):
        element.Primitive.__init__(self, name="ycarriage")
        ycar = yCarriage
        lbHolder = ycar.lbHolder
        height = lbHolder.lb.outerDiameter - lbHolder.clampFactor
        lbCap1 = LinearBearingHolderCap(ycar.lbHolder)
        lbCap2 = copy.deepcopy(lbCap1)
        lbCap1.location = [-lbHolder.wall/2, -lbHolder.width/2, 0]
        lbCap2.location = [-lbHolder.length+lbHolder.wall/2, -lbHolder.width/2, 0]
        d1 = shape.D(radius=ycar.rodEncasementDiameter/2,
                    length=height,
                    extension=lbHolder.wall/2)
        d1.location=[0, -ycar.lb.outerDiameter/2-ycar.plateNut.diameter/2, 0]
        d2 = copy.deepcopy(d1)
        d2.location=[0, ycar.lb.outerDiameter/2+ycar.plateNut.diameter/2, 0]
        d2.rotation.angle = 180
        d2.rotation.axis = [0, 0, 1]
        h1 = element.Hole(radius=ycar.plateNut.diameter/2, height=height+0.2)
        h1.location = d1.location
        h2 = element.Hole(radius=ycar.plateNut.diameter/2, height=height+0.2)
        h2.location = d2.location
        shaft = element.Hole(radius=ycar.lb.innerDiameter/2+1,
                             height=lbHolder.length*2)
        shaft.center = [True, True, True]
        shaft.rotation.angle=90
        shaft.rotation.axis=[0, 1, 0]
        shaft.location = [0, 0, ycar.lb.outerDiameter/2]
        asm = lbCap1 + lbCap2 + d1 + d2 
        asm -= h1 + h2 + shaft
        self.construction = asm


class MotorMount(element.Primitive):
    def __init__(self, rodDiameter=8,
                 mountLength=38,
                 stepper="GenericNEMA17",
                 beltSize="GT2",
                 nutSize="M5",
                 bearingSize="625zz",
                 linearBearing="LM8UU",
                 beltSeperation=16,
                 beltWidth=6,
                 holeDiameter=3.5):
        element.Primitive.__init__(self, name="motormount")
        #Used Elements
        self.belt = belt.TimingBelt(size=beltSize)
        self.bearing = bearing.BallBearing(size=bearingSize)
        self.lb = bearing.LinearBallBearing(size=linearBearing)
        self.beltSeperation = self.bearing.outerDiameter
        self.nut = hardware.Nut(size=nutSize)
        self.yRodMount = YRodMount(rodDiameter=8,
                                   mountLength=mountLength,
                                   stepper=stepper,
                                   beltSize=beltSize,
                                   nutSize=nutSize,
                                   linearBearing = linearBearing,
                                   beltSeperation=beltSeperation,
                                   beltWidth=beltWidth,
                                   holeDiameter=holeDiameter)
        self.stepper = motor.Stepper(size=stepper,
                                     negative=True,
                                     negativeLength=self.yRodMount.height*2+0.1)
        self.bearingHoldRadius = self.bearing.outerDiameter/2 + self.belt.height*1.25
        self.bearingHoldHeight = (self.yRodMount.height - self.yRodMount.beltPass[2])/2
        self.yRodMount.rodEncasementStart = -self.stepper.width/2
        self.yRodMount.rodStart = 0
        self.construction = self._construction()

    def _construction(self):
        mount = self.yRodMount
        mount.update()
        asm = self.yRodMount
        d1 = shape.D(radius=mount.tabRadius,
                    extension=self.stepper.width-mount.tabRadius+mount.length*0.25,
                    length=self.yRodMount.plateThick)
        d2 = shape.D(radius=self.yRodMount.tabRadius,
                    extension=self.stepper.width-mount.tabRadius+mount.length*0.25,
                    length=self.yRodMount.plateThick)
        d1.location=[mount.tabRadius,-self.stepper.width+mount.tabRadius,0]
        d2.location=copy.copy(d1.location)
        d2.location[0] += self.stepper.width-mount.tabRadius*2
        clear1 = element.Cylinder(radius=mount.tabRadius,
                                  height=mount.plateThick+0.2)
        clear2 = copy.deepcopy(clear1)
        clear1.location = mount.mountingHoles[0]
        clear2.location = mount.mountingHoles[2]
        motorPlate = operation.Hull([d1, d2])
        motorPlate -= clear1 + clear2
        asm += motorPlate
        self.stepper.location = [self.stepper.width/2, -self.stepper.width/2, 0]
        asm -= self.stepper
        return asm


class YBearingMount(component.Element):
    def __init__(self, rodDiameter=8,
                 mountLength=38,
                 stepper="GenericNEMA17",
                 beltSize="GT2",
                 nutSize="M5",
                 bearingSize="625zz",
                 linearBearing="LM8UU",
                 beltSeperation=16,
                 beltWidth=6,
                 holeDiameter=3.5):
        component.Element.__init__(self, name="yrodmount")
        #Used Elements
        self.stepper = motor.Stepper(size=stepper)
        self.belt = belt.TimingBelt(size=beltSize)
        self.bearing = bearing.BallBearing(size=bearingSize)
        self.lb = bearing.LinearBallBearing(size=linearBearing)
        self.beltSeperation = self.bearing.outerDiameter
        self.nut = hardware.Nut(size=nutSize)
        self.yRodMount = YRodMount(rodDiameter=8,
                                   mountLength=mountLength,
                                   stepper=stepper,
                                   beltSize=beltSize,
                                   nutSize=nutSize,
                                   linearBearing = linearBearing,
                                   beltSeperation=beltSeperation,
                                   beltWidth=beltWidth,
                                   holeDiameter=holeDiameter)
        self.bearingHoldRadius = self.bearing.outerDiameter/2 + self.belt.height*1.25
        self.bearingHoldHeight = (self.yRodMount.height - self.yRodMount.beltPass[2])/2
        self.yRodMount.rodEncasementStart = self.bearingHoldRadius
        self.yRodMount.rodStart = self.bearingHoldRadius*2 + rodDiameter*0.25
        self.construction = self._construction()

    def _construction(self):
        self.yRodMount.update()
        asm = self.yRodMount
        bearingSub = element.Hole(radius=self.bearingHoldRadius,
                                  height=self.lb.outerDiameter)
        bearingSub.location = [self.yRodMount.width/2,
                               self.bearingHoldRadius,
                               self.yRodMount.plateThick]
        # bearing bottom will interfere with the hole on the yRodMount.
        # This will make it fit tighter and push it upwards which is good
        bearingBot = shape.D(radius=self.bearingHoldRadius,
                            extension=self.yRodMount.length*0.75-self.bearingHoldRadius,
                            length=self.yRodMount.plateThick)
        bearingBot.location = [self.yRodMount.width/2,
                               self.bearingHoldRadius,
                               0]
        hole = element.Hole(radius=self.bearing.innerDiameter/2,
                            height=self.yRodMount.plateThick+0.2)
        hole.location=[self.yRodMount.width/2, self.bearingHoldRadius, -0.1]
        nut = NutTrap(nut=self.nut)
        nut.location = hole.location
        asm += bearingBot
        asm -= bearingSub + hole + nut
        return asm


class YRodMount(element.Primitive):
    def __init__(self,
                 rodDiameter=8,
                 mountLength=38,
                 stepper="GenericNEMA17",
                 beltSize="GT2",
                 nutSize="M5",
                 linearBearing="LM8UU",
                 beltSeperation=16,
                 beltWidth=6,
                 holeDiameter=3.5):
        element.Primitive.__init__(self, name="yrodmount")
        #Used Elements
        self.stepper = motor.Stepper(size=stepper)
        self.belt = belt.TimingBelt(size=beltSize)
        self.nut = hardware.Nut(size=nutSize)
        self.lb = bearing.LinearBallBearing(size=linearBearing)
        self.beltSeperation = beltSeperation
        self.beltWidth = beltWidth
        self.holeDiameter = holeDiameter
        self.beltWidth = beltWidth
        self.plateThick = self.nut.height*2
        self.length = mountLength
        self.width = self.stepper.width
        self.height = self.plateThick + rodDiameter + (self.lb.innerDiameter-self.lb.outerDiameter)/2
        self.rodEncasementDiameter = self.lb.outerDiameter
        self.rodEncasementStart = 0
        self.rodDiameter = rodDiameter
        self.rodCenter = self.plateThick + self.rodDiameter/2
        #should be reassigned by implementation
        self.rodDepth = self.length * 0.8
        #should be reassigned by implementation
        self.rodStart = self.length - self.rodDepth
        # beltpass represents space around the belt
        self.beltPass = [self.belt.height*3, self.width+0.2, self.beltWidth*2]
        # setup hole locations
        leftX = (self.width/2-self.beltSeperation/2-self.beltPass[0])/2
        rightX = self.width - leftX
        self.mountingHoles = [[leftX, self.length*0.25, -0.1],
                              [leftX, self.length*0.75, -0.1],
                              [rightX, self.length*0.25, -0.1],
                              [rightX, self.length*0.75, -0.1]
                             ]
        #tab radius
        self.tabRadius = leftX
        #countersink
        self.countersinkHeight = self.plateThick/3
        #construction
        self.construction = self._construction()

    def _construction(self):
        #asm = element.Cube(size=[self.width, self.length, self.plateThick])
        #rod encasement
        rodEn = shape.D(radius=self.rodEncasementDiameter/2,
                       extension=self.rodCenter,
                       length=self.length-self.rodEncasementStart)
        rodEn = operation.Rotate(axis=[1, 0, 0], angle=-90, elements=[rodEn])
        rodEn.location = [self.width/2, self.rodEncasementStart, self.rodCenter]
        asm = rodEn
        #Mounting holes
        for i in range(0, 4):
            holeTab = shape.D(radius=self.tabRadius,
                             extension=self.width/2-self.tabRadius,
                             length=self.plateThick)
            holeTab.location = copy.copy(self.mountingHoles[i])
            holeTab.location[2] = 0
            if i in [0, 1]:
                holeTab.rotation.angle = -90
                holeTab.rotation.axis = [0, 0, 1]
            if i in [2, 3]:
                holeTab.rotation.angle = 90
                holeTab.rotation.axis = [0, 0, 1]
            hole = element.Hole(radius=self.holeDiameter/2,
                                height=self.plateThick+0.2)
            hole.location = self.mountingHoles[i]
            countersink = element.Cone(topRadius=self.tabRadius,
                                       bottomRadius=self.holeDiameter/2,
                                       height=self.countersinkHeight+0.1)
            countersink.location = copy.copy(self.mountingHoles[i])
            countersink.location[2] = self.plateThick-self.countersinkHeight
            asm = asm + holeTab - hole - countersink
        #rod
        rod = element.Hole(radius=self.rodDiameter/2, height=self.length+0.1)
        rod.rotation.axis = [1, 0, 0]
        rod.rotation.angle = -90
        rod.location = [self.width/2, self.rodStart, self.rodCenter]
        asm -= rod
        return asm

    def update(self):
        self.construction = self._construction()


class LinearBearingHolder(element.Primitive):
    def __init__(self,
                 linearBallBearing="LM8UU",
                 useZipTie=True,
                 zipTieWidth=5,
                 zipTieHeight=2,
                 clampFactor=0.75,
                 lengthTolerance=0,
                 radiusTolerance=0.05
                 ):
        element.Primitive.__init__(self, name="linearbearingholder")
        self.lb = bearing.LinearBallBearing(size=linearBallBearing,
                                            negative=True,
                                            lengthTolerance=lengthTolerance,
                                            radiusTolerance=radiusTolerance)
        self.zipTieWidth = zipTieWidth
        if not useZipTie:
            self.zipTieWidth = 0
            zipTieHeight = 0
        self.wall = self.lb.outerDiameter*0.25
        self.length = self.lb.length + self.wall * 2
        self.width = self.lb.outerDiameter + self.wall * 2
        self.bearingCenter = self.width/2 + zipTieHeight
        self.clampFactor = self.lb.outerDiameter * clampFactor
        self.height = self.clampFactor + self.wall + zipTieHeight
        self.construction = self._construction()

    def _construction(self):
        core = element.Cylinder(radius=self.width/2, height=self.length)
        core.rotation.axis = [0, 1, 0]
        core.rotation.angle = 90
        core.location = [0, self.width/2, self.bearingCenter]
        squareBase1 = element.Cube(size=[(self.length-self.zipTieWidth)/2,
                                         self.width,
                                         self.bearingCenter])
        squareBase2 = copy.deepcopy(squareBase1)
        squareBase2.location = [(self.length+self.zipTieWidth)/2, 0, 0]
        rodClearance = dShapeNeg(radius=self.lb.innerDiameter/2+0.5,
                                 extension=self.lb.outerDiameter,
                                 length=self.length+0.2)
        rodClearance = operation.Rotate(axis=[0, 0, 1],
                                        angle=90,
                                        elements=[rodClearance])
        rodClearance = operation.Rotate(axis=[0, 1, 0],
                                        angle=90,
                                        elements=[rodClearance])
        rodClearance.location = [-0.1, self.width/2, self.bearingCenter]
        lb = self.lb
        lb.rotation.axis = [0, 1, 0]
        lb.rotation.angle = 90
        lb.location = [self.wall, self.width/2, self.bearingCenter]
        keep = element.Cube([self.length, self.width, self.height])
        asm = core + squareBase1 + squareBase2 - rodClearance - lb
        asm = operation.Intersection([asm, keep])
        return asm

    def update(self):
        self.construction = self._construction()


class LinearBearingHolder(element.Primitive):
    def __init__(self,
                 linearBallBearing="LM8UU",
                 useZipTie=True,
                 zipTieWidth=5,
                 zipTieHeight=2,
                 clampFactor=0.75,
                 lengthTolerance=0,
                 radiusTolerance=0.05,
                 ):
        element.Primitive.__init__(self, name="linearbearingholder")
        self.lb = bearing.LinearBallBearing(size=linearBallBearing,
                                            negative=True,
                                            lengthTolerance=lengthTolerance,
                                            radiusTolerance=radiusTolerance)
        self.zipTieWidth = zipTieWidth
        if not useZipTie:
            self.zipTieWidth = 0
            zipTieHeight = 0
        self.wall = self.lb.outerDiameter*0.25
        self.length = self.lb.length + self.wall * 2
        self.width = self.lb.outerDiameter + self.wall * 2
        self.bearingCenter = self.width/2 + zipTieHeight
        self.clampFactor = self.lb.outerDiameter * clampFactor
        self.height = self.clampFactor + self.wall + zipTieHeight
        self.construction = self._construction()

    def _construction(self):
        core = element.Cylinder(radius=self.width/2, height=self.length)
        core.rotation.axis = [0, 1, 0]
        core.rotation.angle = 90
        core.location = [0, self.width/2, self.bearingCenter]
        squareBase1 = element.Cube(size=[(self.length-self.zipTieWidth)/2,
                                         self.width,
                                         self.bearingCenter])
        squareBase2 = copy.deepcopy(squareBase1)
        squareBase2.location = [(self.length+self.zipTieWidth)/2, 0, 0]
        rodClearance = dShapeNeg(radius=self.lb.innerDiameter/2+0.5,
                                 extension=self.lb.outerDiameter,
                                 length=self.length+0.2)
        rodClearance = operation.Rotate(axis=[0, 0, 1],
                                        angle=90,
                                        elements=[rodClearance])
        rodClearance = operation.Rotate(axis=[0, 1, 0],
                                        angle=90,
                                        elements=[rodClearance])
        rodClearance.location = [-0.1, self.width/2, self.bearingCenter]
        lb = self.lb
        lb.rotation.axis = [0, 1, 0]
        lb.rotation.angle = 90
        lb.location = [self.wall, self.width/2, self.bearingCenter]
        keep = element.Cube([self.length, self.width, self.height])
        asm = core + squareBase1 + squareBase2 - rodClearance - lb
        asm = operation.Intersection([asm, keep])
        return asm

    def update(self):
        self.construction = self._construction()


class LinearBearingHolderCap(element.Primitive):
    def __init__(self, linearBearingHolder=None):
        element.Primitive.__init__(self, name="linearbearingholdercap")
        self.holder = linearBearingHolder
        self.construction = self._construction()

    def _construction(self):
        h = self.holder
        base = element.Cube()
        base.size = [h.length, h.width, h.lb.outerDiameter - h.clampFactor]
        core = element.Cylinder()
        core.radius = h.width/2
        core.height = h.length
        core.location = [0, h.width/2, h.lb.outerDiameter/2]
        core.rotation.angle = 90
        core.rotation.axis = [0, 1, 0]
        lb = copy.deepcopy(h.lb)
        lb.rotation = core.rotation
        lb.location = [h.wall, h.width/2, h.lb.outerDiameter/2]
        return operation.Intersection([base, core]) - lb


class BeltRetainer(element.Primitive):
    def __init__(self, yBearingMount=None, height=2):
        element.Primitive.__init__(self, name="beltretainer")
        outer = element.Cylinder(radius=yBearingMount.bearingHoldRadius,
                                 height=height)
        inner = element.Hole(radius=yBearingMount.bearing.innerDiameter/2,
                             height=height+0.2)
        inner.location = [0, 0, -0.1]
        self.construction = outer - inner


class BeltClamp(element.Primitive):
    def __init__(self, xCarriage=None, thickness=3):
        element.Primitive.__init__(self, name="beltClamp")
        self.xcar = xCarriage
        self.thickness = thickness
        self.construction = self._construction()

    def _construction(self):
        xcar = self.xcar
        thickness = self.thickness
        a = element.Cylinder(radius=xcar.nut.width-0.5, height=thickness)
        b = copy.deepcopy(a)
        b.location = [xcar.beltClampHoleSpacing, 0, 0]
        asm = operation.Hull([a, b])
        c = element.Hole(radius=xcar.screw.outerDiameter/2,
                         height=thickness+0.2)
        d = copy.deepcopy(c)
        c.location = [0, 0, -0.1]
        d.location = [xcar.beltClampHoleSpacing, 0, -0.1]
        asm -= c + d
        return asm


def dShapeNeg(radius, extension, length):
    """Makes a D shape to subtract from linear bearing holders"""
    a = element.Hole(radius=radius, height=length)
    b = element.Cube(size=[radius*2, extension, length])
    b.location = [-radius, 0, 0]
    return a + b


class nShape(element.Primitive):
    def __init__(self, radius=2, extension=2, length=2):
        element.Primitive.__init__(self, name="dshape")
        self.radius = radius
        self.extension = extension
        self.length = length
        a = element.Cylinder(radius=radius, height=length)
        b = element.Cube(size=[radius*2, extension, length])
        c = element.Cube(size=[extension, radius*2, length])
        b.location = [-radius, 0, 0]
        c.location = [0, -radius, 0]
        d = element.Cube(size=[extension, extension, length])
        self.construction = a + b + c + d


class NutTrap(element.Primitive):
    def __init__(self, nut=None):
        element.Primitive.__init__(self, name="nuttrap")
        self.construction = element.Ntube(apothem=nut.width/2,
                                          sides=6,
                                          height=nut.height+0.1)


class NutSlot(element.Primitive):
    def __init__(self, nut=None, extension=0):
        element.Primitive.__init__(self, name="nuttrap")
        ext = element.Cube()
        ext.size = [extension, nut.width, nut.height+0.1]
        ext.location = [0, -nut.width/2, 0]
        self.construction = NutTrap(nut=nut) + ext


class DrillTemplate(element.Primitive):
    def __init__(self, yRodMount=None, holeDiameter=2, wall=4, wallHeight=12):
        element.Primitive.__init__(self, name="drilltemplate")
        asm = element.Cube()
        asm.size = [yRodMount.width+yRodMount.length, yRodMount.length+wall, wall]
        sideWall = element.Cube()
        sideWall.size = [yRodMount.width+yRodMount.length, wall, wallHeight]
        sideWall.location = [0, yRodMount.length, 0]
        asm += sideWall
        for holeLocation in yRodMount.mountingHoles:
            hole = element.Hole(radius=holeDiameter/2,
                                height=wall+0.2)
            hole.location = holeLocation
            asm -= hole
        sideHole1 = element.Hole(radius=holeDiameter/2,
                                 height=wall+0.2)
        sideHole2 = element.Hole(radius=holeDiameter/2,
                                 height=wall+0.2)
        sideHole1.location = [yRodMount.width+yRodMount.length*1/3,
                              yRodMount.length*1/3,
                              -0.1]
        sideHole2.location = [yRodMount.width+yRodMount.length*2/3,
                              yRodMount.length*2/3,
                              -0.1]
        asm -= sideHole1 + sideHole2
        self.construction = asm

if __name__ == "__main__":
    #Config
    stepper = "GenericNEMA17"
    linearBearing = "LM8UU"
    screw = "M3"
    beltSize = "GT2"
    beltWidth = 6
    woodWidth = 38.3
    ybearing = YBearingMount(rodDiameter=8,
                             mountLength=woodWidth,
                             stepper=stepper,
                             beltSize=beltSize,
                             bearingSize="625zz",
                             beltWidth=beltWidth,
                             holeDiameter=3.5)
    motorMount = MotorMount(rodDiameter=8,
                            mountLength=woodWidth,
                            stepper=stepper,
                            beltSize=beltSize,
                            bearingSize="625zz",
                            beltWidth=beltWidth,
                            holeDiameter=3.5)
    xcar = XCarriage(linearBallBearing=linearBearing,
                     screw=screw,
                     beltSize=belt,
                     beltWidth=beltWidth
                     )
    beltRetainer = BeltRetainer(yBearingMount=ybearing, height=2)
    beltClamp = BeltClamp(xCarriage=xcar, thickness=4)
    ycar = YCarriage(xCarriage=xcar,
                     bearingMount=ybearing)
    ycarPlate = YCarriagePlate(yCarriage=ycar)
    drillTemplate = DrillTemplate(yRodMount=YRodMount())

    def textcadArgs(name):
        cmd = "textcad -o ./scad/" + name + ".scad ./json/" + name + ".json"
        return cmd.split()

    utility.export(xcar, "./json/xcar.json")
    subprocess.Popen(textcadArgs("xcar"))
    utility.export(ybearing, "./json/ybearing.json")
    subprocess.Popen(textcadArgs("ybearing"))
    utility.export(beltRetainer, "./json/belt_retainer.json")
    subprocess.Popen(textcadArgs("belt_retainer"))
    utility.export(motorMount, "./json/motor_mount.json")
    subprocess.Popen(textcadArgs("motor_mount"))
    utility.export(beltClamp, "./json/belt_clamp.json")
    subprocess.Popen(textcadArgs("belt_clamp"))
    utility.export(ycar, "./json/ycar.json")
    subprocess.Popen(textcadArgs("ycar"))
    utility.export(drillTemplate, "./json/drill_template.json")
    subprocess.Popen(textcadArgs("drill_template"))
    utility.export(ycarPlate, "./json/ycar_plate.json")
    subprocess.Popen(textcadArgs("ycar_plate"))
