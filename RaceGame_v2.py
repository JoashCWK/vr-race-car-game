from __future__ import absolute_import, division, print_function, unicode_literals
import math,random, time
import demo
import pi3d
import serial
import RPi.GPIO as GPIO
import time
import pi3d
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import sys


#----------------------------------------------
#RF24
def SetupRadio():
    GPIO.setmode(GPIO.BCM)

    pipes = [[0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]
    radio = NRF24(GPIO, spidev.SpiDev())
    radio.begin(0,25)

    radio.setPayloadSize(32)
    radio.setChannel(0x76)
    radio.setDataRate(NRF24.BR_1MBPS)
    radio.setPALevel(NRF24.PA_MIN)

    radio.setAutoAck(True)
    radio.enableDynamicPayloads()
    radio.enableAckPayload()

    radio.openReadingPipe(1, pipes[0])
    radio.printDetails()
    radio.startListening()
    return radio

class carChoice():
    def __init__(self, carChoice):
        self.carChoice = pi3d.Model(file_string=carChoice, x = 0, y = 0, z = 15)
        self.carChoice.draw()
class car():
    #To be set before the game begins
    LENGTH_CAR = 0
    WIDTH_CAR = 0
    DIST_CAMfromAxis = 0
    DIST_STEERINGfromAxis = 0
    maxWheelAngle = 0
    
    #Constants
    rads = 0.017453292512
    resistance = 0.1
    maxSpeed = 195
    maxBrake = 0.37
    steeringRatioPower = 1.4
    steeringRatioConstant = maxWheelAngle/(512**steeringRatioPower)
    multiplier = 0.1
    
    #Variables
    brake = 0
    speed = 0
    acceleration = 0
    but1Buffer = 1
    brakeReleaseMode = True
    countBrakeRelease = 1
    finalSpeed = 0
    carAngle = 0
    steeringValue = 0
    wheelAngle = 0
    compWheelAngle = 0
    wheelRadius = 1000000000
    compWheelRadius = 1000000000
    carRadius = 1000000000
    camRadius = 1000000000
    steeringRadius = 1000000000
    carAngularVel = 0
    carRotationVel = 0
    carSpeed = 0
    camSpeed = 0
    camAngleFromTangent = 0
    steeringSpeed = 0
    steeringAngleFromTangent = 0
    speedLeftWheel = 0
    speedRightWheel = 0
    angleLeftWheel = 0
    angleRightWheel = 0
    camParallelSpeed = 0
    camNormalSpeed = 0
    steeringParallelSpeed = 0
    steeringNormalSpeed = 0
    positionCAMERA = [0,0,0,0,0,0]
    
    def __init__(self, carBody, leftWheel, rightWheel, steering):
        self.carBody = pi3d.Model(file_string=carBody)
        self.leftWheel = pi3d.Model(file_string=leftWheel)
        self.rightWheel = pi3d.Model(file_string=rightWheel)
        self.steering = pi3d.Model(file_string=steering)
        self.CAMERA = pi3d.StereoCam(separation = 0, interlace = 0)
        
    def setDimensions(self, length, width, camDist, steerDist, maxWheelAngle):
        self.LENGTH_CAR = length
        self.WIDTH_CAR = width
        self.DIST_CAMfromAxis = camDist
        self.DIST_STEERINGfromAxis = steerDist
        self.maxWheelAngle = maxWheelAngle
        self.steeringRatioConstant = self.maxWheelAngle/(512**1.5)

    def setPosition(self, positionCar, positionLeftWheel, positionRightWheel, positionSteering, positionCAMERA):
        self.carBody.position(positionCar[0], positionCar[1], positionCar[2])
        self.leftWheel.position(positionLeftWheel[0], positionLeftWheel[1], positionLeftWheel[2])
        self.rightWheel.position(positionRightWheel[0], positionRightWheel[1], positionRightWheel[2])
        self.steering.position(positionSteering[0], positionSteering[1], positionSteering[2])
        self.positionCAMERA = positionCAMERA
        self.CAMERA.move_camera((self.positionCAMERA[0], self.positionCAMERA[1], self.positionCAMERA[2]), self.positionCAMERA[3], self.positionCAMERA[4], self.positionCAMERA[5])

    #def setRotation(self, rotCar, rotLeftWheel, rotRightWheel, rotSteering):
        #self.carBody.

    def calcKinematics(self, val1, val2, but1):
        self.brake = val2/200
        if self.brake == 0:
            self.acceleration = val1/200
        if self.brake!= 0:
            self.acceleration = 0
        if not self.brakeReleaseMode:
            self.speed += self.acceleration - self.brake - self.resistance       
        if self.speed < 0:
            self.speed = 0
            self.acceleration = 0
        if self.speed > self.maxSpeed:
            self.speed = self.maxSpeed
        if self.acceleration == 0 and self.brake == 0:
            if self.speed < 3:
                self.brakeReleaseMode = True
        if self.speed == 0 and self.brake != 0:
            self.brakeReleaseMode = True
        if self.brakeReleaseMode:
            if self.brake > self.maxBrake:
                self.speed = 0
            else:
                self.countBrakeRelease += 1
                self.speed = (self.maxBrake-self.brake)*7/math.log(self.countBrakeRelease, 5)
            if self.acceleration != 0:
                self.brakeReleaseMode = False
                self.countBrakeRelease = 1
        if but1 == 0 and self.but1Buffer == 1:
            self.multiplier = - self.multiplier
        self.but1Buffer = but1
        self.finalSpeed = self.speed*self.multiplier

    def calcAngleSpeed(self, val3):
        self.steeringValue = val3 - 512
        self.wheelAngle = abs((self.steeringRatioConstant*(self.steeringValue**self.steeringRatioPower)))
        if self.wheelAngle!= 0:
            self.compWheelAngle = math.degrees(math.atan(self.LENGTH_CAR/((self.LENGTH_CAR/math.tan(self.wheelAngle*self.rads))+self.WIDTH_CAR))) #Direction of the other wheel
        else:
            self.compWheelAngle = 0
        if self.wheelAngle != 0:
            #if not self.carSpeed**2/(0.8*10) > self.carRadius:
            self.wheelRadius = self.LENGTH_CAR/math.sin(self.wheelAngle*self.rads)
            self.compRadius = self.LENGTH_CAR/math.sin(self.compWheelAngle*self.rads)
            self.carRadius = self.WIDTH_CAR/2 + self.LENGTH_CAR/math.tan(self.wheelAngle*self.rads)
            self.camRadius = math.sqrt(self.carRadius**2 + self.DIST_CAMfromAxis**2) 
            self.steeringRadius = math.sqrt(self.carRadius**2 + self.DIST_STEERINGfromAxis**2)
            self.carSpeed = self.finalSpeed
            self.carAngularVel = (self.carSpeed/self.carRadius)/self.rads
            self.carRotationVel = self.carAngularVel/2
            self.carSpeed = self.carSpeed*math.cos(self.carRotationVel*self.rads)
            self.speedWheel = self.carSpeed/self.carRadius*self.wheelRadius
            self.compSpeedWheel = self.carSpeed/self.carRadius*self.compRadius
    
        else:
            self.speedWheel = self.finalSpeed
            self.compSpeedWheel = self.finalSpeed
            self.carRadius = 1000000000000000
            self.camRadius = self.carRadius
            self.steeringRadius = self.carRadius
            self.carAngularVel = 0
            self.carRotationVel = 0
            self.carSpeed = self.finalSpeed
            
        self.camSpeed = self.carSpeed/self.carRadius*self.camRadius
        self.camAngleFromTangent = math.degrees(math.acos(self.carRadius/self.camRadius)) #a constant

        self.steeringSpeed = self.carSpeed/self.carRadius*self.steeringRadius
        self.steeringAngleFromTangent = math.degrees(math.acos(self.carRadius/self.steeringRadius))
    
        #IF turning left
        if self.steeringValue > 0:
            self.angleLeftWheel = -self.wheelAngle
            self.angleRightWheel = -self.compWheelAngle
            self.speedLeftWheel = self.speedWheel
            self.speedRightWheel = self.compSpeedWheel
            self.camAngleFromTangent = -self.camAngleFromTangent
            self.steeringAngleFromTangent = -self.steeringAngleFromTangent
        #IF turning right
        else:
            self.angleLeftWheel = self.compWheelAngle
            self.angleRightWheel = self.wheelAngle
            self.speedLeftWheel = self.compSpeedWheel
            self.speedRightWheel = self.speedWheel
            self.camAngleFromTangent = self.camAngleFromTangent
            self.steeringAngleFromTangent = self.steeringAngleFromTangent

        self.camParallelSpeed = self.camSpeed*math.cos(self.camAngleFromTangent*self.rads)
        self.camNormalSpeed = self.camSpeed*math.sin(self.camAngleFromTangent*self.rads)
        self.steeringParallelSpeed = self.steeringSpeed*math.cos(self.steeringAngleFromTangent*self.rads)
        self.steeringNormalSpeed = self.steeringSpeed*math.sin(self.steeringAngleFromTangent*self.rads)

    def rotateSteering(self):
        self.leftWheel.rotateToY(self.angleLeftWheel + self.carAngle)
        self.rightWheel.rotateToY(self.angleRightWheel + self.carAngle)

    def calcRotation(self):
        if self.steeringValue > 0:
            self.carRotationVel = -self.carRotationVel
        else:
            self.carRotationVel = self.carRotationVel
            
    def rotateModel(self, rotationVel):
        self.carAngle += rotationVel
        self.carBody.rotateIncY(rotationVel)
        self.leftWheel.rotateIncY(rotationVel)
        self.rightWheel.rotateIncY(rotationVel)
        self.steering.rotateIncY(rotationVel)
        self.positionCAMERA[3] -= rotationVel

    def translateModel(self):
        self.carBody.translate(-math.cos(self.carAngle*self.rads)*self.carSpeed, 0, math.sin(self.carAngle*self.rads)*self.carSpeed)
        self.leftWheel.translate(-math.cos((self.angleLeftWheel + self.carAngle)*self.rads)*self.speedLeftWheel, 0, math.sin((self.angleLeftWheel + self.carAngle)*self.rads)*self.speedLeftWheel)
        self.rightWheel.translate(-math.cos((self.angleRightWheel + self.carAngle)*self.rads)*self.speedRightWheel, 0, math.sin((self.angleRightWheel + self.carAngle)*self.rads)*self.speedRightWheel)
        self.steering.translate(-math.cos((self.carAngle)*self.rads)*self.steeringParallelSpeed +math.sin((self.carAngle)*self.rads)*self.steeringNormalSpeed, 0, math.sin((self.carAngle)*self.rads)*self.steeringParallelSpeed + math.cos((self.carAngle)*self.rads)*self.steeringNormalSpeed)
        self.positionCAMERA[0] += -math.cos((self.carAngle)*self.rads)*self.camParallelSpeed + math.sin((self.carAngle)*self.rads)*self.camNormalSpeed
        self.positionCAMERA[2] += math.sin((self.carAngle)*self.rads)*self.camParallelSpeed + math.cos((self.carAngle)*self.rads)*self.camNormalSpeed

    def moveModel(self):
        self.calcRotation()
        self.rotateModel(self.carRotationVel)
        self.CAMERA.move_camera((self.positionCAMERA[0], self.positionCAMERA[1], self.positionCAMERA[2]), self.positionCAMERA[3], self.positionCAMERA[4], self.positionCAMERA[5])
        self.translateModel()
        self.CAMERA.move_camera((self.positionCAMERA[0], self.positionCAMERA[1], self.positionCAMERA[2]), self.positionCAMERA[3], self.positionCAMERA[4], self.positionCAMERA[5])
        self.rotateModel(self.carRotationVel)
        self.CAMERA.move_camera((self.positionCAMERA[0], self.positionCAMERA[1], self.positionCAMERA[2]), self.positionCAMERA[3], self.positionCAMERA[4], self.positionCAMERA[5])
        
    def drawModel(self):
        self.carBody.draw()
        self.leftWheel.draw()
        self.rightWheel.draw()
        self.steering.draw()

class instruments():
    def __init__(self):
        CAMERA2D = pi3d.Camera(is_3d=False)
        FLATSH = pi3d.Shader("uv_flat")
        speedometer_tex = pi3d.Texture("textures/speedometer.png")
        ndl_tex = pi3d.Texture("textures/instrument_needle.png")
        self.speedometerLeft = pi3d.ImageSprite(speedometer_tex, shader, camera=CAMERA2D,
                  w=170, h=170, x=-800, y=-400, z=5)
        self.speedometerRight = pi3d.ImageSprite(speedometer_tex, shader, camera=CAMERA2D,
                  w=170, h=170, x=110, y=-400, z=5)
        self.ndlLeft = pi3d.ImageSprite(ndl_tex, FLATSH, camera=CAMERA2D,
                  w=170, h=170, x=-800, y=-400, z=4)
        self.ndlRight = pi3d.ImageSprite(ndl_tex, FLATSH, camera=CAMERA2D,
                  w=170, h=170, x=110, y=-400, z=4)
    def updateNeedle(self, speed):
        self.ndlLeft.rotateToZ(-320*speed/200 + 160)
        self.ndlRight.rotateToZ(-320*speed/200 + 160)
    def draw(self):
        self.speedometerLeft.draw()
        self.speedometerRight.draw()
        self.ndlLeft.draw()
        self.ndlRight.draw()
"""
DISP_CHOICE = pi3d.Display.create(use_pygame=True, frames_per_second=40)
DISP_CHOICE.set_background(0.4,0.8,0.8,1)

mymouse = pi3d.Mouse(restrict = False)
mymouse.start()
omx, omy = mymouse.position()
mykeys = pi3d.Keyboard()
mouserot = 0
tilt = 0

CAMERA = pi3d.StereoCam(separation = 0, interlace = 0)

carChoice = carChoice("models/RaceCar/carChoice.obj")
while DISP_CHOICE.loop_running():
    k = mykeys.read()
    if k == 27:
        mykeys.close()
        mymouse.stop()
        DISP_CHOICE.stop()
        break
    mx, my = mymouse.position()
    mouserot -= (mx - omx)*0.02
    tilt += (my-omy)*0.02
    omx = mx
    omy = my
    CAMERA.move_camera((0, 0, 0), mouserot, tilt, 0)
    for i in range(2):
        CAMERA.start_capture(i)
        carChoice.carChoice.draw()
        CAMERA.end_capture(i)
    CAMERA.draw()
"""    
DISPLAY = pi3d.Display.create(use_pygame=True, frames_per_second=40)
DISPLAY.set_background(0.4,0.8,0.8,1)

mouserot = 90.0
tilt = -10.0

mymouse = pi3d.Mouse(restrict = False)
mymouse.start()
omx, omy = mymouse.position()

bumptex = pi3d.Texture("textures/floor_nm.jpg")
shinetex = pi3d.Texture("textures/stars.jpg")
texture = pi3d.Texture("textures/weave.png")
shader = pi3d.Shader("uv_bump")
shader2 = pi3d.Shader("uv_flat")

roads = pi3d.Texture("textures/roads.jpg")
road = pi3d.Model(file_string='models/RaceCar/Road.obj')
road2 = pi3d.Model(file_string='models/RaceCar/Road.obj')
road3 = pi3d.Model(file_string='models/RaceCar/Road.obj')
road.set_shader(shader2)
road.set_normal_shine(roads, 16.0, texture, 0.5)
road.rotateToY(-90)
road.position(0.0,-0.2, 0.0)
road2.set_shader(shader2)
road2.set_normal_shine(roads, 16.0, texture, 0.5)
road2.position(0.0,-0.2, 0.0)
road3.set_normal_shine(roads, 16.0, texture, 0.5)
road3.rotateToY(-90)
road3.position(0.0,-0.2, 50.0)

#Variables to store values from control arduino
val1 = 0
val2 = 0
val3 = 0
but1 = 0

#car = car("models/RaceCar/RaceCar.obj","models/RaceCar/WheelLeft.obj","models/RaceCar/WheelRight.obj","models/RaceCar/SteeringWheel.obj")
#car.setDimensions(length = 4.7, width = 2.4, camDist = 1.5, steerDist = 3.0, maxWheelAngle = 30)
#car.setPosition([0,0,0], [-4.7, 0.1, -1.2], [-4.7, 0.1, 1.2], [-3, 0.8, 0], [-1.5, 1.5, 0, 90, -10, 0])
#car.setRotation(0,0,0,30)
car = car("models/F1/F1CarBody.obj","models/F1/F1Right.obj","models/F1/F1Left.obj","models/RaceCar/SteeringWheel.obj")
car.setDimensions(length = 7.3, width = 2.4, camDist = 3.3, steerDist = 4.2, maxWheelAngle = 38)
car.setPosition([0,0,0], [-7.3, 0.1, -1.2], [-7.3, 0.1, 1.2], [-4.2, 0.8, 0], [-3.3, 1.6, 0, 90, -10, 0])
car.drawModel()

instruments = instruments()

mykeys = pi3d.Keyboard()

radio = SetupRadio()


while DISPLAY.loop_running():
    instruments.draw()
    #Receive data from RF24
    while radio.available(0):
        received = []
        radio.read(received, radio.getDynamicPayloadSize())
        val1 = received[0] + received[1]*256
        val2 = received[2] + received[3]*256
        val3 = received[4] + received[5]*256
        but1 = received[6]
        
    car.calcKinematics(val1, val2, but1)
    instruments.updateNeedle(car.speed)
    car.calcAngleSpeed(val3)
    car.rotateSteering()
    car.moveModel()
    mx, my = mymouse.position()
    car.positionCAMERA[3] -= (mx - omx)*0.02
    car.positionCAMERA[4] += (my-omy)*0.02
    mouserot -= (mx - omx)*0.02
    tilt += (my-omy)*0.02
    omx = mx
    omy = my
    car.CAMERA.move_camera((car.positionCAMERA[0], car.positionCAMERA[1], car.positionCAMERA[2]), car.positionCAMERA[3], car.positionCAMERA[4], car.positionCAMERA[5])
    for i in range(2):
        car.CAMERA.start_capture(i)
        car.drawModel()
        road.draw()
        road2.draw()
        road3.draw()
        car.CAMERA.end_capture(i)
    car.CAMERA.draw()

    k = mykeys.read()
    if k == 27:
        mykeys.close()
        mymouse.stop()
        DISPLAY.stop()
        break

