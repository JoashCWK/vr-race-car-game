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

val1 = 0
val2 = 0
but1 = 0
but1_buffer = 0
#-----------------------------------------------------

''' #uncomment for MPU6050
ser = serial.Serial("/dev/ttyACM0", 9600)
ser.baudrate= 9600
'''
i = 0
potValue = 0
buttonState = 0
AngleZ = 0
AngleY = 0
angle = 0
angle_buffer = 0
direction_buffer = 0
AngleZ_buffer = 0
AngleY_buffer = 0
rads = 0.017453292512  # degrees to radians
speed = 0

# Setup display and initialise pi3d
DISPLAY = pi3d.Display.create(use_pygame=True, frames_per_second=30)
DISPLAY.set_background(0.4,0.8,0.8,1) 

#========================================
# this is a bit of a one off because the texture has transparent parts
# comment out and google to see why it's included here.
#pi3d.opengles.glDisable(pi3d.GL_CULL_FACE)
#========================================
# load bump and reflection textures
bumptex = pi3d.Texture("textures/floor_nm.jpg")
shinetex = pi3d.Texture("textures/stars.jpg")
# load model_loadmodel
texture = pi3d.Texture("textures/weave.png")
roads = pi3d.Texture("textures/roads.jpg")
shader = pi3d.Shader("uv_bump")
shader2 = pi3d.Shader("uv_flat")
mymodel = pi3d.Model(file_string='models//RaceCar/RaceCar.obj')
wheelleftmodel = pi3d.Model(file_string='models/RaceCar/WheelLeft.obj')
wheelrightmodel = pi3d.Model(file_string='models/RaceCar/WheelRight.obj')
steeringmodel = pi3d.Model(file_string='models/RaceCar/SteeringWheel.obj')
road = pi3d.Model(file_string='models/RaceCar/Road.obj')
mymodel.set_shader(shader)
wheelleftmodel.set_shader(shader)
wheelrightmodel.set_shader(shader)
steeringmodel.set_shader(shader)
mymodel.set_normal_shine(bumptex, 16.0, texture, 0.5)
road.set_shader(shader2)
road.set_normal_shine(roads, 16.0, texture, 0.5)
road.rotateToY(-90)
road.position(0.0,-0.2, 0.0)

#position vars
#CAM
mouserot = 90.0
tilt = -10.0
avhgt = 1.5
xm = -1.5
zm = 0.0
ym = avhgt
#LeftWheel
xl = -4.7
yl = 0.1
zl = -1.2
#RightWheel
xr = -4.7
yr = 0.1
zr = 1.2
#SteeringWheel
xs = -3.0
ys = 0.8
zs = 0.0
#Car
xc = 0.0
yc = 0.0
zc = 0.0
# Fetch key presses
mykeys = pi3d.Keyboard()
mymouse = pi3d.Mouse(restrict = False)
mymouse.start()
omx, omy = mymouse.position()
angle = 0
#CAMERA = pi3d.Camera.instance()
CAMERA = pi3d.StereoCam(separation = -0, interlace = 0)

#-------------------------------------------------------------------------
#2D models

CAMERA2D = pi3d.Camera(is_3d=False)
FLATSH = pi3d.Shader("uv_flat")
speedometer_tex = pi3d.Texture("textures/speedometer.png")
ndl_tex = pi3d.Texture("textures/instrument_needle.png")
speedometerLeft = pi3d.ImageSprite(speedometer_tex, shader, camera=CAMERA2D,
          w=170, h=170, x=-800, y=-400, z=5)
speedometerRight = pi3d.ImageSprite(speedometer_tex, shader, camera=CAMERA2D,
          w=170, h=170, x=110, y=-400, z=5)
ndlLeft = pi3d.ImageSprite(ndl_tex, FLATSH, camera=CAMERA2D,
          w=170, h=170, x=-800, y=-400, z=4)
ndlRight = pi3d.ImageSprite(ndl_tex, FLATSH, camera=CAMERA2D,
          w=170, h=170, x=110, y=-400, z=4)
#--------------------------------------------------------------------

#Constants
maxSpeed = 65
Mass = 100 #kg
CoeffFric = 0.7
Power = 0.1
Vel = 0.1
Frad = 0
steeringmodel.rotateToZ(30)
Resistance = 0.05
deceleration = 0
brake = 0
speed_target = 0
multiplier = 0.3
brake_buffer = 0
SmallThrust = 1
brakeReleaseMode = False
countBrakeRelease = 1
accn = 0
carRadiusBuffer = 0
while DISPLAY.loop_running():
    speedometerLeft.draw()
    speedometerRight.draw()
    ndlLeft.draw()
    ndlRight.draw()
    '''
    data=str(ser.readline())
    if(len(data) == 13):
        #potValue = int(data[2:6]) - 512
        if int(data[2:6]) != 0 and speed < int(data[2:6]) and brake == 0:
            speed = int(data[2:6])
            speed_target = speed
        elif int(data[2:6]) < speed and int(data[2:6]) >7 and brake == 0:
            speed_target = int(data[2:6])
        if int(data[6:10]) > 3:
            brake = (int(data[6:10]) - 498)/200
        else:
            brake = 0
    '''
# Speed Calculation
#---------------------------------------------------------------------------
    while radio.available(0):
        received = []
        radio.read(received, radio.getDynamicPayloadSize())
        val1 = received[0] + received[1]*256
        val2 = received[2] + received[3]*256
        val3 = received[4] + received[5]*256
        but1 = received[6]
    '''
    if brake == 0:
        speed_target = val1
    brake = val2/300
    if speed > speed_target and brake == 0 and speed_target != 0:
        deceleration = 0.3
    elif speed < speed_target and brake == 0:
        deceleration = -0.3
    elif speed_target == 0 and brake == 0:
        deceleration = 0
    if brake == 0 and not brakeReleaseMode and not deceleration < 0:
        if speed > 3:
            speed -= Resistance + deceleration + brake
    elif (brake != 0 or deceleration < 0) and not brakeReleaseMode:
        speed -= deceleration + brake + Resistance
    '''
    if brake == 0:
        accn = val1/150
    brake = val2/280
    if brake!= 0:
        accn = 0
    if not brakeReleaseMode:
        speed += accn - brake - Resistance       
    if speed < 0:
        speed = 0
        accn = 0
    if speed > maxSpeed:
        speed = maxSpeed
    if accn == 0 and brake == 0:
        if speed < 3:
            brakeReleaseMode = True
    if speed == 0 and brake != 0:
        brakeReleaseMode = True
    if brakeReleaseMode:
        if brake > 0.45:
            speed = 0
        else:
            countBrakeRelease += 1
            speed = (0.45-brake)*7/math.log(countBrakeRelease, 5)
        if accn != 0:
            brakeReleaseMode = False
            countBrakeRelease = 1
    if but1 == 0 and but1_buffer == 1:
        multiplier = - multiplier
    but1_buffer = but1
    finalSpeed = speed*multiplier #mult determins D or R

    #Speedometer
    ndlLeft.rotateToZ(-320*speed/70 + 160)
    ndlRight.rotateToZ(-320*speed/70 + 160)
#----------------------------------------------------------------------
#Calculate angles and speed
    potValue = val3 -512 #Steeringwheel
    wheelAngle = abs(30/512*potValue) #direction of the wheel the direction the steering wheel is turned to (Absolute direction on map and not Relative direction as seen by the driver)
    if wheelAngle!= 0:
        compWheelAngle = math.degrees(math.atan(4.7/((4.7/math.tan(wheelAngle*rads))+2.4))) #Direction of the other wheel
    else:
        compWheelAngle = 0
    steeringmodel.rotateToX(-90/512*potValue)

    if wheelAngle != 0:
        radius = 4.7/math.sin(wheelAngle*rads)
        compRadius = 4.7/math.sin(compWheelAngle*rads)
        carRadius = 1.2 + 4.7/math.tan(wheelAngle*rads)
        camRadius = math.sqrt(carRadius**2 + 1.5**2) 

        carSpeed = finalSpeed #tangential
        carAngularVel = (carSpeed/carRadius)/rads
        carTranslationAngle = carAngularVel/2

        speedWheel = finalSpeed/carRadius*radius
        compSpeedWheel = finalSpeed/carRadius*compRadius
        
        camSpeed = carSpeed/carRadius*camRadius
        camAngleFromTangent = math.degrees(math.acos(carRadius/camRadius)) #a constant
        
    else:
        speedWheel = finalSpeed
        compSpeedWheel = finalSpeed
        carRadius = 1000000000000000
        carAngularVel = 0
        carTranslationAngle = 0
        
    #IF turning left
    if potValue > 0:
        directionLeftWheel = -wheelAngle
        directionRightWheel = -compWheelAngle
        speedLeftWheel = speedWheel
        speedRightWheel = compSpeedWheel
        camAngleFromTangent = -camAngleFromTangent
        
    #IF turning right
    else:
        directionLeftWheel = compWheelAngle
        directionRightWheel = wheelAngle
        speedLeftWheel = compSpeedWheel
        speedRightWheel = speedWheel
        camAngleFromTangent = camAngleFromTangent
    camParallelSpeed = camSpeed*math.cos(camAngleFromTangent*rads)
    camNormalSpeed = camSpeed*math.sin(camAngleFromTangent*rads)
#Turn wheels according to steering
    wheelleftmodel.rotateToY(directionLeftWheel + angle)
    wheelrightmodel.rotateToY(directionRightWheel + angle)

#Circular motion of all models
    if potValue > 0:
        carRotationAngle = -carTranslationAngle
    else:
        carRotationAngle = carTranslationAngle
    angle += carRotationAngle
    
    mymodel.rotateIncY(carRotationAngle)
    #Rotate the wheels in circular motion
    wheelleftmodel.rotateIncY(carRotationAngle)
    wheelrightmodel.rotateIncY(carRotationAngle)
    mouserot -= (carRotationAngle)
    
    zc+=math.sin(angle*rads)*carSpeed
    xc-=math.cos(angle*rads)*carSpeed
    
    #Moving the wheels
    zl+=math.sin((directionLeftWheel+angle)*rads)*speedLeftWheel
    xl-=math.cos((directionLeftWheel+angle)*rads)*speedLeftWheel
    zr+=math.sin((directionRightWheel+angle)*rads)*speedRightWheel
    xr-=math.cos((directionRightWheel+angle)*rads)*speedRightWheel
    zm+=math.sin((angle)*rads)*camParallelSpeed + math.cos((angle)*rads)*camNormalSpeed
    xm+=-math.cos((angle)*rads)*camParallelSpeed +math.sin((angle)*rads)*camNormalSpeed
    
    angle += carRotationAngle
    mymodel.rotateIncY(carRotationAngle)
    #Rotate the wheels in circular motion
    wheelleftmodel.rotateIncY(carRotationAngle)
    wheelrightmodel.rotateIncY(carRotationAngle)
    mouserot -= (carRotationAngle)
    #mouserot += 1
    mx, my = mymouse.position()
    #mouserot += (AngleZ-AngleZ_buffer)
    #tilt += (AngleY-AngleY_buffer)
    mouserot -= (mx - omx)*0.02
    tilt += (my-omy)*0.02
    omx = mx
    omy = my
    #AngleZ_buffer = AngleZ
    #AngleY_buffer = AngleY

    mymodel.position(xc, yc, zc)
    wheelleftmodel.position(xl, yl, zl)
    wheelrightmodel.position(xr, yr, zr)
    steeringmodel.position(xs, ys, zs)
    CAMERA.move_camera((xm,ym,zm), mouserot, tilt, 0)
    
    for i in range(2):
        CAMERA.start_capture(i)
        steeringmodel.draw()
        mymodel.draw()
        wheelleftmodel.draw()
        wheelrightmodel.draw()
        road.draw()
        CAMERA.end_capture(i)
    CAMERA.draw()
    
  #Press ESCAPE to terminate
    k = mykeys.read()
    if k == 27:
        mykeys.close()
        mymouse.stop()
        DISPLAY.stop()
        break

'''
    potValue = val3 -512 #Steeringwheel
    direction = abs(30/512*potValue) #direction of the wheel the direction the steering wheel is turned to (Absolute direction on map and not Relative direction as seen by the driver)
    if direction != 0:
        compDirection = math.degrees(math.atan(4.7/((4.7/math.tan(direction*rads))+2.4))) #Direction of the other wheel
        midAxisAngle = math.degrees(math.atan(4.7/((4.7/math.tan(direction*rads))+1.2)))
    else:
        compDirection = 0
        midAxisAngle = 0
    steeringmodel.rotateToX(-90/512*potValue)
   
    #Speed of each part using F=m*v^2/r
    if direction != 0:
        radius = 4.7/math.sin(direction*rads)
        compRadius = 4.7/math.sin(compDirection*rads)
        midAxisRadius = 4.7/math.sin(midAxisAngle*rads)
        carRadius = 1.2 + 4.7/math.tan(direction*rads)
        camRadius = math.sqrt(carRadius**2 + 1.5**2) 
        speedWheel = finalSpeed/carRadius*radius
        compSpeedWheel = finalSpeed/carRadius*compRadius
        
        carSpeed = finalSpeed #tangential
        carAngularVel = (carSpeed/carRadius)/rads
        carTranslationAngle = carAngularVel/2
        carTranslationSpeed = carSpeed/math.cos(carTranslationAngle*rads)
        
        camSpeed = carSpeed/carRadius*camRadius
        camAngleFromTangent = math.degrees(math.asin(1.5/camRadius))
        camTravellingSpeed = camSpeed*math.cos(camAngleFromTangent)
        camAngularVel = (camSpeed/camRadius)/rads
        camTranslationAngle = camAngularVel/2 + camAngleFromTangent
        camTranslationSpeed = camTravellingSpeed/math.cos(camTranslationAngle*rads)
    else:
        speedWheel = speed
        compSpeedWheel = speed
        carRadius = 100000000
        carAngularVel = 0
        carTranslationAngle = 0
        camAngularVel = 0
        camTranslationAngle = 0
    #IF turning left
    if potValue > 0:
        directionLeftWheel = -direction + angle
        directionRightWheel = -compDirection + angle
        carDirection = -carTranslationAngle + angle
        camDirection = -camTranslationAngle + angle
        directionLeftWheel = -direction + angle
        directionRightWheel = -compDirection + angle
        midAxisAngle = -midAxisAngle + angle
        speedLeftWheel = speedWheel
        speedRightWheel = compSpeedWheel
        carRotationAngle = -carTranslationAngle
    #IF turning right
    else:
        directionLeftWheel = compDirection + angle
        directionRightWheel = direction + angle
        carDirection = carTranslationAngle + angle
        camDirection = camTranslationAngle + angle
        directionLeftWheel = compDirection + angle
        directionRightWheel = direction + angle
        midAxisAngle = midAxisAngle + angle
        speedLeftWheel = compSpeedWheel
        speedRightWheel = speedWheel
        carRotationAngle = carTranslationAngle
    
    
#-------------------------------------------------------------------------
#Car body movement
#-------------------------------------------------------------------------
    angle += carRotationAngle
    
    mymodel.rotateToY(angle)
    #Rotate the wheels accordingly 
    wheelleftmodel.rotateToY(directionLeftWheel)
    wheelrightmodel.rotateToY(directionRightWheel)
    
    zc+=math.sin(angle*rads)*carSpeed
    xc-=math.cos(angle*rads)*carSpeed
    #Moving the wheels
    zl+=math.sin(directionLeftWheel*rads)*speedLeftWheel
    xl-=math.cos(directionLeftWheel*rads)*speedLeftWheel
    zr+=math.sin(directionRightWheel*rads)*speedRightWheel
    xr-=math.cos(directionRightWheel*rads)*speedRightWheel
    
    angle += carRotationAngle
    
    mymodel.rotateToY(angle)
    #Rotate the wheels accordingly 
    wheelleftmodel.rotateToY(directionLeftWheel)
    wheelrightmodel.rotateToY(directionRightWheel)
    #distInc = math.sqrt((math.sin(carDirection*rads)*carTranslationSpeed)**2 + (math.cos(carDirection*rads)*carTranslationSpeed)**2)
    
    #zm+=math.sin(camDirection*rads)*camTranslationSpeed
    #xm-=math.cos(camDirection*rads)*camTranslationSpeed
'''
    

