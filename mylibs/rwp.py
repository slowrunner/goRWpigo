#!/usr/bin/python
#
# File: rwpida.py   
# Doc:        Rug Warrior with Pi Droid Alpha 
#             Primitives for goRWpigo interface
#


import myPDALib
import myPyLib
import time      # for test main
import pigpio	#for constants only, funcs through myPDALib
import encoders

debugLevel = 0		# 0 off, 1 some, 99 all    


# RWP for GoPiGo API SUMMARY

# ### Motor control functions:

# fwd(): Move the GoPiGo forward with PID (better control), stops at enc_tgt if enabled
# motor_fwd(): Move the GoPiGo forward without PID, stops at enc_tgt if enabled
# bwd(): Move the GoPiGo back with PID (better control), stops at enc_tgt if enabled
# motor_bwd(): Move the GoPiGo back without PID, stops at enc_tgt if enabled
# left(): Turn GoPiGo Left slow (one motor off, better control)
# left_rot(): Rotate GoPiGo left in same position 
#             (both motors moving in the opposite direction)
# right(): Turn GoPiGo right slow (one motor off, better control)
# right_rot(): Rotate GoPiGo right in same position 
#             both motors moving in the opposite direction)
# stop(): Stop the GoPiGo


# ### Motor speed Functions:  (default speed is 200)
# Note: gopigo speeds 1-255 are mapped to MinPwr2Mov (~100)..MaxPwr (255) 
#       rwp_speeds[lft,rt] are from -255 to +255
# increase_speed(): Increase the speed of the GoPiGo by 10
# decrease_speed(): Decrease the speed of the GoPiGo by 10
# set_left_speed(speed): Set speed of the left motor 0-255
# set_right_speed(speed): Set speed of the right motor 0-255
# set_speed(speed): Set speeds of both the motors 0-255


# ### Encoder Functions:

# enc_tgt(m1, m2, numEncPulses): Set encoder target m1/2=0:disable 1:enabled, 18 per wheel rev
# enable_encoders(): Enable the encoders
# disable_encoders(): Disable the encoders
# enc_read(motor):    motor=left 1, 0 right  returns distance traveled in cm
# ### Ultrasonic ranger read:

# us_dist(pin): Read distance in cm from the ultrasonic sensor (rwp ignores pin parm)


# ### LED control:

# led(led,pwr): Set LED power level led:  1 left, 0 right  pwr:0-255
# led_on(led): Turn LED on
# led_off(led): Turn LED off


# ### Servo control:

# enable_servo(): Enables the servo
# disable_servo(): Disables the servo
# servo(angle): Set servo position  Left 180 - 0 Right


# ### Status from the GoPiGo:

# volt(): Read battery voltage in V
# fw_ver(): Get the firmware version of the GoPiGo
# enable_com_timeout(ms): Enable communication time-out ms: 0-65536
#                        (stop the motors if no command received in the specified time-out)
# disable_com_timeout(): Disable communication time-out
# read_status(): Read the status register on the GoPiGo returns [encTgtNotSatisfied=1, timeoutNotReached=1] 
# read_enc_status(): Read encoder status returns 0 if tgt reached, 1 if not 
# read_timeout_status(): Read timeout status returns 0 if timeout reached, 1 if not

# ********* ADDED BY ALAN **************
# (rwp) print_status()   # prints out all state variables
# motors_init()   # runs when import rwp.py to configure motor pins
# motors_kill()   # set both motors to coast to stop



# ################### IMPLEMENTATION #######
rwp_speeds = [0,0]            # [left, right]
rwp_edges_1_rev = 32          #  
rwp_enc_tgt = [0,0, rwp_edges_1_rev]  # [m1_enabled, m2_enabled, encoder_tgt] (default off, 1 rev)
rwp_servo_angle = 0                                  
rwp_com_timeout = 10000       # if don't get a command in this mSec, stop motions
rwp_default_speed = 200       # this is equiv to 78% of max speed
rwp_default_turn_speed = 110   # this is equiv to 10% of max speed
rwp_enc_time_status = [0,0]    # 0 when condition met

 

# ### Motor control functions:

def fwd(speed=rwp_default_speed):    # Move the GoPiGo forward with PID (better control)
    if (debugLevel): print "rwp:  fwd(%d) called" % speed
    motor_fwd(speed)                 # no PID for time being

    
    

def motor_fwd(speed=rwp_default_speed):    # Move the GoPiGo forward without PID
    global rwp_speeds, drive_bias_F
    if (debugLevel): print "rwp:  motor_fwd(%d) called" % speed
    if (drive_bias_F > 0):  rwp_speeds = [speed - drive_bias_F, speed]  #decrease left by bias
    else: rwp_speeds = [speed, speed - abs(drive_bias_F)]             #decrease right by bias
    if (debugLevel): print "rwp:motor_fwd: rwp_speeds=",rwp_speeds
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
	
def bwd(speed=rwp_default_speed):    # Move the GoPiGo back with PID (better control)
    if (debugLevel): print "rwp:  bwd() called"
    motor_bwd(speed)                 # no PID for time being
	
def motor_bwd(speed=rwp_default_speed):    # Move the GoPiGo back without PID
    global rwp_speeds, drive_bias_B
    if (debugLevel): print "rwp:  motor_bwd() called"
    if (drive_bias_B < 0):  rwp_speeds = [-speed, -speed - drive_bias_B]  #e.g. -200 - -5=-195 decreased 
    else: rwp_speeds = [-speed + drive_bias_B, -speed]
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
	
def left(speed=rwp_default_turn_speed):    # Turn GoPiGo Left slow (one motor off, better control)
    global rwp_speeds
    if (debugLevel): print "rwp:  left() called"
    rwp_speeds = [0,speed]
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
	
def left_rot(speed=rwp_default_turn_speed):    # Rotate GoPiGo left in same position 
    global rwp_speeds                          # (both motors moving in the opposite direction)
    if (debugLevel): print "rwp:  left_rot() called"
    rwp_speeds = [-speed,speed]
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
	
def right(speed=rwp_default_turn_speed):    # Turn GoPiGo right slow (one motor off, better control)
    global rwp_speeds
    if (debugLevel): print "rwp:  right() called"
    rwp_speeds = [speed,0]
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
    
	
def right_rot(speed=rwp_default_turn_speed):    # Rotate GoPiGo right in same position 
    global rwp_speeds                           # both motors moving in the opposite direction)
    if (debugLevel): print "rwp:  right_rot() called"
    rwp_speeds = [speed,-speed]
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
	
def stop():    # Stop the GoPiGo
    global rwp_speeds
    if (debugLevel): print "rwp:  rwp.stop: motors_coast()"
    rwp_speeds = [0,0]
    motors_coast()
	

# ### Motor speed Functions:

def increase_speed():    # Increase the speed of the GoPiGo by 10
    global rwp_speeds
    new_rwp_speeds = rwp_speeds
    dont_change = False
    if (debugLevel):  print "rwp:  increase_speed() called"
    if (245 >= rwp_speeds[0] > 0):     # lft motor is fwd and will not bust limit
        new_rwp_speeds[0] = rwp_speeds[0] + 10
    elif (-245 <= rwp_speeds[0] < 0):  # lft motor is rev and will not bust limit
        new_rwp_speeds[0] = rwp_speeds[0] - 10
    else: dont_change = (rwp_speeds[0] <> 0)  # true if not zero but will bust limit 
    if not dont_change:     # if left motor ok, test right motor speed
        if (245 >= rwp_speeds[1] > 0):     # rt motor is fwd and will not bust limit
            new_rwp_speeds[1] = rwp_speeds[1] + 10
        elif (-245 <= rwp_speeds[1] < 0):  # rt motor is rev and will not bust limit
            new_rwp_speeds[1] = rwp_speeds[1] - 10
        else: dont_change = (rwp_speeds[1] <> 0)  # true if not zero but will bust limit 
        if not dont_change:
            if (rwp_speeds[0] == rwp_speeds[1] == 0):     # check not change due to 0,0
                if (debugLevel):  print "rwp: rwp_speeds not changed: [lft,rt]", rwp_speeds
            else:  # changed
                rwp_speeds = new_rwp_speeds
                if (debugLevel):    print "rwp:  new rwp_speeds: [lft,rt]", rwp_speeds
                            # not changed because right would bust limit
        elif (debugLevel):  print "rwp:  rwp_speeds not changed: [lft,rt]", rwp_speeds
    else: print "rwp:  rwp_speeds not changed: [lft,rt]", rwp_speeds  # because left would bust limit
    
	
def decrease_speed():    # Decrease the speed of the GoPiGo by 10
    global rwp_speeds
    new_rwp_speeds = rwp_speeds
    dont_change = False
    if (debugLevel):  print "rwp:  decrease_speed() called"
    if (rwp_speeds[0] > 10):     # lft motor is fwd and will not bust limit
        new_rwp_speeds[0] = rwp_speeds[0] - 10
    elif ( rwp_speeds[0] < -10):  # lft motor is rev and will not bust limit
        new_rwp_speeds[0] = rwp_speeds[0] + 10
    else: dont_change = (rwp_speeds[0] != 0)  # true if not zero but will bust limit 
    if not dont_change:     # if left motor ok, test right motor speed
        if (rwp_speeds[1] > 10):     # rt motor is fwd and will not bust limit
            new_rwp_speeds[1] = rwp_speeds[1] - 10
        elif (rwp_speeds[1] < -10):  # rt motor is rev and will not bust limit
            new_rwp_speeds[1] = rwp_speeds[1] + 10
        else: dont_change = (rwp_speeds[1] <> 0)  # true if not zero but will bust limit 
        if not dont_change:
            if (rwp_speeds[0] == rwp_speeds[1] == 0):     # check not change due to 0,0
                if (debugLevel):  print "rwp: rwp_speeds not changed: [lft,rt]", rwp_speeds
            else:  # changed
                rwp_speeds = new_rwp_speeds
                if (debugLevel):    print "rwp:  new rwp_speeds: [lft,rt]", rwp_speeds
                            # not changed because right would bust limit
        elif (debugLevel):  print "rwp:  rwp_speeds not changed: [lft,rt]", rwp_speeds
    else: print "rwp:  rwp_speeds not changed: [lft,rt]", rwp_speeds  # because left would bust limit

	
def set_left_speed(speed=rwp_default_speed):    # Set speed of the left motor
    global rwp_speeds
    rwp_speeds[1] = speed
    if (debugLevel):
        print "rwp:  set_left_speed(%d) called" % speed
    motor(LMotorIndex, rwp_speeds[0])
	
def set_right_speed(speed=rwp_default_speed):    # Set speed of the right motor
    global rwp_speeds
    rwp_speeds[1] = speed
    if (debugLevel):
        print "rwp:  set_right_speed(%d) called" % speed
    motor(RMotorIndex, rwp_speeds[1])
	
def set_speed(speed=rwp_default_speed):    # Set speeds of both the motors
    global rwp_speeds
    if (debugLevel): 
        print "rwp:  set_speed(%d) called" % speed
    rwp_speeds = [speed,speed]
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])
	

# ### Encoder Functions:

def enc_tgt(m1,m2,numEncPulses):    # Set encoder target to move the GoPiGo to a set distance
    global rwp_enc_tgt
    if (debugLevel): print "rwp:  enc_tgt(m1=%d,m2=%d,numEncPulses=%d) called" % (m1,m2,numEncPulses)
    rwp_enc_tgt = [m1, m2, numEncPulses]
	
def enable_encoders():    # Enable the encoders
    if (debugLevel): print "rwp:  enable_encoders() called"
    encoders.enable_encoder_interrupts()

	
def disable_encoders():    # Disable the encoders
    if (debugLevel): print "rwp:  disable_encoders() called"
    encoders.disable_encoder_interrupts()

def enc_read(motor):       # motor=left 1, 0 right  returns distance traveled in cm
    if (debugLevel): print "rwp: enc_read(%d) left 1, 0 right" % motor 
    if (motor==0): enc_count = encoders.rightCount()
    else:       enc_count = encoders.leftCount()
    if (debugLevel): print "rwp.enc_read: enc_count is %d" % enc_count
    return enc_count * encoders.CmPerCount 	

# ### Ultrasonic ranger read:

def us_dist(pin=0):    # Read distance from the ultrasonic sensor
                       # pin parameter not used
    if (debugLevel): print "rwp:  us_dist() called" 
    dist_in_cm = inCm()    # call the rwp ultrasonic range once - return dist in cm
    if (debugLevel): print "rwp:us_dist: returning %f" % dist_in_cm
    return dist_in_cm
	


# ### LED control:

def led(led,pwr):    # Set  LED to  power level
    if (debugLevel): print "rwp:  led() called"
	
def led_on(led):    # Turn  LED on
    if (debugLevel): print "rwp:  led_on() called"
	
def led_off(led):    # Turn LED off
    if (debugLevel): print "rwp:  led_off() called"
	


# ### Servo control:

def enable_servo():    # Enables the servo
    if (debugLevel): print "rwp:  enable_servo() called"
    servos_on()
	
def disable_servo():    # Disables the servo
    if (debugLevel): print "rwp:  disable_servo() called"
    # servos_off()
	
def servo(angle):    # Set servo position
    global gopiogo_servo_angle
    if (debugLevel): print "rwp:  servo(%d) called" % angle
    gopigo_servo_angle = pan_servo(angle)
	


# ### Status from the GoPiGo:

def volt():    # Read battery voltage in V
    if (debugLevel): print "rwp:  volt() called"
    return 7.2
	
def fw_ver():    # Get the firmware version of the GoPiGo
    if (debugLevel): print "rwp:  fw_ver() called"
    return "Alan's gorwpigo.py 6Jun2017"
	
def enable_com_timeout(ms):    # Enable communication time-out
                               #  (stop the motors if no command received in the specified time-out)
    global gopigo_com_timeout
    gopigo_com_timeout = ms    
    if (debugLevel): print "rwp:  enable_com_timeout(ms=%d) called" % ms
	
def disable_com_timeout():    # Disable communication time-out
    if (debugLevel): print "rwp:  disable_com_timeout() called"
	
def read_status():    # Read the status register on the GoPiGo
    global rwp_enc_time_status
    if (debugLevel): print "rwp:  read_status() called"
    return rwp_enc_time_status
	
def read_enc_status():    # Read encoder status
    global rwp_enc_time_status
    if (debugLevel): print "rwp:  read_enc_status() called, returning %i" % rwp_enc_time_status[0]
    return rwp_enc_time_status[0]
	


def read_timeout_status():    # Read timeout status
    global rwp_enc_time_status
    if (debugLevel): print "rwp:  read_timeout_status() called, returning %i" % rwp_enc_time_status[1]
    return rwp_enc_time_status[1]
	
    

# #############################################
# Rug Warrior platform Definition and Vars



LMotorIndex = 0     # LEFT MOTOR
RMotorIndex = 1     # RIGHT MOTOR

RMotorPin = 6   # SRV 6		Motor 1 Speed (PWM)
LMotorPin = 7   # SRV 7		Motor 2 Speed (PWM)

MotorPin = [7,6]  # MotorPin[0] Left, MotorPin[1] Right

M1DirA = 12 # DIO 12 (A4)	Motor 1 Dir A (0=coast 1=F/Brake)
M1DirB = 13 # DIO 13 (A5)	Motor 1 Dir B (0=coast 1=R/Brake)
M2DirA = 14 # DIO 14 (A6)	Motor 2 Dir A (0=coast 1=F/Brake)
M2DirB = 15 # DIO 15 (A7)	Motor 2 Dir B (0=coast 1=R/Brake)
MotorDirA = [14,12]  # 0 left 1 right
MotorDirB = [15,13]  # 0 left 1 right

MinPwr2Mov = 100   # empirical minimum power to move with both wheels
MaxPwr = 255
drive_bias_F = 35   # positive will drive right more than left
drive_bias_B =  0  

# ### MOTORS_INIT()
motorsInitialized = False   # True after motor pins are configured once
def motors_init():   # set up the two speed and four dir pins
    global motorsInitialized
    if (debugLevel): print "rwp:motors_init() called"
    if not motorsInitialized:
        myPDALib.pinMode(RMotorPin,myPDALib.PWM)  # init rt-motor1  speed control pin
        myPDALib.pinMode(LMotorPin,myPDALib.PWM)  # init lft-motor2 speed control pin 

        myPDALib.pinMode(M1DirA,myPDALib.OUTPUT)  #init rt-motor1 dirA/Fwd   enable
        myPDALib.pinMode(M1DirB,myPDALib.OUTPUT)  #init rt-motor1 dirB/Bkwd  enable
        myPDALib.pinMode(M2DirA,myPDALib.OUTPUT)  #init lft-motor2 dirA/Fwd  enable
        myPDALib.pinMode(M2DirB,myPDALib.OUTPUT)  #init lft-motor2 dirB/Bkwd enable

        myPDALib.digitalWrite(M1DirA,0)  #set to off/coast
        myPDALib.digitalWrite(M1DirB,0)  #set to off/coast
        myPDALib.digitalWrite(M2DirA,0)  #set to off/coast
        myPDALib.digitalWrite(M2DirB,0)  #set to off/coast

        # turn off the speed pins
        myPDALib.analogWrite(RMotorPin,0)  #set rt  (motor1,index 1) to zero speed 
        myPDALib.analogWrite(LMotorPin,0)  #set lft (motor2,index 0) to zero speed
        
        motorsInitialized = True   # Don't need to do this again
    return True
dummy = motors_init()   # run when imported by any process to init motors


# ### MOTORS_coast()
def motors_coast():
    # turn off the speed pin 
    myPDALib.analogWrite(RMotorPin,0)  #set motor1 to coast (zero speed)  
    myPDALib.analogWrite(LMotorPin,0)  #set motor2 to coast (zero speed)


# ### MOTOR(INDEX,VEL)

def motor(index,vel):  #mtr 0=lft, 1=rt, +/-255
    avel = myPyLib.clamp(vel,-255,255)  # adjust velocity to limits
    if (vel == 0):
        myPDALib.analogWrite(MotorPin[index],0)  #set motor to zero speed
        return
    if (vel > 0):  # set forward
        myPDALib.digitalWrite(MotorDirA[index],1)  #set to fwd
        myPDALib.digitalWrite(MotorDirB[index],0)  #set to off/coast
        # avel = vel
    else:
        avel = -vel        
        myPDALib.digitalWrite(MotorDirA[index],0)  #set to off/coast
        myPDALib.digitalWrite(MotorDirB[index],1)  #set to bwd
  
    # compute pct to pwr
    pwr = int( (MaxPwr - MinPwr2Mov) * avel/255.0 + MinPwr2Mov)
    if (debugLevel): 
        print "rwp:motor(%d,%d) called" % (index,vel)
        print "rwp:motor: MaxPwr=%d  MinPwr2Mov=%d" % (MaxPwr,MinPwr2Mov)
        print "rwp:motor: pwr=%d or %d %% max speed" % (pwr, int(vel/255.0*100))
    myPDALib.analogWrite(MotorPin[index], pwr)  #set motor pwr level

# ### DRIVE(TRANS_VEL, ROT_VEL)

def drive(trans_vel, rot_vel): #  + = fwd, ccw
    # 
    #
    motor(LMotorIndex, trans_vel - rot_vel)
    motor(RMotorIndex, trans_vel + rot_vel)


# ### DRIVEB(TRANS, ROT)  

def driveb(trans, rot):    # Correct for motor bias
    # 
    #
    rot_bias = (drive_bias_F * trans) / 100
    motor(LMotorIndex, trans - (rot + rot_bias));
    motor(RMotorIndex,trans + (rot + rot_bias))

# Left Encoder - DIO B4 - "myPDALib.pin 20"
# Right Encoder- DIO B3 - "myPDALib.pin 19" 

# ################ usDistance.py ##############################
#
# usDistance.py    HC-SR04 ULTRASONIC DISTANCE SENSOR 
#
# import time
# import pigpio
# import PDALib

# My configuration
TrigPin = 26    #GPIO26 is pin 37 of the PiB+ and Pi3B 40pin connector
EchoPin = 5	 #PDALib "pin" = Servo3 connector (of 1-8) (GPIO18)



def _echo1(gpio, level, tick):
   global _high
   _high = tick
      
def _echo0(gpio, level, tick):
   global _done, _high, _pulseTravelTime
   _pulseTravelTime = tick - _high
   _done = True


def clearEcho():
       global my_echo0, my_echo1
       my_echo1.cancel()
       my_echo0.cancel()

def setEcho(srvopin=EchoPin):
  global my_echo0, my_echo1
  # my_echo1 = pi.callback(22, pigpio.RISING_EDGE,  _echo1)
  # my_echo0 = pi.callback(22, pigpio.FALLING_EDGE, _echo0)

  # Alan - 14Jun2016 Echo on servopin - translate to GPIO pin
  my_echo1 = myPDALib.pi.callback(myPDALib.servopin[srvopin], pigpio.RISING_EDGE,  _echo1)
  my_echo0 = myPDALib.pi.callback(myPDALib.servopin[srvopin], pigpio.FALLING_EDGE, _echo0)


# readDistance2gs(_trig, _echo) for HC-SR04 only
# Alan:   g: trigger is connected direct to a PiB+, Pi2, Pi3B gpio pin
#         s: echo is connected to a PiDroidAlpha servo pin 0..7
#
# Alan: _trigpin is gpioPin  (e.g. 26 for GPIO26 on pin 37)
#       _echopin is a servoPin 0..7
#

def readDistance2gs(_trigpin, _echopin):
   global _done, _pulseTravelTime
   _done = False
   myPDALib.pi.set_mode(_trigpin, pigpio.OUTPUT)
   time.sleep(0.0001)        # does this make it more reliable?
   myPDALib.pi.gpio_trigger(_trigpin,50,1)
   myPDALib.pi.set_mode(myPDALib.servopin[_echopin], pigpio.INPUT)
   time.sleep(0.0001)
   tim = 0
   while not _done:
      time.sleep(0.001)
      tim = tim+1
      if tim > 50:
         return 0
   return _pulseTravelTime / 58.068 # return as cm

# #############################################
# ULTRASONIC DISTANCE SENSOR INTERFACE METHODS

# OFFSET TO PIVOT - distance from front of sensor to pan servo pivot point
offsetInchesToPivot=2.0


# INIT_USSENSOR()
#
# Setup callbacks for trigger pulse and for echo pulse
#
def init_usSensor():
  setEcho()

dummy = init_usSensor()

# inCm()
#
# return Distance in Centimeters (to sensor circuit board)

def inCm():
   distInCm = readDistance2gs(TrigPin,EchoPin)
   return distInCm

# inInches()
#
# return Distance in Inches (to sensor circuit board)

def inInches(readings=75):
    if (readings > 1):
      values = []
      for i in range(0,readings):
        values.append(readDistance2gs(TrigPin,EchoPin) * 0.393701)
        # time.sleep(0.01)
      values.sort()
      usReading = sum(values) / float(len(values)) # average
    else:
      #  Distance from a single reading
      usReading =  readDistance2gs(TrigPin,EchoPin) * 0.393701
    return usReading 


# ###########################################################
# tiltpan.py   TILT PAN 
#
# SG90 Micro Servo

#Specification :

#* Weight : 9 g

#* Size : 22 x 11.5 x 27 mm

#* Operating Speed (4.8V no load): 0.12sec/60 degrees 
#* Stall Torque (4.8V): 17.5oz/in (1.2 kg/cm)

#* Temperature Range: -30 to +60 Degree C
#* Dead Band Width: 7usec
#* Operating Voltage:3.0-7.2 Volts

#Features :
#- Coreless Motor
#- All Nylon Gear
#- Connector Wire Length 150MM

import myPDALib
import myPyLib
import time

TILTSERVO = 0
PANSERVO = 1

PanPosLimitL = 2400
PanPosCenter = 1450
PanPosLimitR =  600

PanDegLimitL = 180
PanDegCenter =  90
PanDegLimitR =   0

# pre calculate one deg angle equals how many "pos" increments
PanDeg2PanPosInc = int((PanPosLimitL-PanPosLimitR) / float(PanDegLimitL-PanDegLimitR))
Pan0Deg2PanPos = PanDeg2PanPosInc * -180 + PanPosLimitL


TiltPosLimitUp = 500
TiltPosCenter = 1400
TiltPosLimitDn = 1900 #2435

TiltDegLimitUp = 90
TiltDegCenter  = 0
TiltDegLimitDn = -30

# pre calculate one deg angle equals how many "pos" increments
TiltDeg2TiltPosInc = int((TiltPosLimitUp-TiltPosCenter) / float(TiltDegLimitUp - TiltDegCenter))
Tilt0Deg2TiltPos = TiltPosCenter


def center_servos():
    servos_on()
    myPDALib.servoWrite(TILTSERVO, TiltPosCenter)
    myPDALib.servoWrite(PANSERVO, PanPosCenter)
    if (debugLevel): print "center_servos() called"

def servos_off():
    myPDALib.pinMode(TILTSERVO,myPDALib.INPUT)    # init Tilt servo off
    myPDALib.pinMode(PANSERVO,myPDALib.INPUT)     # init motor2 servo off
    if (debugLevel): print "servos_off() called"
  
def servos_on():
    myPDALib.pinMode(TILTSERVO, myPDALib.SERVO)    # init Tilt servo pin to SERVO mode
    myPDALib.pinMode(PANSERVO, myPDALib.SERVO )    # init Pan  servo pin to SERVO mode
    if (debugLevel): print "servos_on() called"

def init_servos():
    if (debugLevel): print "init_servos() called"
    servos_on()
    time.sleep(0.1)
    center_servos()
    # servos_off()

dummy = init_servos()                     # initialize when module is loaded

def pos_servo(servo,pos):
    if (debugLevel): print "rwp:pos_servo(Tilt0Pan1=%d, pos=%d)" % (servo,pos) 
    if (servo == PANSERVO):
        cpos = myPyLib.clamp(pos,PanPosLimitR,PanPosLimitL)
    elif (servo == TILTSERVO):
        cpos = myPyLib.clamp(pos,TiltPosLimitUp,TiltPosLimitDn)
    if (debugLevel): print "setting Tilt0Pan1=%d to pos: %d)" % (servo, cpos) 
    myPDALib.servoWrite(servo, cpos)   # set to new position
    return cpos

def gopigoDeg2panPos(angle):
    pos = angle*PanDeg2PanPosInc + Pan0Deg2PanPos
    if (debugLevel):
        print "rwp:gopigoDeg2panPos(angle=%d) called" % angle
        print "rwp:gopigoDeg2panPos: returning pos:",pos
    return pos

def rwpPos2gopigoPanDeg(pos):
    angle = (pos - Pan0Deg2PanPos)/PanDeg2PanPosInc
    if (debugLevel):
        print "rwp:rwpPos2gopioPanDeg(pos=%d) called" % pos
        print "rwp:rwpPos2gopioPanDeg: returning angle:",angle
    return angle
    
def tiltDeg2TiltPos(angle):
    pos = angle*TiltDeg2TiltPosInc + Tilt0Deg2TiltPos
    if (debugLevel):
        print "rwp:tiltDeg2TiltPos(angle=%d) called" % angle
        print "rwp:tiltDeg2TiltPos: returning pos:",pos
    return pos

def rwpTiltPos2TiltDeg(pos):
    angle = (pos-Tilt0Deg2TiltPos)/TiltDeg2TiltPosInc
    if (debugLevel):
        print "rwp:rwpTiltPos2TiltDeg(pos=%d) called" % pos
        print "rwp:rwpTiltPos2TiltDeg: returning angle:",angle
    return angle
    
def tilt_servo(angle):
    if (debugLevel): print "rwp:  tilt_servo(angle=%d) called" % angle 
    pos = tiltDeg2TiltPos(angle)
    cpos = pos_servo(TILTSERVO, pos)
    cangle = rwpTiltPos2TiltDeg(cpos)                 
    if (debugLevel): print "rwp:tilt_servo: pos=%d cpos=%d cangle=%f" % (pos,cpos,cangle)
    return cangle
    

def pan_servo(angle):
    if (debugLevel): print "rwp:  pan_servo(angle=%d) called" % angle 
    pos = gopigoDeg2panPos(angle)
    cpos = pos_servo(PANSERVO, pos)
    cangle = rwpPos2gopigoPanDeg(cpos)                 
    if (debugLevel): print "rwp:pan_servo: pos=%d cpos=%d cangle=%f" % (pos,cpos,cangle)
    return cangle


def print_status():
    if (debugLevel): print "rwp:  print_status() called"
    print "rwp.print_status: encoders[left,right] = [%f,%f] cm" % (enc_read(1),enc_read(0))    
    

# ############ RWP TEST MAIN ######################
	
def main():
    global tiltAngle
    print "rwp:main: *** RWP TEST MAIN ***"
    
    servo_range = [2,3,4,5,6,7,8]
    tiltAngle = 0

    def key_input(event):
        global tiltAngle
        key_press = event  # ALAN  for Tkinter was = event.keysym.lower()
        print(key_press)

        if key_press == '?':
            print """
            w: fwd
            s: bwd
            a: left
            d: right
            q: rotate left
            e: rotate right
            space: stop
            u: ultrasonic dist
            2-8: servo position
            +: increase speed
            -: decrease speed
            >: increase drive_bias "rt wheel"
            <: decrease drive_bias             
            =: print all variables
            v: do set_speed(125), set_left(0), set_right(0)
            ^: tilt sensor platform up 10 deg
            V: tilt sensor platform dn 10 deg            
            c: center servos
            i: enable and reset encoders
            
            ctrl-c: quit
        
            """
        if key_press == 'w':
            fwd()
        elif key_press == 's':
            bwd()
        elif key_press == 'a':
            left()
        elif key_press == 'd':
            right()
        elif key_press == 'q':
            left_rot()
        elif key_press == 'e':
            right_rot()
        elif key_press == ' ':     # was 'space'
            stop()
        elif key_press == '+':
            increase_speed()
        elif key_press == '-':
            decrease_speed()
        elif key_press == 'u':
            print inCm()," cm"
        elif key_press == '=':
            print_status()
        elif key_press == '>':
            rwp.drive_bias += 1
        elif key_press == '<':
            rwp.drive_bias -= 1
        elif key_press.isdigit():
            if int(key_press) in servo_range:
                enable_servo()
                servo((8-int(key_press))*30)
                time.sleep(1)
                disable_servo()
        elif key_press == 'v':
            set_speed(125)
            time.sleep(3)
            set_left_speed(0)
            time.sleep(3)
            set_right_speed(0)
        elif key_press == '^':
                enable_servo()
                tiltAngle += 10
                cmdDeg = tiltAngle
                print "cmd Tilt Angle: %d", cmdDeg
                actualDeg = tilt_servo(cmdDeg)
                print "Tilt servo set to %f deg" % actualDeg
                time.sleep(1)
                servos_off()
        elif key_press == 'V':
                enable_servo()
                tiltAngle -= 10
                cmdDeg = tiltAngle
                print "cmd Tilt Angle: %d", cmdDeg
                actualDeg = tilt_servo(cmdDeg)
                print "Tilt servo set to %f deg" % actualDeg
                time.sleep(1)
                servos_off()
        elif key_press == 'c':
                center_servos()   # calls servos_on() 
        elif key_press == 'i':
                enable_encoders()     
            

    # command = tk.Tk()
    # command.bind_all('<Key>', key_input)  # ALAN  '' changed to '<Key>'
    # command.mainloop()

    ### created for command line execution cntl-C to quit

    while True:
      event=raw_input("cmd? ") 
      key_input(event)


	   
	   

if __name__ == "__main__":
    main()	    
