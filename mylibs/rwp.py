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

debugLevel = 99		# 0 off, 1 some, 99 all    


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

# ### Ultrasonic ranger read:

# us_dist(pin): Read distance in cm from the ultrasonic sensor (pin=1 or 15)


# ### LED control:

# led(led,pwr): Set LED power level led:  1 left, 0 right  pwr:0-255
# led_on(led): Turn LED on
# led_off(led): Turn LED off


# ### Servo control:

# enable_servo(): Enables the servo
# disable_servo(): Disables the servo
# servo(angle): Set servo position  0-180


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
# print_state()   # prints out all state variables
# motors_init()   # runs when import rwp.py to configure motor pins
# motors_kill()   # set both motors to coast to stop



# ################### IMPLEMENTATION #######
rwp_speeds = [0,0]            # [left, right]
rwp_edges_1_rev = 32          #  
rwp_enc_tgt = [0,0,0]   
rwp_servo_angle = 0                                  
rwp_com_timeout = 10000  
rwp_default_speed = 200       # this is equiv to 78% of max speed
rwp_default_turn_speed = 110   # this is equiv to 10% of max speed

 

# ### Motor control functions:

def fwd(speed=rwp_default_speed):    # Move the GoPiGo forward with PID (better control)
    if (debugLevel): print "rwp:  fwd(%d) called" % speed
    motor_fwd(speed)                 # no PID for time being

    
    

def motor_fwd(speed=rwp_default_speed):    # Move the GoPiGo forward without PID
    global rwp_speeds, drive_bias
    if (debugLevel): print "rwp:  motor_fwd(%d) called" % speed
    if (drive_bias > 0):  rwp_speeds = [speed - drive_bias, speed]  #decrease left by bias
    else: rwp_speeds = [speed, speed - abs(drive_bias)]             #decrease right by bias
    if (debugLevel): print "rwp:motor_fwd: rwp_speeds=",rwp_speeds
    motor(LMotorIndex, rwp_speeds[0])
    motor(RMotorIndex, rwp_speeds[1])    
	
def bwd(speed=rwp_default_speed):    # Move the GoPiGo back with PID (better control)
    if (debugLevel): print "rwp:  bwd() called"
    motor_bwd(speed)                 # no PID for time being
	
def motor_bwd(speed=rwp_default_speed):    # Move the GoPiGo back without PID
    global rwp_speeds, drive_bias
    if (debugLevel): print "rwp:  motor_bwd() called"
    if (drive_bias < 0):  rwp_speeds = [-speed, -speed - drive_bias]  #e.g. -200 - -5=-195 decreased 
    else: rwp_speeds = [-speed + drive_bias, -speed]
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
    if (debugLevel): print "rwp:  enc_tgt(m1=%d,m2=%d,numEncPulses=%d) called" % (m1,m2,numEncPulses)
    gopigo_enc_tgt = [m1, m2, numEncPulses]
	
def enable_encoders():    # Enable the encoders
    if (debugLevel): print "rwp:  enable_encoders() called"
	
def disable_encoders():    # Disable the encoders
    if (debugLevel): print "rwp:  disable_encoders() called"
	

# ### Ultrasonic ranger read:

def us_dist(pin=0):    # Read distance from the ultrasonic sensor
                       # pin parameter not used
    if (debugLevel): print "rwp:  us_dist() called" 
    dist_in_cm = 10.1234567
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
	
def disable_servo():    # Disables the servo
    if (debugLevel): print "rwp:  disable_servo() called"
	
def servo(angle):    # Set servo position
    global gopiogo_servo_angle
    if (debugLevel): print "rwp:  servo(%d) called" % angle
    gopigo_servo_angle = angle
	


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
    global gopigo_status
    if (debugLevel): print "rwp:  read_status() called"
    return gopigo_status
	
def read_enc_status():    # Read encoder status
    global gopigo_status
    if (debugLevel): print "rwp:  read_enc_status() called"
    return gopigo_status[0]
	


def read_timeout_status():    # Read timeout status
    global gopigo_status
    if (debugLevel): print "rwp:  read_timeout_status() called"
    return gopigo_status[1]
	
    

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
drive_bias = -5   # positive will drive right more than left

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
    rot_bias = (drive_bias * trans) / 100
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
   global _done, _high, _time
   _time = tick - _high
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
# Alan: _trig is gpioPin  (e.g. 26 for GPIO26 on pin 37)
#       _echo is a servoPin 0..7
#

def readDistance2gs(_trig, _echo):
   global pi, _done, _time
   _done = False
   myPDALib.pi.set_mode(_trig, pigpio.OUTPUT)
   myPDALib.pi.gpio_trigger(_trig,50,1)
   myPDALib.pi.set_mode(myPDALib.servopin[_echo], pigpio.INPUT)
   time.sleep(0.0001)
   tim = 0
   while not _done:
      time.sleep(0.001)
      tim = tim+1
      if tim > 50:
         return 0
   return _time / 58.068 # return as cm

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

usSensorInit = init_usSensor()

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
#Operating Voltage:3.0-7.2 Volts

#Features :
#- Coreless Motor
#- All Nylon Gear
#- Connector Wire Length 150MM

#import PDALib
#import myPDALib
#import myPyLib
#import time

TILTSERVO = 0
PANSERVO = 1

PanPosLimitL = 2500
PanPosCenter = 1500
PanPosLimitR =  630

PanDegLimitL = -90
PanDegCenter = 90
PanDegLimitR = +90

TiltPosLimitUp = 700  #550
TiltPosCenter = 1375
TiltPosLimitDn = 1900 #2435

TiltDegLimitUp = 90
TiltDegCenter  = 0
TiltDegLimitDn = -30


def center_servos():
  myPDALib.servoWrite(TILTSERVO, TiltPosCenter)
  myPDALib.servoWrite(PANSERVO, PanPosCenter)

def servos_off():
  myPDALib.pinMode(TILTSERVO,myPDALib.INPUT)    # init Tilt servo off
  myPDALib.pinMode(PANSERVO,myPDALib.INPUT)     # init motor2 servo off

def setup_servo_pins():
  myPDALib.pinMode(TILTSERVO, myPDALib.SERVO)    # init Tilt servo pin to SERVO mode
  myPDALib.pinMode(PANSERVO, myPDALib.SERVO )    # init Pan  servo pin to SERVO mode
  center_servos()
  servos_off()

dummy = setup_servo_pins()                     # initialize when module is loaded

def pos_servo(servo,pos=1500):
    if (debugLevel): print "rwp:pos_servo(Tilt0Pan1=%d, pos=%d)" % (servo,pos) 
    if (servo == PANSERVO):
        cpos = myPyLib.clamp(pos,PanPosLimitR,PanPosLimitL)
    elif (servo == TILTSERVO):
        cpos = myPyLib.clamp(pos,TiltPosLimitDn,TiltPosLimitUp)
    if (debugLevel): print "setting Tilt0Pan1=%d to pos: %d)" % (servo, cpos) 
    myPDALib.servoWrite(servo, cpos)   # set to new position


    
    

# ############ RWP TEST MAIN ######################
	
def main():
    print "rwp:main: *** RWP TEST MAIN ***"
    
    servo_range = [2,3,4,5,6,7,8]

    def key_input(event):
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
            2..8: servo position
            +: increase speed
            -: decrease speed
            >: increase drive_bias "rt wheel"
            <: decrease drive_bias             
            =: print all variables
            v: do set_speed(125), set_left(0), set_right(0)
            
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
            print_state()
        elif key_press == '>':
            rwp.drive_bias += 1
        elif key_press == '<':
            rwp.drive_bias -= 1
        elif key_press.isdigit():
            if int(key_press) in servo_range:
                enable_servo()
                servo(int(key_press)*14)
                time.sleep(1)
                disable_servo()
        elif key_press == 'v':
            set_speed(125)
            time.sleep(3)
            set_left_speed(0)
            time.sleep(3)
            set_right_speed(0)
            

    # command = tk.Tk()
    # command.bind_all('<Key>', key_input)  # ALAN  '' changed to '<Key>'
    # command.mainloop()

    ### created for command line execution cntl-C to quit

    while True:
      event=raw_input("cmd? ") 
      key_input(event)


	   
	   

if __name__ == "__main__":
    main()	    
