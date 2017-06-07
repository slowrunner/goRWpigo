#!/usr/bin/python
#
# File: rwpida.py   
# Doc:        Rug Warrior with Pi Droid Alpha 
#             Primitives for goRWpigo interface
#

import PDALib
import myPDALib
import myPyLib
import time      # for test main

import motorsClass.py



# GoPiGo API SUMMARY

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
# motors_init()   # if not done prior, configures motor pins
# motors_kill()   # set both motors to coast to stop



# ################### IMPLEMENTATION #######
gopigo_status = '\x00\x00'       #  gopigo_status[0]= Encoder targeting status: 0=target reached
                                 #  gopigo_status[1]= Timeout status: 0=timeout reached
gopigo_speed = [0,0]
gopigo_edges_per_rev = 18        #  
gopigo_enc_tgt = [0,0,2 * gopigo_edges_per_rev]   
gopigo_servo_angle = 90                                  
gopigo_com_timeout = 10000  

debugLevel = 99		# 0 off, 1 some, 99 all    
 

# ### Motor control functions:

def fwd():    # Move the GoPiGo forward with PID (better control)
    print "fwd() called"

def motor_fwd():    # Move the GoPiGo forward without PID
    print "motor_fwd() called"
	
def bwd():    # Move the GoPiGo back with PID (better control)
    print "bwd() called"
	
def motor_bwd():    # Move the GoPiGo back without PID
    print "motor_bwd() called"
	
def left():    # Turn GoPiGo Left slow (one motor off, better control)
    print "left() called"
	
def left_rot():    # Rotate GoPiGo left in same position 
                   # (both motors moving in the opposite direction)
    print "left_rot() called"
	
def right():    # Turn GoPiGo right slow (one motor off, better control)
    print "right() called"
	
def right_rot():    # Rotate GoPiGo right in same position 
                    # both motors moving in the opposite direction)
    print "right_rot() called"
	
def stop():    # Stop the GoPiGo
    print "rwp.stop: motors_kill()"
    motors_kill()
	

# ### Motor speed Functions:

def increase_speed():    # Increase the speed of the GoPiGo by 10
    global gopigo_speed
    gopigo_speed = map(lambda x: x+10, gopigo_speed )
    print "increase_speed() called"
    print "gopigo_speed:",gopigo_speed
    
	
def decrease_speed():    # Decrease the speed of the GoPiGo by 10
    global gopigo_speed
    gopigo_speed = map(lambda x: x-10, gopigo_speed )
    print "decrease_speed() called"
    print "gopigo_speed:",gopigo_speed    

	
def set_left_speed(speed=200):    # Set speed of the left motor
    global gopigo_speed
    gopigo_speed[0] = speed
    print "set_left_speed(%d) called" % speed
    print "gopigo_speed:", gopigo_speed    
	
def set_right_speed(speed=200):    # Set speed of the right motor
    global gopigo_speed
    gopigo_speed[1] = speed
    print "set_right_speed(%d) called" % speed
    print "gopigo_speed:", gopigo_speed
	
def set_speed(speed=200):    # Set speeds of both the motors
    print "set_speed(%d) called" % speed
    gopigo_speed = [speed,speed]
    print "gopigo_speed:", gopigo_speed
	

# ### Encoder Functions:

def enc_tgt(m1,m2,numEncPulses):    # Set encoder target to move the GoPiGo to a set distance
    print "enc_tgt(m1=%d,m2=%d,numEncPulses=%d) called" % (m1,m2,numEncPulses)
    gopigo_enc_tgt = [m1, m2, numEncPulses]
	
def enable_encoders():    # Enable the encoders
    print "enable_encoders() called"
	
def disable_encoders():    # Disable the encoders
    print "disable_encoders() called"
	

# ### Ultrasonic ranger read:

def us_dist(pin):    # Read distance from the ultrasonic sensor
    print "us_dist(pin=%d) called" % pin
    return random.uniform(0,50)
	


# ### LED control:

def led(led,pwr):    # Set  LED to  power level
    print "led() called"
	
def led_on(led):    # Turn  LED on
    print "led_on() called"
	
def led_off(led):    # Turn LED off
    print "led_off() called"
	


# ### Servo control:

def enable_servo():    # Enables the servo
    print "enable_servo() called"
	
def disable_servo():    # Disables the servo
    print "disable_servo() called"
	
def servo(angle):    # Set servo position
    global gopiogo_servo_angle
    print "servo(%d) called" % angle
    gopigo_servo_angle = angle
	


# ### Status from the GoPiGo:

def volt():    # Read battery voltage in V
    print "volt() called"
    return 7.2
	
def fw_ver():    # Get the firmware version of the GoPiGo
    print "fw_ver() called"
    return "Alan's gorwpigo.py 6Jun2017"
	
def enable_com_timeout(ms):    # Enable communication time-out
                               #  (stop the motors if no command received in the specified time-out)
    global gopigo_com_timeout
    gopigo_com_timeout = ms    
    print "enable_com_timeout(ms=%d) called" % ms
	
def disable_com_timeout():    # Disable communication time-out
    print "disable_com_timeout() called"
	
def read_status():    # Read the status register on the GoPiGo
    global gopigo_status
    print "read_status() called"
    return gopigo_status
	
def read_enc_status():    # Read encoder status
    global gopigo_status
    print "read_enc_status() called"
    return gopigo_status[0]
	


def read_timeout_status():    # Read timeout status
    global gopigo_status
    print "read_timeout_status() called"
    return gopigo_status[1]
	
    
# ---------- ALAN's FUNCS ------
def print_state():
        global gopigo_status, gopigo_speed, gopigo_edges_per_rev, gopigo_enc_tgt, gopigo_com_timeout, gopigo_servo_angle
        print "goRWpigo State Variables:"
        print "gopigo_status: ", repr(gopigo_status)
        print "gopigo_speed:", gopigo_speed
        print "gopigo_edges_per_rev:", gopigo_edges_per_rev
        print "gopigo_enc_tgt:", gopigo_enc_tgt
        print "gopigo_com_timeout:", gopigo_com_timeout
        print "gopigo_servo_angle:", gopigo_servo_angle


# #############################################
# Rug Warrior platform Definition and Vars

global drive_bias

LMotorIndex = 0     # LEFT MOTOR
RMotorIndex = 1    # RIGHT MOTOR

# Motor Pins 
# SRV 6		Motor 1 Speed (PWM)
# SRV 7		Motor 2 Speed (PWM)

RMotorPin = 6
LMotorPin = 7

MotorPin = [7,6]  # MotorPin[0] Left, MotorPin[1] Right

# DIO 12 (A4)	Motor 1 Dir A (0=coast 1=F/Brake)
# DIO 13 (A5)	Motor 1 Dir B (0=coast 1=R/Brake)

# DIO 14 (A6)	Motor 2 Dir A (0=coast 1=F/Brake)
# DIO 15 (A7)	Motor 2 Dir B (0=coast 1=R/Brake)

M1DirA = 12
M1DirB = 13
M2DirA = 14
M2DirB = 15
MotorDirA = [14,12]  # 0 left 1 right
MotorDirB = [15,13]  # 0 left 1 right

MinPwr2Move = 100
MaxPwr = 255

# No two drive motors respond in exactly the same way to the same
#   applied voltage.  Use the drive_bias term to correct for biases
#   in your robot.  If your robot arcs to the right, make drive_bias
#   positive, arcs to the left require a negative correction. 

drive_bias = 0   # Open loop correction term for drive motors


# ### MOTORS_INIT()

motorsInitialized = False

def motors_init():   # set up the pwm and two dir pins for each motor
    global motors_initialized
  
    if (!motorsInitialized):
        PDALib.pinMode(RMotorPin,PDALib.PWM)  # init motor1 speed control pin
        PDALib.pinMode(LMotorPin,PDALib.PWM)  # init motor2 speed control pin 

        PDALib.pinMode(M1DirA,PDALib.OUTPUT)  #init motor1 dirA/Fwd    enable
        PDALib.pinMode(M1DirB,PDALib.OUTPUT)  #init motor1 dirB/Bkwd  enable
        PDALib.pinMode(M2DirA,PDALib.OUTPUT)  #init motor2 dirA/Fwd    enable
        PDALib.pinMode(M2DirB,PDALib.OUTPUT)  #init motor2 dirB/Bkwd  enable

        # init all direction pins to off
        PDALib.digitalWrite(M1DirA,0)  #set to off/coast
        PDALib.digitalWrite(M1DirB,0)  #set to off/coast
        PDALib.digitalWrite(M2DirA,0)  #set to off/coast
        PDALib.digitalWrite(M2DirB,0)  #set to off/coast

        # turn off the speed pins
        PDALib.analogWrite(Motors.RMotor,0)  #set motor1 to zero speed 
        PDALib.analogWrite(Motors.LMotor,0)  #set motor2 to zero speed
        motors_initialized = True

# ### MOTORS_KILL()

def motors_kill():
    motors_init()	# make sure motors are initialized
    # turn off the speed pin 
    PDALib.analogWrite(RMotorPin,0)  #set motor1 to coast (zero speed)  
    PDALib.analogWrite(LMotorPin,0)  #set motor2 to coast (zero speed)


# ### MOTOR(INDEX,VEL)

def motor(index,vel):  #mtr 0=lft, 1=rt, +/-100
    vel = myPyLib.clamp(vel,-100,100)
    avel = vel
    if (vel == 0):
        PDALib.analogWrite(MotorPin[index],0)  #set motor to zero speed
        return
    if (vel > 0):  # set forward
        PDALib.digitalWrite(MotorDirA[index],1)  #set to fwd
        PDALib.digitalWrite(MotorDirB[index],0)  #set to off/coast
        avel = vel
    else:
        avel = -vel        
        PDALib.digitalWrite(MotorDirA[index],0)  #set to off/coast
        PDALib.digitalWrite(MotorDirB[index],1)  #set to bwd
  
    # compute pct to pwr
    pwr = int( (MaxPwr - MinPwr2Move) * avel/100.0 + MinPwr2Move)    
    PDALib.analogWrite(MotorPin[index], pwr)  #set motor pwr level

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

# Left Encoder - DIO B4 - "PDALib.pin 20"
# Right Encoder- DIO B3 - "PDALib.pin 19" 

    
    

# ############ TEST MAIN ######################
	
def main():
    print "--- rwp TEST MAIN STARTED"

    print "--- rwp TEST MAIN Completed"
	   
	   
	   

if __name__ == "__main__":
    main()	    