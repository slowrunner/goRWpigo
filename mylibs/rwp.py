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
rwp_speeds = [0,0]
rwp_edges_per_rev = 32        #  
rwp_enc_tgt = [0,0,0]   
rwp_servo_angle = 0                                  
rwp_com_timeout = 10000  
rwp_default_speed = 200

debugLevel = 99		# 0 off, 1 some, 99 all    
 

# ### Motor control functions:

def fwd(speed=rwp_default_speed):    # Move the GoPiGo forward with PID (better control)
    if (debugLevel): print "rwp:  fwd() called"

    
    

def motor_fwd(speed=rwp_default_speed):    # Move the GoPiGo forward without PID
    global rwp_speeds, drive_bias
    if (debugLevel): print "rwp:  motor_fwd() called"
    if (drive_bias > 0):  rwp_speeds = [speed, speed - drive_bias]
    else: rwp_speeds = [speed - abs(drive_bias), speed]
    motor(LMotorIndex, rwp_speeds[1])
    motor(RMotorIndex, rwp_speeds[0])    
	
def bwd():    # Move the GoPiGo back with PID (better control)
    if (debugLevel): print "rwp:  bwd() called"
	
def motor_bwd():    # Move the GoPiGo back without PID
    if (debugLevel): print "rwp:  motor_bwd() called"
	
def left():    # Turn GoPiGo Left slow (one motor off, better control)
    if (debugLevel): print "rwp:  left() called"
	
def left_rot():    # Rotate GoPiGo left in same position 
                   # (both motors moving in the opposite direction)
    if (debugLevel): print "rwp:  left_rot() called"
	
def right():    # Turn GoPiGo right slow (one motor off, better control)
    if (debugLevel): print "rwp:  right() called"
	
def right_rot():    # Rotate GoPiGo right in same position 
                    # both motors moving in the opposite direction)
    if (debugLevel): print "rwp:  right_rot() called"
	
def stop():    # Stop the GoPiGo
    if (debugLevel): print "rwp:  rwp.stop: motors_coast()"
    motors_coast()
	

# ### Motor speed Functions:

def increase_speed():    # Increase the speed of the GoPiGo by 10
    global gopigo_speed
    gopigo_speed = map(lambda x: x+10, gopigo_speed )
    if (debugLevel): print "rwp:  increase_speed() called"
    if (debugLevel): print "rwp:  gopigo_speed:",gopigo_speed
    
	
def decrease_speed():    # Decrease the speed of the GoPiGo by 10
    global gopigo_speed
    gopigo_speed = map(lambda x: x-10, gopigo_speed )
    if (debugLevel): print "rwp:  decrease_speed() called"
    if (debugLevel): print "rwp:  gopigo_speed:",gopigo_speed    

	
def set_left_speed(speed=200):    # Set speed of the left motor
    global gopigo_speed
    gopigo_speed[0] = speed
    if (debugLevel): print "rwp:  set_left_speed(%d) called" % speed
    if (debugLevel): print "rwp:  gopigo_speed:", gopigo_speed    
	
def set_right_speed(speed=200):    # Set speed of the right motor
    global gopigo_speed
    gopigo_speed[1] = speed
    if (debugLevel): print "rwp:  set_right_speed(%d) called" % speed
    if (debugLevel): print "rwp:  gopigo_speed:", gopigo_speed
	
def set_speed(speed=200):    # Set speeds of both the motors
    if (debugLevel): print "rwp:  set_speed(%d) called" % speed
    gopigo_speed = [speed,speed]
    if (debugLevel): print "rwp:  gopigo_speed:", gopigo_speed
	

# ### Encoder Functions:

def enc_tgt(m1,m2,numEncPulses):    # Set encoder target to move the GoPiGo to a set distance
    if (debugLevel): print "rwp:  enc_tgt(m1=%d,m2=%d,numEncPulses=%d) called" % (m1,m2,numEncPulses)
    gopigo_enc_tgt = [m1, m2, numEncPulses]
	
def enable_encoders():    # Enable the encoders
    if (debugLevel): print "rwp:  enable_encoders() called"
	
def disable_encoders():    # Disable the encoders
    if (debugLevel): print "rwp:  disable_encoders() called"
	

# ### Ultrasonic ranger read:

def us_dist(pin):    # Read distance from the ultrasonic sensor
    if (debugLevel): print "rwp:  us_dist(pin=%d) called" % pin
    return random.uniform(0,50)
	


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

MinPwr2Move = 100   # empirical minimum power to move with both wheels
MaxPwr = 255
drive_bias = -5   # positive will drive right more than left

# ### MOTORS_INIT()
motorsInitialized = False   # True after motor pins are configured once
def motors_init():   # set up the two speed and four dir pins
    global motors_initialized
  
    if not motorsInitialized:
        PDALib.pinMode(RMotorPin,PDALib.PWM)  # init rt-motor1  speed control pin
        PDALib.pinMode(LMotorPin,PDALib.PWM)  # init lft-motor2 speed control pin 

        PDALib.pinMode(M1DirA,PDALib.OUTPUT)  #init rt-motor1 dirA/Fwd   enable
        PDALib.pinMode(M1DirB,PDALib.OUTPUT)  #init rt-motor1 dirB/Bkwd  enable
        PDALib.pinMode(M2DirA,PDALib.OUTPUT)  #init lft-motor2 dirA/Fwd  enable
        PDALib.pinMode(M2DirB,PDALib.OUTPUT)  #init lft-motor2 dirB/Bkwd enable

        PDALib.digitalWrite(M1DirA,0)  #set to off/coast
        PDALib.digitalWrite(M1DirB,0)  #set to off/coast
        PDALib.digitalWrite(M2DirA,0)  #set to off/coast
        PDALib.digitalWrite(M2DirB,0)  #set to off/coast

        # turn off the speed pins
        PDALib.analogWrite(Motors.RMotor,0)  #set rt  (motor1,index 1) to zero speed 
        PDALib.analogWrite(Motors.LMotor,0)  #set lft (motor2,index 0) to zero speed
        motors_initialized = True   # Don't need to do this again

# ### MOTORS_coast()
def motors_coast():
    motors_init()	# make sure motors are initialized
    # turn off the speed pin 
    PDALib.analogWrite(RMotorPin,0)  #set motor1 to coast (zero speed)  
    PDALib.analogWrite(LMotorPin,0)  #set motor2 to coast (zero speed)


# ### MOTOR(INDEX,VEL)

def motor(index,vel):  #mtr 0=lft, 1=rt, +/-255
    avel = myPyLib.clamp(vel,-255,255)  # adjust velocity to limits
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