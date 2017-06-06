#!/usr/bin/python
# File:  goRWpigo.py
# Doc:   RugWarriorPi version of Dexter Industries GoPiGo API

import time   # for test main
import random  #randint(0,9)  0..9  uniform(0,50) 0.000 to 50.000


# GoPiGo API

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



# ################### IMPLEMENTATION #######
gopigo_status = '\x00\x00'       #  gopigo_status[0]= Encoder targeting status: 0=target reached
                                 #  gopigo_status[1]= Timeout status: 0=timeout reached
gopigo_speed = [0,0]
gopigo_edges_per_rev = 18        #  
gopigo_enc_tgt = [0,0,2 * gopigo_edges_per_rev]   
gopigo_servo_angle = 90                                  
gopigo_com_timeout = 10000       

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
    print "stop() called"
	

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
    

# ############ TEST MAIN ######################
	
def main():
    testspeed = 100  # speed for teseting  0-255 default 200
    angle = 45   # 45 deg left for servo test
    print "--- goRWpigo TEST MAIN STARTED"
    fwd()    # Move the GoPiGo forward with PID (better control)
    time.sleep(1); stop()

    motor_fwd()    # Move the GoPiGo forward without PID
    time.sleep(1); stop()

    bwd()    # Move the GoPiGo back with PID (better control)
    time.sleep(1); stop()

    motor_bwd()    # Move the GoPiGo back without PID
    time.sleep(1); stop()

    left()    # Turn GoPiGo Left slow (one motor off, better control)
    time.sleep(1); stop()

    left_rot()    # Rotate GoPiGo left in same position 
#             (both motors moving in the opposite direction)
    time.sleep(1); stop()

    right()    # Turn GoPiGo right slow (one motor off, better control)
    time.sleep(1); stop()

    right_rot()    # Rotate GoPiGo right in same position 
#             both motors moving in the opposite direction)
    time.sleep(1)

    stop()    # Stop the GoPiGo


# ### Motor speed Functions:

    increase_speed()    # Increase the speed of the GoPiGo by 10
    decrease_speed()    # Decrease the speed of the GoPiGo by 10
    set_left_speed(testspeed)     # Set speed of the left motor
    set_right_speed(testspeed)    # Set speed of the right motor
    set_speed(testspeed/2)    # Set speeds of both the motors


# ### Encoder Functions:

    enc_tgt(1,1,36)    # Set encoder target to move the GoPiGo 2 wheel revolutions
    fwd()              # drive 2 wheel revs
	
    enable_encoders()    # Enable the encoders 
    disable_encoders()    # Disable the encoders

# ### Ultrasonic ranger read:

    print us_dist(1)    # Read distance from the ultrasonic sensor


# ### LED control:

    led(1,255)    # Set left LED max power level
    led_on(0)    # Turn right LED on
    led_off(1)    # Turn left LED off


# ### Servo control:

    enable_servo()    # Enables the servo
    servo(angle)    # Set servo position 
    disable_servo()    # Disables the servo


# ### Status from the GoPiGo:

    print volt()    # Read battery voltage in V
    print fw_ver()    # Get the firmware version of the GoPiGo
	
    fwd()     # start going for test of com timeout
    enable_com_timeout(1000)    # Enable communication time-out
                                # (stop the motors if no command received in 1 sec)
    disable_com_timeout()    # Disable communication time-out
    print read_status()    # Read the status register on the GoPiGo
    print read_enc_status()    # Read encoder status


    print read_timeout_status()    # Read timeout status	
    
 # ### Alan's funcs:

    print_state() 
    
    print "--- goRWpigo TEST MAIN Completed"
	   
	   
	   

if __name__ == "__main__":
    main()	    