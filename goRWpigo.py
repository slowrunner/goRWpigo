#!/usr/bin/python
# File:  goRWpigo.py
# Doc:   RugWarriorPi version of Dexter Industries GoPiGo API

import time   # for test main
from mylibs import rwp
import random   # to simulate until implemented
from mylibs import myPyLib
from enum import Enum

debugLevel = 1                  # 0=off 1=some  99=all


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
#  Alan: gopigo speeds 1-255 mapped to MinPwr2Mov..MaxPwr
#  Alan: default speed of 200 is roughly 78% max speed
#  Alan: default_turn_speed of 25 is roughly 9% max speed

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
# print_state()   # prints out all state variables



# ################### IMPLEMENTATION #######
#gopigo_motor_modes = Enum('gopigo_motor_modes','init stop fwd bwd left right left_rot right_rot')
#gopigo_motor_modes = Enum('init','stop','fwd','bwd','left','right','left_rot','right_rot')
# sudo pip install enum34  to give python 3.4 compatible Enum type
class gopigo_motor_modes(Enum):
    init = 0
    stop = 1
    fwd  = 2
    bwd  = 3
    left = 4
    right = 5
    left_rot = 6
    right_rot = 7
    
gopigo_motor_mode = gopigo_motor_modes.init
gopigo_status = [0,0]       #  gopigo_status[0]= Encoder targeting status: 0=target reached
                                 #  gopigo_status[1]= Timeout status: 0=timeout reached
gopigo_speed = 0                 #  0-255
gopigo_enc_1_rev = 18            #  
gopigo_enc_tgt = [0,0,2 * gopigo_enc_1_rev]   
gopigo_servo_angle = 90                                  
gopigo_com_timeout = 10000       
gopigo_default_speed = 200
gopigo_default_turn_speed = 25

# ### Motor control functions:

def fwd(speed=gopigo_default_speed):    # Move the GoPiGo forward with PID (better control)
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  fwd() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.fwd
    rwp.fwd(speed)

def motor_fwd(speed=gopigo_default_speed):    # Move the GoPiGo forward without PID
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  motor_fwd() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.fwd
    rwp.motor_fwd(speed)
	
def bwd(speed=gopigo_default_speed):    # Move the GoPiGo back with PID (better control)
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  bwd() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.bwd
    rwp.motor_bwd(speed)
	
def motor_bwd(speed=gopigo_default_speed):    # Move the GoPiGo back without PID
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  motor_bwd() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.bwd
    rwp.motor_bwd(speed)
	
def left(speed=gopigo_default_turn_speed):    # Turn GoPiGo Left slow (one motor off, better control)
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  left() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.left
    rwp.left(speed)
	
def left_rot(speed=gopigo_default_turn_speed):    # Rotate GoPiGo left in same position 
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
                   # (both motors moving in the opposite direction)
    if (debugLevel): print "goRWpigo:  left_rot() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.left_rot
    rwp.left_rot(speed)
	
def right(speed=gopigo_default_turn_speed):    # Turn GoPiGo right slow (one motor off, better control)
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  right() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.right
    rwp.right(speed)
	
def right_rot(speed=gopigo_default_turn_speed):    # Rotate GoPiGo right in same position 
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
                    # both motors moving in the opposite direction)
    if (debugLevel): print "goRWpigo:  right_rot() called"
    gopigo_speed = speed
    gopigo_motor_mode=gopigo_motor_modes.right_rot
    rwp.right_rot(speed)
	
def stop():    # Stop the GoPiGo
    global gopigo_speed,gopigo_motor_modes, gopigo_motor_mode
    if (debugLevel): print "goRWpigo:  stop() called"
    gopigo_speed=0
    gopigo_motor_mode=gopigo_motor_modes.stop
    rwp.stop()
	

# ### Motor speed Functions:

def increase_speed():    # Increase the speed of moving motors by 10
    global gopigo_speed
    gopigo_speed = myPyLib.clamp(gopigo_speed + 10,0,255)  # adjust velocity to limits
    if (debugLevel): 
        print "goRWpigo:increase_speed() called"
        print "goRWpigo:  gopigo_speed:",gopigo_speed
    rwp.increase_speed()
    
	
def decrease_speed():    # Decrease the speed of the GoPiGo by 10
    global gopigo_speed
    gopigo_speed = myPyLib.clamp(gopigo_speed - 10,0,255)  # adjust velocity to limits
    if (debugLevel): 
        print "goRWpigo:decrease_speed() called"
        print "goRWpigo:  gopigo_speed:",gopigo_speed    
    rwp.decrease_speed()
	
def set_left_speed(speed=gopigo_default_speed):    # Set speed of the left motor
    global gopigo_speed
    gopigo_speed = speed
    if (debugLevel):
        print "goRWpigo:set_left_speed(%d) called" % speed
        print "goRWpigo:  gopigo_speed:", gopigo_speed
    rwp.set_left_speed(speed)
	
def set_right_speed(speed=gopigo_default_speed):    # Set speed of the right motor
    global gopigo_speed
    gopigo_speed = speed
    if (debugLevel):
        print "goRWpigo:set_right_speed(%d) called" % speed
        print "goRWpigo:  gopigo_speed:", gopigo_speed
    rwp.set_right_speed(speed)
	
def set_speed(speed=gopigo_default_speed):    # Set speeds of both the motors
    if (debugLevel): print "goRWpigo:set_speed(%d) called" % speed
    gopigo_speed = speed
    if (debugLevel): print "goRWpigo:gopigo_speed:", gopigo_speed
    rwp.set_speed(speed)
	

# ### Encoder Functions:

def enc_tgt(m1,m2,numEncPulses):    # Set encoder target to move the GoPiGo to a set distance
    if (debugLevel): print "goRWpigo:enc_tgt(m1=%d,m2=%d,numEncPulses=%d) called" % (m1,m2,numEncPulses)
    gopigo_enc_tgt = [m1, m2, numEncPulses]
	
def enable_encoders():    # Enable the encoders
    if (debugLevel): print "goRWpigo:enable_encoders() called"
	
def disable_encoders():    # Disable the encoders
    if (debugLevel): print "goRWpigo:disable_encoders() called"
	

# ### Ultrasonic ranger read:

def us_dist(pin):    # Read distance from the ultrasonic sensor
    if (debugLevel): print "goRWpigo:us_dist(pin=%d) called" % pin
    dist=rwp.us_dist()
    if (debugLevel): print "goRWpigo:us_dist: returning %f" % dist
    return dist
	


# ### LED control:

def led(led,pwr):    # Set  LED to  power level
    if (debugLevel): print "goRWpigo:led() called"
	
def led_on(led):    # Turn  LED on
    if (debugLevel): print "goRWpigo:led_on() called"
	
def led_off(led):    # Turn LED off
    if (debugLevel): print "goRWpigo:led_off() called"
	


# ### Servo control:

def enable_servo():    # Enables the servo
    if (debugLevel): print "goRWpigo:enable_servo() called"
    rwp.servos_on()
	
def disable_servo():    # Disables the servo
    if (debugLevel): print "goRWpigo:disable_servo() called"
    rwp.servos_off()
	
def servo(angle):    # Set servo position 0-180  0=full left, 180=full right
    global gopigo_servo_angle
    if (debugLevel): print "goRWpigo:servo(%d) called" % angle
    gopigo_servo_angle = angle
    gopigo_servo_angle = rwp.pan_servo(angle)  #call returns the rwp limited angle
    
	


# ### Status from the GoPiGo:

def volt():    # Read battery voltage in V
    if (debugLevel): print "goRWpigo:volt() called"
    return 7.2
	
def fw_ver():    # Get the firmware version of the GoPiGo
    if (debugLevel): print "goRWpigo:fw_ver() called"
    return "Alan's goRWpigo.py 7Jun2017"
	
def enable_com_timeout(ms):    # Enable communication time-out
                               #  (stop the motors if no command received in the specified time-out)
    global gopigo_com_timeout
    gopigo_com_timeout = ms    
    if (debugLevel): print "goRWpigo:enable_com_timeout(ms=%d) called" % ms
	
def disable_com_timeout():    # Disable communication time-out
    if (debugLevel): print "goRWpigo:disable_com_timeout() called"
	
def read_status():    # Read the status register on the GoPiGo
    global gopigo_status
    if (debugLevel): print "goRWpigo:read_status() called"
    return gopigo_status
	
def read_enc_status():    # Read encoder status
    global gopigo_status
    if (debugLevel): print "goRWpigo:read_enc_status() called"
    return gopigo_status[0]
	


def read_timeout_status():    # Read timeout status
    global gopigo_status
    if (debugLevel): print "goRWpigo:read_timeout_status() called"
    return gopigo_status[1]
	
    
# ---------- ALAN's FUNCS ------
def print_state():
        global gopigo_status, gopigo_speed, gopigo_enc_1_rev, gopigo_enc_tgt, gopigo_com_timeout, gopigo_servo_angle
        print "goRWpigo State Variables:"
        print "gopigo_status:    ", repr(gopigo_status)
        print "gopigo_motor_mode:", gopigo_motor_mode
        print "gopigo_speed:     ", gopigo_speed
        print "rwp_speeds:lft,rt ", repr(rwp.rwp_speeds)
        print "rwp.drive_bias:   ", rwp.drive_bias
        print "gopigo_enc_1_rev: ", gopigo_enc_1_rev
        print "gopigo_enc_tgt:   ", gopigo_enc_tgt
        print "gopigo_com_timeout:", gopigo_com_timeout
        print "gopigo_servo_angle:", gopigo_servo_angle
    

# ############ TEST MAIN ######################
	
def main():
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
            print(us_dist(15))
        elif key_press == '=':
            print_state()
        elif key_press == '>':
            rwp.drive_bias += 1
        elif key_press == '<':
            rwp.drive_bias -= 1
        elif key_press.isdigit():
            if int(key_press) in servo_range:
                enable_servo()
                cmdDeg = (8-int(key_press))*30 
                servo(cmdDeg)   # Alan lft 2=180 5=90 8=0 rt
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
    print "--- goRWpigo TEST MAIN STARTED"

    while True:
      event=raw_input("cmd? ") 
      key_input(event)












'''
    testspeed = 100  # speed for teseting  0-255 default 200
    angle = 45   # 45 deg left for servo test
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
'''    
	   
	   
	   

if __name__ == "__main__":
    main()	    
