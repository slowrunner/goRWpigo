#!/usr/bin/python
#  File: goRWpigo_remote_cli.python
#
#  Based on "Programming Autonomy - Robotics with Python Raspberry Pi and GoPiGo p.6"
#            From https://pythonprogramming.net/programming-autonomous-robot-gopigo-tutorial/

# ALAN - changes:  1) changed gopigo api import to goRWpigo
#                  2) 

from goRWpigo import *
import time
import random

min_distance = 70

def autonomy():
    print '\nAutonomy:  --- STARTING AUTONOMY p6 ---'
    no_problem = True
    while no_problem:
        servo(70)
        time.sleep(1)
        dist = us_dist(15)
        if dist > min_distance:
            print '\nAutonomy: Forward is fine with me', dist
            fwd()
            time.sleep(1)
        else:
            print '\nAutonomy: Stuff is in the way', dist
            stop()
            servo(28)
            time.sleep(1)
            left_dir = us_dist(15)
            time.sleep(1)
            servo(112)
            right_dir = us_dist(15)
            time.sleep(1)

            if left_dir > right_dir and left_dir > min_distance:
                print '\nAutonomy: Choose left!'
                left()
                time.sleep(1)
            elif left_dir < right_dir and right_dir > min_distance:
                print '\nAutonomy: Choose Right!'
                right()
                time.sleep(1)
            else:
                print '\nAutonomy: No good option, REVERSE!'
                bwd()
                time.sleep(2)
                rot_choices = [right_rot, left_rot]
                rotation = rot_choices[random.randrange(0,2)]
                rotation()
                time.sleep(1)

            stop()
            print '\nAutonomy: Take a break'
                
stop()
enable_servo()
servo(70)
time.sleep(3)
autonomy()