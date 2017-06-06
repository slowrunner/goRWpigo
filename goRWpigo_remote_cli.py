#!/usr/bin/python
#  File: goRWpigo_remote_cli.python
#
#  Based on "Programming Remote Control - Robotics with Python Raspberry Pi and GoPiGo p.4"
#            From https://pythonprogramming.net/remote-control-robot-programming-tutorial

# ALAN - changes:  1) changed gopigo api import
#                  2) rm tkinter design
#                  3) added <Key> to make the bind_all work on Windows7/Python2.7
#                  4) expanded commands

# import Tkinter as tk
from goRWpigo import *     # was from gopigo import *

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
        =: print all variables
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


    elif key_press.isdigit():
        if int(key_press) in servo_range:
            enable_servo()
            servo(int(key_press)*14)
            time.sleep(1)
            disable_servo()

# command = tk.Tk()
# command.bind_all('<Key>', key_input)  # ALAN  '' changed to '<Key>'
# command.mainloop()

### created for command line execution cntl-C to quit

while True:
  event=raw_input("cmd? ") 
  key_input(event)