#!/usr/bin/python
#
# File: pigpio.py   
# Doc:        simulated pi gpio module 
#             
#
debugLevel = 1
FALLING_EDGE = 0
RISING_EDGE  = 1
EITHER_EDGE  = 2

class pi():
    def __init__(self):
        if (debugLevel): print "pigpio:pi class init"        

    def set_PWM_frequency(self,pin,freq):
            if (debugLevel): print "pigpio:set_PWM_frequency(%d,%d)"  % (pin,freq)        
