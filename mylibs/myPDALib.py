#
# myPDALib.py   
#
# v0.2  07June2017  Merged PDALib.py v.93plus
# v0.1	14June2016  LibExit()

import myPyLib
import pigpio

sim = False

##### SUPPLEMENTAL Pi Droid Alpha FUNCTIONS
# MORE CONSTANTS FOR MCP23S17 PDALib.setDioBit, clearDioBit

DIO_INTEN = 0x04 # interupt enable, 1 enables interrupt-on-change for pin
DIO_DEFVAL = 0x06 # default value, for interrupt if different than value on pin
DIO_INTCON = 0x08 # interrupt control, 0: compare to prev val, 1: to default val 
DIO_IOCON = 0x0A  # register containing interrupt polarity bit
DIO_INTPOLbit = 1 # sets polarity of INT output pin: 0=Active-low 1=Active-high
DIO_INTMIRRORbit = 6 # 0=separate 1=bankA or B sets both interrupt pins
DIO_INTF = 0x0E   # bit set indicates which (DioBit) caused the interrupt
DIO_INTCAP = 0x10 # values at time of interrupt (reading resets interrupt)
DIO_INTCAPB = 0x11 # values of portB at time of int (reading resets int)

# INTERRUPT ON PIN
# See what pin caused interrupt
def interruptOnPin():  
    if (sim):  return 0
    # call to see what PDALib pin caused the interrupt
    interruptPins = readDio(DIO_INTF)
    # print ("interruptOnPin(): 0x%x" % interruptPins)
    # convert from register of bits to PDALib pin
    interruptPin = interruptPins  # for the time being
    return interruptPin

# READ AND RESET INTERRUPT
# return state of PortB at time of interrupt
# reading state also resets interrupt state
 
def readAndResetInterrupt():  
    if (sim):  return 0
    # read the MCP23S17 combined dio bank, resets interrupt

    #dioBitsAtInt = readDio2(DIO_INTCAPB)   # only reads PortB
    #return dioBitsAtInt                    # return portB bits

    dioBitsAtInt = readDio(DIO_INTCAP)  # reads both PortA and PortB
    return dioBitsAtInt>>8    # only send Port B

# overload for RoboPiExit()
def PiExit():
    if (sim):  return 0
    pi.stop()
    print "myPDALib.PiExit():  pi.stop() called"

#
# Read specified MCP23S17 register (8 bits)
# 

def readDio2(reg):
    if (sim):  return 0
    with dio_lock:
        spi.open(0,0)  
        r = spi.xfer2([0x41,reg,0])
        spi.close()
        return r[2]
  
FALLING_EDGE = pigpio.FALLING_EDGE
RISING_EDGE = pigpio.RISING_EDGE
EITHER_EDGE = pigpio.EITHER_EDGE

hPi_Callback = None

# SET CALLBACK   sets pigpio callback on a GPIOnn pin
#
# pin: PiDroidAlpha Servo pin (0-7)
# edge: FALLING_EDGE, RISING_EDGE, EITHER_EDGE
# fCallback: function to call when edge condition met 
def setCallback(pin,edge,fCallback):
    global hPi_Callback
    if (sim):  return 0
    hPi_Callback = pi.callback(servopin[pin], edge, fCallback)

# CLEAR CALLBACK  removes pigpio callback on a GPIOnn pin
def clearCallback():
    global hPi_Callback
    if (sim):  return 0
    hPi_Callback.cancel()


#
# read SPI data from MCP3208 chip, 8 possible adc's (0 thru 7)
#

def analogRead12bit(adcnum):
    if (sim):  return 0
    with dio_lock:
      if adcnum < 0:
        return -1   
    if adcnum > 7:
      return -1   
    spi.open(0,1) 
    r = spi.xfer2(req12[adcnum])
    spi.close()
    adcout = ((r[1] & 0b1111) << 8) + r[2]
    return adcout



# ##################
#  readVoltage(adcPin)  return voltage in Volts
#
# MCP3208 12bit
VperBit = (4.94/4095)    # measured 5v with multi meter at ADC

# MCP3008  10bit  (original ADC with PiDroidAlpha)
# VperBit = (4.87/1024)  #4.89 gives same value as multi-meter at 4.68v

#
# adcPin values between 0..7 (others return -1)
# returns voltage in Volts or -1 Error
# 
def readVoltage(adcPin):
    if (sim):  return 0
    if ( (adcPin <0) | (adcPin>7) ):
      return -1
    # obsolete 10bit
    # return (VperBit * analogRead(adcPin))

    return (VperBit * analogRead12bit(adcPin))
  
  
#### PDALib from micronauts.combined
#!/usr/bin/python
#
# PDALib.py	Python API for Pi Droid Alpha educational robot controller
#
# Copyright 2015 William Henning
# http://Mikronauts.com
#
# PDALib provides an implementation of the RoboPi advanced robot controller for
# the Pi Droid Alpha educational robot controller.
#
# Please see the Pi Droid Alpha User Manual for documentation
#
# Aug.25/2015: v0.90 release (readDistance() not reliable)
# Sep.21/2015: v0.91 fixed Pi GPIO INPUT/OUTPUT constants, byte endian issue
# Feb.26/2015  v0.92 added readDistance2(trig,echo) support for HC-SR04
# see http://www.mikronauts.com/raspberry-pi/gpio-experiments/raspberry-pi-and-hc-sr04-distance-sensor-interfacing-with-c-and-python/
# Apr.12/2016: v0.93 fixed spi open/close mismatches
#

# Jul.05/2016: v0.93plus  # ALAN added thread-safe critical section locking 
#                         # ALAN removed duplicate digitalWrite()


import time
import spidev
#import pigpio
import threading     # ALAN  for threading.Lock() critical sectioning

# pin 0-7  = Pi GPIO's
# pin 8-23 = Dio bits

INPUT  = 0
OUTPUT = 1
PWM    = 2
SERVO  = 3
PING   = 4
TRIG   = 5
ECHO   = 6

mode     = [0,0,0,0,0,0,0,0]
diomode  = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
servopin = [4, 17, 18, 27, 22, 23, 24, 25]
servoval = [0, 0, 0, 0, 0, 0, 0, 0]
req12    = [[6,0,0], [6,64,0],[6,128,0],[6,192,0],[7,0,0], [7,64,0],[7,128,0],[7,192,0]]
req10    = [[1,128,0], [1,144,0],[1,160,0],[1,176,0],[1,192,0], [1,208,0],[1,224,0],[1,240,0]]

DIO_DIR  = 0x00 # I/O direction register, 0=output, 1=input
DIO_POL  = 0x02 # polarity invert config, if 1 inverted logic for pin
DIO_GPPU = 0x0C # pullup configuration register, 1 enables pullup for pin
DIO_GPIO = 0x12 # pin values
DIO_OLAT = 0x14 # output latch

dio_lock = threading.Lock()   # with dio_lock: to make any method thread safe # ALAN

pi       = pigpio.pi()
spi      = spidev.SpiDev()

spi.open(0,0)              # open(port,CS) port=0, CS=0|1
spi.max_speed_hz=1953000   # default speed undervalued divided voltages
spi.xfer([0x40,0xff,0xff]) # sets all pins as inputs
spi.close()

#
# Initialize library - for RoboPi compatibility, ignores arguments
#

def RoboPiInit(device, bps):
  return 0

#
# Shut down library
#

def RoboPiExit():
  pi.stop()

#
# return the current mode of digital pins (0..23)
#
# 0..7  are Raspberry Pi pins on the Servo header (PWM, SERVO, INPUT, OUTPUT, PING)
# 8..23 are DIO A0-A7 and B0-B7 respectively (INPUT, OUTPUT)
#

def readMode(pin):
  if pin < 0:
    return -1
  if pin < 8:
    return mode[pin]
  if pin < 24:
    return diomode[pin-8]
  return -1

#
# set the current mode of digital pins (0..23)
#
# 0..7  are Raspberry Pi pins on the Servo header (PWM, SERVO, INPUT, OUTPUT, PING)
# 8..23 are DIO A0-A7 and B0-B7 respectively (INPUT, OUTPUT)
#

def _echo1(gpio, level, tick):
   global _high
   _high = tick
      
def _echo0(gpio, level, tick):
   global _done, _high, _time
   _time = tick - _high
   _done = True
               
def pinMode(pin,newmode):
  global my_echo0, my_echo1
  if pin < 0:
    return -1
  if pin < 8: # only accept valid modes
    if mode[pin] == PWM: # stop PWM if setting for input or output
       pi.set_PWM_dutycycle(servopin[pin],0)
    if mode[pin] == TRIG:
       mode[pin] = newmode
    if mode[pin] == ECHO:
       mode[pin] = newmode
       my_echo1.cancel()
       my_echo0.cancel()
    if newmode == INPUT:
       mode[pin] = newmode
       pi.set_mode(servopin[pin], pigpio.INPUT)
    if newmode == OUTPUT:
       mode[pin] = newmode
       pi.set_mode(servopin[pin], pigpio.OUTPUT)
    if newmode == PWM:
       mode[pin] = newmode
       pi.set_PWM_frequency(servopin[pin],490)
    if newmode == PING:
       mode[pin] = newmode
    if newmode == TRIG:
       mode[pin] = newmode
    if newmode == ECHO:
       mode[pin] = newmode
       my_echo1 = pi.callback(22, pigpio.RISING_EDGE,  _echo1)
       my_echo0 = pi.callback(22, pigpio.FALLING_EDGE, _echo0)
    return 0
  if pin < 24: # only accept valid modes 
    if newmode == INPUT:
       diomode[pin-8] = newmode
       setDioBit(0,pin-8)
       return 0
    if newmode == OUTPUT:
       diomode[pin-8] = newmode
       clearDioBit(0,pin-8)
       return 0
  return -1
  
#
# Read current value of digital pin (0..23)
#
# 0..7  are Raspberry Pi pins on the Servo header (PWM, SERVO, INPUT, OUTPUT, PING)
# 8..23 are DIO A0-A7 and B0-B7 respectively (INPUT, OUTPUT)
#

def digitalRead(pin):
  if pin < 0:
    return -1
  if pin < 8:
    return pi.read(servopin[pin])
  if pin < 24:
    return getDioBit(DIO_GPIO,pin-8)
  return -1
  
#
# Set the value of digital pins (0..23)
#
# 0..7  are Raspberry Pi pins on the Servo header (PWM, SERVO, INPUT, OUTPUT, PING)
# 8..23 are DIO A0-A7 and B0-B7 respectively (INPUT, OUTPUT)
#

def digitalWrite(pin,val):
  if pin < 0:
    return -1
  if pin < 8:
    return pi.write(servopin[pin],val)
  if pin < 24:
    if val:   
      return setDioBit(DIO_OLAT,pin-8)
    else:
      return clearDioBit(DIO_OLAT,pin-8)
  return -1
  
def digitalWrite(pin,val):
  if pin < 0:
    return -1
  if pin < 8:
    return pi.write(servopin[pin],val)
  if pin < 24:
    if val:   
      return setDioBit(DIO_OLAT,pin-8)
    else:
      return clearDioBit(DIO_OLAT,pin-8)
  return -1
  
#
# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
#

def analogRead(adcnum):
 with dio_lock:
  if adcnum < 0:
    return -1   
  if adcnum > 7:
    return -1   
  spi.open(0,1) 
  r = spi.xfer2(req10[adcnum])
  spi.close()
  adcout = ((r[1]&3) << 8) + r[2]
  return adcout
    
#
# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
#
# shift left two bits to present 10 bit result as 12 bit result
# for RoboPi compatibility
#

def analogReadRaw(adcnum):
  return analogRead(adcnum)<<2
  
#
# Set motor speed 0..255 for specified pin (mode must be PWM)
#
# default to 490Hz like Arduino
#

def analogWrite(pin,val):
  if pin < 0:
    return -1
  if pin > 7:
    return -1
  pi.set_PWM_dutycycle(servopin[pin],val)
  return 0
  
#
# Get servo position last written to specified pin
#
  
def servoRead(pin):
  if pin < 0:
    return -1
  if pin > 7:
    return -1
  return servoval[pin]

#
# Set servo position for specified pin (mode must be SERVO)
#
  
def servoWrite(pin,val):
  if pin < 0:
    return -1
  if pin > 7:
    return -1
  servoval[pin] = val
  pi.set_servo_pulsewidth(servopin[pin], val)
  return 0
  
#
# readDistance() is not working properly yet - look for it in future releases
#

def readDistance(pin):
  if pin < 0:
    return -1
  if pin > 7:
    return -1
  if mode[pin] == PING:
    pi.set_mode(servopin[pin], pi.OUTPUT)
    pi.gpio_trigger(servopin[pin], 50, 1) # 50us, high pulse
    pi.set_mode(servopin[pin], pi.INPUT)
    ok = pi.wait_for_edge(servopin[pin], pigpio.RISING_EDGE, 0.01)
    if ok:
      t0 = pi.get_current_tick()
      ok = pi.wait_for_edge(servopin[pin], pigpio.FALLING_EDGE, 0.1)
      if ok:
        t1 = pi.get_current_tick()
        return (t1 - t0) # returns microseconds     
  return -1
  
#
# readDistance2() for HC-SR04 only
#
# for an explanation, see my article:
#
# http://www.mikronauts.com/raspberry-pi/gpio-experiments/raspberry-pi-and-hc-sr04-distance-sensor-interfacing-with-c-and-python/
#

def readDistance2(_trig, _echo):
   global pi, _done, _time
   _done = False
   pi.set_mode(servopin[_trig], pigpio.OUTPUT)
   pi.gpio_trigger(servopin[_trig],50,1)
   pi.set_mode(servopin[_echo], pigpio.INPUT)
   time.sleep(0.0001)
   tim = 0
   while not _done:
      time.sleep(0.001)
      tim = tim+1
      if tim > 50:
         return 0
   return _time / 58.068 # 29.034 # return as mm


#
# The following functions are not meant for public use, and may not be supported
# in the future. If you use them, do not be surprised if future versions of the
# library do not make them available.
#
# The following functions are NOT available on RoboPi
#

#
# Read specified MCP23S17 register (16 bits)
#

def readDio(reg):
 with dio_lock:
  spi.open(0,0)  
  r = spi.xfer([0x41,reg,0,0])
  spi.close()
  return (r[3]<<8)+r[2]
  
#
# Write to specified MCP23S17 register (16 bits)
#

def writeDio(reg,val):
  with dio_lock:
    spi.open(0,0)
    r = spi.xfer([0x40,reg,val&255,val>>8])
    spi.close()
    return r

#
# Set specific bit in specified register
#
    
def setDioBit(reg,bit):
  if bit < 0:
    return -1
  if bit > 15:
    return -1 
  t = readDio(reg)
  t = t | (1<<bit)
  writeDio(reg,t) 
  return 0

#
# Clear specific bit in specified register
#
    
def clearDioBit(reg,bit):
  if bit < 0:
    return -1
  if bit > 15:
    return -1 
  t = readDio(reg)
  t = t & ~(1<<bit)
  writeDio(reg,t)  
  return 0

#
# Flip value of specific bit in specified register
#
    
def flipDioBit(reg,bit):
  if bit < 0:
    return -1
  if bit > 15:
    return -1 
  t = readDio(reg)
  t = t ^ (1<<bit)
  writeDio(reg,t) 
  return 0

#
# Get value of specific bit in specified register
#
    
def getDioBit(reg,bit):
  if bit < 0:
    return -1
  if bit > 15:
    return -1 
  if readDio(reg) & (1<<bit):
    return 1
  return 0  

#
# Print out contents of all the MCP23S17 registers (16 bit)
#
    
def dumpDio():
    print "reg val"
    print "--------"
    for r in range(0,16):
      print "", format(r+r, '02X'), format(readDio(r+r),'04X')


  
  

