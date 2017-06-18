#!/usr/bin/python
#
# encoders.py   ENCODERS BY INTERRUPT 
# 
# Each encoder count is 0.239 inches in either direction
#
# methods:
#    enable_encoder_interrupts()  # sets up callback 
#    disable_encoder_interrupts() # disable callback and interrupts
#    cancel()                     # convenience func to disable encoder interrupts
#    reset()                      # reset all encoder counts to 0 and resets interrupt state
#    distanceTraveled()           # ditance since last reset() or setInitialCounts()
#    readEncoders()               # manual read - interrupts should be disabled
#    leftCount()                  # getter 
#    rightCount()                 # getter 
#    leftState()                  # getter 0/1
#    rightState()                 # getter 0/1
#    bias()                       # getter diff (right - left), positive is ccw_fwd, cw_bwd
#    printStatus()                # state, count, bias

# internal methods:   
#    init()                       # initialize encoder and interrupt pin
#    callback()                   # called when either encoder changes state
#    interruptState()             # direct read of interrupt pin
#    setInitialCounts()           # sets distance counters to current counters (no reset)

import myPDALib
import myPyLib
import time
import sys
import signal
from datetime import datetime
import threading

debugLevel = 0

# ################# Encoder ###########
#
# Left Encoder - DIO B4 - "myPDALib.pin 20"
# Right Encoder- DIO B3 - "myPDALib.pin 19" 
# Encoder Int  - ServoPin5 of 1..8  myPDALib pin 4 GPIO22
# 

LeftEncPin = 20
RightEncPin = 19
InterruptPin = 4 

LeftEncDioBit = LeftEncPin - 8
RightEncDioBit = RightEncPin - 8

InchesPerCount = 0.239   # inches per encoder count
CmPerCount = InchesPerCount * 2.54

_leftEncState=0
_rightEncState=0
_leftEncCount=0
_rightEncCount=0
_bias=0              # positive is ccw going forward, is cw going backward
_lastLeftEncState=0
_lastRightEncState=0
_interruptState = 1  # Active-low
INTERRUPT_ACTIVE = 0

initialLeftCount = 0  # place to store the counter value when starting motion
initialRightCount = 0
initialMeanCount = 0
initialized = False

def init():         # initial encoder pins and interrupt pin
  if (debugLevel):  print "encoders.init: called"
  # Set up DIO channels as input
  myPDALib.pinMode(LeftEncPin,myPDALib.INPUT)
  myPDALib.pinMode(RightEncPin,myPDALib.INPUT)

  # try with pullups  no effect
  #myPDALib.setDioBit( myPDALib.DIO_GPPU, LeftEncDioBit ) 
  #myPDALib.setDioBit( myPDALib.DIO_GPPU, RightEncDioBit )
  

  # Interrupt PortB (MCP23S17 pin19) is wired to myPDALib pin 4  Servo pin 5 (1..8)
  # set as an input to Raspberry Pi - can also read it with interruptState()
  myPDALib.pinMode(InterruptPin,myPDALib.INPUT)

  reset()
  initialized = True

def enable_encoder_interrupts():

    if not initialized: init()

    # encoders->MCP23S17 PortB->PortB interrupt->Pi GPIO (pigpiod)
    # pigpiod calls encoders.callback() when interrupt (active-low)
    # encoders.callback() uses myPDALib.readAndResetInterrupt to read encoders
    # 

    # Set MCP23S17 interrupt-on-on change for each encoder
    # to compare against prior value (not use DEFVAL register)
    #
    myPDALib.clearDioBit( myPDALib.DIO_INTCON, LeftEncDioBit ) 
    myPDALib.clearDioBit( myPDALib.DIO_INTCON, RightEncDioBit ) 

    # Set MCP23S17 interrupt pin active-low polarity (default, but make sure)
    if (debugLevel > 0): print "before set: DIO_IOCON 0x%x" % myPDALib.readDio(myPDALib.DIO_IOCON)
    myPDALib.clearDioBit( myPDALib.DIO_IOCON, myPDALib.DIO_INTPOLbit ) 
    if (debugLevel > 0): print "after set: DIO_IOCON 0x%x" % myPDALib.readDio(myPDALib.DIO_IOCON)


    reset()
  
    # tell pigpio daemon the encoder interrupt handler method
    myPDALib.setCallback(InterruptPin,myPDALib.FALLING_EDGE,callback)

    # Enable MCP23S17 interrupt for each encoder
    myPDALib.setDioBit( myPDALib.DIO_INTEN, LeftEncDioBit ) 
    myPDALib.setDioBit( myPDALib.DIO_INTEN, RightEncDioBit ) 



def disable_encoder_interrupts():
    # Disable MCP23S17 interrupt for each encoder
    if (debugLevel > 0): print "disable_encoder_interrupts called"
    myPDALib.clearDioBit( myPDALib.DIO_INTEN, LeftEncDioBit ) 
    myPDALib.clearDioBit( myPDALib.DIO_INTEN, RightEncDioBit ) 
    
    # disable GPIO / pigpio interrupt callback
    myPDALib.clearCallback()

    

# interrupt callback 
encoderLock = threading.Lock()
numPulsesPerRev = 32
maxSpeedRobot = 8  # inches (or cm - be consistent)
wheelDia = 2.5     # inches (or cm - be consistent)
wheelCircum = 2.5 * 3.14159
revPerSec = maxSpeedRobot / wheelCircum
minPulseWidth = 1.0/(revPerSec * numPulsesPerRev)
debounceWait = minPulseWidth / 8.0  # after int, wait before reading

reentry=0

def callback(gpio=None,level=None,tick=None):
    global _leftEncState,  _lastLeftEncState,  _leftEncCount
    global _rightEncState, _lastRightEncState, _rightEncCount
    global _bias, reentry
  
  # encoders.callback() 
  #    reads state of portB pins at time of interrupt
  #    if 
 
  #with encoderLock:  
    reentry +=1
    if reentry>1: print "!!!!!!!!!!! encoder callback reentry !!!!!!!!!!!!!!"
    if (debugLevel > 0): print "\ncallback: encoders callback",datetime.now()
    while interruptState() == 0:
        # time.sleep(debounceWait)  # delay after interrupt to debounce   
        dioBitsAtInt = myPDALib.readAndResetInterrupt()  # reads state of PortB pins at interrupt 
        if (debugLevel > 0): print ("callback: dioBitsAtInt 0x%x" % dioBitsAtInt)
        _rightEncState = (dioBitsAtInt >> (RightEncDioBit-8)) & 0x01  #  get encoder state at time of interrupt 
        _leftEncState = (dioBitsAtInt >> (LeftEncDioBit-8)) & 0x01
        if (debugLevel > 0): print ("callback: _leftEncState: %d _rightEncState: %d" % (_leftEncState,_rightEncState))

        # if a left stripe transition, increase count, 
        #  
        if (_leftEncState != _lastLeftEncState): 
            _leftEncCount += 1
            if (debugLevel > 0): print ("callback: inc left count: %d" % _leftEncCount)
            _lastLeftEncState  = _leftEncState

       
        # if a right stripe transition, increase count, 
        if (_rightEncState != _lastRightEncState): 
            _rightEncCount += 1
            if (debugLevel > 0): print ("callback: inc right count: %d" % _rightEncCount)
            _lastRightEncState = _rightEncState
   
        _bias = _rightEncCount - _leftEncCount  # maintain bias value
        if (debugLevel > 0): print "callback: ******* end while interrupt loop"
    if (debugLevel > 0): print "callback: end interrupt",datetime.now() 
    reentry -=1
    return 0
    



def cancel():
    if (debugLevel > 0): print "encoders.cancel() called"
    disable_encoder_interrupts()





def reset():         # set left,right encoder counts to zero
    global _leftEncState,  _lastLeftEncState,  _leftEncCount
    global _rightEncState, _lastRightEncState, _rightEncCount
    global _bias

    if (debugLevel > 0): print "encoders.reset() called"
    _leftEncState=myPDALib.digitalRead(LeftEncPin)
    _rightEncState=myPDALib.digitalRead(RightEncPin)

    if (debugLevel > 0): print ("reset(): left %d right %d" % (_leftEncState, _rightEncState))

    _lastLeftEncState  = _leftEncState
    _lastRightEncState = _rightEncState

    _leftEncCount=0
    _rightEncCount=0
    if (debugLevel > 0): print ("reset(): left count %d right count %d" % (_leftEncCount, _rightEncCount))

    _bias=0


    if (interruptState() == 0):
        if (debugLevel > 0): print ("reset(): interruptState %d" % interruptState())
        dioBitsAtInt = myPDALib.readAndResetInterrupt()  # reads state of PortB pins at interrupt 
        time.sleep(0.01)
        dioBitsAtInt = myPDALib.readAndResetInterrupt()  # reads state of PortB pins at interrupt 
        time.sleep(0.01)
        if (debugLevel > 0): print ("reset(): interruptState %d" % interruptState())

    setInitialCounts()      # reset distance traveled counters


        
# ### READ ENCODERS  - manual read encoders (polled, not used with interrupts)
def readEncoders():
    global _leftEncState,  _lastLeftEncState,  _leftEncCount
    global _rightEncState, _lastRightEncState, _rightEncCount
    global _bias

    _leftEncState=myPDALib.digitalRead(LeftEncPin)
    _rightEncState=myPDALib.digitalRead(RightEncPin)

    if (_leftEncState != _lastLeftEncState):
        if (debugLevel > 0): print ("readEncoders: before inc left count: %d" % _leftEncCount)
        _leftEncCount+=1
        if (debugLevel > 0): print ("readEncoders: after inc left count: %d" % _leftEncCount)
        _lastLeftEncState=_leftEncState

    if (_rightEncState != _lastRightEncState):
        if (debugLevel > 0): print ("readEncoders: before inc right count: %d" % _rightEncCount)
        _rightEncCount+=1
        if (debugLevel > 0): print ("readEncoders: after inc right count: %d" % _rightEncCount)
        _lastRightEncState=_rightEncState

    _bias= _rightEncCount - _leftEncCount


def leftState():
  global _leftEncState
  return _leftEncState

def rightState():
  global _rightEncState
  return _rightEncState

def leftCount():
  global _leftEncCount
  if (debugLevel): print "encoders.leftCount: returning _leftEncCount=%d" % _leftEncCount
  return _leftEncCount

def rightCount():
  global _rightEncCount
  if (debugLevel): print "encoders.rightCount: returning _rightEncCount=%d" % _rightEncCount
  return _rightEncCount

def bias():
  global _bias
  return _bias

def interruptState():
  global _interruptState
  _interruptState = myPDALib.digitalRead(InterruptPin)
  return _interruptState
  
def printStatus():
    print "encoder status:"
    print ("    left.state: %d count: %d | right: %d count: %d | bias: %d" % (
               leftState(), leftCount(), rightState(), rightCount(), bias() ))
    if (interruptState() == INTERRUPT_ACTIVE):  print "interruptState: ACTIVE"
    else: print "    interruptState: NOT ACTIVE"


def setInitialCounts():
    initialLeftCount=leftCount()
    initialRightCount=rightCount()
    initialMeanCount=(initialLeftCount+initialRightCount)/2.0 

def distanceTraveled():
    currentLeftCount = leftCount()
    currentRightCount = rightCount()
    currentMeanCount = ( currentLeftCount + currentRightCount) / 2.0
    countsTraveled = (currentMeanCount - initialMeanCount)
    distance=countsTraveled * InchesPerCount
    return distance

dummy=init()        # initialize encoder pins when module is imported

# ##### MAIN ######
def main():
  print "encoders.main:  Encoders.py main() Started"
  sleepTime=10
  myPyLib.set_cntl_c_handler(cancel)  # Set CNTL-C handler 

  try:
    enable_encoder_interrupts()
    
    # Loop displaying encoder values
    while True:
        print "\nencoders.main: encoders reset()"
        reset()
        print "encoders.main: sleep(%d)" % sleepTime
        time.sleep(sleepTime)
        printStatus()
        print "    Distance Traveled: %.1f inches" % distanceTraveled()
        
  except SystemExit:  
      myPDALib.PiExit()
      print "encoders Test Main shutting down"


if __name__ == "__main__":
    main()

