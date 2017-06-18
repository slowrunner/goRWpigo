#!/usr/bin/python
#
# encoders.py   ENCODERS BY INTERRUPT 
#

import myPDALib
import myPyLib
import time
import sys
import signal
from datetime import datetime
import threading

debugLevel = 0

# ################# Encoder TEST ###########

# Left Encoder - DIO B4 - "PDALib.pin 20"
# Right Encoder- DIO B3 - "PDALib.pin 19" 

LeftEncPin = 20
RightEncPin = 19

InterruptPin = 4 # ServoPin5 1..8  PDALib pin 4 GPIO22

LeftEncDioBit = LeftEncPin - 8
RightEncDioBit = RightEncPin - 8

InchesPerCount = 0.239   # inches per encoder count

_leftEncState=0
_rightEncState=0
_leftEncCount=0
_rightEncCount=0
_bias=0              # going forward, is cw going backward
_lastLeftEncState=0
_lastRightEncState=0

_interruptState = 1  # Active-low
INTERRUPT_ACTIVE = 0

def init():
  # Set up DIO channels as input
  PDALib.pinMode(LeftEncPin,PDALib.INPUT)
  PDALib.pinMode(RightEncPin,PDALib.INPUT)

  # try with pullups  no effect
  #PDALib.setDioBit( PDALib.DIO_GPPU, LeftEncDioBit ) 
  #PDALib.setDioBit( PDALib.DIO_GPPU, RightEncDioBit )
  

  # Interrupt PortB (MCP23S17 pin19) is wired to PDALib pin 4  Servo pin 5 (1..8)
  # set as an input to Raspberry Pi - can also read it with interruptState()
  PDALib.pinMode(InterruptPin,PDALib.INPUT)

  reset()

def enable_encoder_interrupts():

    # encoders->MCP23S17 PortB->PortB interrupt->Pi GPIO (pigpiod)
    # pigpiod calls encoders.callback() when interrupt (active-low)
    # encoders.callback() uses myPDALib.readAndResetInterrupt to read encoders
    # 

    # Set MCP23S17 interrupt-on-on change for each encoder
    # to compare against prior value (not use DEFVAL register)
    #
    PDALib.clearDioBit( myPDALib.DIO_INTCON, LeftEncDioBit ) 
    PDALib.clearDioBit( myPDALib.DIO_INTCON, RightEncDioBit ) 

    # Set MCP23S17 interrupt pin active-low polarity (default, but make sure)
    if (debugLevel > 0): print "before set: DIO_IOCON 0x%x" % PDALib.readDio(myPDALib.DIO_IOCON)
    PDALib.clearDioBit( myPDALib.DIO_IOCON, myPDALib.DIO_INTPOLbit ) 
    if (debugLevel > 0): print "after set: DIO_IOCON 0x%x" % PDALib.readDio(myPDALib.DIO_IOCON)


    reset()
  
    # tell pigpio daemon the encoder interrupt handler method
    myPDALib.setCallback(InterruptPin,myPDALib.FALLING_EDGE,callback)

    # Enable MCP23S17 interrupt for each encoder
    PDALib.setDioBit( myPDALib.DIO_INTEN, LeftEncDioBit ) 
    PDALib.setDioBit( myPDALib.DIO_INTEN, RightEncDioBit ) 



def disable_encoder_interrupts():
    # Disable MCP23S17 interrupt for each encoder
    if (debugLevel > 0): print "disable_encoder_interrupts called"
    PDALib.clearDioBit( myPDALib.DIO_INTEN, LeftEncDioBit ) 
    PDALib.clearDioBit( myPDALib.DIO_INTEN, RightEncDioBit ) 
    
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
    if reentry>1: print "!!!!!!!!!!! callback reentry !!!!!!!!!!!!!!"
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





def reset():
    global _leftEncState,  _lastLeftEncState,  _leftEncCount
    global _rightEncState, _lastRightEncState, _rightEncCount
    global _bias

    if (debugLevel > 0): print "encoders.reset() called"
    _leftEncState=PDALib.digitalRead(LeftEncPin)
    _rightEncState=PDALib.digitalRead(RightEncPin)

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



        
# ### READ ENCODERS  - manual read encoders (polled, not used with interrupts)
def readEncoders():
    global _leftEncState,  _lastLeftEncState,  _leftEncCount
    global _rightEncState, _lastRightEncState, _rightEncCount
    global _bias

    _leftEncState=PDALib.digitalRead(LeftEncPin)
    _rightEncState=PDALib.digitalRead(RightEncPin)

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
  return _leftEncCount

def rightCount():
  global _rightEncCount
  return _rightEncCount

def bias():
  global _bias
  return _bias

def interruptState():
  global _interruptState
  _interruptState = PDALib.digitalRead(InterruptPin)
  return _interruptState
  
def printStatus():
    print "encoder status:"
    print ("    left.state: %d count: %d | right: %d count: %d | bias: %d" % (
               leftState(), leftCount(), rightState(), rightCount(), bias() ))
    if (interruptState() == INTERRUPT_ACTIVE):  print "interruptState: ACTIVE"
    else: print "    interruptState: NOT ACTIVE"


# ##### MAIN ######
def main():

  myPyLib.set_cntl_c_handler(cancel)  # Set CNTL-C handler 
  initialLeftCount = 0  # place to store the counter value when starting motion
  initialRightCount = 0
  initialMeanCount = 0

  # ### encoder methods

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


  try:
    init()
    enable_encoder_interrupts()
    setInitialCounts()
    
    # Loop displaying encoder values
    while True:
        print "\n"
        printStatus()
        time.sleep(0.5)
        print "Distance Traveled: %.1f inches" % distanceTraveled()
        
        #if (interruptState() == 0) : 
            #print "\nmain: ***** INTERRUPT ACTIVE LOW"

            #print ("main: intOnPin(): 0x%x"% intOnPin)
            #callback()

            #print "\n"
            #printStatus()
     
            
    #end while
  except SystemExit:  
      myPDALib.PiExit()
      print "encoders Test Main shutting down"


if __name__ == "__main__":
    main()

