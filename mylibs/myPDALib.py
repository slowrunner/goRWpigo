#
# myPDALib.py   SUPPLIMENTAL Pi Droid Alpha FUNCTIONS
#
# v0.1	14June2016  LibExit()

import PDALib
import myPyLib
import pigpio

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
    # call to see what PDALib pin caused the interrupt
    interruptPins = PDALib.readDio(DIO_INTF)
    # print ("interruptOnPin(): 0x%x" % interruptPins)
    # convert from register of bits to PDALib pin
    interruptPin = interruptPins  # for the time being
    return interruptPin

# READ AND RESET INTERRUPT
# return state of PortB at time of interrupt
# reading state also resets interrupt state
 
def readAndResetInterrupt():  
    # read the MCP23S17 combined dio bank, resets interrupt

    #dioBitsAtInt = readDio2(DIO_INTCAPB)   # only reads PortB
    #return dioBitsAtInt                    # return portB bits

    dioBitsAtInt = PDALib.readDio(DIO_INTCAP)  # reads both PortA and PortB
    return dioBitsAtInt>>8    # only send Port B

# overload for RoboPiExit()
def PiExit():
  PDALib.pi.stop()
  print "myPDALib.PiExit():  PDALib.pi.stop() called"

#
# Read specified MCP23S17 register (8 bits)
# 

def readDio2(reg):
    with PDALib.dio_lock:
        PDALib.spi.open(0,0)  
        r = PDALib.spi.xfer2([0x41,reg,0])
        PDALib.spi.close()
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
    hPi_Callback = PDALib.pi.callback(PDALib.servopin[pin], edge, fCallback)

# CLEAR CALLBACK  removes pigpio callback on a GPIOnn pin
def clearCallback():
    global hPi_Callback
    hPi_Callback.cancel()


#
# read SPI data from MCP3208 chip, 8 possible adc's (0 thru 7)
#

def analogRead12bit(adcnum):
 with PDALib.dio_lock:
  if adcnum < 0:
    return -1   
  if adcnum > 7:
    return -1   
  PDALib.spi.open(0,1) 
  r = PDALib.spi.xfer2(PDALib.req12[adcnum])
  PDALib.spi.close()
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
  if ( (adcPin <0) | (adcPin>7) ):
    return -1
  # obsolete 10bit
  # return (VperBit * PDALib.analogRead(adcPin))

  return (VperBit * analogRead12bit(adcPin))
  

