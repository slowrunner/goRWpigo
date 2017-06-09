#!/usr/bin/python
#
# File: spidev.py   
# Doc:        simulated spidev module 
#             
#

debugLevel = 0

class SpiDev():
    def __init__(self):
        if (debugLevel): print "spidev:SpiDev class init"    

    def open(self,port,chipselect):
        if (debugLevel): print "spidev:open(%d,%d) called" % (port,chipselect)
        
    def xfer(self,args):
        if (debugLevel): print "spidev:xfer(%s) called" % ' '.join(str(e) for e in args)
        return [0,0,0,0]
        
    def close(self):    
        if (debugLevel): print "spidev:close called"    
