#!/usr/bin/env python
# coding: latin-1
#requirements
#sudo apt-get install python-visual

from visual import *

#Time for sleep function()
import time

# Load the XLoBorg library
import XLoBorg

# Tell the library to disable diagnostic printouts
XLoBorg.printFunction = XLoBorg.NoPrint

# Start the XLoBorg module (sets up devices)
XLoBorg.Init()

compass = arrow(pos=(0,0,0), axis=(0,0,0), shaftwidth=1, color=(1,0,0))
boat=arrow(pos=(0,0,0), axis=(0,0,0), shaftwidth=1, color=(0,0,1))
while True:
    time.sleep(1)
    # Read and display the raw accelerometer readings
    print 'X = %+01.4f G, Y = %+01.4f G, Z = %+01.4f G' % XLoBorg.ReadAccelerometer()
    u,w,v=XLoBorg.ReadAccelerometer()
    u *= 1000.0
    v *= 1000.0
    w *= 1000.0
    # Read and display the raw magnetometer readings
    print 'mX = %+06d, mY = %+06d, mZ = %+06d' % XLoBorg.ReadCompassRaw()
    x,z,y=XLoBorg.ReadCompassRaw()
    compass.visible=False
    compass = arrow(pos=(0,0,0), axis=(x,y,z), shaftwidth=1, color=(1,0,0))
    boat.visible=False
    boat=arrow(pos=(0,0,0), axis=(u,v,w), shaftwidth=1, color=(0,0,1))
