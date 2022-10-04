#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
from __future__ import print_function
import os
from gps import *
from time import *
import time
import threading
import math as M

def bearing(lat1, lat2, lon1, lon2):
    """
    http://stackoverflow.com/questions/17624310/geopy-calculating-gps-heading-bearing
    """
    dLon = lon2 - lon1
    y = M.sin(dLon) * M.cos(lat2)
    x = M.cos(lat1)*M.sin(lat2)-M.sin(lat1)*M.cos(lat2)*M.cos(dLon)
    brng = M.degrees(M.atan2(y, x))
    if (brng < 0):
        brng += 360
    return brng


f=open('/home/pi/xloborg/gpsdata.txt','w') 
gpsd = None #setting the global variable

os.system("'sudo killall gpsd; sleep(2)'") 
os.system('sudo gpsd /dev/ttyACM0 -F /var/run/gpsd.sock') 
 
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
 
if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread
  gpsp.start() # start it up
  count = 0
  while (count<6000):
    if (count==0):
      print(gpsd.fix.latitude,',',gpsd.fix.longitude,',',gpsd.fix.speed,',0.0', file=f)
      lat1=gpsd.fix.latitude
      lon1=gpsd.fix.longitude
    else:
      lat2=gpsd.fix.latitude
      lon2=gpsd.fix.longitude
      print(gpsd.fix.latitude,',',gpsd.fix.longitude,',',gpsd.fix.speed,',',bearing(lat1,lat2,lon1,lon2), file=f)
      lat1=lat2
      lon1=lon2
    time.sleep(1) #set to whatever
    count = count + 1
  f.close()
  gpsp.running = False
  os.system("'sudo killall gpsd; sleep(2)'") 
  gpsp.join() # wait for the thread to finish what it's doing
