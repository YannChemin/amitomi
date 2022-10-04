# amitomi

The original website is stil up (https://sites.google.com/site/amitomiautoboat/home)

## Introduction

This is the home of a hobby project: amitomi, an "unmanned" RC sailing boat.

The name was given by Ceylian (my son) for "Ami" (French for "friend") and Eva (my daughter) for "Tommy" like the name. By concatenating the names "amitomi" was born on September 8th, 2013.

Unmanned here refers to the fact that no remote control is used, the boat is autonomous.

It uses a phantom RC boat from hobby king:
http://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=25147
that I got for ~90 USD (not including shipment)

As you can see on the picture, it also has a RaspberryPI (RPI) on it (not yet made a design plan to set it up inside the hull...)

2013: RaspberryPIs can be purchased online from many places, actually I bought from 3 online shops, all correct in delivery time (25 USD or 30 Euros depending from where you order). In 2015, Raspberry PI v.2 are available for a similar price.

you can essentially see on the picture a rainbow coloured box from http://www.pibow.com, the power supply is not the final one, it is a minigorilla from an airport duty-free shop (~70 USD), the final one most probably will come from the MotorPiTX bundle (see below).

Additionally from this:

    Xloborg extension for RPI from http://www.piborg.org/xloborg for 3D Accelerometer and 3D Compass (< 10 GBP).
    2013: GPS with 2m usb cable from http://www.semsons.com/covusbgpsre6.html, sells 33 USD today, but ~45 USD when I bought it.
    2015: Adafruit GPS Hat (~44 USD)
    2013: MotorPiTX (2014 delivered) from http://www.kickstarter.com/projects/1436996303/motorpitx-motor-board-and-power-supply-for-a-raspb for sails/rudder servo control
    2015: Adafruit Stepper Motor Hat (~ 30 USD)

The boat should sail alone:

    A skipper software for sailing boat, to be coded yet.
    A waypoint sorter, to sail the survey downwind (safer, less turns)

Practical use of such a boat:

    Phase 1: temperature monitoring of a lake/tank/reservoir (below boat, water surface, top of mast) for evaporation modeling
    Phase 2: (if I can find a way to build it) bathymetry mapping with echo-sounding (very light) technology

Since the main interest is geographical survey, I will build a close link to GRASS-GIS (http://grass.osgeo.org/) once data starts to be acquired (Real-Time Processing) in the boat. If I can manage a live internet link, I will set up a PyWPS (http://pywps.wald.intevation.org/) web service connection to process the data live on a WebGIS server using GRASS-GIS too.

I recently found this, while visiting IFREMER (Brest, France):

https://www.ensta-bretagne.fr/jaulin/vaimos.html


## Attitude Sensing
This page is essentially about the Xloborg from http://www.piborg.org/xloborg
and how the boat uses it both for bearing estimation and estimating sails tension to keep within optimum navigation roll.

The python-visual output from 3D compass (red) and 3D Accelerometer (Blue) when the boat is rolling mast to the right (up picture) and to the left (down picture) with data received on the right side console.

~~~Python
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
    
~~~

## Positioning
The platform's location is tracked by a GPS with 2m usb cable from http://www.semsons.com/covusbgpsre6.html, sells 33USD today (20130908), but ~45USD when I bought it. It works "out of the box" with the RaspberryPI.

I have modified a code from Dan Mandle (http://dan.mandle.me) to visualize the gps data every time step, though xgps (http://gpsd.berlios.de/xgps-sample.html) also work very well through the ssh -X command (see right side picture). You need to get the /dev/ttyXXXX name by parsing "dmesg" for it, a simple grep does the job, in this RPI, /dev/ttyACM0 it is.

~~~~Python
#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
from __future__ import print_function
import os
from gps import *
from time import *
import time
import threading
gpsd = None #setting the global variable

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
  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      print(' GPS reading')
      print('----------------------------------------')
      print('latitude    ' , gpsd.fix.latitude)
      print('longitude   ' , gpsd.fix.longitude)
      print('time utc    ' , gpsd.utc,' + ', gpsd.fix.time)
      print('altitude (m)' , gpsd.fix.altitude)
      print('eps         ' , gpsd.fix.eps)
      print('epx         ' , gpsd.fix.epx)
      print('epv         ' , gpsd.fix.epv)
      print('ept         ' , gpsd.fix.ept)
      print('speed (m/s) ' , gpsd.fix.speed)
      print('climb       ' , gpsd.fix.climb)
      print('track       ' , gpsd.fix.track)
      print('mode        ' , gpsd.fix.mode)
      print('sats        ' , gpsd.satellites)
      time.sleep(1) #set to whatever

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("\nKilling Thread...")
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print("Done.\nExiting.
~~~~

##  Bearing
Bearing is an addition to Positioning:
~~~~Python
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
~~~~

## Skipper

This page is about the skipper software that will define navigational strategies and servo control & feed back for bearing adjustment and boat stabilization.


##  SkipperLibrary
The library is meant to hold functions for the skipper main code to be readable.
	
~~~~Python
def bbox(waypoints):
    #Step 1: create a bounding box for the waypoints coordinates
 
def bbox_center(bnd):
    #Step 2: define the central point of the bounding box
 
def bbox_half_diag_length(bnd,bnd_center):
    #Step 3: define the half diagonale length
 
def upwind_virtual_waypoint(upwinddir,bnd_center,radius):
    #step 4: Define upwind virtual waypoint with radius distance from bbox center
 
def upwind_waypoint_find(waypoints,upw):
    #step 5: find most upwind waypoint by distance to virtual upwind waypoint
 
def waypoint_dist_to_firstwp(waypoints,first_waypoint):
    #step 6: waypoints distance to first waypoint
 
def distangle(waypoints,upwinddir,upw):
    #step 7: Create matrices of distance and angle from wind
 
#Boat should start sailing to first waypoint from here...

def route(waypoints, distM, angleM, first_waypoint):
    #step 8: Create a downwind route through the waypoints by ordering them

#End of routing steps, the final route has been computed and stored on disk


#Navigational elements
def getbearing():
    # Read and display the raw magnetometer readings

def makeactualpos(last_bearing, last_true_gps_pos, last_kalman_pos, last_time, last_velocity, last_attitude):


#Still Working on those...
def navigate(mode,waypoint):    
    

def survey(mode,waypoint):
    #set course to that waypoint
~~~~

## Stepper motors

Since the beginning of this project, essentially September 2013, and after my accident in 2014, a long time passed, and a new Raspberry PI (V.2) arrived, I decided to revamp the whole design by using stepper motors instead of energy hungry servos. 

Along with that the GPS is now an Adafruit Hat and the stepper motor control too.

Only remained the faithful XLoborg from the previous incarnation.

The whole thing is now very tight and clean. Once I will be through the stepper motors preparation, I will change the order of the Hats so that the GPS is coming up above to catch a better signal.

I choose two steppers I already had previously, so no brainer for this, a small one (below) for the rudder and a bigger one for the sails already attached for testing.

##  Waypoint sorting
The experimental assumption is that the boat will be given a basic set of waypoints to pass by during the survey. This is a fair assumption even if it is to limit the safe water boundaries of the survey. From the point of release onward, the boat will have to think about the best route to take.

Waypoint sorting becomes important here, as the least energy consumption will be sailing side wind around 80-120 degrees from the wind. going up upwind may require to turn the boat from one side to another many times to reach upwind location, this is hazardous and energy hungry.

The strategy here is to find the most upwind waypoint ("first_waypoint") given and sail to it from the point of release, it may take risk and energy, but once passed, it is only a downwind ride going through all of the remaining waypoints.

### Sort the waypoints by most upwind first

    Step 1: create a bounding box for the waypoints coordinates
    Step 2: define the central point of the bounding box
    Step 3: define the half diagonal length
    Step 4: define upwind virtual waypoint with radius distance from bbox center
    Step 5: find most upwind waypoint by distance to virtual upwind waypoint

### At this point the boat can start travelling to the first waypoint
### Meanwhile the sorting of the remaining waypoints can happen

Waypoint sorting will involve starting from first_waypoint and locate the next upper wind waypoint to set course to, holding a check on number of turns needed to solve the required path from the waypoints set. 

### Set Route through next waypoints

    Step 6: create matrices of distance and angle from wind
    Step 7: Route waypoints with openopt tsp() algorithm

Found http://openopt.org with a python implementation of a traveler salesman problem optimizer. OK, it sounds overkill for 4 waypoints, but on a larger lake it maybe a more complex pattern of waypoints required to be visited regularly... Constraints can be distance and angle of vector from wind direction.



A simple route tracing indeed, the routing algorithm found the same order as the original waypoint listing... Following least distance and constrained with most right side normal to upwind navigation.

~~~~Python
#!/usr/bin/env python
import numpy as np
#Navigation test engine
#Loads fake datasets and runs the skipper through the data

releasepoint = np.genfromtxt('releasepoint001.csv',delimiter=',')
waypoints = np.genfromtxt('waypoints001.csv',delimiter=',')
upwinddir = np.genfromtxt('upwinddir001.csv',delimiter=',')
print 'releasepoint = ', releasepoint
print 'waypoints = w', waypoints
print 'upwinddir = ', upwinddir, '[0.0=North]'

#set training velocity at 1m/s (motorbike dataset)
v=1.0

#Sort the waypoints by most upwind first
#Step 1: create a bounding box for the waypoints coordinates
def bbox(waypoints):
    #First Column is latitude
    bbox_north=waypoints[:,0].max()
    bbox_south=waypoints[:,0].min()
    bbox_east=waypoints[:,1].max()
    bbox_west=waypoints[:,1].min()
    print 'BOUNDING BOX'
    print '\t',bbox_north
    print bbox_west,bbox_east
    print '\t',bbox_south
    return(bbox_north,bbox_south,bbox_east,bbox_west)

bnd = bbox(waypoints)

#Step 2: define the central point of the bounding box
def bbox_center(bnd):
    lat = bnd[1] + (bnd[0]-bnd[1])/2.0
    lon = bnd[3] + (bnd[2]-bnd[3])/2.0
    return(lat,lon)

bnd_center = bbox_center(bnd)

#Step 3: define the half diagonal length
def bbox_half_diag_length(bnd,bnd_center):
    #load upper right coordinates
    ur_lat = bnd[0]
    ur_lon = bnd[2]
    #"Euclidian" distance
    d = np.sqrt(pow((ur_lat-bnd_center[0]),2)+pow((ur_lon-bnd_center[1]),2))
    #return radius of bearing circle
    return(d)

radius = bbox_half_diag_length(bnd,bnd_center)

#step 4: Define upwind virtual waypoint with radius distance from bbox center
def upwind_virtual_waypoint(upwinddir,bnd_center,radius):
    #set the triangle with upwinddir, radius, bnd_center
    #conversion from clock-wise bearing to arithmetic East-bound X axis start of angle
    temp = upwinddir - 90.0
    if(temp<0):
        temp += 360.0
    a = (temp * (-1)) + 360.0
    #compute circle coordinates with bbox center point offset
    lat=bnd_center[0]+radius*np.sin(a)
    lon=bnd_center[1]+radius*np.cos(a)
    return(lat,lon)

upwwp = upwind_virtual_waypoint(upwinddir,bnd_center,radius)

#step 5: find most upwind waypoint by distance to virtual upwind waypoint
def upwind_waypoint_find(waypoints,upw):
    distance = []
    dmin = 10000000
    for i in range (len(waypoints)):
        distance.append(np.sqrt(pow((upw[0]-waypoints[i,0]),2)+pow((upw[1]-waypoints[i,1]),2)))
        if(distance[i]<dmin):
            dmin = distance[i]
            upwind_wp_idx = i

    print upwind_wp_idx
    print distance
    return(waypoints[upwind_wp_idx,0],waypoints[upwind_wp_idx,1])

first_waypoint = upwind_waypoint_find(waypoints,upwwp)

#At this point the boat can start travelling to the first waypoint
import os
#delete previously used waypoints
os.system("rm -f skipper_waypoints.py &")
os.system("python skipper.py "+first_waypoint[0]+" "+first_waypoint[1]+" &")
#Skipper keeps trying to load the waypoint list on course to first_waypoint
#If not ready by first_waypoint, it will failsafe back to release point

#Meanwhile the sorting of the remaining waypoints can happen
print "#step 6: Create matrices of distance and angle from wind"
#step 6: Create matrices of distance and angle from wind
def distangle(waypoints,upwinddir):
        #make matrices of heading <-> upwbearing angles and distances between waypoints
        angle = np.zeros((len(waypoints),len(waypoints)))
        distance = np.zeros((len(waypoints),len(waypoints)))
        for i in range(len(waypoints)):
                for j in range(len(waypoints)):
                        if(i!=j):
                                distance[i,j] = np.sqrt(pow((waypoints[j,0]-waypoints[i,0]),2)+pow((waypoints[j,1]-waypoints[i,1]),2))
                                #make an upwind waypoint from wpt i with a distance (i,j)
                                upw = upwind_virtual_waypoint(upwinddir,waypoints[i,0:2],distance[i,j])
                                #create vector i,upw
                                i_upw = [upw[0]-waypoints[i,0],upw[1]-waypoints[i,1]]
                                #create vector i,j
                                i_j = [waypoints[j,0]-waypoints[i,0],waypoints[j,1]-waypoints[i,1]]
                                #angle of i_j with i_upw
                                angle[i,j] = atan2(i_j[0],i_j[1])-atan2(i_upw[0],i_upw[1])
                                if(angle[i,j]<0):
                                        angle[i,j] += 2*np.pi
                                if(angle[i,j]>360.0):
                                        angle[i,j] -= 2*np.pi
                                angle[i,j] *= (360.0 / (2*np.pi))
                        else:
                                distance[i,j] = np.nan
                                angle[i,j] = np.nan

        return(distance,angle)

distM, angleM = distangle(waypoints,upwinddir)

print "#step 7: Route waypoints"
# 1- the distance of any wpt from virtual upwind wpt (dist2upw)
# 2- the distance of any wapt to any other wpt (distM)
# 3- the angle from wind of any two waypoints vector (angleM)
def route(waypoints, distM, angleM, first_waypoint):
        #sudo apt-get install python-openopt python-networkx
        import openopt as oo
        from numpy import sin, cos
        import networkx as nx

        N=len(waypoints)
        G = nx.Graph()
        G.add_edges_from([(i,j,{'angle': 360.0-angleM[i,j], 'distance': distM[i,j]}) for i in range(N) for j in range(N) if i != j ])
        objfn = lambda values: values['distance'] + 10*values['angle']
        p = oo.TSP(G, objective = objfn, start = first_waypoint[2], returnToStart=False)
        r = p.solve('glpk')
        print("nodes = ", r.nodes)
        print("edges = ", r.edges)
        route_list_lat = np.zeros(N)
        route_list_lon = np.zeros(N)
        route_list_idx = np.zeros(N)
        for i in range(N):
                route_list_lat[i] = waypoints[r.nodes[i]][0]
                route_list_lon[i] = waypoints[r.nodes[i]][1]
                route_list_idx[i] = r.nodes[i]
        return(route_list_lat,route_list_lon,route_list_idx)

rlat,rlon,ridx = route(waypoints, distM, angleM, first_waypoint)

import pylab as pb
pb.plot(releasepoint[1],releasepoint[0],"o",c="red")
pb.plot(rlon,rlat,"o")
for i in range(len(rlon)):
        pb.text(rlon[i],rlat[i],"#"+str(i)+"#"+str(ridx[i]))
pb.show()
~~~~

##  Wind Direction

Barry Thomas (https://www.facebook.com/barry.thomas.9634) very kindly shared his expertise about making small autonomous boats on the subject of making a small wind vane.

He used a "AS5040 Rotary Position Sensor" from http://www.ams.com/eng/Products/Magnetic-Position-Sensors/Rotary-Magnetic-Position-Sensors/AS5040.

Got the sensor (actually 3 samples, thanks AMS) and got around to solder voltage (5V on pin 16, orange cable) and ground (pin 7, black cable). I am now stuck to know the initial state of the device as there are 3 working modes (output mode 3 being the absolute direction mode with output on pin 4, blue cable). It seems that an OTP (One-Time Programmable) register should be programmed through pin 8.

DONE:

    Locate a micro-soldering tool (used by mobile shops in Sri Lanka)

TODO:

    Get capacitors and pcb (thin) pieces
    Get help from Barry when I am ready !
    Create the rotating flag, and get an appropriate small magnet
    Get containers for the flags and the sensor


------
The Aber Sailbot guys (https://www.facebook.com/Abersailbot) also answered my call:
We use a contactless mechanical wind sensor, which sends a pwm signal to the arduino. The raspberry pi requests the current reading of the wind sensor every few seconds over serial. It's based off the sensor described in part III B of this paper: http://cadair.aber.ac.uk/dspace/handle/2160/3103

##  Payload
Payload fitting concept is extended here to "brain" (RPI) + sensors, as we are on very small project.
### Version 1: Temperature Profiler

The payload for this Version 1 is a set of three temperature sensors, one below the boat, one trailing the boat at the surface of water and the last one on top of the mast, inside a radiation shield.


### Version 2: Bathymetric survey

I found this sonar on some youtube (https://www.youtube.com/watch?v=RggOuAy33XM), showing a RC boat with a sonar capability (200USD though...):
https://buy.garmin.com/en-US/digital/prod26510.html

Waterproofing:

This interesting video (https://www.youtube.com/watch?v=yw2nvuEFTKA) that Veronica (https://plus.google.com/u/0/113279748027723193998) sent me about using NeverWet (http://www.neverwet.com/) hydrophobic coating on a RPI, very impressive ! Will try to get some of it to ensure survival of the RPI in the hull...

##  Arduino

I am following a set of evening classes on Arduino, and Lahiru, the instructor, forwarded me about the connectivity of Arduino and PaspberryPI, after digging a bit I found a servo control Python code commanding the Arduino from the Raspberry PI...

http://www.themagpi.com/issue/issue-7/article/raspberry-pi-arduino/

~~~~Shell
sudo apt-get install python-serial subversion

cd ~/dev/
svn co https://github.com/tino/pyFirmata

cd pyFirmata/trunk/
sudo python setup.py install

Example uses tkinter:
sudo apt-get install python-tk
~~~~

Servo command example:
http://scruss.com/blog/2012/10/28/servo-control-from-pyfirmata-arduino/

##  Camera
Got a PiCam along with other stuff today, including some stickers, so the best way to show them was to use the RPI newly fitted camera. Yes, Tux is there too !

Just used the raspistill basic mode...

~~~~Shell
raspistill -fp --output still_example.jpg
~~~~
I am going to set up a 30 seconds loop to record the trip, hopefully with a full exif geojpeg format... However, on this boat it is just to have a nice trip log more than any useful information...

Note: Also learnt in the process that links2 can display pictures without starting X..
~~~~Python
links2 -g still_example.jpg
~~~~
Along the survey I will record short movies, using raspivid, I am still thinking about the method to water proof the camera and locating the whole RPI in the hull close enough... Anyway, the command to be sent to the system at interesting times (reaching waypoints, changing directions, etc.) is:
~~~~Python
raspivid -o $(date +"%Y%j%H%M%S")\_video.h264 -t 10000 -w 600 -h 400
~~~~
the date command is formatting the beginning of the name with YYYYJJJHHMMSS_video.h264
YYYY=2013
JJJ=172 (Julian day)
HHMMSS (hour minute seconds)

To be light on the cpu, the resolution is 600x400, which gives a decent movie, though limited to 10 seconds to be sure other functions are not waiting too long if memory is becoming limited...
 
##  MotorPiTX
Photo on the right: 
MotorPiTX onboard + spare one on the left.

The board is available at:
http://www.boeeerb.co.uk/shop/

The Python library is already up, check it out with svn
svn co https://github.com/Boeeerb/MotorPiTX


Just tried the test for servos, works OK:

~~~~Shell
# Go to the svn trunk directory
cd ~/MotorPiTX/trunk

# Get the Daemon up
sudo ./servod

#Copy motorpitx.py to examples dir
cp motorpitx.py examples/

#go to examples dir
cd examples/

#run servotest.py
sudo python servotest.py
~~~~

I just sent a comment to Jason on how to code a winch-type servo with his board. I am stuck with a 0-180 degrees interval, when the sails cables need a few rotations of the winch to get in. I will also investigate the use of gears to get more rotations out of a 0-180 degrees range.


##  RaspberryPI
The Raspberry PI, seen here without casing, showing the Xloborg extension card on the GPIO.

The OS in the RPI is the rather standard Raspbian (http://www.raspbian.org/) that has few configurations using raspi-config:

    Disabled the graphical interface boot
    Enabled the ssh daemon
    No CPU overclocking (power... though maybe required depending on workload)

RPI does not like to ssh through wifi it seems, so a standard hub does the job.

Once the MotorPiTX extension will be here, I will still be able to plug the xloborg ontop the GPIO, however, I will need to also plug the temperature sensors on the same GPIO... Still an issue to solve, maybe the piborg GPIO triplicator will do it...

Then, there will be queueing issues for GPIO querying access, fortunately sensors need only short time and maybe different pins...


Redundancy dream...

Redundancy, a common necessity in space probes and satellites, two RPI on-board!

    One for survey (default)
    One for emergency (optional, would be nice, but weight will be an issue here)

The reason is the safety of the boat, in case of failure of the first RPI, a second one would kick in and do a single job: sail back to the release point in the most power efficient way. Today, it is just a dream to have this captain-in-second, but the idea is simple:

    One RPI
    One smaller/lighter battery set
    One small GPS (for initial point of release and navigation)
    One Xloborg (for initial upwind direction and secondary bearing)
    One MotorPiTX (for servo control)
    One LED

The sequence will have two steps:

1 - Switch on during pre-release stage (same as default RPI) but with a different switch (no electric connections between the two RPIs), record up wind direction, record best fix of GPS, write them both to file, blink LED to tell it is done, self shutdown.

2 - Emergency switch on if power to default RPI is gone, set an electric device (https://www.sparkfun.com/products/8883) to detect battery failure ("open circuit") of default RPI. On failure, the device switches on the power for the emergency RPI. For the maximum amount of time from limited batteries, the emergency RPI will go towards the initial point of release with the less amount of servo control possible.

May 2015: Got a new set:
Raspberry Pi V2 with Adafruit gps hat & motor hat, and of course the XLoBorg from PiBorg.

##  Temperature
The initial sensor payload for this design is a triple set of waterproof temperature sensors (DS18B20) at 0.5C accuracy. They can be ordered from https://www.sparkfun.com/products/11050 ( < 10USD per unit, not including shipping) but be ready to order a 4.7kOhms resistor to operate them (One resistor is enough for any number of pooled temperature sensors).

Location of the sensors:
1 - With/Below the Ballast (~30 cm below water line, changing with boat rolling angle)
2 - Floating behind the boat at the water surface (0cm height)
3 - On top of mast inside a (to be made) micro radiation shield (~1.6m height above the water surface, changing with boat rolling angle)

Note: Boat rolling is a problem for data standardization, initially a trimaran was thought of, however, the cost of such boat was not within my intended range for a first experiment. I am however looking for information (or help) to build a trimaran of similar size to correct this data acquisition inaccuracy.

Steps

    Get the sensors together plugged on the GPIO of the RPI + 4.7kO resistor.
    Communicate with all of them (code)
    Log the data into GIS format
    Import in GRASS GIS (http://grass.osgeo.org/) while surveying

1 - Get the sensors together plugged on the GPIO of the RPI + 4.7kO resistor.
2 - Communicate with all of them (code)
~~~~Python
#Reading DS18B20
#http://learn.adafruit.com/adafruits-raspberry-pi-lesson-11-ds18b20-temperature-sensing/software

import os
import glob
import time

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        #temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c

while True:
    print(read_temp())
    time.sleep(1)
~~~~

Notes:

I have found high accuracy waterproof temperature sensors, but they are in the range of 50 Euros per unit, so I postponed getting those for that project. To be complete, I just ran another search and found a waterproof PT100 from http://www.lightobject.com/Water-resistant-PT100-RTD-01-degree-Sensor-Probe-P592.aspx at ~17USD per unit (not including shipping) with 0.1C accuracy...

Got two free samples from https://shop.maxim-ic.com/storefront/searchsample.do?menuitem=Sample&event=SampleSearchLoad for the DS18B20 temperature sensors. Thank you !


    The green cable is linked to the red one of the probe and goes to 3V3 pin(#1) also plugged to yellow colour ring of the 4.7kO resistor
    The white cable is linked to the white one of the probe and goes to the data pin(#4) also plugged to the silver/gold? ring of the 4.7kO resistor
    The dark cable is linked to the black one from the probe and goes to the ground pin 3rd on the other side.

##  Web
For the short time of a survey, if internet connection is available, how much fun would it be to let the http://openweathermap.org engine get the air temperature data?

One line shell script is enough...
(http://openweathermap.org/wiki/Soft/for_stations)
~~~~Shell
#!/bin/bash
#Manual experiment
temperature=29.7
lat=6.89988
long=79.942061667
alt=5.0
curl -d 'temp='$temperature'&lat='$lat'&long='$long'&alt='$alt'&name=AmiTomi' \
        --user 'Yann:*****' http://openweathermap.org/data/post
~~~~
I received a confirmation string like this:
~~~~Shell
{"message":"","cod":"200","id":"64019"}
~~~~
and the screen capture on my account page gives the last uploaded value by amitomi.
	
