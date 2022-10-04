#! /usr/local/bin/python
#-*- coding: utf-8 -*-
import gps, os, time

os.system("'sudo killall gpsd; sleep(2)'") 
os.system('sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock') 

#g = gps.gps(mode=gps.WATCH_NEWSTYLE)
g = gps.gps()
#g.poll()
g.stream()
while 1:
	time.sleep(2)
	os.system('clear')
	#g.poll()
	if gps.PACKET_SET:
		g.stream()
		print
		print ' GPS reading'
		print '----------------------------------------'
		print 'latitude ' , g.fix.latitude
		print 'longitude ' , g.fix.longitude
		print 'time utc ' , g.utc,' + ', g.fix.time
		print 'altitude ' , g.fix.altitude
		print 'epc ' , g.fix.epc
		print 'epd ' , g.fix.epd
		print 'eps ' , g.fix.eps
		print 'epx ' , g.fix.epx
		print 'epv ' , g.fix.epv
		print 'ept ' , g.fix.ept
		print 'speed ' , g.fix.speed
		print 'climb ' , g.fix.climb
		print 'track ' , g.fix.track
		print 'mode ' , g.fix.mode
		print
		print 'sats ' , g.satellites
