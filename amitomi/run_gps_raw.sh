#!/bin/bash
sudo killall gpsd
sleep 2 
sudo gpsd /dev/ttyACM0 -F /var/run/gpsd.sock

#-d causes gpspipe to run as a daemon.
#-l causes gpspipe to sleep for ten seconds before attempting to connect to gpsd. This is very useful when running as a daemon, giving gpsd time to start before attempting a connection.
#-r causes raw NMEA sentences to be output.
#-o option causes the collected data to be written to the specified file. Use of this option is mandatory if gpspipe is run as a daemon.
gpspipe -d -l -r -o /home/pi/gps_raw.txt &


#sudo killall gpsd
#sleep 2 

