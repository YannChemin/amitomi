#!/usr/bin/python
from lib.lib_rpilcd import CharLCD
from subprocess import *
from time import sleep, strftime
from datetime import datetime

lcd = CharLCD()

cmd = "ip addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1"

lcd.begin(16,1)

def run_cmd(cmd):
	p = Popen(cmd, shell=True, stdout=PIPE)
	output = p.communicate()[0]
	return output

while 1:
	lcd.clear()
	ipaddr = run_cmd(cmd)
	lcd.message(datetime.now().strftime('%b %d %H:%M:%S\n'))
	lcd.message('IP %s' % ( ipaddr ) )
	exit(1)#One time display !
	#sleep(120)

