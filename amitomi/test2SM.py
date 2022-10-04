#!/usr/bin/python
def warning():
	"""
	#----------------------------
	Careful with threading 
	it will take over an already
	running stepper
	thus not executing all steps
	and loosing the counters
	#----------------------------
	"""
	print("Threading is a problem with counters careful !")

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_StepperMotor
import time
import atexit
import threading
import random

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT()

# create empty threads (these will hold the stepper 1 and 2 threads)
st1 = threading.Thread()
st2 = threading.Thread()


# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

#Stepper 1 is the SAIL WINCH, FASTER
S1 = mh.getStepper(64, 1)  	# 200 steps/rev, motor port #1
#Steppet 2 is the RUDDER, SLOW
S2 = mh.getStepper(64, 2)  	# 200 steps/rev, motor port #2
#Set Speed
S1.setSpeed(6400)  		# 30 RPM
S2.setSpeed(30)  		# 30 RPM

#Not useful right now, we use SINGLE
stepstyles = [Adafruit_MotorHAT.SINGLE, Adafruit_MotorHAT.DOUBLE, Adafruit_MotorHAT.INTERLEAVE, Adafruit_MotorHAT.MICROSTEP]

#Define the Stepper Counters
counterS1=0
counterS2=0

def stepper_worker(stepper, numsteps, direction, style):
	stepper.step(numsteps, direction, style)
	#Inform the stepper counter 
	#in which direction we increment
	if(direction==Adafruit_MotorHAT.FORWARD):
		return(numsteps)
	else:
		return(-1*numsteps)

while (True):
		print("Stepper 1"),
		direc = Adafruit_MotorHAT.FORWARD
		print("forward"),
		steps = 300		
		print("%d steps" % steps)
		counterS1+=stepper_worker(S1,steps,direc,stepstyles[1])
		print(counterS1)
		#direc = Adafruit_MotorHAT.BACKWARD
		#print("backward"),
		#steps = 300		
		#print("%d steps" % steps)
		#stepper_worker(S1,steps,direc,stepstyles[1])
		#st1 = threading.Thread(target=stepper_worker, args=(S1,steps,direc,stepstyles[1],))
		#st1.start()
		print("Stepper 2"),
		direc = Adafruit_MotorHAT.FORWARD
		print("forward"),
		steps = 64		
		print("%d steps" % steps)
		counterS2+=stepper_worker(S2,steps,direc,stepstyles[2])
		print(counterS2)
		direc = Adafruit_MotorHAT.BACKWARD
		print("backward"),
		steps = 63		
		print("%d steps" % steps)
		counterS2+=stepper_worker(S2,steps,direc,stepstyles[2])
		print(counterS2)
		#st2 = threading.Thread(target=stepper_worker, args=(S2,steps,direc,stepstyles[1],))
		#st2.start()
