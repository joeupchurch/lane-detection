#Step 0: Preamble
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#Program Title  : easy_stepper.py 
#Code Written by: Salty Scott
#Current Project: www.rowboboat.com
#This code is a very basic example of using python to control a spark fun
# easy driver.  The spark fun easy driver that I am using in this example
# is connected to a 42HS4013A4 stepper motor and my raspberry pi.  Pin 23
# is the direction control and pin 24 is the step control.  I am using
# these components in the www.rowboboat.com project version 2.0 and I
# hope someone finds this a useful and simple example.
# This program expects two arguments: direction and steps
# Example usage: sudo python easy_stepper.py left 1600
# The above example would turn a 200 step motor one full revolution as by
# default the easy driver 4.4 is in 1/8 microstep mode. However the stepper driver 
# selected by gtaagii will default to one full step per step pulse, #microstepping can
# be selected if desired.
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
#Step 1: Import necessary libraries 
#------------------------------------------------------------------------
#------------------------------------------------------------------------
import sys
import RPi.GPIO as gpio #https://pypi.python.org/pypi/RPi.GPIO more info
import time
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
#Step 2: Read arguements https://www.youtube.com/watch?v=kQFKtI6gn9Y
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#read the direction and number of steps; if steps are 0 exit 
#using 16 microsteps per step 3200 for a full revoloution
try: 
	direction = "left"
	steps1 = 200
	steps2 = 400
	steps3 = 200
	
except:
	steps = 0
 
##print which direction and how many steps 
##print("You told me to turn %s %s steps.") % (direction, steps)
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 3: Setup the raspberry pi's GPIOs
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#use the broadcom layout for the gpio
gpio.setmode(gpio.BCM)
#GPIO23 = Direction
#GPIO24 = Step
#GPIO25 = Enable
gpio.setwarnings(False)
gpio.setup(23, gpio.OUT)
gpio.setup(24, gpio.OUT)
gpio.setup(25, gpio.OUT)
gpio.output(23, False)
gpio.output(24, False)
gpio.output(25, False)
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 4: Set direction of rotation
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#set the output to true for left and false for right
#------------------------------------------------------------------------
#------------------------------------------------------------------------
if direction == 'left':
	gpio.output(23, True)
elif direction == 'right':
	gpio.output(23, False)

print "Direction",
print direction
time.sleep(0.5)


gpio.output(25,True)
print "MSD415 Enabled"
time.sleep(0.5)
 
	
	
#Step 5: Setup step counter and speed control variables
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#track the number of steps taken
StepCounter = 0
 
#waittime controls speed
WaitTime = 0.01
WaitTimeAccel = 0.1
WaitTimeDecel =0.01
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 6: Let the magic happen
#------------------------------------------------------------------------
#------------------------------------------------------------------------
# Start main loop
while StepCounter < steps1:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(0.0015)
	gpio.output(24, False)
	print "steps1",
	print StepCounter
	StepCounter += 1
	WaitTimeAccel = 0.0005
	time.sleep(WaitTimeAccel)
	
Stepcounter = 0
while StepCounter < steps2:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(0.0015)
	gpio.output(24, False)
	print "steps2",
	print StepCounter
	StepCounter += 1
	time.sleep(WaitTime)
	
StepCounter = 0
while StepCounter < steps3:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(0.0015)
	gpio.output(24, False)
	print "steps3",
	print StepCounter
	StepCounter += 1
	WaitTimeDecel = 0.0005
	time.sleep(WaitTimeDecel)
	gpio.output(25, False)
   
	
#------------------------------------------------------------------------
#------------------------------------------------------------------------

#Reversing the direction to return to original position
##try: 
##	direction = "right"
##	time.sleep(0.005)

gpio.output(23, False)

Stepcounter = 0
while StepCounter < steps1:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(0.0015)
	gpio.output(24, False)
	print "steps1",
	print StepCounter
	StepCounter += 1
	WaitTimeAccel = 0.0005
	time.sleep(WaitTimeAccel)
	
Stepcounter = 0
while StepCounter < steps2:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(0.0015)
	gpio.output(24, False)
	print "steps2",
	print StepCounter
	StepCounter += 1
	time.sleep(WaitTime)
	
StepCounter = 0
while StepCounter < steps3:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(0.0015)
	gpio.output(24, False)
	print "steps3",
	print StepCounter
	StepCounter += 1
	WaitTimeDecel = 0.0005
	time.sleep(WaitTimeDecel)
	
	
gpio.output(25,True)
print "MSD415 disabled"
time.sleep(0.5)
	
	
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 7: Clear the GPIOs so that some other program might enjoy them
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#relase the GPIO
gpio.cleanup()
#------------------------------------------------------------------------
#------------------------------------------------------------------------
