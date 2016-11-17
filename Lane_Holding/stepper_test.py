#Step 1: Import necessary libraries 
#------------------------------------------------------------------------
#------------------------------------------------------------------------
import sys
import RPi.GPIO as gpio #https://pypi.python.org/pypi/RPi.GPIO more info
import time
from timeit import default_timer as timer
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
#Step 2: Read arguements https://www.youtube.com/watch?v=kQFKtI6gn9Y
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#read the direction and number of steps; if steps are 0 exit 
#using 16 microsteps per step 3200 for a full revoloution
try: 
	direction = "left"
	steps1 = 5*1036
	steps2 = 380
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
##gpio.output(23, False)
##gpio.output(24, False)
##gpio.output(25, False)

time.sleep(1)
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
WaitTimeAccel = 0.0005
WaitTimeAccelRate = 0.00002
WaitTimeDecel =0.01
#------------------------------------------------------------------------
#------------------------------------------------------------------------


#Step 6: Let the magic happen
#------------------------------------------------------------------------
#------------------------------------------------------------------------
# Start main loop
start = timer()
while StepCounter < steps1:
 
	#turning the gpio on and off tells the easy driver to take one step
	gpio.output(24, True)
	time.sleep(.0005)
	gpio.output(24, False)
	print "steps1",
	print StepCounter
	StepCounter += 1
	WaitTimeAccel = .001
	time.sleep(WaitTimeAccel)

0##StepCounter = 0
##while StepCounter < steps2:
## 
##	#turning the gpio on and off tells the easy driver to take one step
##	gpio.output(24, True)
##	time.sleep(.000015)
##	gpio.output(24, False)
##	print "steps2",
##	print StepCounter
##	StepCounter += 1
##	WaitTimeAccel = 0.0001
##	time.sleep(WaitTimeAccel)

end = timer()
print end-start
##gpio.cleanup()
