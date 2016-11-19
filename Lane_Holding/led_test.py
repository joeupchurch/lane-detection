import RPi.GPIO as gpio
import time
import os
import atexit

def exit_handler():
    gpio.output(led_gpio,False)
    gpio.cleanup()
    print "LED Script Ending"

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
led_gpio = 18

gpio.setup(led_gpio,gpio.OUT)
gpio.output(led_gpio,False)

print "LED Script Started"

while True:
    try:
        gpio.output(led_gpio,True)
        time.sleep(2)
        gpio.output(led_gpio,False)
        time.sleep(2)
    except KeyboardInterrupt:
        break

atexit.register(exit_handler)
