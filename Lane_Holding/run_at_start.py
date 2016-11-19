import RPi.GPIO as gpio
import time
import subprocess, os
import signal

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio_switch = 23
gpio.setup(gpio_switch,gpio.IN,pull_up_down=gpio.PUD_UP)

try:
    run = 0
    while True:
        if gpio.input(gpio_switch)==0 and run == 0:
            print 'Started'
            rpistr = "python /home/pi/Code/Lane_Holding/led_test.py"
            p=subprocess.Popen(rpistr,shell=True, preexec_fn=os.setsid)
            run = 1
            while gpio.input(gpio_switch)==0:
                time.sleep(1)
        if gpio.input(gpio_switch)==1 and run == 1:
            print "Stopped"
            run = 0
            os.killpg(p.pid, signal.SIGTERM)
            while gpio.input(gpio_switch)==1:
                time.sleep(1)

except KeyboardInterrupt:
    os.killpg(p.pid, signal.SIGTERM)
    print "Quit"
    gpio.cleanup()
