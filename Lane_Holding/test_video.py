##raspivid -o testVideo.h264 -w 360 -h 240 -t 10000 -fps 32 -rot 180

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from timeit import default_timer as timer
import cv2
import datetime
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
height = 480#730#972#480
width = 640#1296#640
camera.resolution = (width, height)
framerate = 60
camera.framerate = framerate

# Set Camera Settings
##camera.sharpness = 0
##camera.contrast = 0
##camera.brightness = 50
##camera.saturation = 0
##camera.ISO = 0
##camera.video_stabilization = False
##camera.exposure_compensation = 0
##camera.exposure_mode = 'auto'
##camera.meter_mode = 'average'
##camera.awb_mode = 'auto'
##camera.image_effect = 'none'
##camera.color_effects = None
camera.rotation = 180
##camera.hflip = False
##camera.vflip = False
camera.crop = (0.0, 0.0, 1.0, 1.0)

##hood_row = 1/2

rawCapture = PiRGBArray(camera, size=(width, height))
 
# allow the camera to warmup
time.sleep(0.1)

start = timer()
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	cv2.line(image,(width/2,0),(width/2,height),(0,0,255),2)
	cv2.line(image,(0,height*3/5),(width,height*3/5),(0,0,255),2)
 
	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

        end = timer()
        print 'Frame Rate :: ',1/(end-start)
        start = timer()
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# Create 10 second recording
t = datetime.datetime.now()

camera.start_recording('testVideo%d%d%d%d%d%d.h264' %(t.year,t.month,t.day,\
       t.hour,t.minute,t.second))
time.sleep(30)
camera.stop_recording()
