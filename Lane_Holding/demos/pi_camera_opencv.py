### import the necessary packages
##from picamera.array import PiRGBArray
##from picamera import PiCamera
##import time
##import cv2
##import numpy as np
## 
### initialize the camera and grab a reference to the raw camera capture
##camera = PiCamera()
##rawCapture = PiRGBArray(camera)
## 
### allow the camera to warmup
##time.sleep(0.1)
## 
### grab an image from the camera
##camera.rotation = 180
##camera.capture(rawCapture, format="bgr")
##img = rawCapture.array
##
##face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
##eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
##
##gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
##
##faces = face_cascade.detectMultiScale(gray, 1.3, 5)
##for (x,y,w,h) in faces:
##    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
##    roi_gray = gray[y:y+h, x:x+w]
##    roi_color = img[y:y+h, x:x+w]
##    eyes = eye_cascade.detectMultiScale(roi_gray)
##    for (ex,ey,ew,eh) in eyes:
##        cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
##
##cv2.imshow('img',img)
##cv2.waitKey(0)
##cv2.destroyAllWindows()

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (480, 320)
camera.rotation = 0
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(480, 320))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	img = frame.array

	face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
                
	# show the frame
	cv2.imshow("Frame", img)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
                cv2.imwrite("output.jpg", img)
		break
