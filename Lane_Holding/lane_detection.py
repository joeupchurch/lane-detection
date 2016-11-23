# import the necessary packages
import cv2
import numpy as np
from matplotlib import pyplot as plt
from timeit import default_timer as timer
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import sys
import RPi.GPIO as gpio
import atexit

# initialize the camera and grab a reference to the raw camera capture
capture_width = 368
capture_height = 240
camera = PiCamera()
camera.resolution = (capture_width, capture_height)
camera.framerate = 32
camera.rotation = 180
camera.contrast = 0
camera.saturation = 0
camera.brightness = 50
rawCapture = PiRGBArray(camera, size=(capture_width, capture_height))

# Set initial values for lane, line, and camera parameters
hood_range_perc = 0.20
hood_range_update_perc = 0.01
horizon_range_perc = 0.05
min_line_length_perc = 0.5
upper_frame_row = capture_height*3/12
horizon_row = capture_height*1/12
hood_row = capture_height*3/7
height = hood_row-upper_frame_row
right_line_hood_range = [capture_width/2,capture_width]
left_line_hood_range = [0,capture_width/2]
horizon_range = [capture_width/2*(1-horizon_range_perc),capture_width/2*(1+horizon_range_perc)]
lane_width = capture_width*19/32
lane_middle = capture_width/2
mean_lane_middle = int(capture_width/2)

# Set GPIO Pins for stepper
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio_pulse = 13
gpio_direction = 19
gpio.setup(gpio_pulse, gpio.OUT) # Pulse
gpio.setup(gpio_direction, gpio.OUT) # Direction

# Set PID Values for error loop
error = [0]
error_time = [timer()]
Kp = input('Kp :: ')
Ki = input('Ki :: ')
Kd = input('Kd :: ')

# Define Exit Procedure
def exit_handler():
    cv2.destroyAllWindows()
    gpio.output(gpio_pulse,False)
    gpio.output(gpio_direction,False)
    gpio.cleanup()
    print "Lane Detection Script Ending"
 
# allow the camera to warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # Start Timer for Framerate
    start = timer()

    # Save frame to img
    img = frame.array

    # Clip image outside of road
    trunc = img[upper_frame_row:hood_row,0:capture_width]

    # Convert BGR to Gray
    gray = cv2.cvtColor(trunc,cv2.COLOR_BGR2GRAY)

    # Apply histogram equalization to gray image
    equ = cv2.equalizeHist(gray)

    # Threshold for equalized image
    ret,thresh = cv2.threshold(equ,250,255,cv2.THRESH_BINARY)

##    cv2.imshow('gray',thresh)

    # Bitwise-AND mask and original image
    resw = cv2.bitwise_and(trunc,trunc, mask= thresh)

    # Perform Canny Edge Detection
    edges = cv2.Canny(resw,100,200,apertureSize = 3)

    # Perform Hough Line Detection
    lines = cv2.HoughLinesP(edges,1,np.pi/180,\
            int(height*min_line_length_perc/2),\
            minLineLength=int(height*min_line_length_perc),\
            maxLineGap=height*0.2)
    lines_mat = np.matrix(lines,'float')

##    cv2.imshow('edges',edges)
##    print lines
##    cv2.waitKey(0)
##    cv2.line(edges,lines,(0,255,0),1)
    
    if lines_mat.size>1:

        # Calculate slope, hood, and horizon intercept for each line
        slope = np.divide(np.subtract(lines_mat[:,3],lines_mat[:,1]),np.subtract(lines_mat[:,2],lines_mat[:,0]))
        b = np.subtract(lines_mat[:,1],np.multiply(slope,lines_mat[:,0]))+(upper_frame_row-horizon_row)
        hood_col = np.divide(np.subtract(np.subtract(hood_row,horizon_row),b),slope)
        horizon_col = np.divide(-b,slope)

        # Parse right lines based on hood/horizon intercepts and slope
        right_lines = np.logical_and(hood_col>right_line_hood_range[0],hood_col<right_line_hood_range[1])
        right_lines = np.logical_and(right_lines,horizon_col>horizon_range[0])
        right_lines = np.logical_and(right_lines,horizon_col<horizon_range[1])
        right_lines = np.logical_and(right_lines,slope>0)

        # Parse left lines based on hood/horizon intercepts and slope
        left_lines = np.logical_and(hood_col>left_line_hood_range[0],hood_col<left_line_hood_range[1])
        left_lines = np.logical_and(left_lines,horizon_col>horizon_range[0])
        left_lines = np.logical_and(left_lines,horizon_col<horizon_range[1])
        left_lines = np.logical_and(left_lines,slope<0)

        if any((any(right_lines),any(left_lines))):

            if any(right_lines):
                # Parse right lines based on max slope then minimum hood intercept
                right_line = np.logical_and(right_lines,slope==np.max(slope[right_lines]))
                right_line = np.logical_and(right_line,hood_col==np.min(hood_col[right_line]))

                # Pull right line hood/horizon intercept and lines_mat index
                right_line_hood = hood_col[right_line]
                right_line_horizon = horizon_col[right_line]
                right_row_idx,right_col_idx = np.where(right_line)
                
            if any(left_lines):
                # Parse left lines based on max slope then maximum hood intercept
                left_line = np.logical_and(left_lines,slope==np.min(slope[left_lines]))
                left_line = np.logical_and(left_line,hood_col==np.max(hood_col[left_line]))

                # Pull left line hood/horizon intercept and lines_mat index
                left_line_hood = hood_col[left_line]
                left_line_horizon = horizon_col[left_line]
                left_row_idx,left_col_idx = np.where(left_line)

            # Both lines detected
            if all((any(right_lines),any(left_lines))):
                # Update right line hood search regions based on latest line data
                right_line_hood_range = [right_line_hood-capture_width*hood_range_perc/2,right_line_hood+capture_width*hood_range_perc/2]

                # Update left line hood search regions based on latest line data
                left_line_hood_range = [left_line_hood-capture_width*hood_range_perc/2,left_line_hood+capture_width*hood_range_perc/2]

                # Calculate middle of lane from difference of hood intercepts
                lane_middle = np.append([lane_middle],\
                              int(np.mean([left_line_hood,right_line_hood])))
                # Consider only last five frames
                lane_middle = lane_middle[-5:]
                # Take mean of lane middles
                mean_lane_middle = int(np.mean(lane_middle))

                # Draw lines on image for left and right lines
                cv2.line(img,(lines_mat[left_row_idx,0],lines_mat[left_row_idx,1]+upper_frame_row),(lines_mat[left_row_idx,2],lines_mat[left_row_idx,3]+upper_frame_row),(0,0,255),2)
                cv2.line(img,(lines_mat[right_row_idx,0],lines_mat[right_row_idx,1]+upper_frame_row),(lines_mat[right_row_idx,2],lines_mat[right_row_idx,3]+upper_frame_row),(0,255,0),2)
            
            elif any(right_lines):

                # Update right line hood search regions based on latest line data
                right_line_hood_range = [right_line_hood-capture_width*hood_range_perc/2,right_line_hood+capture_width*hood_range_perc/2]
                
                # Expand left line hood search regions based on right line if no left lines detected
                left_line_hood_range[0] = right_line_hood_range[0]-lane_width
                left_line_hood_range[1] = right_line_hood_range[1]-lane_width
                
                # Calculate middle of lane from difference of hood intercepts
                lane_middle = np.append([lane_middle],\
                              int(right_line_hood-lane_width/2))
                # Consider only last five frames
                lane_middle = lane_middle[-5:]
                # Take mean of lane middles
                mean_lane_middle = int(np.mean(lane_middle))
                
                # Draw line for on image right line
                cv2.line(img,(lines_mat[right_row_idx,0],lines_mat[right_row_idx,1]+upper_frame_row),(lines_mat[right_row_idx,2],lines_mat[right_row_idx,3]+upper_frame_row),(0,255,0),2)


            elif any(left_lines):
                # Update left line hood search regions based on latest line data
                left_line_hood_range = [left_line_hood-capture_width*hood_range_perc/2,left_line_hood+capture_width*hood_range_perc/2]
                
                # Expand right line hood search regions based on left line if no right lines detected
                right_line_hood_range[0] = left_line_hood_range[0]+lane_width
                right_line_hood_range[1] = left_line_hood_range[1]+lane_width
                
                # Calculate middle of lane from difference of hood intercepts
                lane_middle = np.append([lane_middle],\
                              int(left_line_hood+lane_width/2))
                # Consider only last five frames
                lane_middle = lane_middle[-5:]
                # Take mean of lane middles
                mean_lane_middle = int(np.mean(lane_middle))

                # Draw line for on image left line
                cv2.line(img,(lines_mat[left_row_idx,0],lines_mat[left_row_idx,1]+upper_frame_row),(lines_mat[left_row_idx,2],lines_mat[left_row_idx,3]+upper_frame_row),(0,0,255),2)

            # Determine number of stepper motor steps to take
            error = np.append([error],mean_lane_middle-capture_width/2)
            error = error[-2:]
            error_time = np.append([error_time],timer())
            error_time = error_time[-2:]
            steps_to_take = int(Kp*error[-1]\
                +Ki*np.mean(error)*(error_time[1]-error_time[0])\
                +Kd*(error[1]-error[0])/(error_time[1]-error_time[0]))

        # No lines detected
        else:
            # Update line hood search regions based on last line data
            right_line_hood_range = [right_line_hood_range[0]-capture_width*hood_range_update_perc/2,right_line_hood_range[1]+capture_width*hood_range_update_perc/2]
            left_line_hood_range = [left_line_hood_range[0]-capture_width*hood_range_update_perc/2,left_line_hood_range[1]+capture_width*hood_range_update_perc/2]

            if right_line_hood_range[1]-right_line_hood_range[0]>capture_width/2:
                right_line_hood_range[0] = capture_width*1/2
                right_line_hood_range[1] = capture_width

            if left_line_hood_range[1]-left_line_hood_range[0]>capture_width/2:
                left_line_hood_range[0] = 0
                left_line_hood_range[1] = capture_width*1/2
                

            # Hold wheel position
            steps_to_take = 0

    # If no lines detected in lines_mat
    else:
        # Hold wheel position
        steps_to_take = 0

    # Drive stepper motor
    StepCounter = 0
    if steps_to_take !=0:
        # Set Direction
        if steps_to_take>0:
            gpio.output(gpio_direction, False)
        else:
            gpio.output(gpio_direction, True)
            
        # Pulse Steps        
        while StepCounter<abs(steps_to_take):
            gpio.output(gpio_pulse, True)
	    time.sleep(.0005)
	    gpio.output(gpio_pulse, False)
	    time.sleep(.001)
	    StepCounter +=1
	    
    # End timer for frame rate calculation
    end = timer()

    # Display information in terminal
    print ''
    print 'Error :: ',error
    print 'Steps to Take :: ',steps_to_take
    print 'Lane Middle :: ', mean_lane_middle
    print 'Frame Rate :: ',1/(end - start)

    # Draw line on image for correction amount, blue for right, white for left
    if (mean_lane_middle-capture_width/2)<0:
            cv2.line(img,(capture_width/2,hood_row-5),(mean_lane_middle,hood_row-5),(255,255,255),2)
    elif(mean_lane_middle-capture_width/2)>0:
            cv2.line(img,(capture_width/2,hood_row-5),(mean_lane_middle,hood_row-5),(255,0,0),2)

    cv2.line(img,(0,hood_row),(capture_width,hood_row),(0,0,255),1)
    cv2.line(img,(0,horizon_row),(capture_width,horizon_row),(0,0,255),1)
    cv2.line(img,(0,upper_frame_row),(capture_width,upper_frame_row),(0,0,255),1)
    cv2.line(img,(int(left_line_hood_range[0]),0),(int(left_line_hood_range[0]),capture_height),(255,0,0),2)
    cv2.line(img,(int(left_line_hood_range[0]),0),(int(left_line_hood_range[0]),capture_height),(255,0,0),2)
    cv2.line(img,(int(left_line_hood_range[1]),0),(int(left_line_hood_range[1]),capture_height),(255,0,0),2)
    cv2.line(img,(int(right_line_hood_range[0]),0),(int(right_line_hood_range[0]),capture_height),(255,0,0),2)
    cv2.line(img,(int(right_line_hood_range[1]),0),(int(right_line_hood_range[1]),capture_height),(255,0,0),2)

    cv2.line(img,(mean_lane_middle-lane_width/2,0),(mean_lane_middle-lane_width/2,capture_height),(0,0,255),1)
    cv2.line(img,(mean_lane_middle+lane_width/2,0),(mean_lane_middle+lane_width/2,capture_height),(0,0,255),1)

    error_text = '%d %d' %(error[-1],steps_to_take)
    cv2.putText(img,error_text,(capture_width/2-30,capture_height-4),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),1)

    # Display image, break if 'q' pressed
    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
    cv2.imshow('Image',img)
	
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

atexit.register(exit_handler)
