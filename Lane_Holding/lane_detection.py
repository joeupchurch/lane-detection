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
horizon_range_perc = 0.25
min_line_length_perc = 0.5
upper_frame_row = capture_height*3/12
horizon_row = capture_height*1/16
hood_row = capture_height*4/7
height = hood_row-upper_frame_row
horizon_range = [capture_width/2*(1-horizon_range_perc),capture_width/2*(1+horizon_range_perc)]
lane_width = capture_width*55/64
lane_middle = [capture_width/2]
mean_lane_middle = int(capture_width/2)
right_line_hood_range = [int(mean_lane_middle+lane_width/2\
                         -hood_range_perc/2*capture_width),\
                         int(mean_lane_middle+lane_width/2\
                         +hood_range_perc/2*capture_width)]
left_line_hood_range = [int(mean_lane_middle-lane_width/2\
                         -hood_range_perc/2*capture_width),\
                         int(mean_lane_middle-lane_width/2\
                         +hood_range_perc/2*capture_width)]
add_width = int(height*4/3-hood_range_perc/2*capture_width)
line_counter = 0
line_counter_max = 5

# Limit wheel position to +/- 10 degrees
wheel_pos = int(0) # In raw steps
wheel_pos_lim_amt = 5
wheel_pos_lim = [int((57*5*200*(-wheel_pos_lim_amt))/(11*360)),\
                 int((57*5*200*(wheel_pos_lim_amt))/(11*360))]

# Set max steps to take in one frame to 5 degrees
max_wheel_steps = int((57*5*200*(2))/(11*360))

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

    right_line_hood_capture_range = right_line_hood_range[:]
    left_line_hood_capture_range = left_line_hood_range[:]

    # Limit ranges to capture width
    if right_line_hood_capture_range[1]>capture_width:
        right_line_hood_capture_range[1] = int(capture_width)
    if right_line_hood_capture_range[0]<capture_width/2:
        right_line_hood_capture_range[0] = int(capture_width/2)
    if left_line_hood_capture_range[0]<0:
        left_line_hood_capture_range[0] = 0
    if left_line_hood_capture_range[1]>capture_width/2:
        left_line_hood_capture_range[1]=capture_width/2

    if right_line_hood_capture_range[1]<right_line_hood_capture_range[0]:
        right_line_hood_capture_range[0] = int(right_line_hood_capture_range[1]-capture_width*hood_range_perc)
    if left_line_hood_capture_range[1]<left_line_hood_capture_range[0]:
        left_line_hood_capture_range[1] = int(left_line_hood_capture_range[0]+capture_width*hood_range_perc)

##                
##    print right_line_hood_range
##    print left_line_hood_range
##    print right_line_hood_capture_range
##    print left_line_hood_capture_range
        
    # Clip image outside of road
    truncl = img[upper_frame_row:hood_row,\
                 left_line_hood_capture_range[0]:left_line_hood_capture_range[1]+add_width]
    truncr = img[upper_frame_row:hood_row,\
                 right_line_hood_capture_range[0]-add_width:right_line_hood_capture_range[1]]
    
    # Convert BGR to Gray
    grayl = cv2.cvtColor(truncl,cv2.COLOR_BGR2GRAY)
    grayr = cv2.cvtColor(truncr,cv2.COLOR_BGR2GRAY)
    
    # Apply histogram equalization to gray image
    equl = cv2.equalizeHist(grayl)
    equr = cv2.equalizeHist(grayr)
    
    # Threshold for equalized image
    retl,threshl = cv2.threshold(equl,250,255,cv2.THRESH_BINARY)
    retr,threshr = cv2.threshold(equr,250,255,cv2.THRESH_BINARY)
##    cv2.imshow('grayl',threshl)
##    cv2.imshow('grayr',threshr)

    # Bitwise-AND mask and original image
    resl = cv2.bitwise_and(truncl,truncl, mask= threshl)
    resr = cv2.bitwise_and(truncr,truncr, mask= threshr)

    # Perform Canny Edge Detection
    edgesl = cv2.Canny(resl,100,200,apertureSize = 3)
    edgesr = cv2.Canny(resr,100,200,apertureSize = 3)

    # Perform Hough Line Detection
    linesl = cv2.HoughLinesP(edgesl,1,np.pi/180,\
            int(height*min_line_length_perc/3),\
            minLineLength=int(height*min_line_length_perc),\
            maxLineGap=height*.2)
    linesr = cv2.HoughLinesP(edgesr,1,np.pi/180,\
        int(height*min_line_length_perc/3),\
        minLineLength=int(height*min_line_length_perc),\
        maxLineGap=height*.2)
##    cv2.imshow('edgesl',edgesl)
##    cv2.imshow('edgesr',edgesr)
    
    lines_matl = np.matrix(linesl,'float')
    lines_matr = np.matrix(linesr,'float')

##    if lines_matl.size>1:
##        for line in linesl:
##            x1,y1,x2,y2 = line[0]
##            cv2.line(img,(x1+left_line_hood_range[0],upper_frame_row+y1),(x2+left_line_hood_range[0],upper_frame_row+y2),(255,255,255),2)
##
##    if lines_matr.size>1:
##        for line in linesr:
##            x1,y1,x2,y2 = line[0]
##            cv2.line(img,(x1+right_line_hood_range[0]-add_width,upper_frame_row+y1),(x2+right_line_hood_range[0]-add_width,upper_frame_row+y2),(255,255,255),2)
##
##    cv2.waitKey(0)
    
    if lines_matl.size>1:

        # Calculate slope, hood, and horizon intercept for each line
        slope = np.divide(np.subtract(lines_matl[:,3],lines_matl[:,1]),\
                          np.subtract(lines_matl[:,2],lines_matl[:,0]))
        b = np.subtract(lines_matl[:,1]+upper_frame_row,np.multiply(slope,\
                        (lines_matl[:,0]+left_line_hood_capture_range[0])))
        hood_col = np.divide(np.subtract(upper_frame_row+height,b),slope)
        horizon_col = np.divide(np.subtract(horizon_row,b),slope)
        
        # Parse left lines based on hood/horizon intercepts and slope
        left_lines = np.logical_and(hood_col>left_line_hood_range[0],hood_col<left_line_hood_range[1])
        left_lines = np.logical_and(left_lines,horizon_col>horizon_range[0])
        left_lines = np.logical_and(left_lines,horizon_col<horizon_range[1])
        left_lines = np.logical_and(left_lines,slope<0)

        if any(left_lines):
            # Parse left lines based on max slope then maximum hood intercept
            left_line = np.logical_and(left_lines,hood_col==np.max(hood_col[left_lines]))
            left_line = np.logical_and(left_line,slope==np.min(slope[left_line]))

            # Pull left line hood/horizon intercept and lines_mat index
            left_line_hood = hood_col[left_line]
            left_line_horizon = horizon_col[left_line]
            left_row_idx,left_col_idx = np.where(left_line)

    else:
        left_lines = []

    if lines_matr.size>1:

        # Calculate slope, hood, and horizon intercept for each line
        slope = np.divide(np.subtract(lines_matr[:,3],lines_matr[:,1]),\
                          np.subtract(lines_matr[:,2],lines_matr[:,0]))
        b = np.subtract(lines_matr[:,1]+upper_frame_row,np.multiply(slope,\
                        (lines_matr[:,0]+right_line_hood_capture_range[0]-add_width)))
        hood_col = np.divide(np.subtract(upper_frame_row+height,b),slope)
        horizon_col = np.divide(np.subtract(horizon_row,b),slope)

        # Parse right lines based on hood/horizon intercepts and slope
        right_lines = np.logical_and(hood_col>right_line_hood_range[0],hood_col<right_line_hood_range[1])
        right_lines = np.logical_and(right_lines,horizon_col>horizon_range[0])
        right_lines = np.logical_and(right_lines,horizon_col<horizon_range[1])
        right_lines = np.logical_and(right_lines,slope>0)

        if any(right_lines):
            # Parse right lines on max slope then minimum hood intercept
            right_line = np.logical_and(right_lines,hood_col==np.min(hood_col[right_lines]))
            right_line = np.logical_and(right_line,slope==np.max(slope[right_line]))

            # Pull right line hood/horizon intercept and lines_mat index
            right_line_hood = hood_col[right_line]
            right_line_horizon = horizon_col[right_line]
            right_row_idx,right_col_idx = np.where(right_line)

    else:
        right_lines = []
        
    if any((any(right_lines),any(left_lines))):

        line_counter=0

        # Both lines detected
        if all((any(right_lines),any(left_lines))):
            
            # Draw lines on image for left and right lines
            cv2.line(img,(lines_matl[left_row_idx,0]+left_line_hood_capture_range[0],\
                          lines_matl[left_row_idx,1]+upper_frame_row),\
                     (lines_matl[left_row_idx,2]+left_line_hood_capture_range[0],\
                      lines_matl[left_row_idx,3]+upper_frame_row),(0,0,255),2)
            cv2.line(img,(lines_matr[right_row_idx,0]+right_line_hood_capture_range[0]-add_width,\
                          lines_matr[right_row_idx,1]+upper_frame_row),\
                     (lines_matr[right_row_idx,2]+right_line_hood_capture_range[0]-add_width,\
                      lines_matr[right_row_idx,3]+upper_frame_row),(0,255,0),2)
            
            # Update right line hood search regions based on latest line data
            right_line_hood_range = [int(right_line_hood-capture_width*hood_range_perc/2),\
                                     int(right_line_hood+capture_width*hood_range_perc/2)]

            # Update left line hood search regions based on latest line data
            left_line_hood_range = [int(left_line_hood-capture_width*hood_range_perc/2),\
                                    int(left_line_hood+capture_width*hood_range_perc/2)]

            # Calculate middle of lane from difference of hood intercepts
            lane_middle = np.append([lane_middle],\
                          int(np.mean([left_line_hood,right_line_hood])))
            # Consider only last five frames
            lane_middle = lane_middle[-1:]
            # Take mean of lane middles
            mean_lane_middle = int(np.mean(lane_middle))

        elif any(right_lines):
            # Draw line for on image right line
            cv2.line(img,(lines_matr[right_row_idx,0]+right_line_hood_capture_range[0]-add_width,\
                          lines_matr[right_row_idx,1]+upper_frame_row),\
                     (lines_matr[right_row_idx,2]+right_line_hood_capture_range[0]-add_width,\
                      lines_matr[right_row_idx,3]+upper_frame_row),(0,255,0),2)
            
            # Update right line hood search regions based on latest line data
            right_line_hood_range = [int(right_line_hood-capture_width*hood_range_perc/2),\
                                     int(right_line_hood+capture_width*hood_range_perc/2)]
            
            # Expand left line hood search regions based on right line if no left lines detected
            left_line_hood_range[0] = int(right_line_hood_range[0]-lane_width)
            left_line_hood_range[1] = int(right_line_hood_range[1]-lane_width)
            print left_line_hood_range
            
            # Calculate middle of lane from difference of hood intercepts
            lane_middle = np.append([lane_middle],int(right_line_hood-lane_width/2))
            # Consider only last frame
            lane_middle = lane_middle[-1:]
            # Take mean of lane middles
            mean_lane_middle = int(np.mean(lane_middle))

        elif any(left_lines):
            # Draw line for on image left line
            cv2.line(img,(lines_matl[left_row_idx,0]+left_line_hood_capture_range[0],\
                          lines_matl[left_row_idx,1]+upper_frame_row),\
                     (lines_matl[left_row_idx,2]+left_line_hood_capture_range[0],\
                      lines_matl[left_row_idx,3]+upper_frame_row),(0,0,255),2)
            
            # Update left line hood search regions based on latest line data
            left_line_hood_range = [int(left_line_hood-capture_width*hood_range_perc/2),\
                                    int(left_line_hood+capture_width*hood_range_perc/2)]
            
            # Expand right line hood search regions based on left line if no right lines detected
            right_line_hood_range[0] = int(left_line_hood_range[0]+lane_width)
            right_line_hood_range[1] = int(left_line_hood_range[1]+lane_width)
            
            # Calculate middle of lane from difference of hood intercepts
            lane_middle = np.append([lane_middle],int(left_line_hood+lane_width/2))
            # Consider only last frame
            lane_middle = lane_middle[-1:]
            # Take mean of lane middles
            mean_lane_middle = int(np.mean(lane_middle))

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
        right_line_hood_range = [int(right_line_hood_range[0]-capture_width*hood_range_update_perc/2),\
                                 int(right_line_hood_range[1]+capture_width*hood_range_update_perc/2)]
        left_line_hood_range = [int(left_line_hood_range[0]-capture_width*hood_range_update_perc/2),\
                                int(left_line_hood_range[1]+capture_width*hood_range_update_perc/2)]

        # Hold wheel position
        steps_to_take = int(0)
        line_counter += 1
        # Reset hood ranges
        if line_counter>line_counter_max:
            print 'Line Counter Reset'
            right_line_hood_range = [int(capture_width/2),\
                         int(capture_width*3/2)]
            left_line_hood_range = [int(-capture_width/2),\
                         int(capture_width/2)]

    # Limit wheel turning
    if abs(steps_to_take) > max_wheel_steps:
        steps_to_take = int(max_wheel_steps*steps_to_take/abs(steps_to_take))
        print 'Steps limited'
    if wheel_pos+steps_to_take>wheel_pos_lim[1]:
        steps_to_take = wheel_pos_lim[1]-wheel_pos
        print 'Exceeded max position'
    elif wheel_pos+steps_to_take<wheel_pos_lim[0]:
        steps_to_take = wheel_pos_lim[0]-wheel_pos
        print 'Exceeded min position'
    wheel_pos = wheel_pos + steps_to_take
   
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
    
    cv2.line(img,(0,hood_row),(capture_width,hood_row),(0,0,255),1)
    cv2.line(img,(0,horizon_row),(capture_width,horizon_row),(0,0,255),1)
    cv2.line(img,(0,upper_frame_row),(capture_width,upper_frame_row),(0,0,255),1)
    cv2.line(img,(int(left_line_hood_range[0]),upper_frame_row),(int(left_line_hood_range[0]),hood_row),(255,0,0),2)
    cv2.line(img,(int(left_line_hood_range[1]+add_width),upper_frame_row),(int(left_line_hood_range[1]+add_width),hood_row),(255,0,0),2)
    cv2.line(img,(int(left_line_hood_range[1]),hood_row-10),(int(left_line_hood_range[1]),hood_row),(255,0,0),2)
    cv2.line(img,(int(right_line_hood_range[0]),hood_row-10),(int(right_line_hood_range[0]),hood_row),(255,0,0),2)
    cv2.line(img,(int(right_line_hood_range[0]-add_width),upper_frame_row),(int(right_line_hood_range[0]-add_width),hood_row),(255,0,0),2)
    cv2.line(img,(int(right_line_hood_range[1]),upper_frame_row),(int(right_line_hood_range[1]),hood_row),(255,0,0),2)

    # Draw line on image for correction amount, blue for right, white for left
    if (mean_lane_middle-capture_width/2)<0:
            cv2.line(img,(capture_width/2,hood_row-5),(mean_lane_middle,hood_row-5),(255,255,255),2)
    elif(mean_lane_middle-capture_width/2)>0:
            cv2.line(img,(capture_width/2,hood_row-5),(mean_lane_middle,hood_row-5),(255,0,0),2)

    cv2.line(img,(capture_width/2,0),(capture_width/2,capture_height),(0,0,255),1)
    cv2.line(img,(mean_lane_middle-lane_width/2,0),(mean_lane_middle-lane_width/2,capture_height),(0,0,255),1)
    cv2.line(img,(mean_lane_middle+lane_width/2,0),(mean_lane_middle+lane_width/2,capture_height),(0,0,255),1)

    error_text = '%d %d' %(error[-1],steps_to_take)
    cv2.putText(img,error_text,(capture_width/2-30,capture_height-4),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),1)

    # Display image, break if 'q' pressed
##    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
##    cv2.imshow('Image',img)
##	
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

atexit.register(exit_handler)
