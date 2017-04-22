# import the necessary packages
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
from timeit import default_timer as timer
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import sys
import RPi.GPIO as gpio
import atexit
from imutils.video.pivideostream import PiVideoStream
from threading import Thread

test_video = 0
display_image = 1

if test_video==1:
    cap = cv2.VideoCapture('/home/pi/Code/Lane_Holding/test_videos/testVideo201732919336.h264')
    ret, frame = cap.read()
    
    # Define camera height and width (defaults)
    capture_height,capture_width,channels = frame.shape
    print frame.shape

else:
    # Define camera height and width (defaults)
    capture_width = 640
    capture_height = 480
    
    # Start Video Stream and warm-up
    vs = PiVideoStream().start()
    time.sleep(2.0)
    
# Set PID Values for error loop
error = [0]
error_time = [float(timer())]
error_frames = 1
error_mean_frames = float(10)
error_multiplier = 2/(error_mean_frames+1)
p_error = 0
i_error = 0
d_error = 0

Kp = 64/float(capture_width)#input('Kp :: ')
Ki = 0/float(capture_width)#125#.5#input('Ki :: ')
Kd = 128/float(capture_width)#input('Kd :: ')
print 'Kp :: ',Kp
print 'Ki :: ',Ki
print 'Kd ;; ',Kd

# Set frame rate variables
min_steps_to_take = 0
max_steps_to_take = 16
max_frame_rate = float(40)

# Set initial values for lane, line, and camera parameters
hood_range_perc = 0.10
hood_range_update_perc = 0.05
hood_row = capture_height*3/5
hood_range_max_width = 5*hood_range_perc*capture_width
hood_mean_frames = float(10)
hood_mean_multiplier = 2/(hood_mean_frames+1)

upper_frame_range_perc = 0.02
upper_frame_range_update_perc = 0.20
upper_frame_row = capture_height*1/12
upper_frame_range_max_width = 20*upper_frame_range_perc*capture_width
upper_frame_mean_frames = float(10)
upper_frame_mean_multiplier = 2/(upper_frame_mean_frames+1)

lane_width_init = int(750*capture_width/640)
upper_frame_width_init = int(85*capture_width/640)

min_line_length_perc = 0.5
slope_factor = 0.025

reset_threshold = 60
resets_counter = 0
resets_counter_threshold = 5

height = hood_row-upper_frame_row
frame_number = 0
missed_frame_counter = 0

# Reset variables
lane_width = lane_width_init
upper_frame_width = upper_frame_width_init

lane_middle = [capture_width/2]
mean_lane_middle = int(capture_width/2)

hood_middle = int(capture_width/2)

right_hood_range = [int(mean_lane_middle+lane_width/2\
                         -hood_range_perc/2*capture_width),\
                         int(mean_lane_middle+lane_width/2\
                         +hood_range_perc/2*capture_width)]
left_hood_range = [int(mean_lane_middle-lane_width/2\
                         -hood_range_perc/2*capture_width),\
                         int(mean_lane_middle-lane_width/2\
                         +hood_range_perc/2*capture_width)]

right_upper_frame_range = [int(mean_lane_middle+upper_frame_width/2\
                            -upper_frame_range_perc/2*capture_width),\
                            int(mean_lane_middle+upper_frame_width/2\
                            +upper_frame_range_perc/2*capture_width)]
left_upper_frame_range = [int(mean_lane_middle-upper_frame_width/2\
                            -upper_frame_range_perc/2*capture_width),\
                            int(mean_lane_middle-upper_frame_width/2\
                            +upper_frame_range_perc/2*capture_width)]

right_hood_int = int(mean_lane_middle+lane_width/2)
left_hood_int = int(mean_lane_middle-lane_width/2)

right_upper_frame_int = int(mean_lane_middle+upper_frame_width/2)
left_upper_frame_int = int(mean_lane_middle-upper_frame_width/2)

right_line_slope_check = False
left_line_slope_check = False

reset_counter = 0

# Set GPIO Pins for stepper
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio_pulse = 13
gpio_direction = 19
gpio.setup(gpio_pulse, gpio.OUT) # Pulse
gpio.setup(gpio_direction, gpio.OUT) # Direction

# Define Exit Procedure
def exit_handler():
    cv2.destroyAllWindows()
    if test_video==0:
        vs.stop()
    gpio.output(gpio_pulse,False)
    gpio.output(gpio_direction,False)
    gpio.cleanup()
    print 'Lane Detection Script Ending'

# Define Stepper Function
def stepperFunc(steps_to_take):
    StepCounter = 0
    # Drive stepper motor
    if steps_to_take<0:
        gpio.output(19, True)
    else:
        gpio.output(19, False)
    if abs(steps_to_take)>max_steps_to_take:
        steps_to_take = max_steps_to_take
    while StepCounter<abs(steps_to_take):
        time.sleep(.00075)
        gpio.output(13, True)
        time.sleep(.00075)
        gpio.output(13, False)
        StepCounter +=1

# Start Timer for Framerate
start = timer()

while True:
    # Grab frame differently if test video
    if test_video==1:
        ret, frame = cap.read()
    else:
        # Grab frame from video stream
        frame = vs.read()

    try:
        # Index Frame Number
        frame_number+=1

        if display_image==1:
            print ' '
            print 'Frame Number :: ',frame_number

        # Save frame to img
        img = frame
            
        # Calculate slopes
        left_line_slope = float(upper_frame_row-hood_row)/float(left_upper_frame_int-left_hood_int)
        right_line_slope = float(upper_frame_row-hood_row)/float(right_upper_frame_int-right_hood_int)

        # Calculate boundary edge intercepts
        left_line_int = hood_row-left_line_slope*left_hood_range[0]
        right_line_int = hood_row-right_line_slope*right_hood_range[1]

        # Calculate frame intercepts
        left_line_frame_int = int(left_line_int)
        right_line_frame_int = int(right_line_slope*capture_width+right_line_int)

        # Calculate hood range widths
        right_hood_range_width = right_hood_range[1]-right_hood_range[0]
        left_hood_range_width = left_hood_range[1]-left_hood_range[0]

        # Calculate upper frame range widths
        right_upper_frame_range_width = right_upper_frame_range[1]-right_upper_frame_range[0]
        left_upper_frame_range_width = left_upper_frame_range[1]-left_upper_frame_range[0]

        # Define points for line transformations
        if right_line_frame_int<hood_row:
            right_transform_width = capture_width - right_hood_range_width
            right_transform_height = right_line_frame_int-upper_frame_row
            right_transform_height = int(right_transform_height)
                
            right_line_pts1 = np.float32([[right_upper_frame_int-right_hood_range_width/2,upper_frame_row],\
                                          [right_upper_frame_int+right_hood_range_width/2,upper_frame_row],\
                                          [capture_width-right_hood_range_width,right_line_frame_int],\
                                          [capture_width,right_line_frame_int]])
            right_line_pts2 = np.float32([[0,0],[right_hood_range_width,0],\
                                         [0,right_transform_height],\
                                         [right_hood_range_width,right_transform_height]])
        else:
            right_transform_width = right_hood_range[0]
            right_transform_height = height
            right_transform_height = int(right_transform_height)

            right_line_pts1 = np.float32([[right_upper_frame_int-right_hood_range_width/2,upper_frame_row],\
                                          [right_upper_frame_int+right_hood_range_width/2,upper_frame_row],\
                                          [right_hood_range[0],hood_row],\
                                          [right_hood_range[1],hood_row]])
            right_line_pts2 = np.float32([[0,0],[right_hood_range_width,0],\
                                         [0,height],\
                                         [right_hood_range_width,height]])

        if left_line_frame_int<hood_row:
            left_transform_width = 0
            left_transform_height = left_line_frame_int-upper_frame_row
            left_transform_height = int(left_transform_height)

            left_line_pts1 = np.float32([[left_upper_frame_int-left_hood_range_width/2,upper_frame_row],\
                                          [left_upper_frame_int+left_hood_range_width/2,upper_frame_row],\
                                          [0,left_line_frame_int],\
                                          [left_hood_range_width,left_line_frame_int]])
            left_line_pts2 = np.float32([[0,0],[left_hood_range_width,0],\
                                         [0,left_transform_height],\
                                         [left_hood_range_width,left_transform_height]])
        else:
            left_transform_width = left_hood_range[0]
            left_transform_height = height
            left_transform_height = int(left_transform_height)

            left_line_pts1 = np.float32([[left_upper_frame_int-left_hood_range_width/2,upper_frame_row],\
                                          [left_upper_frame_int+left_hood_range_width/2,upper_frame_row],\
                                          [left_hood_range[0],hood_row],\
                                          [left_hood_range[1],hood_row]])
            left_line_pts2 = np.float32([[0,0],[left_hood_range_width,0],\
                                         [0,height],\
                                         [left_hood_range_width,height]])

        # Define transform matrices
        M_right = cv2.getPerspectiveTransform(right_line_pts1,right_line_pts2)
        M_left  = cv2.getPerspectiveTransform(left_line_pts1,left_line_pts2)

        # Perform transform
        dst_right = cv2.warpPerspective(img,M_right,(int(right_hood_range_width),int(right_transform_height)))
        dst_left  = cv2.warpPerspective(img,M_left,(int(left_hood_range_width),int(left_transform_height)))
                                             
        # Convert BGR to Gray
        grayl = cv2.cvtColor(dst_left,cv2.COLOR_BGR2GRAY)
        grayr = cv2.cvtColor(dst_right,cv2.COLOR_BGR2GRAY)

        # Apply histogram equalization to gray image
        equl = cv2.equalizeHist(grayl)
        equr = cv2.equalizeHist(grayr)
        
        # Threshold for equalized image
        retl,threshl = cv2.threshold(equl,245,255,cv2.THRESH_BINARY)
        retr,threshr = cv2.threshold(equr,245,255,cv2.THRESH_BINARY)

        # Bitwise-AND mask original image
        resl = cv2.bitwise_and(dst_left,dst_left, mask= threshl)
        resr = cv2.bitwise_and(dst_right,dst_right, mask= threshr)

        # Perform Canny Edge Detection
        edgesl = cv2.Canny(resl,100,200,apertureSize = 3)
        edgesr = cv2.Canny(resr,100,200,apertureSize = 3)

        # Perform Hough Line Detection
        linesl = cv2.HoughLinesP(edgesl,1,np.pi/180,\
                int(left_transform_height*min_line_length_perc/5),\
                minLineLength=int(left_transform_height*min_line_length_perc),\
                maxLineGap=left_transform_height*.5)
        linesr = cv2.HoughLinesP(edgesr,1,np.pi/180,\
            int(right_transform_height*min_line_length_perc/5),\
            minLineLength=int(right_transform_height*min_line_length_perc),\
            maxLineGap=right_transform_height*.5)

        lines_matl = np.matrix(linesl,'float')
        lines_matr = np.matrix(linesr,'float')

        points_matl = np.matrix(left_line_pts1,'int')
        points_matr = np.matrix(right_line_pts1,'int')

        # Show all lines on image
        if lines_matl.size>1:

            lines_matl[:,1] = np.add(lines_matl[:,1],upper_frame_row)
            lines_matl[:,3] = np.add(lines_matl[:,3],upper_frame_row)
            lines_matl[:,0] = np.add(lines_matl[:,0],np.divide(np.subtract(lines_matl[:,1],left_line_int),left_line_slope))
            lines_matl[:,2] = np.add(lines_matl[:,2],np.divide(np.subtract(lines_matl[:,3],left_line_int),left_line_slope))

            for line in linesl:
                x1,y1,x2,y2 = line[0]
                cv2.line(img,(int((y1+upper_frame_row-left_line_int)/left_line_slope+x1),y1+upper_frame_row),\
                         (int((y2+upper_frame_row-left_line_int)/left_line_slope+x2),y2+upper_frame_row),(255,255,255),1)

        if lines_matr.size>1:

            lines_matr[:,1] = np.add(lines_matr[:,1],upper_frame_row)
            lines_matr[:,3] = np.add(lines_matr[:,3],upper_frame_row)
            lines_matr[:,0] = np.add(lines_matr[:,0],np.subtract(np.divide(np.subtract(lines_matr[:,1],right_line_int),right_line_slope),right_hood_range_width))
            lines_matr[:,2] = np.add(lines_matr[:,2],np.subtract(np.divide(np.subtract(lines_matr[:,3],right_line_int),right_line_slope),right_hood_range_width))
            
            for line in linesr:
                x1,y1,x2,y2 = line[0]
                cv2.line(img,(int((y1+upper_frame_row-right_line_int)/right_line_slope+x1-right_hood_range_width),y1+upper_frame_row),\
                         (int((y2+upper_frame_row-right_line_int)/right_line_slope+x2-right_hood_range_width),y2+upper_frame_row),(255,255,255),1)
        
        if lines_matl.size>1:
            try:
                # Calculate slope/hood intercept for each line
                slope = np.divide(np.subtract(lines_matl[:,3],lines_matl[:,1]),\
                                  np.subtract(lines_matl[:,2],lines_matl[:,0]))
                b = np.subtract(lines_matl[:,1],np.multiply(slope,lines_matl[:,0]))
                hood_col = np.divide(np.subtract(upper_frame_row+height,b),slope)
                upper_frame_col = np.divide(np.subtract(upper_frame_row,b),slope)

                # Parse left lines based on hood/upper frame intercepts and slope
                left_lines = np.logical_and(hood_col>left_hood_range[0],hood_col<left_hood_range[1])
                left_lines = np.logical_and(left_lines,upper_frame_col>left_upper_frame_range[0])
                left_lines = np.logical_and(left_lines,upper_frame_col<left_upper_frame_range[1])
                if left_line_slope_check:
                    left_lines = np.logical_and(left_lines,slope>left_line_slope*(1+slope_factor))
                    left_lines = np.logical_and(left_lines,slope<left_line_slope*(1-slope_factor))
                else:
                    left_lines = np.logical_and(left_lines,slope<0)

                if any(left_lines):
                    # Parse left lines based on max slope then maximum hood intercept
                    left_line = np.logical_and(left_lines,hood_col==np.max(hood_col[left_lines]))
                    left_line = np.logical_and(left_line,slope==np.min(slope[left_line]))

                    # Pull left line hood intercept and lines_mat index
                    left_hood_int = hood_col[left_line]
                    left_row_idx,left_col_idx = np.where(left_line)
                    left_upper_frame_int = upper_frame_col[left_line]

                    left_line_slope = slope[left_line]
                    left_line_slope_check = True
            except:
                left_lines = []
        else:
            left_lines = []

        if lines_matr.size>1:
            try:
                # Calculate slope/hood intercept for each line
                slope = np.divide(np.subtract(lines_matr[:,3],lines_matr[:,1]),\
                                  np.subtract(lines_matr[:,2],lines_matr[:,0]))
                b = np.subtract(lines_matr[:,1],np.multiply(slope,lines_matr[:,0]))
                hood_col = np.divide(np.subtract(upper_frame_row+height,b),slope)
                upper_frame_col = np.divide(np.subtract(upper_frame_row,b),slope)

                # Parse right lines based on hood intercepts and slope
                right_lines = np.logical_and(hood_col>right_hood_range[0],hood_col<right_hood_range[1])
                right_lines = np.logical_and(right_lines,upper_frame_col>right_upper_frame_range[0])
                right_lines = np.logical_and(right_lines,upper_frame_col<right_upper_frame_range[1])

                if right_line_slope_check:
                    right_lines = np.logical_and(right_lines,slope<right_line_slope*(1+slope_factor))
                    right_lines = np.logical_and(right_lines,slope>right_line_slope*(1-slope_factor))
                else:
                    right_lines = np.logical_and(right_lines,slope>0)

                if any(right_lines):
                    # Parse right lines on max slope then minimum hood intercept
                    right_line = np.logical_and(right_lines,hood_col==np.min(hood_col[right_lines]))
                    right_line = np.logical_and(right_line,slope==np.min(abs(slope[right_line])))

                    # Pull right line hood intercept and lines_mat index
                    right_hood_int = hood_col[right_line]
                    right_row_idx,right_col_idx = np.where(right_line)
                    right_upper_frame_int = upper_frame_col[right_line]

                    right_line_slope = slope[right_line]
                    right_line_slope_check = True
            except:
                right_lines = []
        else:
            right_lines = []
            
        if any((any(right_lines),any(left_lines))):

            reset_counter = 0
            resets_counter = 0

            # Both lines detected
            if all((any(right_lines),any(left_lines))):
                
                # Calculate middle of upper frame and middle of hood
                upper_frame_middle = np.mean([left_upper_frame_int,right_upper_frame_int])
                hood_middle = np.mean([left_hood_int,right_hood_int])

                if display_image==1:
                    # Draw lines on image for left and right lines
                    cv2.line(img,(lines_matl[left_row_idx,0],lines_matl[left_row_idx,1]),\
                             (lines_matl[left_row_idx,2],lines_matl[left_row_idx,3]),(0,0,255),2)
                    cv2.line(img,(lines_matr[right_row_idx,0],lines_matr[right_row_idx,1]),\
                             (lines_matr[right_row_idx,2],lines_matr[right_row_idx,3]),(0,255,0),2)
                
                # Update hood regions based on latest line data
                right_hood_range = [int(right_hood_int-capture_width*hood_range_perc/2),\
                                    int(right_hood_int+capture_width*hood_range_perc/2)]
                left_hood_range = [int(left_hood_int-capture_width*hood_range_perc/2),\
                                   int(left_hood_int+capture_width*hood_range_perc/2)]

                # Update upper frame regions based on latest line data
                right_upper_frame_range = [int(right_upper_frame_int-capture_width*upper_frame_range_perc/2),\
                                           int(right_upper_frame_int+capture_width*upper_frame_range_perc/2)]
                left_upper_frame_range = [int(left_upper_frame_int-capture_width*upper_frame_range_perc/2),\
                                          int(left_upper_frame_int+capture_width*upper_frame_range_perc/2)]

                # Update lane width and upper frame width using EMA
                lane_width = int(((right_hood_int-left_hood_int)-lane_width)*hood_mean_multiplier+lane_width)
                upper_frame_width = int(((right_upper_frame_int-left_upper_frame_int)-upper_frame_width)*upper_frame_mean_multiplier+upper_frame_width)

                # Print lane, upper_frame widths
                if display_image==1:
                    print 'Lane Width :: ', lane_width
                    print 'Upper Frame Width :: ', upper_frame_width

                # Calculate middle of lane from difference of upper frame intercepts
                lane_middle = np.append([lane_middle],int(upper_frame_middle))
                
                # Consider only last frames
                lane_middle = lane_middle[-1:]
                
                # Take mean of lane middles
                mean_lane_middle = np.mean(lane_middle)

            elif any(right_lines):

                if display_image==1:
                    # Draw line for on image right line
                    cv2.line(img,(lines_matr[right_row_idx,0],lines_matr[right_row_idx,1]),\
                             (lines_matr[right_row_idx,2],lines_matr[right_row_idx,3]),(0,255,0),2)

                # Calculate middle of upper frame and middle of hood
                upper_frame_middle = right_upper_frame_int-upper_frame_width/2
                hood_middle = right_hood_int-lane_width/2

                # Save data for left line intercepts
                left_upper_frame_int = right_upper_frame_int-upper_frame_width
                left_hood_int = right_hood_int-lane_width

                # Update right line hood search regions based on latest line data
                right_hood_range = [int(right_hood_int-capture_width*hood_range_perc/2),\
                                         int(right_hood_int+capture_width*hood_range_perc/2)]
                
                # Expand left line hood search regions based on right line if no left lines detected
                if left_hood_range_width < hood_range_max_width:
                    left_hood_range[0] = int(right_hood_int-lane_width-left_hood_range_width*(1+hood_range_update_perc)/2)
                    left_hood_range[1] = int(right_hood_int-lane_width+left_hood_range_width*(1+hood_range_update_perc)/2)
                else:
                    left_hood_range[0] = int(right_hood_int-lane_width-(left_hood_range_width)/2)
                    left_hood_range[1] = int(right_hood_int-lane_width+(left_hood_range_width)/2)

                # Update upper frame region based on latest line_data
                right_upper_frame_range = [int(right_upper_frame_int-capture_width*upper_frame_range_perc/2),\
                                           int(right_upper_frame_int+capture_width*upper_frame_range_perc/2)]

                # Expand left line upper frame region based on right line if no left lines detected
                if left_upper_frame_range_width < upper_frame_range_max_width:
                    left_upper_frame_range[0] = int(right_upper_frame_int-upper_frame_width-(left_upper_frame_range_width)*(1+upper_frame_range_update_perc)/2)
                    left_upper_frame_range[1] = int(right_upper_frame_int-upper_frame_width+(left_upper_frame_range_width)*(1+upper_frame_range_update_perc)/2)
                else:
                    left_upper_frame_range[0] = int(right_upper_frame_int-upper_frame_width-(left_upper_frame_range_width)/2)
                    left_upper_frame_range[1] = int(right_upper_frame_int-upper_frame_width+(left_upper_frame_range_width)/2)
                
                # Calculate middle of lane from difference of upper frame intercepts
                lane_middle = np.append([lane_middle],int(upper_frame_middle))
                
                # Consider only last frame
                lane_middle = lane_middle[-1:]

                # Reset left line slope
                left_line_slope_check = False                        
                
                # Take mean of lane middles
                mean_lane_middle = np.mean(lane_middle)

            elif any(left_lines):
                
                # Draw line for on image left line
                if display_image==1:
                    cv2.line(img,(lines_matl[left_row_idx,0],lines_matl[left_row_idx,1]),\
                             (lines_matl[left_row_idx,2],lines_matl[left_row_idx,3]),(0,0,255),2)

                # Calculate middle of upper frame and middle of hood
                upper_frame_middle = left_upper_frame_int+upper_frame_width/2
                hood_middle = left_hood_int+lane_width/2

                # Save data for right line intercepts
                right_upper_frame_int = left_upper_frame_int+upper_frame_width
                right_hood_int = left_hood_int+lane_width
                
                # Update left line hood search regions based on latest line data
                left_hood_range = [int(left_hood_int-capture_width*hood_range_perc/2),\
                                        int(left_hood_int+capture_width*hood_range_perc/2)]
                
                # Expand right line hood search regions based on left line if no right lines detected
                if right_hood_range_width < hood_range_max_width:
                    right_hood_range[0] = int(left_hood_int+lane_width-right_hood_range_width*(1+hood_range_update_perc)/2)
                    right_hood_range[1] = int(left_hood_int+lane_width+right_hood_range_width*(1+hood_range_update_perc)/2)
                else:
                    right_hood_range[0] = int(left_hood_int+lane_width-(right_hood_range_width)/2)
                    right_hood_range[1] = int(left_hood_int+lane_width+(right_hood_range_width)/2)                

                # Update upper frame region based on latest line_data
                left_upper_frame_range = [int(left_upper_frame_int-capture_width*upper_frame_range_perc/2),\
                                          int(left_upper_frame_int+capture_width*upper_frame_range_perc/2)]

                # Expand left line upper frame region based on right line if no left lines detected
                if right_upper_frame_range_width < upper_frame_range_max_width:
                    right_upper_frame_range[0] = int(left_upper_frame_int+upper_frame_width-(right_upper_frame_range_width)*(1+upper_frame_range_update_perc)/2)
                    right_upper_frame_range[1] = int(left_upper_frame_int+upper_frame_width+(right_upper_frame_range_width)*(1+upper_frame_range_update_perc)/2)
                else:
                    right_upper_frame_range[0] = int(left_upper_frame_int+upper_frame_width-(right_upper_frame_range_width)/2)
                    right_upper_frame_range[1] = int(left_upper_frame_int+upper_frame_width+(right_upper_frame_range_width)/2)
                
                # Calculate middle of lane from difference of upper frame intercepts
                lane_middle = np.append([lane_middle],int(upper_frame_middle))

                # Consider only last frame
                lane_middle = lane_middle[-1:]

                # Reset right line slope
                right_line_slope_check = False
                
                # Take mean of lane middles
                mean_lane_middle = np.mean(lane_middle)

            # Calculate Error and Error Times Using EMA
            error_current = ((mean_lane_middle-capture_width/2)-error[-1])*error_multiplier+error[-1]
            error = np.append([error],error_current)
            error = error[-2:]
            if test_video!=1:
                error_time = np.append([error_time],float(timer()))
            else:
                error_time = np.append([error_time],float(frame_number*1.0/60.0))
            error_time = error_time[-2:]

            a_error = mean_lane_middle-capture_width/2
            p_error = error[-1]
            i_error = np.mean(error)*(error_time[-1]-error_time[0])
            d_error = (error[-1]-error[0])/(error_time[-1]-error_time[0])
            
            # Determine number of stepper motor steps to take
            steps_to_take = int(round(Kp*p_error+Ki*i_error+Kd*d_error))

            if display_image==1:
                error_text = 'A: %d P:%d I:%d D:%d S:%d' %(a_error,p_error,i_error,d_error,steps_to_take)
                cv2.putText(img,error_text,(capture_width/3,capture_height-4),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),1)

                print 'A :: ',a_error
                print 'P :: ',p_error
                print 'I :: ',i_error
                print 'D :: ',d_error
                
        # No lines detected
        else:
            # Update line hood search regions
            if right_hood_range_width<hood_range_max_width:
                right_hood_range = [int(right_hood_range[0]-right_hood_range_width*hood_range_update_perc/2),\
                                    int(right_hood_range[1]+right_hood_range_width*hood_range_update_perc/2)]
            if left_hood_range_width<hood_range_max_width:  
                left_hood_range = [int(left_hood_range[0]-left_hood_range_width*hood_range_update_perc/2),\
                                   int(left_hood_range[1]+left_hood_range_width*hood_range_update_perc/2)]

            # Update upper frame regions
            if right_upper_frame_range_width<upper_frame_range_max_width:
                right_upper_frame_range = [int(right_upper_frame_range[0]-right_upper_frame_range_width*upper_frame_range_update_perc/2),\
                                           int(right_upper_frame_range[1]+right_upper_frame_range_width*upper_frame_range_update_perc/2)]
            if left_upper_frame_range_width<upper_frame_range_max_width:
                left_upper_frame_range = [int(left_upper_frame_range[0]-left_upper_frame_range_width*upper_frame_range_update_perc/2),\
                                          int(left_upper_frame_range[1]+left_upper_frame_range_width*upper_frame_range_update_perc/2)]

            # Don't allow hood ranges across middle or wider than one lane width from middle
            if right_hood_range[1]>capture_width/2*(1+hood_range_perc)+lane_width:
                right_hood_range[1] = capture_width/2*(1+hood_range_perc)+lane_width
            if right_hood_range[0]<capture_width/2*(1-hood_range_perc):
                right_hood_range[0] = capture_width/2*(1-hood_range_perc)
            if left_hood_range[0]<capture_width/2*(1-hood_range_perc)-lane_width:
                left_hood_range[0] = capture_width/2*(1-hood_range_perc)-lane_width
            if left_hood_range[1]>capture_width/2*(1+hood_range_perc):
                left_hood_range[1]=capture_width/2*(1+hood_range_perc)

            # Hold wheel position
            steps_to_take = int(0)

            # Reset both line slopes
            left_line_slope_check = False
            right_line_slope_check = False

            # Add to missed frame counter
            missed_frame_counter +=1
            
            # Record error and time to prevent I from exploding
            error_current = ((mean_lane_middle-capture_width/2)-error[-1])*error_multiplier+error[-1]
            error = np.append([error],error_current)
            error = error[-2:]
            if test_video!=1:
                error_time = np.append([error_time],float(timer()))
            else:
                error_time = np.append([error_time],float(frame_number*1.0/60.0))

            reset_counter+=1

            if display_image==1:
                print 'Missed Lines :: ', reset_counter

            if reset_counter>=reset_threshold:
                # Reset variables
                lane_width = lane_width_init
                upper_frame_width = upper_frame_width_init

                lane_middle = [capture_width/2]
                mean_lane_middle = int(capture_width/2)

                hood_middle = int(capture_width/2)

                right_hood_range = [int(mean_lane_middle+lane_width/2\
                                         -hood_range_perc/2*capture_width),\
                                         int(mean_lane_middle+lane_width/2\
                                         +hood_range_perc/2*capture_width)]
                left_hood_range = [int(mean_lane_middle-lane_width/2\
                                         -hood_range_perc/2*capture_width),\
                                         int(mean_lane_middle-lane_width/2\
                                         +hood_range_perc/2*capture_width)]

                right_upper_frame_range = [int(mean_lane_middle+upper_frame_width/2\
                                            -upper_frame_range_perc/2*capture_width),\
                                            int(mean_lane_middle+upper_frame_width/2\
                                            +upper_frame_range_perc/2*capture_width)]
                left_upper_frame_range = [int(mean_lane_middle-upper_frame_width/2\
                                            -upper_frame_range_perc/2*capture_width),\
                                            int(mean_lane_middle-upper_frame_width/2\
                                            +upper_frame_range_perc/2*capture_width)]

                right_hood_int = int(mean_lane_middle+lane_width/2)
                left_hood_int = int(mean_lane_middle-lane_width/2)

                right_upper_frame_int = int(mean_lane_middle+upper_frame_width/2)
                left_upper_frame_int = int(mean_lane_middle-upper_frame_width/2)

                right_line_slope_check = False
                left_line_slope_check = False

                reset_counter = 0
                print 'Reset Ranges!'
                time.sleep(0.25)

        if abs(steps_to_take)>min_steps_to_take:
            stepperStart = Thread(target=stepperFunc, args=(steps_to_take,))
            stepperStart.start()

        if display_image ==1:    
            # Draw lines on image for ranges
            cv2.line(img,(0,hood_row),(capture_width,hood_row),(0,0,255),1)
            cv2.line(img,(0,upper_frame_row),(capture_width,upper_frame_row),(0,0,255),1)
            cv2.line(img,(int(mean_lane_middle-upper_frame_width/2),upper_frame_row-5),(int(mean_lane_middle-upper_frame_width/2),upper_frame_row+5),(0,0,255),1)
            cv2.line(img,(int(mean_lane_middle+upper_frame_width/2),upper_frame_row-5),(int(mean_lane_middle+upper_frame_width/2),upper_frame_row+5),(0,255,0),1)
            cv2.line(img,(int(hood_middle-lane_width/2),hood_row-5),(int(hood_middle-lane_width/2),hood_row+5),(0,0,255),1)
            cv2.line(img,(int(hood_middle+lane_width/2),hood_row-5),(int(hood_middle+lane_width/2),hood_row+5),(0,0,255),1)
            cv2.line(img,(left_upper_frame_range[0],upper_frame_row-5),(left_upper_frame_range[0],upper_frame_row+5),(0,0,255),1)
            cv2.line(img,(left_upper_frame_range[1],upper_frame_row-5),(left_upper_frame_range[1],upper_frame_row+5),(0,0,255),1)
            cv2.line(img,(right_upper_frame_range[0],upper_frame_row-5),(right_upper_frame_range[0],upper_frame_row+5),(0,255,0),1)
            cv2.line(img,(right_upper_frame_range[1],upper_frame_row-5),(right_upper_frame_range[1],upper_frame_row+5),(0,255,0),1)

            cv2.circle(img,(points_matr[0,0],points_matr[0,1]),2,(0,255,0),2)
            cv2.circle(img,(points_matr[1,0],points_matr[1,1]),2,(0,255,0),2)
            cv2.circle(img,(points_matr[2,0],points_matr[2,1]),2,(0,255,0),2)
            cv2.circle(img,(points_matr[3,0],points_matr[3,1]),2,(0,255,0),2)

            cv2.circle(img,(points_matl[0,0],points_matl[0,1]),2,(0,0,255),2)
            cv2.circle(img,(points_matl[1,0],points_matl[1,1]),2,(0,0,255),2)
            cv2.circle(img,(points_matl[2,0],points_matl[2,1]),2,(0,0,255),2)
            cv2.circle(img,(points_matl[3,0],points_matl[3,1]),2,(0,0,255),2)

            # Draw line on image for correction amount, blue for right, white for left
            if p_error<0:
                    cv2.line(img,(capture_width/2,upper_frame_row),(int(p_error+capture_width/2),upper_frame_row),(255,255,255),2)
            elif p_error>0:
                    cv2.line(img,(capture_width/2,upper_frame_row),(int(p_error+capture_width/2),upper_frame_row),(255,0,0),2)

            # Display image, break if 'q' pressed
            cv2.namedWindow('Image')
            cv2.imshow('Image',img)
       
        # End timer for frame rate calculation
        end = float(timer())

        # Calculate frame rate
        frame_rate = 1/(end - start)

        # Pause to not exceed max_frame_rate
        if frame_rate>max_frame_rate:
            sleep_time = float(1/max_frame_rate)-float(1/frame_rate)
            time.sleep(sleep_time)

        # Recalculate frame rate
        end = float(timer())
        frame_rate = 1/(end - start)

        if display_image==1:
            # Display information in terminal
            print 'Steps to Take :: ',steps_to_take
            print 'Frame Rate :: ',frame_rate
            print 'Loop Time :: ', end-start
            print 'Missed Percent :: ',int(100*missed_frame_counter/frame_number)

    ##    if cv2.waitKey(0) & 0xFF == ord('q'):
    ##        break

        # Start Timer for Framerate
        start = float(timer())
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break    

atexit.register(exit_handler)
