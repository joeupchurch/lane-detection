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
from imutils.video.pivideostream import PiVideoStream
from threading import Thread

cap = cv2.VideoCapture('/home/pi/Code/Lane_Holding/testVideo201732919135.h264')
ret, frame = cap.read()

# Define camera height and width (defaults)
capture_height,capture_width,channels = frame.shape
print frame.shape

# Set PID Values for error loop
error = [0]
error_time = [float(timer())]
error_frames = 10
integral_error =[0]
Kp = 0.1#input('Kp :: ')
Ki = 0.5#input('Ki :: ')
Kd = 0.02#input('Kd :: ')
display_image = 1

# Set initial values for lane, line, and camera parameters
hood_range_perc = 0.10
hood_range_update_perc = 0.02
horizon_range_perc = 0.125
min_line_length_perc = 0.5

upper_frame_row = capture_height*1/12
horizon_row = capture_height*1/42
hood_row = capture_height*3/5
lane_width_init = int(758*capture_width/480)
upper_frame_width_init = int(82*capture_width/480)

slope_factor = 0.025

max_steps_to_take = 30
max_frame_rate = float(32)

# Initialize variables
mean_lane_width = []
mean_upper_frame_width = []

lane_width = lane_width_init
upper_frame_width = upper_frame_width_init

height = hood_row-upper_frame_row
horizon_range = [capture_width/2*(1-horizon_range_perc),capture_width/2*(1+horizon_range_perc)]

lane_middle = [capture_width/2]
mean_lane_middle = int(capture_width/2)
hood_middle = int(capture_width/2)
right_line_hood_range = [int(mean_lane_middle+lane_width/2\
                         -hood_range_perc/2*capture_width),\
                         int(mean_lane_middle+lane_width/2\
                         +hood_range_perc/2*capture_width)]
left_line_hood_range = [int(mean_lane_middle-lane_width/2\
                         -hood_range_perc/2*capture_width),\
                         int(mean_lane_middle-lane_width/2\
                         +hood_range_perc/2*capture_width)]

right_line_slope_check = False
left_line_slope_check = False
right_line_slope = 0.7
left_line_slope = -0.7

frame_number = 0
missed_frame_counter = 0

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
    vs.stop()
    gpio.output(gpio_pulse,False)
    gpio.output(gpio_direction,False)
    gpio.cleanup()
    print 'Lane Detection Script Ending'

def stepperFunc(steps_to_take):
    StepCounter = 0
    # Drive stepper motor
    if steps_to_take>0:
        gpio.output(19, True)
    else:
        gpio.output(19, False)
    if abs(steps_to_take)>max_steps_to_take:
        steps_to_take = max_steps_to_take
    while StepCounter<abs(steps_to_take):
        gpio.output(13, True)
        time.sleep(.1)
        gpio.output(13, False)
        time.sleep(.1)
        StepCounter +=1

# Set PID Values for error loop
error = [0]
error_time = [float(timer())]
error_frames = 10
integral_error =[0]
Kp = 0.1#input('Kp :: ')
Ki = 0.5#input('Ki :: ')
Kd = 0.02#input('Kd :: ')

# Start Timer for Framerate
start = timer()

while(cap.isOpened()):
    try:
        ret, frame = cap.read()
        
        print ' '
        # Index Frame Number
        frame_number+=1

        # Save frame to img
        img = frame

        right_line_hood_capture_range = right_line_hood_range[:]
        left_line_hood_capture_range = left_line_hood_range[:]

        # Limit ranges to capture window
        if right_line_hood_capture_range[1]>capture_width:
            right_line_hood_capture_range[1] = capture_width
        if left_line_hood_capture_range[0]<0:
            left_line_hood_capture_range[0] = 0

        # Apply additional window width to account for line slope
        right_add_width = int(height/right_line_slope)
        left_add_width = int(height/abs(left_line_slope))

        # Limit ranges across middle of lane
        if right_line_hood_capture_range[0]-right_add_width>int(capture_width/2*(1-hood_range_perc)):
            right_line_hood_capture_range[0] = right_line_hood_capture_range[0]-right_add_width
        else:
            right_add_width = right_line_hood_capture_range[0]-int(capture_width/2*(1-hood_range_perc))
            right_line_hood_capture_range[0] = int(capture_width/2*(1-hood_range_perc))
            
        if left_line_hood_capture_range[1]+left_add_width<int(capture_width/2*(1+hood_range_perc)):
            left_line_hood_capture_range[1] = left_line_hood_capture_range[1]+left_add_width
        else:
            left_add_width = int(capture_width/2*(1+hood_range_perc)) - left_line_hood_capture_range[1]
            left_line_hood_capture_range[1] = int(capture_width/2*(1+hood_range_perc))

        # Ensure always within capture window
        if right_line_hood_capture_range[1]<=right_line_hood_capture_range[0]:
            right_line_hood_capture_range[0] = int(right_line_hood_capture_range[1]-capture_width*hood_range_perc)
        if left_line_hood_capture_range[1]<=left_line_hood_capture_range[0]:
            left_line_hood_capture_range[1] = int(left_line_hood_capture_range[0]+capture_width*hood_range_perc)

        # Clip image outside of road
        truncl = img[upper_frame_row:hood_row,left_line_hood_capture_range[0]:left_line_hood_capture_range[1]]
        truncr = img[upper_frame_row:hood_row,right_line_hood_capture_range[0]:right_line_hood_capture_range[1]]
            
        # Convert BGR to Gray
        grayl = cv2.cvtColor(truncl,cv2.COLOR_BGR2GRAY)
        grayr = cv2.cvtColor(truncr,cv2.COLOR_BGR2GRAY)
        
        # Apply histogram equalization to gray image
        equl = cv2.equalizeHist(grayl)
        equr = cv2.equalizeHist(grayr)
        
        # Threshold for equalized image
        retl,threshl = cv2.threshold(equl,250,255,cv2.THRESH_BINARY)
        retr,threshr = cv2.threshold(equr,250,255,cv2.THRESH_BINARY)

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
                maxLineGap=height*.5)
        linesr = cv2.HoughLinesP(edgesr,1,np.pi/180,\
            int(height*min_line_length_perc/3),\
            minLineLength=int(height*min_line_length_perc),\
            maxLineGap=height*.2)
        
        lines_matl = np.matrix(linesl,'float')
        lines_matr = np.matrix(linesr,'float')

        if lines_matl.size>1:
            for line in linesl:
                x1,y1,x2,y2 = line[0]
                cv2.line(img,(x1+left_line_hood_capture_range[0],upper_frame_row+y1),\
                         (x2+left_line_hood_capture_range[0],upper_frame_row+y2),(255,255,255),2)

        if lines_matr.size>1:
            for line in linesr:
                x1,y1,x2,y2 = line[0]
                cv2.line(img,(x1+right_line_hood_capture_range[0],upper_frame_row+y1),\
                         (x2+right_line_hood_capture_range[0],upper_frame_row+y2),(255,255,255),2)
        
        if lines_matl.size>1:
            try:
                # Calculate slope, hood, and horizon intercept for each line
                slope = np.divide(np.subtract(lines_matl[:,3],lines_matl[:,1]),\
                                  np.subtract(lines_matl[:,2],lines_matl[:,0]))
                b = np.subtract(lines_matl[:,1]+upper_frame_row,np.multiply(slope,\
                                (lines_matl[:,0]+left_line_hood_capture_range[0])))
                hood_col = np.divide(np.subtract(upper_frame_row+height,b),slope)
                horizon_col = np.divide(np.subtract(horizon_row,b),slope)
                upper_frame_col = np.divide(np.subtract(upper_frame_row,b),slope)

                # Parse left lines based on hood/horizon intercepts and slope
                left_lines = np.logical_and(hood_col>left_line_hood_range[0],hood_col<left_line_hood_range[1])
                left_lines = np.logical_and(left_lines,horizon_col>horizon_range[0])
                left_lines = np.logical_and(left_lines,horizon_col<horizon_range[1])
                if left_line_slope_check:
                    left_lines = np.logical_and(left_lines,slope>left_line_slope*(1+slope_factor))
                    left_lines = np.logical_and(left_lines,slope<left_line_slope*(1-slope_factor))
                    print 'Slope Range :: ',left_line_slope*(1+slope_factor),left_line_slope*(1-slope_factor)
                else:
                    left_lines = np.logical_and(left_lines,slope<0)


                if any(left_lines):
                    # Parse left lines based on max slope then maximum hood intercept
                    left_line = np.logical_and(left_lines,hood_col==np.max(hood_col[left_lines]))
                    left_line = np.logical_and(left_line,slope==np.min(slope[left_line]))

                    # Pull left line hood/horizon intercept and lines_mat index
                    left_line_hood = hood_col[left_line]
                    left_line_horizon = horizon_col[left_line]
                    left_row_idx,left_col_idx = np.where(left_line)
                    left_line_upper_frame = upper_frame_col[left_line]

                    left_line_slope = slope[left_line]
                    print left_line_slope
                    left_line_slope_check = True
            except:
                left_lines = []
        else:
            left_lines = []

        if lines_matr.size>1:
            try:
                # Calculate slope, hood, and horizon intercept for each line
                slope = np.divide(np.subtract(lines_matr[:,3],lines_matr[:,1]),\
                                  np.subtract(lines_matr[:,2],lines_matr[:,0]))
                b = np.subtract(lines_matr[:,1]+upper_frame_row,np.multiply(slope,\
                                (lines_matr[:,0]+right_line_hood_capture_range[0])))
                hood_col = np.divide(np.subtract(upper_frame_row+height,b),slope)
                horizon_col = np.divide(np.subtract(horizon_row,b),slope)
                upper_frame_col = np.divide(np.subtract(upper_frame_row,b),slope)

                # Parse right lines based on hood/horizon intercepts and slope
                right_lines = np.logical_and(hood_col>right_line_hood_range[0],hood_col<right_line_hood_range[1])
                right_lines = np.logical_and(right_lines,horizon_col>horizon_range[0])
                right_lines = np.logical_and(right_lines,horizon_col<horizon_range[1])

                if right_line_slope_check:
                    right_lines = np.logical_and(right_lines,slope<right_line_slope*(1+slope_factor))
                    right_lines = np.logical_and(right_lines,slope>right_line_slope*(1-slope_factor))
                else:
                    right_lines = np.logical_and(right_lines,slope>0)

                if any(right_lines):
                    # Parse right lines on max slope then minimum hood intercept
                    right_line = np.logical_and(right_lines,hood_col==np.min(hood_col[right_lines]))
                    right_line = np.logical_and(right_line,slope==np.min(abs(slope[right_line])))

                    # Pull right line hood/horizon intercept and lines_mat index
                    right_line_hood = hood_col[right_line]
                    right_line_horizon = horizon_col[right_line]
                    right_row_idx,right_col_idx = np.where(right_line)
                    right_line_upper_frame = upper_frame_col[right_line]

                    right_line_slope = slope[right_line]
                    right_line_slope_check = True
            except:
                right_lines = []
        else:
            right_lines = []
            
        if any((any(right_lines),any(left_lines))):

            line_counter=0

            # Both lines detected
            if all((any(right_lines),any(left_lines))):
                
                # Calculate middle of upper frame and middle of hood
                upper_frame_middle = np.mean([left_line_upper_frame,right_line_upper_frame])
                hood_middle = np.mean([left_line_hood,right_line_hood])

                # Draw lines on image for left and right lines
                cv2.line(img,(lines_matl[left_row_idx,0]+left_line_hood_capture_range[0],\
                              lines_matl[left_row_idx,1]+upper_frame_row),\
                         (lines_matl[left_row_idx,2]+left_line_hood_capture_range[0],\
                          lines_matl[left_row_idx,3]+upper_frame_row),(0,0,255),2)
                cv2.line(img,(lines_matr[right_row_idx,0]+right_line_hood_capture_range[0],\
                              lines_matr[right_row_idx,1]+upper_frame_row),\
                         (lines_matr[right_row_idx,2]+right_line_hood_capture_range[0],\
                          lines_matr[right_row_idx,3]+upper_frame_row),(0,255,0),2)
                
                # Update right line hood search regions based on latest line data
                right_line_hood_range = [int(right_line_hood-capture_width*hood_range_perc/2),\
                                         int(right_line_hood+capture_width*hood_range_perc/2)]

                # Update left line hood search regions based on latest line data
                left_line_hood_range = [int(left_line_hood-capture_width*hood_range_perc/2),\
                                        int(left_line_hood+capture_width*hood_range_perc/2)]

                # Update lane width and upper frame width
                lane_width = int(right_line_hood-left_line_hood)
                upper_frame_width = int(right_line_upper_frame-left_line_upper_frame)

                # Calculate mean lane, upper_frame widths
                mean_lane_width = np.append([mean_lane_width],lane_width)
                mean_upper_frame_width = np.append([mean_upper_frame_width],upper_frame_width)

                # Calculate middle of lane from difference of upper frame intercepts
                lane_middle = np.append([lane_middle],int(upper_frame_middle))
                
                # Consider only last frames
                lane_middle = lane_middle[-1:]
                
                # Take mean of lane middles
                mean_lane_middle = int(np.mean(lane_middle))

            elif any(right_lines):
                
                # Draw line for on image right line
                cv2.line(img,(lines_matr[right_row_idx,0]+right_line_hood_capture_range[0],\
                              lines_matr[right_row_idx,1]+upper_frame_row),\
                         (lines_matr[right_row_idx,2]+right_line_hood_capture_range[0],\
                          lines_matr[right_row_idx,3]+upper_frame_row),(0,255,0),2)

                # Calculate middle of upper frame and middle of hood
                upper_frame_middle = right_line_upper_frame-upper_frame_width/2
                hood_middle = right_line_hood-lane_width/2

                # Update right line hood search regions based on latest line data
                right_line_hood_range = [int(right_line_hood-capture_width*hood_range_perc/2),\
                                         int(right_line_hood+capture_width*hood_range_perc/2)]
                
                # Expand left line hood search regions based on right line if no left lines detected
                last_hood_range = left_line_hood_range[1]-left_line_hood_range[0]
                left_line_hood_range[0] = int(np.mean([right_line_hood_range[1],right_line_hood_range[0]])-lane_width-\
                                              (last_hood_range)*(1+hood_range_update_perc)/2)
                left_line_hood_range[1] = int(np.mean([right_line_hood_range[1],right_line_hood_range[0]])-lane_width+\
                                              (last_hood_range)*(1+hood_range_update_perc)/2)
                
                # Calculate middle of lane from difference of upper frame intercepts
                lane_middle = np.append([lane_middle],int(upper_frame_middle))
                
                # Consider only last frame
                lane_middle = lane_middle[-1:]

                # Reset left line slope
                left_line_slope_check = False                        
                
                # Take mean of lane middles
                mean_lane_middle = int(np.mean(lane_middle))

            elif any(left_lines):
                
                # Draw line for on image left line
                cv2.line(img,(lines_matl[left_row_idx,0]+left_line_hood_capture_range[0],\
                              lines_matl[left_row_idx,1]+upper_frame_row),\
                         (lines_matl[left_row_idx,2]+left_line_hood_capture_range[0],\
                          lines_matl[left_row_idx,3]+upper_frame_row),(0,0,255),2)

                # Calculate middle of upper frame and middle of hood
                upper_frame_middle = left_line_upper_frame+upper_frame_width/2
                hood_middle = left_line_hood+lane_width/2
                
                # Update left line hood search regions based on latest line data
                left_line_hood_range = [int(left_line_hood-capture_width*hood_range_perc/2),\
                                        int(left_line_hood+capture_width*hood_range_perc/2)]
                
                # Expand right line hood search regions based on left line if no right lines detected
                last_hood_range = right_line_hood_range[1]-right_line_hood_range[0]
                right_line_hood_range[0] = int(np.mean([left_line_hood_range[1],left_line_hood_range[0]])+lane_width-\
                                              (last_hood_range)*(1+hood_range_update_perc)/2)
                right_line_hood_range[1] = int(np.mean([left_line_hood_range[1],left_line_hood_range[0]])+lane_width+\
                                              (last_hood_range)*(1+hood_range_update_perc)/2)
                
                # Calculate middle of lane from difference of upper frame intercepts
                lane_middle = np.append([lane_middle],int(upper_frame_middle))

                # Consider only last frame
                lane_middle = lane_middle[-1:]

                # Reset right line slope
                right_line_slope_check = False
                
                # Take mean of lane middles
                mean_lane_middle = int(np.mean(lane_middle))

            # Calculate Error and Error Times
            error = np.append([error],mean_lane_middle-capture_width/2)
            error = error[-2:]
            error_time = np.append([error_time],float(timer()))
            error_time = error_time[-2:]

            integral_error = np.append([integral_error],np.mean(error)*(error_time[-1]-error_time[0]))
            integral_error = integral_error[-error_frames:]
            
            # Determine number of stepper motor steps to take
            steps_to_take = int(round(Kp*error[-1]\
                +Ki*np.sum(integral_error)\
                +Kd*(error[-1]-error[0])/(error_time[-1]-error_time[0])))

            error_text = 'P:%d I:%d D:%d S:%d' %(error[-1],\
                    np.sum(integral_error),\
                    (error[1]-error[0])/(error_time[1]-error_time[0]),steps_to_take)
            cv2.putText(img,error_text,(capture_width/2-60,capture_height-4),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),1)

            print 'Frame Number :: ',frame_number
            print 'Error :: ',error
            print 'P :: ',error[-1]
            print 'I :: ',np.sum(integral_error)
            print 'D :: ',(error[1]-error[0])/(error_time[1]-error_time[0])
            print 'Error Time :: ',(error_time[1]-error_time[0])
            
        # No lines detected
        else:
            # Update line hood search regions based on last line data
            right_line_hood_range = [int(right_line_hood_range[0]-capture_width*hood_range_update_perc/2),\
                                     int(right_line_hood_range[1]+capture_width*hood_range_update_perc/2)]
            left_line_hood_range = [int(left_line_hood_range[0]-capture_width*hood_range_update_perc/2),\
                                    int(left_line_hood_range[1]+capture_width*hood_range_update_perc/2)]

            if right_line_hood_range[1]>capture_width/2*(1+hood_range_perc)+lane_width:
                right_line_hood_range[1] = capture_width/2*(1+hood_range_perc)+lane_width
            if right_line_hood_range[0]<capture_width/2*(1-hood_range_perc):
                right_line_hood_range[0] = capture_width/2*(1-hood_range_perc)
            if left_line_hood_range[0]<capture_width/2*(1-hood_range_perc)-lane_width:
                left_line_hood_range[0] = capture_width/2*(1-hood_range_perc)-lane_width
            if left_line_hood_range[1]>capture_width/2*(1+hood_range_perc):
                left_line_hood_range[1]=capture_width/2*(1+hood_range_perc)

            # Hold wheel position
            steps_to_take = int(0)

            # Reset both line slopes
            left_line_slope_check = False
            right_line_slope_check = False

            # Add to missed frame counter
            missed_frame_counter +=1
            
            # Record error time to prevent I from exploding
            error_time = np.append([error_time],float(timer()))
            error_time = error_time[-2:]

        if steps_to_take !=0:
            stepperStart = Thread(target=stepperFunc, args=(steps_to_take,))
            stepperStart.start()

        if display_image ==1:    
            # Draw lines on image for ranges
            #CAPTURE RANGES
            cv2.line(img,(0,hood_row),(capture_width,hood_row),(0,0,255),1)
            cv2.line(img,(0,horizon_row),(capture_width,horizon_row),(0,0,255),1)
            cv2.line(img,(0,upper_frame_row),(capture_width,upper_frame_row),(0,0,255),1)
            cv2.line(img,(int(left_line_hood_range[0]),upper_frame_row),(int(left_line_hood_range[0]),hood_row),(0,0,255),1)
            cv2.line(img,(int(left_line_hood_capture_range[1]),upper_frame_row),(int(left_line_hood_capture_range[1]),hood_row),(0,0,255),1)
            cv2.line(img,(int(left_line_hood_range[1]),hood_row-10),(int(left_line_hood_range[1]),hood_row),(0,0,255),1)
            cv2.line(img,(int(right_line_hood_range[0]),hood_row-10),(int(right_line_hood_range[0]),hood_row),(0,255,0),1)
            cv2.line(img,(int(right_line_hood_capture_range[0]),upper_frame_row),(int(right_line_hood_capture_range[0]),hood_row),(0,255,0),1)
            cv2.line(img,(int(right_line_hood_range[1]),upper_frame_row),(int(right_line_hood_range[1]),hood_row),(0,255,0),1)
            cv2.line(img,(mean_lane_middle-upper_frame_width/2,upper_frame_row-5),(mean_lane_middle-upper_frame_width/2,upper_frame_row+5),(0,0,255),1)
            cv2.line(img,(mean_lane_middle+upper_frame_width/2,upper_frame_row-5),(mean_lane_middle+upper_frame_width/2,upper_frame_row+5),(0,0,255),1)
            cv2.line(img,(int(hood_middle-lane_width/2),hood_row-5),(int(hood_middle-lane_width/2),hood_row+5),(0,0,255),1)
            cv2.line(img,(int(hood_middle+lane_width/2),hood_row-5),(int(hood_middle+lane_width/2),hood_row+5),(0,0,255),1)

            # Draw line on image for correction amount, blue for right, white for left
            if (mean_lane_middle-capture_width/2)<0:
                    cv2.line(img,(capture_width/2,upper_frame_row),(mean_lane_middle,upper_frame_row),(255,255,255),2)
            elif(mean_lane_middle-capture_width/2)>0:
                    cv2.line(img,(capture_width/2,upper_frame_row),(mean_lane_middle,upper_frame_row),(255,0,0),2)

            # Display image, break if 'q' pressed
##            cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
            cv2.imshow('Image',img)
       
        # End timer for frame rate calculation
        end = float(timer())

        # Calculate frame rate
        frame_rate = 1/(end - start)

        # Pause to not exceed max_frame_rate
        if frame_rate>max_frame_rate:
            sleep_time = float(1/max_frame_rate)-float(1/frame_rate)
            print 'Sleep Time :: ',sleep_time
            time.sleep(sleep_time)

        # Recalculate frame rate
        end = float(timer())
        frame_rate = 1/(end - start)

        # Display information in terminal
        print 'Steps to Take :: ',steps_to_take
        print 'Frame Rate :: ',frame_rate
        print 'Loop Time :: ', end-start

        # Start Timer for Framerate
        start = float(timer())
##        cv2.waitKey(0)
##        time.sleep(0.2)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break    

        print 'Missed Percent :: ',int(100*missed_frame_counter/frame_number)

    except:
        print 'Mean Lane Width :: ', np.mean(mean_lane_width)
        print 'Mean Upper Frame Width :: ',np.mean(mean_upper_frame_width)
        cv2.destroyAllWindows()
        cap.release()
        gpio.output(gpio_pulse,False)
        gpio.output(gpio_direction,False)
        gpio.cleanup()
        print 'Line Test Script Ending'
