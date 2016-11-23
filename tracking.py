# Import necessary packages
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from timeit import default_timer as timer
import RPi.GPIO as gpio
import atexit
import Adafruit_PCA9685
import math

# Set camera settings
capture_width = 480
capture_height = 368
camera = PiCamera()
camera.resolution = (capture_width,capture_height)#(960,540)
camera.framerate = 24
camera.brightness = 0#0
camera.saturation = 0#100
camera.contrast = 100#100
rawCapture = PiRGBArray(camera,size=(capture_width,capture_height))

print 'here'

# Set GPIO settings
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio_on_led = 4
gpio.setup(gpio_on_led,gpio.OUT)
gpio.output(gpio_on_led,True)

# Set threshold for missing lights
miss_thresh = 200
miss_counter = 0
single_miss_thresh = 10
single_miss_counter = 0

# Define set point parameters
max_set_dist = 170
min_set_dist = 100
max_vert_sep = 30

# define range of red color in HSV
lower_red = np.array([-10,127,50])
upper_red = np.array([10,255,255])

# define range of green color in HSV
lower_green = np.array([50,127,10])
upper_green = np.array([70,255,255])

# Set initial light ranges
green_range = [0,capture_height,0, capture_width]
red_range = [0,capture_height,0, capture_width]
range_perc_height = 0.03
range_expand_perc_height = 0.03
range_perc_width= 0.20
range_expand_perc_width = 0.1

# Initialize the PCA9685 using default address
pwm = Adafruit_PCA9685.PCA9685()

# Set pwm frequency (Hz)
pwm.set_pwm_freq(100)
pwm_dc = [0]
pwm_dc_rot = [0]
min_pwm = 0.2
max_pwm = 0.5
max_pwm_rot = min_pwm
ramp_step = 200
ramp_time = 0.01
jump_time = 0.01
jump_steps = 5
jump_steps_ramp = 4
jump_top_pwm = 0.5

# Set I2C locations for motors
left_forward = 12
left_backward = 13
right_forward = 15
right_backward = 14

# Set PID Values for error loop
error = [0]
error_time = [timer()]
error_rot = [0]
error_time_rot = [timer()]
Kp_rot = 0.005#input('Kp (rot) :: ')
Ki_rot = 0#input('Ki (rot):: ')
Kd_rot = 0#input('Kd (rot):: ' )
Kp = .005#input('Kp :: ') # 0.010 oscillates
Ki = .075#input('Ki :: ')
Kd = .01#input('Kd :: ' )

# Define exit parameters
def exit_handler():
##    cv2.destroyAllWindows()
    gpio.output(gpio_on_led,False)
    gpio.cleanup()
    pwm.set_pwm(left_forward,0,0)
    pwm.set_pwm(right_forward,0,0)
    pwm.set_pwm(left_backward,0,0)
    pwm.set_pwm(right_backward,0,0)
    print 'Tracking Script Ended (Error)'

# Allow camera time to warmup
time.sleep(0.5)

# Set point
centerr_set = []
centerg_set = []
center_set = []

for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    # Start timer for frame rate
    start = timer()

    # Take each frame as an array
    image = frame.array
    
    # Truncate image for green search area
    truncg = image[green_range[0]:green_range[1],green_range[2]:green_range[3]]
    truncr = image[red_range[0]:red_range[1],red_range[2]:red_range[3]]

    # Convert BGR to Gray
    grayg = cv2.cvtColor(truncg,cv2.COLOR_BGR2GRAY)
    grayr = cv2.cvtColor(truncr,cv2.COLOR_BGR2GRAY)

    # Equalize brightness
    equg = cv2.equalizeHist(grayg)
    equr = cv2.equalizeHist(grayr)

    #  Create threshold mask
    retg,threshg = cv2.threshold(equg,252,255,cv2.THRESH_BINARY)
    retr,threshr = cv2.threshold(equr,252,255,cv2.THRESH_BINARY)

    # Dilate LED areas
    kernel = np.ones((21,21),np.uint8)
    graymaskg = cv2.dilate(threshg,kernel,iterations=1)
    graymaskr = cv2.dilate(threshr,kernel,iterations=1)

    # Mask Original Image based on brightness
    resg = cv2.bitwise_and(truncg,truncg,mask=graymaskg)
    resr = cv2.bitwise_and(truncr,truncr,mask=graymaskr)

    # Convert brightness-masked image to hsv
    hsvg = cv2.cvtColor(resg, cv2.COLOR_BGR2HSV)
    hsvr = cv2.cvtColor(resr, cv2.COLOR_BGR2HSV)
    
    # Threshold the HSV image to get only blue colors
    maskg = cv2.inRange(hsvg, lower_green, upper_green)
    maskr = cv2.inRange(hsvr, lower_red, upper_red)
    
    # Define circular kernel
    kernel_circ = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

    # Morphological Close to fill in LED area
    closingg = cv2.morphologyEx(maskg,cv2.MORPH_GRADIENT, kernel_circ)
    closingr = cv2.morphologyEx(maskr,cv2.MORPH_GRADIENT, kernel_circ)

    # Find contours for each mask
    im2g,contoursg,hierarchyg = cv2.findContours(closingg,cv2.RETR_EXTERNAL,\
                                              cv2.CHAIN_APPROX_NONE)
    im2r,contoursr,hierarchyr = cv2.findContours(closingr,cv2.RETR_EXTERNAL,\
                                              cv2.CHAIN_APPROX_NONE)

    # If green cluster detected
    if len(contoursg)>0:
        cntg = contoursg[0]
        
        # Find center and radius of detected contours
        (xg,yg),radiusg = cv2.minEnclosingCircle(cntg)
        centerg = (int(xg)+green_range[2],int(yg)+green_range[0])
        radiusg = int(radiusg)
        
        # Draw circle on image
        cv2.circle(image,centerg,radiusg,(0,255,0),2)
        
        # Redefine search range based on most recent center
        green_range = [int(centerg[1]-capture_height*range_perc_height),\
                       int(centerg[1]+capture_height*range_perc_height),\
                       int(centerg[0]-capture_width*range_perc_width),\
                       int(centerg[0]+capture_width*range_perc_width)]

    # If red cluster detected 
    if len(contoursr)>0:
        cntr = contoursr[0]
        
        # Find center and radius of detected contours
        (xr,yr),radiusr = cv2.minEnclosingCircle(cntr)
        centerr = (int(xr)+red_range[2],int(yr)+red_range[0])
        radiusr = int(radiusr)
        
        # Draw circle on image
        cv2.circle(image,centerr,radiusr,(0,0,255),2)
        
        # Redefine search range based on most recent center
        red_range = [int(centerr[1]-capture_height*range_perc_height),\
               int(centerr[1]+capture_height*range_perc_height),\
               int(centerr[0]-capture_width*range_perc_width),\
               int(centerr[0]+capture_width*range_perc_width)]
        
    # If both clusters detected or one light and below threshold
    if  any((all((len(contoursr)>0,len(contoursg)>0)),\
            all((single_miss_counter<single_miss_thresh,\
                 any((len(contoursr)>0,len(contoursg)>0)),\
                 len(center_set)!=0)))):

        if not all((len(contoursr)>0,len(contoursg)>0)):
            single_miss_counter +=1
            if len(contoursg)>0:
                centerr = (centerg[0]-horz_sep,centerg[1]-vert_sep)
                red_range = [int(centerr[1]-capture_height*range_perc_height),\
                   int(centerr[1]+capture_height*range_perc_height),\
                   int(centerr[0]-capture_width*range_perc_width),\
                   int(centerr[0]+capture_width*range_perc_width)]
            if len(contoursr)>0:
                centerg = (centerr[0]+horz_sep,centerr[1]+vert_sep)
                green_range = [int(centerg[1]-capture_height*range_perc_height),\
                   int(centerg[1]+capture_height*range_perc_height),\
                   int(centerg[0]-capture_width*range_perc_width),\
                   int(centerg[0]+capture_width*range_perc_width)]
                
            print 'Single Miss :: ', single_miss_counter
        else:
            single_miss_counter = 0
        # Define midpoint of line between two clusters
        xc = int(np.mean((centerg[0],centerr[0])))
        yc = int(np.mean((centerg[1],centerr[1])))
        cv2.circle(image,(xc,yc),2,(255,0,0),8)

        gr_dist = math.sqrt(math.pow(centerg[0]-centerr[0],2)+math.pow(centerg[1]-centerr[1],2))
        vert_sep = centerg[1]-centerr[1]
        horz_sep = centerg[0]-centerr[0]

        if all((gr_dist>min_set_dist,gr_dist<max_set_dist)):
        
            # Reset missed lights counter
            miss_counter = 0

            if all((len(center_set)==0,abs(vert_sep<max_vert_sep))):
                center_set = (xc,yc)
                vert_sep_set = vert_sep
                gr_dist_set = gr_dist
                print center_set
                
            # Else calculate error as difference from setpoint (only y direction)
            elif len(center_set)>0:
                error = np.append([error],center_set[1]-yc)
                error = error[-2:]
                error_rot = np.append([error_rot],xc-center_set[0])
                error_rot = error_rot[-2:]
                error_time = np.append([error_time],timer())
                error_time = error_time[-2:]

                # Calculate duty cycle as function of error
                pwm_dc = np.append([pwm_dc],Kp*error[-1]\
                         +Ki*np.mean(error)*(error_time[1]-error_time[0])\
                         +Kd*(error[1]-error[0])/(error_time[1]-error_time[0]))
                pwm_dc_rot = np.append([pwm_dc_rot],Kp_rot*error_rot[-1]\
                         +Ki_rot*np.mean(error_rot)*(error_time[1]-error_time[0])\
                         +Kd_rot*(error_rot[1]-error_rot[0])/(error_time[1]-error_time[0]))

                # Set duty cycle to max pwm if greater than
                if abs(pwm_dc[-1])>max_pwm:
                    pwm_dc[-1] = max_pwm*pwm_dc[-1]/abs(pwm_dc[-1])
                if abs(pwm_dc_rot[-1])>max_pwm_rot:
                    pwm_dc_rot[-1] = max_pwm_rot*pwm_dc_rot[-1]/abs(pwm_dc_rot[-1])
                # Set duty cycle to 0 if less than min pwm
                if abs(pwm_dc[-1])<min_pwm:
                    pwm_dc[-1] = 0
                pwm_dc = pwm_dc[-2:]
                pwm_dc_rot = pwm_dc_rot[-2:]
                
                print 'PWM :: ',pwm_dc
                print 'Rotation :: ', (pwm_dc[-1]+pwm_dc_rot[-1],pwm_dc[-1]-pwm_dc_rot[-1],)
                
                # Drive PWM for both motors opposite
                if pwm_dc[-1]>0:
                    if pwm_dc[0]<0:
                        for ramp_factor in range(int(pwm_dc[0]*4096),0,ramp_step):
                            pwm.set_pwm(right_backward,0,abs(ramp_factor))
                            pwm.set_pwm(left_backward,0,abs(ramp_factor))
                            time.sleep(ramp_time)
                        for jump_pwm in range(0,int(jump_top_pwm*4096),int(jump_top_pwm*4096/jump_steps_ramp)):
                            pwm.set_pwm(right_forward,0,jump_pwm)
                            pwm.set_pwm(left_forward,0,jump_pwm)
                            time.sleep(jump_time)
                        for a in range(0,jump_steps,1):
                            time.sleep(jump_time)
                        pwm.set_pwm(right_forward,0,int((pwm_dc[-1]-pwm_dc_rot[-1])*4096))
                        pwm.set_pwm(left_forward,0,int((pwm_dc[-1]+pwm_dc_rot[-1])*4096))
                    elif pwm_dc[-1]>=pwm_dc[0]:
                        if pwm_dc[0]==0:
                            for jump_pwm in range(0,int(jump_top_pwm*4096),int(jump_top_pwm*4096/jump_steps_ramp)):
                                pwm.set_pwm(right_forward,0,jump_pwm)
                                pwm.set_pwm(left_forward,0,jump_pwm)
                                time.sleep(jump_time)
                            for a in range(0,jump_steps,1):
                                time.sleep(jump_time)
                            pwm.set_pwm(right_forward,0,int((pwm_dc[-1]-pwm_dc_rot[-1])*4096))
                            pwm.set_pwm(left_forward,0,int((pwm_dc[-1]+pwm_dc_rot[-1])*4096))
                        else:
                            for ramp_factor in range(int(pwm_dc[0]*4096),int(pwm_dc[-1]*4096),ramp_step):
                                pwm.set_pwm(right_forward,0,ramp_factor)
                                pwm.set_pwm(left_forward,0,ramp_factor)
                                time.sleep(ramp_time)
                            pwm.set_pwm(right_forward,0,int((pwm_dc[-1]-pwm_dc_rot[-1])*4096))
                            pwm.set_pwm(left_forward,0,int((pwm_dc[-1]+pwm_dc_rot[-1])*4096))

                    else:
                        for ramp_factor in range(int(pwm_dc[0]*4096),int(pwm_dc[-1]*4096),-ramp_step):
                            pwm.set_pwm(right_forward,0,ramp_factor)
                            pwm.set_pwm(left_forward,0,ramp_factor)
                            time.sleep(ramp_time)
                        pwm.set_pwm(right_forward,0,int((pwm_dc[-1]-pwm_dc_rot[-1])*4096))
                        pwm.set_pwm(left_forward,0,int((pwm_dc[-1]+pwm_dc_rot[-1])*4096))
                # Drive brackward
                elif pwm_dc[-1]<0:
                    if pwm_dc[0]>0:
                        for ramp_factor in range(int(pwm_dc[0]*4096),0,-ramp_step):
                            pwm.set_pwm(right_forward,0,ramp_factor)
                            pwm.set_pwm(left_forward,0,ramp_factor)
                            time.sleep(ramp_time)
                        for jump_pwm in range(0,int(jump_top_pwm*4096),int(jump_top_pwm*4096/jump_steps_ramp)):
                            pwm.set_pwm(right_backward,0,jump_pwm)
                            pwm.set_pwm(left_backward,0,jump_pwm)
                            time.sleep(jump_time)
                        for a in range(0,jump_steps,1):
                            time.sleep(jump_time)
                        pwm.set_pwm(right_backward,0,abs(int((pwm_dc[-1]-pwm_dc_rot[-1])*4096)))
                        pwm.set_pwm(left_backward,0,abs(int((pwm_dc[-1]+pwm_dc_rot[-1])*4096)))
                    elif pwm_dc[-1]<=pwm_dc[0]:
                        if pwm_dc[0]==0:
                            for jump_pwm in range(0,int(jump_top_pwm*4096),int(jump_top_pwm*4096/jump_steps_ramp)):
                                pwm.set_pwm(right_backward,0,jump_pwm)
                                pwm.set_pwm(left_backward,0,jump_pwm)
                                time.sleep(jump_time)
                            for a in range(0,jump_steps,1):
                                time.sleep(jump_time)
                            pwm.set_pwm(right_backward,0,abs(int((pwm_dc[-1]-pwm_dc_rot[-1])*4096)))
                            pwm.set_pwm(left_backward,0,abs(int((pwm_dc[-1]+pwm_dc_rot[-1])*4096)))
                        else:
                            for ramp_factor in range(int(pwm_dc[0]*4096),int(4096*pwm_dc[-1]),-ramp_step):
                                pwm.set_pwm(right_backward,0,abs(ramp_factor))
                                pwm.set_pwm(left_backward,0,abs(ramp_factor))
                                time.sleep(ramp_time)
                            pwm.set_pwm(right_backward,0,abs(int((pwm_dc[-1]-pwm_dc_rot[-1])*4096)))
                            pwm.set_pwm(left_backward,0,abs(int((pwm_dc[-1]+pwm_dc_rot[-1])*4096)))
                    else:
                        for ramp_factor in range(int(4096*pwm_dc_rot[0]),int(4096*pwm_dc[-1]),ramp_step):
                            pwm.set_pwm(right_backward,0,abs(ramp_factor))
                            pwm.set_pwm(left_backward,0,abs(ramp_factor))
                            time.sleep(ramp_time)
                        pwm.set_pwm(right_backward,0,abs(int((pwm_dc[-1]-pwm_dc_rot[-1])*4096)))
                        pwm.set_pwm(left_backward,0,abs(int((pwm_dc[-1]+pwm_dc_rot[-1])*4096)))
                else:
                    pwm.set_pwm(left_backward,0,0)
                    pwm.set_pwm(right_backward,0,0)
                    pwm.set_pwm(left_forward,0,0)
                    pwm.set_pwm(right_forward,0,0)
                    
        # Incorrect lights detected
        else:
            green_range[0]=0
            green_range[1]=capture_height
            green_range[2]=0
            green_range[3]=capture_width

            red_range[0]=0
            red_range[1]=capture_height
            red_range[2]=0
            red_range[3]=capture_width
            
    # If both lights not detected
    else:
        # Increment miss counter
        miss_counter += 1
        print 'Miss'

        pwm_dc[-1] = 0
        
        # Stop PWMs
        pwm.set_pwm(left_backward,0,0)
        pwm.set_pwm(right_backward,0,0)
        pwm.set_pwm(left_forward,0,0)
        pwm.set_pwm(right_forward,0,0)

        # Expand search ranges
        green_range = [int(green_range[0]-capture_height*range_expand_perc_height),\
                       int(green_range[1]+capture_height*range_expand_perc_height),\
                       int(green_range[2]-capture_width*range_expand_perc_width),\
                       int(green_range[3]+capture_width*range_expand_perc_width)]
        red_range = [int(red_range[0]-capture_height*range_expand_perc_height),\
               int(red_range[1]+capture_height*range_expand_perc_height),\
               int(red_range[2]-capture_width*range_expand_perc_width),\
               int(red_range[3]+capture_width*range_expand_perc_width)]

    # Prevent green range from exceeding captured frame
    if green_range[0]<0:
        green_range[0]=0
    if green_range[1]>capture_height:
        green_range[1]=capture_height
    if green_range[2]<0:
        green_range[2]=0
    if green_range[3]>capture_width:
        green_range[3]=capture_width

    # Prevent red range from exceeding captured frame
    if red_range[0]<0:
        red_range[0]=0
    if red_range[1]>capture_height:
        red_range[1]=capture_height
    if red_range[2]<0:
        red_range[2]=0
    if red_range[3]>capture_width:
        red_range[3]=capture_width

    # End loop if lights missing
    if miss_counter > miss_thresh:
        print 'Lost signal lights'
        break
    
    # Draw search rectangles
    cv2.rectangle(image,(green_range[2],green_range[0]),\
                  (green_range[3],green_range[1]),(0,255,0),2,8,0)
    cv2.rectangle(image,(red_range[2],red_range[0]),\
                  (red_range[3],red_range[1]),(0,0,255),2,8,0)
 
    # Display image
    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
    cv2.imshow('Image',image)

    # End timer for frame rate
    end = timer()
##    print 'Frame Rate :: ',1/(end-start)

    # Clear capture for next frame
    rawCapture.truncate(0)

    # End if 'q' key is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Invoke exit handler
atexit.register(exit_handler)
