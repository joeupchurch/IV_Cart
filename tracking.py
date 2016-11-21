# Import necessary packages
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from timeit import default_timer as timer

# Set camera settings
capture_width = 480
capture_height = 368
camera = PiCamera()

camera.resolution = (capture_width,capture_height)#(960,540)
camera.framerate = 32
camera.brightness = 0#0
camera.saturation = 0#100
camera.contrast = 100#100
rawCapture = PiRGBArray(camera,size=(capture_width,capture_height))

# Set threshold for missing lights
miss_thresh = 200
miss_counter = 0

# define range of red color in HSV
lower_red = np.array([-10,127,50])
upper_red = np.array([10,255,255])

# define range of green color in HSV
lower_green = np.array([50,127,10])
upper_green = np.array([70,255,255])

# Set initial light ranges
green_range = [0,capture_height,0, capture_width]
red_range = [0,capture_height,0, capture_width]
range_perc = 0.05
range_expand_perc = 0.005

# Allow camera time to warmup
time.sleep(0.5)

# Set point
centerr_set = []
centerg_set = []
center_set = []

for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):

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

    # Determine center of each light cluster
    if len(contoursg)>0:
        cntg = contoursg[0]
        (xg,yg),radiusg = cv2.minEnclosingCircle(cntg)
        centerg = (int(xg)+green_range[2],int(yg)+green_range[0])
        radiusg = int(radiusg)
        cv2.circle(image,centerg,radiusg,(0,255,0),2)

        green_range = [int(centerg[1]-capture_height*range_perc),\
                       int(centerg[1]+capture_height*range_perc),\
                       int(centerg[0]-capture_width*range_perc),\
                       int(centerg[0]+capture_width*range_perc)]

        if len(centerg_set)==0:
            centerg_set = centerg
            
    else:
        green_range = [int(green_range[0]-capture_height*range_expand_perc),\
                       int(green_range[1]+capture_height*range_expand_perc),\
                       int(green_range[2]-capture_width*range_expand_perc),\
                       int(green_range[3]+capture_width*range_expand_perc)]
        
    if len(contoursr)>0:
        cntr = contoursr[0]
        (xr,yr),radiusr = cv2.minEnclosingCircle(cntr)
        centerr = (int(xr)+red_range[2],int(yr)+red_range[0])
        radiusr = int(radiusr)
        cv2.circle(image,centerr,radiusr,(0,0,255),2)

        red_range = [int(centerr[1]-capture_height*range_perc),\
               int(centerr[1]+capture_height*range_perc),\
               int(centerr[0]-capture_width*range_perc),\
               int(centerr[0]+capture_width*range_perc)]

        if len(centerr_set)==0:
            centerr_set = centerr
        
    else:
        red_range = [int(red_range[0]-capture_height*range_expand_perc),\
               int(red_range[1]+capture_height*range_expand_perc),\
               int(red_range[2]-capture_width*range_expand_perc),\
               int(red_range[3]+capture_width*range_expand_perc)]
        
    if  all((len(contoursr)>0,len(contoursg)>0)):
        xc = int(np.mean((centerg[0],centerr[0])))
        yc = int(np.mean((centerg[1],centerr[1])))
        cv2.circle(image,(xc,yc),2,(255,0,0),2)
        miss_counter = 0
        if len(center_set)==0:
            center_set = (xc,yc)
            print center_set
        else:
            print 'Error :: ', yc-center_set[1]    
    else:
        miss_counter += 1

    if green_range[0]<0:
        green_range[0]=0
    if green_range[1]>capture_height:
        green_range[1]=capture_height
    if green_range[2]<0:
        green_range[2]=0
    if green_range[3]>capture_width:
        green_range[3]=capture_width

    if red_range[0]<0:
        red_range[0]=0
    if red_range[1]>capture_height:
        red_range[1]=capture_height
    if red_range[2]<0:
        red_range[2]=0
    if red_range[3]>capture_width:
        red_range[3]=capture_width

    if miss_counter > miss_thresh:
        print 'Lost signal lights'
        break

    cv2.rectangle(image,(green_range[2],green_range[0]),\
                  (green_range[3],green_range[1]),(0,255,0),2,8,0)
    cv2.rectangle(image,(red_range[2],red_range[0]),\
                  (red_range[3],red_range[1]),(0,0,255),2,8,0)
    
    cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
    cv2.imshow('Image',image)

    end = timer()
##    print 'Frame Rate :: ',1/(end-start)

    # Clear capture for next frame
    rawCapture.truncate(0)

    # End if 'q' key is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
