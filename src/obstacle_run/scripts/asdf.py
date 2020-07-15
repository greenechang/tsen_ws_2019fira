#!/usr/bin/env python

import cv2
import numpy as np

#HSV color range
BLUE_LOWER = np.array([85, 80, 55])
BLUE_UPPER = np.array([125, 255, 255])


BLUE_Y_LOWER = np.array([0, 0, 10])
BLUE_Y_UPPER = np.array([20, 100, 120])

BLUE_Y2_LOWER = np.array([160, 0, 60])
BLUE_Y2_UPPER = np.array([180, 100, 160])

YELLOW_LOWER = np.array([10, 100, 90])
YELLOW_UPPER = np.array([30, 230, 255])

RED_LOWER1 = np.array([0, 90, 10])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 90, 10])
RED_UPPER2 = np.array([180, 255, 255])


def frame_process(frame):
    #cv2.imshow('webcam',frame)
    '''Main process
    
    '''
    '''Main image processing area.

    Args:
        frame: A numpy array, image catched from webcam.
    '''

    hsv_img = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    kernel = np.ones((3,3),np.uint8)
    
    global mask_blue
    mask_blue = cv2.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
    mask_y_blue = cv2.inRange(hsv_img, BLUE_Y_LOWER, BLUE_Y_UPPER)
    mask_y2_blue = cv2.inRange(hsv_img, BLUE_Y2_LOWER, BLUE_Y2_UPPER)
        

    mask_blue = np.bitwise_or(np.bitwise_or(mask_blue,mask_y_blue),mask_y2_blue)
    mask_blue = cv2.dilate(mask_blue, np.ones((3, 3)))
    mask_blue = cv2.erode(mask_blue, np.ones((3, 3)))
    mask_blue = cv2.erode(mask_blue, np.ones((3, 3)),iterations = 1)
    mask_blue = cv2.dilate(mask_blue, np.ones((3, 3)),iterations = 1)
 
    mask_yellow = cv2.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)
    mask_yellow = cv2.erode(mask_yellow, np.ones((3, 3)))
    mask_yellow = cv2.dilate(mask_yellow, np.ones((3, 3)))
    mask_yellow = cv2.dilate(mask_yellow, np.ones((3, 3)))
    mask_yellow = cv2.erode(mask_yellow, np.ones((3, 3)))


    mask_red1 = cv2.inRange(hsv_img, RED_LOWER1, RED_UPPER1)
    mask_red2 = cv2.inRange(hsv_img, RED_LOWER2, RED_UPPER2)
    mask_red = np.bitwise_or(mask_red1, mask_red2)
    mask_red = cv2.erode(mask_red, np.ones((3, 3)))
    mask_red = cv2.dilate(mask_red, np.ones((3, 3)))
    mask_red = cv2.dilate(mask_red, np.ones((3, 3)))
    mask_red = cv2.erode(mask_red, np.ones((3, 3)))

    global roi_near
    roi_near = mask_blue[int(mask_blue.shape[0] * 0.625):, :]

    side_height = 0.3
    side_width = 0.3
    side_margin = 0.1

    roi_left_yellow = mask_yellow[int(mask_yellow.shape[0] * (1-side_height)):, int(mask_yellow.shape[1] * side_margin):int(mask_yellow.shape[1] * (side_width+side_margin))]
    roi_left = gray_img[int(mask_yellow.shape[0] * (1-side_height)):, int(mask_yellow.shape[1] * side_margin):int(mask_yellow.shape[1] * (side_width+side_margin))]
    roi_right_yellow = mask_yellow[int(mask_yellow.shape[0] * (1-side_height)):, int(mask_yellow.shape[1] * (1-side_width-side_margin)):int(mask_yellow.shape[1] * (1-side_margin))]
    roi_right = gray_img[int(mask_yellow.shape[0] * (1-side_height)):, int(mask_yellow.shape[1] * (1-side_width-side_margin)):int(mask_yellow.shape[1] * (1-side_margin))]

    roi_near_yellow = mask_yellow[int(mask_yellow.shape[0] * 0.8):, int(mask_yellow.shape[1] * 0.25):int(mask_yellow.shape[1] * 0.75)]


    roi_red = mask_red[int(mask_red.shape[0] * 0.24):int(mask_red.shape[0] * 0.28),\
int(mask_red.shape[1] * 0.485):int(mask_red.shape[1] * 0.515)]
    cv2.rectangle(frame, (int(mask_red.shape[1] * 0.485), int(mask_red.shape[0] * 0.24)),\
(int(mask_red.shape[1] * 0.515), int(mask_red.shape[0] * 0.28)), (0, 0, 255), 1)


    roi_yellow = np.concatenate((np.concatenate((roi_left,roi_right), axis = 1), np.concatenate(((roi_left_yellow,roi_right_yellow)),axis = 1)),axis = 0)
    cv2.line(roi_yellow,(0,roi_yellow.shape[0]/2),(roi_yellow.shape[1],roi_yellow.shape[0]/2),255,1)
    cv2.line(roi_yellow,(roi_yellow.shape[1]/2,0),(roi_yellow.shape[1]/2,roi_yellow.shape[0]),255,1)

    gray_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    
    vis = np.concatenate((hsv_img, gray_img, frame), axis=1)

    global lava
    lava = cv2.dilate(mask_blue, np.ones((200, 200))) 

    rbl = np.concatenate((mask_red, mask_blue, lava), axis = 1)
    cv2.line(rbl,(mask_red.shape[1],0),(mask_red.shape[1],mask_red.shape[0]),255,1)
    cv2.line(rbl,(mask_red.shape[1]*2,0),(mask_red.shape[1]*2,mask_red.shape[0]),255,1)

    #cv2.imshow('vis', vis)
    #cv2.imshow("near", roi_near)
    #cv2.imshow("near yellow",roi_near_yellow)
    #cv2.imshow('yellow',roi_yellow)
    #cv2.imshow('red, blue, lava', rbl)

    #cv2.moveWindow('vis',500,0)
    #cv2.moveWindow("near",500,240)
    #cv2.moveWindow("near yellow",500,360)
    #cv2.moveWindow('yellow',500,540)
    #cv2.moveWindow('red, blue, lava',500,720)

    global state

    global encounter_wall
    global encounter_gate
    global left_wall_area
    global right_wall_area
    global encounter_left_border9
    global encounter_right_border
    global encounter_yellow

    if white_area(roi_red) > 0.7:
        encounter_gate = 1
    else:
        encounter_gate = 0

    if white_area(roi_near) > 0.2:
        encounter_wall = 1
    else:
        encounter_wall = 0

    if white_area(roi_left_yellow) > 0.3:
        encounter_left_border = 1
    else:
        encounter_left_border = 0

    if white_area(roi_right_yellow) > 0.3:
        encounter_right_border = 1
    else:
        encounter_right_border = 0
        
    if white_area(roi_near_yellow) > 0.1:
        encounter_yellow = 1
    else:
        encounter_yellow = 0

    if cv2.waitKey(1) == 27:
        rospy.signal_shutdown('ESC is pressed!')


def white_area(frame):
    '''Calculate white(value == 255) pixels in a "1-channel" image, in ratio.
    '''
    pd_img = pd.DataFrame(data=frame[:, :])
    return float(pd_img.sum().sum())/(255*frame.shape[0]*frame.shape[1])

def lava_safe(lava):

    lava_pd = pd.DataFrame(data=lava[:, :])
    rospy.loginfo('lava: ' + str(lava_pd.sum()[lava.shape[1]*0.5]/255))
    rospy.loginfo(lava.shape[0])
    if (lava_pd.sum()[lava.shape[1]*0.5]/255) < (lava.shape[0] * 0.7):
        return True
    else:
        return False
    

