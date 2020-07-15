#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import random as rng
import math
from collections import deque

def get_image(img):
    return img, True

def detect_single_color(img,cvtParam,params):

    lower = params[0]
    upper = params[1]
    img = cv2.cvtColor(img,cvtParam)
    mask = cv2.inRange(img, lower, upper)
    

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5)))
    
    roi_near = mask[int(mask.shape[0]*0.5):,:]
    mask_canny = cv2.Canny(roi_near, 10, 20)
    _, contours, _ = cv2.findContours(mask_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    try:
        mu = [None]*len(contours)
        for i in range(len(contours)):
            mu[i] = cv2.moment(contours[i])
            pass

        mc = [None]*len(contours)
        for i in range(len(contours)):
            mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))

        drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3),dtype = np.uint8)

        for i in range(len(contours)):
            color = (rng.radint(0,256), rng.radint(0,256), rng.radint(0,256))
            cv2.drawContours(drawing, contours, i, color, 2)
            cv2.circle(drawing, (int(mc[i][0]), int(mc[i][1])), 4, color, 3)
    except:
        #print('no contours')
        pass
    return mask, True

def detect_line(img,params):
   
    line_end = False
    hsv_params = params
    yellow_lower = hsv_params.yellow_lower
    yellow_upper = hsv_params.yellow_upper
    red_lower1 = hsv_params.red_lower1
    red_upper1 = hsv_params.red_upper1
    red_lower2 = hsv_params.red_lower2
    red_upper2 = hsv_params.red_upper2

    img_shape = img.shape

    result_img = img.copy()

    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
    
    red_mask1 = cv2.inRange(hsv_img, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv_img, red_lower2, red_upper2)
    
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    centre_dot = []
    roi_height = int(img_shape[0] / 2)

    contour_color = (0,255,0)


    mask = cv2.bitwise_or(red_mask,yellow_mask)
    mask_near = mask[roi_height:-30,:]
    _, line_contours, _ = cv2.findContours(mask_near.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sorted_line_contours = []

    if len(line_contours) > 0:
        sorted_line_contours = sorted(line_contours, key=cv2.contourArea, reverse=True)[:3]
    cntr_area = 0
    if len(sorted_line_contours) > 0:
        line_cntr = sorted_line_contours[0]
        cntr_area = cv2.contourArea(line_cntr)

        if cntr_area > 0:

            
            moving_average_area = contour_average_area(cntr_area)
            # print(moving_average_area)
            if moving_average_area < 1500:
                line_end = True


            
            box_x, box_y, box_w, box_h = cv2.boundingRect(line_cntr)
            box_y = roi_height + box_y
            boline_centre_x_x = int(box_x + box_w / 2)
            boline_centre_x_y = int(box_y + box_h / 2)
            cv2.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), contour_color, 1)
            centreDot = (boline_centre_x_x, boline_centre_x_y)
            cv2.circle(result_img, centreDot, 5, (255, 0, 0), -1)
            centre_dot.append(centreDot)
    
    x1 = 0
    x2 = 0
    angle = -888
    line_centre_x = -1
    line_centre_y = -1

    if len(centre_dot) > 0:
        cv2.arrowedLine(result_img, centre_dot[-1], centre_dot[0], (0,255,255), 2)
        x1, y1 = centre_dot[-1]
        x2, y2 = centre_dot[0]
        c_dot = (x2,y1)
        cv2.circle(result_img,c_dot,5,(255,0,0),-1)
        h_length = np.linalg.norm(np.array(centre_dot[-1])-np.array(centre_dot[0]))
        a_length = np.linalg.norm(np.array(centre_dot[0])-np.array(c_dot))

        line_centre_x = x1
        line_centre_y = y1

        angle_sign = 1

        if x2 >= x1:
            angle_sign = 1
        elif x2 < x1:
            angle_sign = -1

        angle = (90 - (math.asin(a_length / h_length) * 180 / np.pi)) * angle_sign
    
    elif len(centre_dot) == 1:
        line_centre_x, line_centre_y = centre_dot[0]

    return (result_img, angle, [line_centre_x, line_centre_y], line_end, cntr_area), True

def marker_detect(img,templates):
    res = [None] * len(templates)
    for index, template in enumerate(templates):
        lt, rb, max_val, res = template_matching(img,template)
        res[index] = [lt, rb, max_val, res]
        pass
    return res, True

def template_matching(img,template):
    result = img.copy()
    match = cv2.matchTemplate(img,templaye,cv2.TM_CCOEFF_NORMED)
    match_result = cv2.minMaxLoc(match)

    max_val = match_result[1]
    left_top = match_result[3] # depends on which method used

    right_bot = (left_top[0]+template.shape[0],left_top[1]+template.shape[1])
    cv2.rectangle(result,left_top,right_bot,(128,0,0),3)

    return left_top, right_bot, max_val, result


que = deque((0,0))
def contour_average_area(area,length=10):
    
    
    que.appendleft(area)
    if len(que) > length:
        que.pop()
        print(sum(que)/length)
        return sum(que)/length
        
    else:
        print('too few elements')
        







