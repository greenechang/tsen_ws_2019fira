import cv2
import sys
import random
import numpy as np
import math
from math import sqrt, acos, atan2

ratio = 3
min_thres = 20

def click_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        img = param
        print('x: ' + str(x) + ' y: ' + str(y))
        print(img[y,x])


if __name__ == '__main__':

    argv = sys.argv

    cap = cv2.VideoCapture(argv[1])

    while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame,(0,0), fx=0.5, fy=0.5)
        frame_shape = frame.shape

        result_img = frame.copy()

        hsv_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        yellow_lower = (20,90,90)
        yellow_upper = (30,255,255)

        yellow_mask = cv2.inRange(hsv_frame,yellow_lower,yellow_upper)

        centre_dot = []
        roi_height = int(frame_shape[0] / 2)

        contour_color = (0,255,0)

        mask_near = yellow_mask[roi_height:,:]
        _, line_contours, _ = cv2.findContours(mask_near.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sorted_line_contours = []

        if len(line_contours) > 0:
            sorted_line_contours = sorted(line_contours, key=cv2.contourArea, reverse=True)[:3]
        
        if len(sorted_line_contours) > 0:
            line_cntr = sorted_line_contours[0]
            cntr_area = cv2.contourArea(line_cntr)

            if cntr_area > 0:
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

            angle = (90 - (math.asin(a_length / h_length) * 180 / math.pi)) * angle_sign
        
        elif len(centre_dot) == 1:
            line_centre_x, line_centre_y = centre_dot[0]

        print("Angle = " + str(angle))
        print("Line Pos = " + str([line_centre_x, line_centre_y]))


        cv2.imshow('rst',result_img)
        cv2.imshow('video', frame)
        cv2.waitKey(1)
