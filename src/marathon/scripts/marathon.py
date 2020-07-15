#!/usr/bin/env python

import rospy
import numpy as np

from ImageProcess import *

from op3_utils.op3_utils import *
from op3_utils.vision import * 
from copy import copy
import sys
from os import path
from marker_detection import *

marker_list = ['left','straight','left','left','right',
            'left', 'straight','left','right','straight',
            'right','right','straight','right'] 
marker_list_counter = 0

class States:
    INIT = -1
    TRACK = 1
    FALL_FORWARD = 2
    FALL_BACKWARD = 3
    ALIGN_MARKER = 4 # reach end of the line
    DO_MARKER = 5 # follow marker
    FIND_MARKER = 6
    SEE_MARKER = 0

class Mode:
    VISION = 0
    MARATHON = 1

class DetectLineParams():
    yellow_lower = np.array([10,30,50])
    yellow_upper = np.array([30,255,255])
    red_lower1 = np.array([0,15,40])
    red_upper1 = np.array([7,255,255])
    red_lower2 = np.array([160,15,40])
    red_upper2 = np.array([180,255,255])

TR_X = 0
TR_Y = 10
TR_A = -9
TR_TIME = 6  # Time robot spends on turn right 90 degree with speeds above

def turn_right(robot,angle):
    
    robot.walkVelocities(x=TR_X,y=TR_Y,th=TR_A)
    robot.walkStart()
    print(angle)
    rospy.sleep(TR_TIME * angle / 90)
    robot.walkStop()
    rospy.sleep(2)


TL_X = 0
TL_Y = -15
TL_A = 12
TL_TIME = 6.0  # Time robot spends on turn left 90 degree with speeds above
def turn_left(robot,angle):
    
    robot.walkVelocities(x=TL_X,y=TL_Y,th=TL_A)
    robot.walkStart()
    rospy.sleep(TL_TIME * angle / 90)
    robot.walkStop()
    rospy.sleep(2)


STR_X = 15
STR_Y = 0
STR_A = 2

def walk_straight(robot):
    robot.walkVelocities(x=STR_X,y=STR_Y,th=STR_A)
    robot.walkStart()
    rospy.sleep(10)
    robot.walkStop()


detect_line_params = DetectLineParams()

pipeline_funcs = [get_image,
                  detect_single_color,
                  detect_line]
pipeline_args = [(),
                (cv2.COLOR_BGR2Lab,(np.array([170, 110, 170]),
                    np.array([230, 130, 190])),),
                (detect_line_params,)]

vision = VisionSystem(pipeline_funcs, pipeline_args)

rospy.Subscriber('/cv_camera/image_raw', Image, vision.read, queue_size=1)

rospy.init_node('marathon')
robot = Robot('marathon')
robot.walk_set_param_pub.publish(robot.walking_params[0])

rospy.sleep(4) # Take a break

def clickColor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        img = param
        rospy.loginfo('x: ' + str(x) + ' y: ' + str(y))
        rospy.loginfo(img[y,x])


if __name__ == '__main__':
   
    robot.setGeneralControlModule('walking_module')
    robot.setJointsControlModule(['head_pan','head_tilt'],['none'])
    rospy.sleep(1)
    rospy.loginfo('move gripper, head')
    robot.moveGripper(20,20)
    robot.moveHead(0,-0.7)

    rospy.loginfo(robot.PACKAGE)
    
    tickrate = 33
    rate = rospy.Rate(tickrate)
    
    current_head_pan = 0.0
    
    current_state = States.INIT
    current_mode = Mode().VISION

    while not rospy.is_shutdown():
        but = robot.get_pressed_button()
        if but == 'user':
            go_straight(robot)
        if(cv2.waitKey(1) == 27):
            robot.setGripperTorqueOff()
            break

        
        frame = vision.results[0]
        cv2.imshow('frame',frame)
        #cv2.setMouseCallback('frame',clickColor,frames['frame'])

        #cv2.imshow('lab',frames['lab'])
        #mask, result = vision.results[1], vision.status[1]

        line = vision.results[2][0]
        angle = vision.results[2][1]
        line_centre = vision.results[2][2]
        line_end = vision.results[2][3]
        cntr_area = vision.results[2][4]
        cv2.imshow('line',line)

        #rospy.loginfo('line_centre: {}'.format(line_centre))
        
        width = float(frame.shape[1])

        head_pan_delta = (width/2 - line_centre[0]) / width * 0.3
        #print('current head: {},  head_pan_delta: {}'.format(current_head_pan,head_pan_delta))
        if but == 'mode':
            rospy.loginfo('mode')
            current_mode ^= 1
            current_state = States.INIT
        
        if current_mode == 0:
            robot.moveHead(0,None)
            rospy.loginfo('vision_mode')
            robot.walkStop()

        elif current_mode == 1:
            if(line_centre[0]!=-1) and current_mode == 1 and cntr_area>300:
                head_pan_delta = min(max(head_pan_delta,-0.1),0.1)
                if current_state == States.TRACK or current_state == States.INIT:
                    current_head_pan += head_pan_delta
                    robot.moveHead(current_head_pan,None)
            


            if current_state == States.INIT:
                if but == 'start':
                    rospy.loginfo('start')
                    current_state = States.TRACK
                    robot.walkStart()
               
            elif current_state == States.TRACK:
                print('track')

                if line_end:
                    rospy.loginfo("It's the end")
                    robot.walkStop()
                    rospy.sleep(1.2)
                    tick_count = 0
                    current_state = States.SEE_MARKER

                    frame_counter = 0
                    marker_counter = 0
                    angles_list = []
                    fail_count = 0
                    continue

                # print('tracking.....')
                ratio = 0
                robot.walkVelocities(x=12,y=current_head_pan*ratio,
                        th=current_head_pan/DEGREE2RADIAN/2)

                # print(current_head_pan*ratio)


            elif current_state == States.SEE_MARKER:
                robot.moveHead(0,-1.0)
                img = vision.results[0]
                res = detectMarkers(img)
                frame_counter += 1

                if res is not None and len(res) > 0:
                    print('found marker')
                    marker, angle = res[0]
                    marker_counter += 1
                    angles_list.append(angle)
                else:
                    print('no marker found')

                if frame_counter >= 2 * tickrate:
                    if marker_counter >= 2:
                        align_angle = np.mean(angles_list)
                        current_state = States.ALIGN_MARKER
                        angles_list = []
                        frame_counter, marker_counter = 0, 0

                    else:
                        print('can\'t see marker')
                        robot.walkVelocities(x=-5,y=-10,th=3)
                        robot.walkStart()
                        rospy.sleep(1)
                        robot.walkStop()
                        frame_counter, marker_counter = 0, 0
                        angle_list = []
                        fail_count += 1
                        rospy.sleep(1)

                        if fail_count >= 3:
                            robot.moveHead(0,-0.7)
                            rospy.sleep(2)
                            current_state = States.DO_MARKER
                            marker = marker_list[marker_list_counter]


            elif current_state == States.ALIGN_MARKER:
                
                print('align: {}'.format(align_angle))
                if align_angle > 5:
                    print('align to left')
                    turn_left(robot,align_angle)
                    current_state = States.SEE_MARKER

                elif align_angle < -5:
                    print('align to right')
                    turn_right(robot,-align_angle)
                    current_state = States.SEE_MARKER
                else:
                    current_state = States.DO_MARKER

            elif current_state == States.DO_MARKER:
                print(marker)
                marker_list_counter += 1

                if marker == 'left':
                    turn_left(robot,90)
                    print('go')
                    walk_straight(robot)
                elif marker == 'right':
                    turn_right(robot,90)
                    print('go')
                    walk_straight(robot)
                elif marker == 'straight':
                    print('go')
                    walk_straight(robot)
                else:
                    put_some_undifined_function()
                    anyway_it_will_never_be_here()

                current_state = States.TRACK
                robot.moveHead(None,-0.7)
                STR_A = 1

                pass

            elif current_state == States.FIND_MARKER:
                robot.walkVelocities(x=0,y=0,th=0)
                robot.walkStart()
                rospy.sleep(1)
                robot.walkStop()
                current_state = States.ALIGN_MARKER
                pass
        rate.sleep()
        pass #while not rospy.is_shutdown()

