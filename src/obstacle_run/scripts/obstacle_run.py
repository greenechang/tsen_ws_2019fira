#!/usr/bin/env python
'''obstacle run
'''

import os
import numpy as np
import yaml
import pandas as pd

import rospy
import rosnode
from pygame import mixer

import cv2
from cv_bridge import CvBridge, CvBridgeError

import pygame as pg

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam

from robotis_controller_msgs.srv import SetModule, SetJointModule
from op3_action_module_msgs.srv import IsRunning

#from op3_ros_function import *
from op3_utils.op3_utils import * 
from op3_utils.vision import VisionSystem
from ImageProcess import *

DEBUG = True

pipeline_funcs = [convertColor,frame_process]
pipeline_args = [(()),(())] 

vision = VisionSystem(pipeline_funcs,pipeline_args)

rospy.Subscriber('/cv_camera/image_raw', Image, vision.read, queue_size=1)

# ros init
rospy.init_node('obstacle_run_node') 
robot = Robot('obstacle_run')
mixer.init()
mixer.music.load('/home/robotis/sound/windowsXP_startup.mp3')
mixer.music.play()

# check if op3_manager is working
while not rospy.is_shutdown():
    if '/op3_manager' in rosnode.get_node_names():
        rospy.loginfo('Found op3 manager')
        break
    else:
        rospy.loginfo('Waiting for op3 manager')
    rospy.Rate(20).sleep()


DEGREE2RADIAN = np.pi / 180

MODE_LIST = ['READY',
             'OBSTACLE_RUN',
             'MODE_NUM']

class Mode():
    READY = 0 
    OBSTACLE_RUN = 1
    MODE_NUM = 2
mode = Mode()
STR_X = 8
STR_Y = 0
STR_A = -0.3

SL_X = -2
SL_Y = 12
SL_A = 1 

SR_X = -2
SR_Y = -12
SR_A = -0

TR_X = -2
TR_Y = -0.000
TR_A = -3

TL_X = -2 
TL_Y = 0.000
TL_A = 3


is_start = 0



class State():
    INIT = -1
    READY = 0
    WALKING = 1
    LEFT_WALL = 2
    RIGHT_WALL = 3
    GATE = 4
    LEFT_BORDER = 5
    RIGHT_BORDER = 6
    LOOK_LEFT = 7
    LOOK_RIGHT = 8
    GATE_SHIFT_LEFT = 9
    GATE_SHIFT_RIGHT = 10
    CREEP = 11

current_state = State.INIT

encounter_wall = 0
encounter_gate = 0
left_wall_area = 0
right_wall_area = 0
encounter_left_border = 0
encounter_right_border = 0
encounter_yellow = 0


mask_blue = 123 
frame = 123
lava = 123
roi_near = 123



if __name__ == '__main__':


    rospy.sleep(4)
    
    robot.setJointsControlModule(['head_pan','head_tilt'],['none'])
    robot.setGeneralControlModule('walking_module')

    robot.moveHead(0,-0.75)
    robot.moveGripper(35,35)
    cv2.waitKey(33)
    current_state = State.READY
    current_mode = 0
    is_start = False

    while not rospy.is_shutdown():
        pressed_button = robot.get_pressed_button()
        
        if(pressed_button == 'mode'):
            current_mode += 1
            is_start = False
            mixer.music.play()
            if current_mode == mode.MODE_NUM:
                current_mode = 0
        
        elif pressed_button == 'start':
            is_start ^= True
            if current_mode == 1:
                mixer.music.load('/home/robotis/sound/rocky.mp3')
                mixer.music.play(-1)
        
        
        
        res = vision.results[1]
        
        rospy.loginfo('Mode: {}'.format(current_mode))
        rospy.loginfo('State: {}'.format(current_state))    
        
        vis, roi_near, roi_yellow, lava = res['vis'], res['roi_near'], res['roi_yellow'], res['lava']
        rbl = res['rbl']
        roi_near_yellow = res['roi_near_yellow']
        
        gate, wall, l_border, r_border = res['gate'], res['wall'], res['l_border'], res['r_border']
        yellow = res['yellow']
        mask_blue = res['mask_blue']
        rospy.loginfo('gate: {} wall: {} l_bor: {} r_bor: {}'.format(gate,wall,l_border,r_border))
        
        if DEBUG: 
            cv2.imshow('vis',vis)
            cv2.imshow('roi_near',roi_near)
            cv2.imshow('roi_yellow',roi_yellow)
            cv2.imshow('lava',lava)        
            cv2.imshow('rbl',rbl)
            cv2.imshow('near_yellow',roi_near_yellow)
            if cv2.waitKey(33) == 27:
                robot.setGripperTorqueOff()
                break
        
        # obstacle run mode
        if current_mode == 1 and is_start:
            rospy.loginfo('Obstacle run mode.')
            
            if current_state == State.READY:
                rospy.loginfo('take a break.')
                current_state = State.WALKING

            elif current_state == State.WALKING:
            
                robot.walkVelocities(STR_X,0,0)
                
                if gate:
                    rospy.loginfo('Encounter a gate, stop!')
                    robot.walkStop()
                    rospy.Rate(1).sleep()
                    current_state = State.GATE
                    
                elif r_border:
                    rospy.loginfo('Encounter right border, stop!')
                    current_state = State.RIGHT_BORDER
                    
                elif l_border:
                    rospy.loginfo('Encounter left border, stop!')
                    current_state = State.LEFT_BORDER

                elif wall:
                    rospy.loginfo('Encounter a wall, stop!')
                    robot.walkStop()
                    rospy.Rate(1).sleep()
                    ###############move head to left
                    robot.moveHead(0.65,-0.75)
                    rospy.sleep(1)

                    current_state = State.LOOK_LEFT

                else:
                    robot.moveHead(0,-0.75)

                    robot.walkVelocities(STR_X,STR_Y,STR_A)
                    robot.walkStart()


            elif current_state == State.LOOK_LEFT:
                rospy.Rate(1).sleep()

                left_wall_area = white_area(mask_blue)
                if yellow:
                    rospy.loginfo('encounter yellow!')
                    current_state = State.LEFT_BORDER
                    robot.moveHead(0,-0.75)
                else:
                    current_state = State.LOOK_RIGHT
                     #################move head to right
                    robot.moveHead(-0.75,-0.75)

                    rospy.Rate(1).sleep()

            elif current_state == State.LOOK_RIGHT:
                rospy.Rate(1).sleep()
                right_wall_area = white_area(mask_blue)
                print(left_wall_area, right_wall_area)
                ################move head to mid
                robot.moveHead(0,-0.75)


                if yellow:
                    rospy.loginfo('encounter yellow!')
                    current_state = State.RIGHT_BORDER
                    robot.moveHead(0,-0.75)
                    
                elif left_wall_area > right_wall_area:
                    current_state = State.LEFT_WALL
                else:
                    current_state = State.RIGHT_WALL

                left_wall_area = 0
                right_wall_area = 0
                
            elif current_state == State.RIGHT_WALL:
                robot.moveHead(0,-0.75)
                
                robot.walkVelocities(SL_X,SL_Y,SL_A)
                robot.walkStart()

                if lava_safe(lava[int(lava.shape[0]*0.65):lava.shape[0],:]):
                    current_state = State.READY


               #########danger
            elif current_state == State.LEFT_WALL:
                robot.moveHead(0,-0.75)
                robot.walkVelocities(SR_X,SR_Y,SR_A)
                robot.walkStart()
                
                if lava_safe(lava[int(lava.shape[0]*0.65):lava.shape[0],:]):
                    current_state = State.READY

            elif current_state == State.LEFT_BORDER:
                robot.walkVelocities(TR_X,TR_Y,TR_A)
                robot.walkStart()
                
                rospy.Rate(1).sleep()
                current_state = State.WALKING

            elif current_state == State.RIGHT_BORDER:
                robot.walkVelocities(TL_X,TL_Y,TL_A)
                robot.walkStart()

                rospy.Rate(1).sleep()
                current_state = State.WALKING

            elif current_state == State.GATE:
                robot.walkStop()
                
                roi_pd = pd.DataFrame(data=roi_near[:, int(roi_near.shape[1]*0):int(roi_near.shape[1]*1)])
                
                if roi_pd.sum()[0] > 0:
                    current_state = State.GATE_SHIFT_RIGHT
                elif roi_pd.sum()[lava.shape[0]-1] > 0:
                    current_state = State.GATE_SHIFT_LEFT
                else:
                    current_state = State.CREEP
            
            elif current_state == State.GATE_SHIFT_RIGHT:
                robot.walkVelocities(SR_X,SR_Y,SR_A)
                robot.walkStart()

                lava = cv2.dilate(lava,np.ones((20,20)))
                
                if lava_safe(lava[int(lava.shape[0]*0.7):lava.shape[0],:]):
                    current_state = State.CREEP                

            elif current_state == State.GATE_SHIFT_LEFT:
                robot.walkVelocities(SL_X,SL_Y,SL_A)
                robot.walkStart()

                lava = cv2.dilate(lava,np.ones((20,20)))

                if lava_safe(lava[int(lava.shape[0]*0.7):lava.shape[0],:]):
                    current_state = State.CREEP
            elif current_state == State.CREEP:
                robot.setJointsControlModule(HEAD,['none'])
                robot.setJointsControlModule(WHOLE_BODY_WITHOUT_HEAD,['action_module'])
                robot.moveHead(0,1.57)

                robot.playMotion(33)     #lie down
                robot.playMotion(36)     #creep
                robot.playMotion(36)     #creep
                robot.playMotion(36)     #creep
                robot.playMotion(36)     #creep
                robot.playMotion(36)     #creep
                robot.playMotion(36)     #creep
                robot.playMotion(36)     #creep
                robot.playMotion(32)     #stand up


                robot.setJointsControlModule(['head_pan','head_tilt'],['none'])
                robot.setGeneralControlModule('walking_module')               
                    
                current_state = State.WALKING
                robot.moveHead(0,-0.75)
        else:
            robot.walkStop()
            rospy.loginfo('READY_MODE')
            
        rospy.Rate(10).sleep()


        pass#end while not is_shutdown

    rospy.spin()
