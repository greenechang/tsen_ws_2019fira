#!/usr/bin/env python

import os
import numpy as np
import yaml
import pandas as pd

import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Point


import rosnode

from sensor_msgs.msg import JointState

from op3_ros_function import *
from op3_kinematic import *



DATA_PATH = '/home/robotis/Tsen_ws/src/obstacle_run_slam/data/'
CONFIG_PATH = '/home/robotis/Tsen_ws/src/obstacle_run_slam/config/'
PACKAGE_PATH = '/home/robotis/Tsen_ws/src/obstacle_run_slam/'

DEGREE2RADIAN = np.pi / 180
gripper_action_pub = rospy.Publisher('/gripper_action',Point,queue_size = 5)
    
LINKS = np.array([[0       ,3  ,0   ,0       ],
                  [np.pi/2 ,0  ,0   ,0       ],
                  [0       ,11 ,0   ,0       ],
                  [0       ,11 ,0   ,0       ],
                  [-np.pi/2,0  ,0   ,-np.pi/2],
                  [np.pi/2 ,0  ,-3  ,np.pi   ],
                  [np.pi   ,3.7,16.5,np.pi/2 ],
                  [np.pi/2 ,1  ,0   ,np.pi/2 ],
                  [np.pi/2 ,5  ,1   ,np.pi/2 ]])
                  
OP3 = robot(9,LINKS)



class persp:
    p1 = np.array([-39,   0])
    p2 = np.array([ 39,   0])
    p3 = np.array([-39,21.9])
    p4 = np.array([ 39,21.9])
    
    pts1 = np.float32([[0  ,180],
                       [639,180],
                       [0  ,359],
                       [639,358]])


    def get_trans_matrix(self):
        rospy.loginfo(FRAME.shape)
        p11,p12 = cramer(OP3.T_u_c,self.p1)
        p21,p22 = cramer(OP3.T_u_c,self.p2)
        p31,p32 = cramer(OP3.T_u_c,self.p3)
        p41,p42 = cramer(OP3.T_u_c,self.p4)
        
        self.pts2 = np.float32([[p11,-p12],
                                [p21,-p22],
                                [p31,-p32],
                                [p41,-p42]])

        self.pts2 +=np.float32([[400,450],
                           [400,450],
                           [400,450],
                           [400,450]])
        rospy.loginfo('pts2')
        rospy.loginfo(self.pts2)

        M = cv2.getPerspectiveTransform(self.pts1,self.pts2)
        return M

PER = persp()
def present_joint_state_callback(msg):

    position = msg.position
    new_link = LINKS.copy()

    theta = np.float64([[-position[ 3]],
                        [ position[ 2]],
                        [-position[ 8]],
                        [ position[14]],
                        [ position[15]],
                        [-position[ 7]],
                        [ position[ 0]],
                        [ position[ 1]]])
    
    new_link[:-1,3:4] += theta

    OP3.set_link_params(new_link)

    p = np.array([[0],[0],[0],[1]])
    rospy.loginfo('Pcamera related to frame{0}:')
    rospy.loginfo(np.dot(OP3.trans_matrix,p))
    y,z=cramer(OP3.T_u_c,np.array([0,0]))
    rospy.loginfo('(y,z):')
    rospy.loginfo((y,z))
    

    pass




def status_callback(msg):
    '''Callback finction when receive a message from rostopic /robotis/status
    1. Enable walking module after Init Pose.

    '''
#   rospy.loginfo('Got Status Msg: '+msg.status_msg)

    #Active walking module after initialization
    if msg.status_msg == 'Finish Init Pose':
        module_name = 'walking_module'
        rospy.loginfo('Set ctrl module: ' + module_name +' result:'+ \
str(set_ctrl_module_client(module_name)))
        module_name = 'none'
        rospy.loginfo('Set joint ctrl module: ' + module_name +' result:'+ \
str(set_joint_ctrl_module_client('head_pan', module_name)))
        rospy.loginfo('Set joint ctrl module: ' + module_name +' result:'+ \
str(set_joint_ctrl_module_client('head_tilt', module_name)))
        rospy.Rate(1).sleep()
        
        gripper_action_pub.publish(45,45,0)
        move_head(0,-0.4)



FRAME = 123
BRIDGE = CvBridge()
MAP = np.zeros((450,800,3), np.uint8)
cv2.circle(MAP,(400,450),7,(0,255,255),-1)

def camera_callback(msg):
    try:
        cv2_img = BRIDGE.imgmsg_to_cv2(msg,"bgr8")
        global FRAME
        FRAME = cv2.resize(cv2_img,(0,0), fx=0.5, fy=0.5)
    except CvBridgeError, err:
        rospy.loginfo(err)
    else:
        frame_process(FRAME)

YELLOW_LOWER = np.array([10, 100, 90])
YELLOW_UPPER = np.array([30, 255, 255])

def frame_process(frame):
#    rospy.loginfo('got a frame')
    cv2.imshow('webcam',frame)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,YELLOW_LOWER,YELLOW_UPPER)
    cv2.imshow('yellow',yellow)
    cv2.imshow('hsv',hsv)

#    new_map = MAP
#    cv2.imshow('MAP',new_map)

#    for ix,iy in np.ndindex(yellow.shape):
#        rospy.loginfo(yellow[ix,iy] != 0)
    dst = cv2.warpPerspective(hsv,PER.get_trans_matrix(),(800,450))
    cv2.imshow('dst',dst)
    cv2.waitKey(33)


def plot_on_map(point):
    y,x,u = cramer(OP3.T_u_c,(point[0],point[1]))
    return


if __name__ == '__main__':

    #Rosnode init
    rospy.init_node('obstacle_run_slam')
    rospy.loginfo('Obstacle run node open.')

    #Subscriber setup    
    rospy.Subscriber('/robotis/present_joint_states', JointState, present_joint_state_callback,queue_size=1)
    rospy.Subscriber('/cv_camera/image_raw', Image, camera_callback)
    rospy.Subscriber("/robotis/status", StatusMsg, status_callback)
    
    rospy.loginfo(OP3.trans_matrix)
    rospy.loginfo(OP3.link_params)
    
    rospy.spin()
