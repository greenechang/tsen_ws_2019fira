#!/usr/bin/env python
'''obstacle run
'''
import os
import numpy as np
import yaml
import pandas as pd

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

#import rosparam
import rosnode

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam

from robotis_controller_msgs.srv import SetModule, SetJointModule
from op3_action_module_msgs.srv import IsRunning


from op3_ros_function import *
DATA_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/data/'
CONFIG_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/config/'
PACKAGE_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/'

DEGREE2RADIAN = np.pi / 180

MODE_LIST = ['READY',
             'SPIN',
             'MODE_NUM']




current_mode = 0
is_start = 0

#Publisher setup
walk_command_pub = rospy.Publisher('/robotis/walking/command', String, queue_size=1)
walk_set_param_pub = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=5)
action_page_pub = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=5)
base_init_pub = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=5)
set_joint_states_pub = rospy.Publisher('/robotis/set_joint_states',JointState, queue_size=5)

gripper_action_pub = rospy.Publisher('/gripper_action',Point,queue_size = 5)



WALKING_STRAIGHT = init_walking_param('param.yaml')

def button_callback(msg):
    '''Callback function when receive a message from rostpoic /robotis/open_cr/button

    To manage modes in obstacle_run by global variables "current_mode" and "is_start"

    Args:
        msg: std_msgs.msg.String
    Returns:
        none

    '''

    global is_start

    rospy.loginfo('button pressed: ' + msg.data)
    if msg.data == 'mode' or msg.data == 'mode_long':
        global current_mode
        is_start = 0
        current_mode += 1
        if current_mode == 2:
            current_mode = 0

        rospy.loginfo('current mode:' + MODE_LIST[current_mode])

    elif msg.data == 'start' or msg.data == 'start_long':
        is_start = 1
        page_num = 1
        action_page_pub.publish(1)
        while action_is_running_client():
            rospy.Rate(1000).sleep()
            rospy.loginfo('waiting other action')
        action_page_pub.publish(15)
        rospy.loginfo('Start mode: ' + MODE_LIST[current_mode])






def status_callback(msg):
    '''Callback finction when receive a message from rostopic /robotis/status
    1. Enable walking module after Init Pose.

    '''
#   rospy.loginfo('Got Status Msg: '+msg.status_msg)

    #Active walking module after initialization
    if msg.status_msg == 'Finish Init Pose':
        module_name = 'action_module'
        rospy.loginfo('Set ctrl module: ' + module_name +' result:'+ \
str(set_ctrl_module_client(module_name)))        
        action_page_pub.publish(76)     #lie down
        while action_is_running_client():
            rospy.Rate(1000).sleep()
        
        gripper_action_pub.publish(45,45,0)



BRIDGE = CvBridge()
def camera_callback(msg):
    ''' Callback function when receive a message from rostopic /cv_camera/image_raw

    Use CvBridge to convert image_raw to numpy array.
    Then call frame_process().

    Args:
        msg: sensor_msgs.msg.Image
    Returns:
        none
    '''
    try:
        cv2_img = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(cv2_img, (0, 0), fx=0.5, fy=0.5)
    except CvBridgeError, err:
        rospy.loginfo(err)
    else:
        frame_process(frame)




def frame_process(frame):
    '''Main process
    
    '''
    '''Main image processing area.

    Args:
        frame: A numpy array, image catched from webcam.
    '''

    hsv_img = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    
    vis = np.concatenate((hsv_img, gray_img, frame), axis=1)

    if cv2.waitKey(1) == 27:
        rospy.signal_shutdown('ESC is pressed!')




if __name__ == '__main__':



    cv2.waitKey(33)

    rospy.init_node('obstacle_run', disable_signals=True)


    rospy.loginfo('Obstacle run node open.')

    #Subscriber setup
    rospy.Subscriber("/robotis/open_cr/button", String, button_callback)
    rospy.Subscriber("/cv_camera/image_raw", Image, camera_callback)
    rospy.Subscriber("/robotis/status", StatusMsg, status_callback)

    # check if op3_manager is working
    while not rospy.is_shutdown():
        if '/op3_manager' in rosnode.get_node_names():
            rospy.loginfo('Found op3 manager')
            break
        else:
            rospy.loginfo('Waiting for op3 manager')
        rospy.Rate(20).sleep()

    global WALKING_STRAIGHT
    
    while not rospy.is_shutdown():
    
       
        
        # spin fan mode
        if current_mode == 1:
            action_page_pub.publish(77)     #wave hand
            
            while action_is_running_client():
                rospy.Rate(1000).sleep()            

    #end while not is_shutdown
        rospy.Rate(10).sleep()
    rospy.spin()
