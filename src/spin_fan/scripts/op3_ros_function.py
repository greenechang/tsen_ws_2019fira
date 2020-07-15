#!/usr/bin/env python

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

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam

from robotis_controller_msgs.srv import SetModule, SetJointModule
from op3_action_module_msgs.srv import IsRunning

walk_command_pub = rospy.Publisher('/robotis/walking/command',String,queue_size=5)
CONFIG_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/config/'
DEGREE2RADIAN = np.pi / 180


set_joint_states_pub = rospy.Publisher('/robotis/set_joint_states',JointState, queue_size=5)

def walk_command(command):
    walk_command_pub.publish(command)
    walk_command_pub.publish(command)
    walk_command_pub.publish(command)
    

def init_walking_param(file_name='param.yaml'):
    '''set walking param function
    Parameters besides x, y, a_move_amp are loaded from param.yaml file.
    '''
    yaml_file = open(CONFIG_PATH + file_name)

    yf = yaml.load(yaml_file)
    walking_param_ = WalkingParam()

    # parse movement time
    walking_param_.init_x_offset = yf["x_offset"]
    walking_param_.init_y_offset = yf["y_offset"]
    walking_param_.init_z_offset = yf["z_offset"]
    walking_param_.init_roll_offset = yf["roll_offset"] * DEGREE2RADIAN
    walking_param_.init_pitch_offset = yf["pitch_offset"] * DEGREE2RADIAN
    walking_param_.init_yaw_offset = yf["yaw_offset"] * DEGREE2RADIAN
    walking_param_.hip_pitch_offset = yf["hip_pitch_offset"] * DEGREE2RADIAN
    # time
    walking_param_.period_time = yf["period_time"] * 0.001    # ms -> s
    walking_param_.dsp_ratio = yf["dsp_ratio"]
    walking_param_.step_fb_ratio = yf["step_forward_back_ratio"]
    # walking
    walking_param_.x_move_amplitude = 0
    walking_param_.y_move_amplitude = 0
    walking_param_.z_move_amplitude = yf["foot_height"]
    walking_param_.angle_move_amplitude = 0

    # balance
    walking_param_.balance_hip_roll_gain = yf["balance_hip_roll_gain"]
    walking_param_.balance_knee_gain = yf["balance_knee_gain"]
    walking_param_.balance_ankle_roll_gain = yf["balance_ankle_roll_gain"]
    walking_param_.balance_ankle_pitch_gain = yf["balance_ankle_pitch_gain"]
    walking_param_.y_swap_amplitude = yf["swing_right_left"]
    walking_param_.z_swap_amplitude = yf["swing_top_down"]
    walking_param_.pelvis_offset = yf["pelvis_offset"] * DEGREE2RADIAN
    walking_param_.arm_swing_gain = yf["arm_swing_gain"]
    # gain
    walking_param_.p_gain = yf["p_gain"]
    walking_param_.i_gain = yf["i_gain"]
    walking_param_.d_gain = yf["d_gain"]

    walking_param_.move_aim_on = False
    walking_param_.balance_enable = False

    return walking_param_

WALKING_STRAIGHT = init_walking_param('param.yaml')
WALKING_SHIFT = init_walking_param('shift_walk.yaml')

def move_joint(joint_name,position):
    msg = JointState()
    msg.name = [joint_name]
    msg.position = [position]
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)


def set_ctrl_module_client(module_name):
    '''Client of rosservice /robotis/set_present_ctrl_modules

    '''

    try:
        set_module = rospy.ServiceProxy('robotis/set_present_ctrl_modules', SetModule)
        res = set_module(module_name)
        return res.result
    except rospy.ServiceException, err:
        rospy.loginfo(err)

def set_joint_ctrl_module_client(joint_name, module_name):
    '''Client of rosservice /robotis/set_present_ctrl_modules

    '''

    try:
        set_joint_module = rospy.ServiceProxy('robotis/set_present_joint_ctrl_modules',\
 SetJointModule)
        res = set_joint_module([joint_name], [module_name])
        return res.result
    except rospy.ServiceException, err:
        rospy.loginfo(err)


def action_is_running_client():
    '''Is running
    '''

    try:
        is_running = rospy.ServiceProxy('robotis/action/is_running', IsRunning)
        res = is_running()
        return res.is_running
    except rospy.ServiceException, err:
        rospy.loginfo(err)

def move_head(pan,tilt):
    msg = JointState()
    msg.name = ['head_pan']
    msg.position = [pan]
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)
    
    msg.name = ['head_tilt']
    msg.position = [tilt]
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)
    set_joint_states_pub.publish(msg)



