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
import rospkg
from copy import copy

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Int32

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam
from op3_gripper.msg import GripperPosition

from robotis_controller_msgs.srv import SetModule, SetJointModule, SetJointModuleRequest
from op3_action_module_msgs.srv import IsRunning

from os import path
DEGREE2RADIAN = np.pi / 180

rospack = rospkg.RosPack()
set_joint_states_pub = rospy.Publisher('/robotis/set_joint_states',JointState, queue_size=5)
gripper_position_pub = rospy.Publisher('/gripper_action',GripperPosition,queue_size=1)
walk_command_pub = rospy.Publisher('/robotis/walking/command',String,queue_size=5)

def move_gripper(left,right):
    msg = GripperPosition()
    msg.Left, msg.Right = left, right
    gripper_position_pub.publish(msg)

def initWalkingParam(package_name, file_name='param.yaml'):
    '''set walking param function
    Parameters besides x, y, a_move_amp are loaded from param.yaml file.
    '''
    
    config_path = path.join(rospack.get_path(package_name),'config')
    
    yaml_file = open(path.join(config_path, file_name))

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
    walking_param_.balance_enable = True 

    return walking_param_

def move_joint(joint_name,position):
    msg = JointState()
    msg.name = [joint_name]
    msg.position = [position]
    set_joint_states_pub.publish(msg)


class Robot:

    def __init__(self,package_name):
        '''initialize publishers, subscribers, clients and other needed variables
        '''
        self.package_name = package_name

        self.joint_state_pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=1)
        self.action_pub = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=1)
        self.walk_command_pub = rospy.Publisher('/robotis/walking/command', String, queue_size=1)
        self.walk_set_param_pub = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=1)

        self.base_walking_params = initWalkingParam(self.package_name)

        self.set_joint_ctrl_module = rospy.ServiceProxy('/robotis/set_present_joint_ctrl_modules',SetJointModule)
        self.set_ctrl_module = rospy.ServiceProxy('/robotis/set_present_ctrl_modules', SetModule)

        self.action_is_running = rospy.ServiceProxy('/robotis/action/is_running', IsRunning)

        
    def move_head(self, pan, tilt):
        self.setJointPos(['head_pan','head_tilt'],[pan,tilt])


    def setControlModule(self, module_name):
        self.set_ctrl_module(module_name)

    def setJointControlModule(self, joint_names, module_names):
        req = SetJointModuleRequest()

        if len(module_names) == 1:
            modlue_names = module_names * len(joint_names)
            
        req.joint_name = joint_names
        req.module_name = module_names
        self.set_joint_ctrl_module(req)

    def setJointPos(self, joint, pos):
        msg = JointState()
        msg.name = joint
        msg.position = pos

        self.joint_state_pub.publish(msg)

    def actionPage(self, action_page, wait_for_end=False):
        self.action_pub.publish(action_page)
        if wait_for_end:
            while self.action_is_running().is_running:
                rospy.sleep(0.1)

    def updateJointPos(self, msg):
        for i, joint_name in enumerate(msg.name):
            self.joint_pos[joint_name] = msg.position[i]

    def walkStart(self):
        self.walk_command_pub.publish('start')
    
    def walkStop(self):
        self.walk_command_pub.publish('stop')

    def walkVelocities(self, x=0, y=0, a=0):
        mod_params = copy(self.base_walking_params)
        mod_params.x_move_amplitude = x / 1000.0
        mod_params.y_move_amplitude = y / 1000.0
        mod_params.angle_move_amplitude = a * DEGREE2RADIAN

        self.walk_set_param_pub.publish(mod_params)


