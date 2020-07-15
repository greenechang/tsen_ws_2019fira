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

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam

from robotis_controller_msgs.srv import SetModule, SetJointModule
from op3_action_module_msgs.srv import IsRunning

DATA_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/data/'
CONFIG_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/config/'
PACKAGE_PATH = '/home/robotis/Tsen_ws/src/obstacle_run/'

DEGREE2RADIAN = np.pi / 180





MODE_LIST = ['READY',
             'OBSTACLE_RUN',
             'MODE_NUM']


current_mode = 0
is_start = 0

#Publisher setup
walk_command_pub = rospy.Publisher('/robotis/walking/command', String, queue_size=1)
walk_set_param_pub = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=1)
action_page_pub = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=1)
base_init_pub = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=1)
set_joint_states_pub = rospy.Publisher('/robotis/set_joint_states',JointState, queue_size=1)



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
    walking_param_.balance_enable = True

    return walking_param_

WALKING_STRAIGHT = init_walking_param('param.yaml')
WALKING_SHIFT = init_walking_param('shift_walk.yaml')

def move_joint(joint_name,position):
    msg = JointState()
    msg.name = [joint_name]
    msg.position = [position]
    set_joint_states_pub.publish(msg)
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
        module_name = 'walking_module'
        rospy.loginfo('Set ctrl module: ' + module_name +' result:'+ \
str(set_ctrl_module_client(module_name)))
        module_name = 'none'
        rospy.loginfo('Set joint ctrl module: ' + module_name +' result:'+ \
str(set_joint_ctrl_module_client('head_pan', module_name)))
        rospy.loginfo('Set joint ctrl module: ' + module_name +' result:'+ \
str(set_joint_ctrl_module_client('head_tilt', module_name)))
        rospy.Rate(1).sleep()
        move_joint('head_pan',0)
        move_joint('head_tilt',-0.75)
        move_joint('head_tilt',-0.75)
        move_joint('head_tilt',-0.75)
        move_joint('head_tilt',-0.75)


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

#HSV color range
BLUE_LOWER = np.array([95, 100, 60])
BLUE_UPPER = np.array([120, 255, 255])


YELLOW_LOWER = np.array([20, 80, 80])
YELLOW_UPPER = np.array([40, 255, 255])

RED_LOWER1 = np.array([0, 130, 60])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 130, 60])
RED_UPPER2 = np.array([180, 255, 255])



#States list
READY            = 0
WALKING          = 1
LEFT_WALL        = 2
RIGHT_WALL       = 3
GATE             = 4
LEFT_BORDER      = 5
RIGHT_BORDER     = 6
LOOK_LEFT        = 7
LOOK_RIGHT       = 8
GATE_SHIFT_LEFT  = 9
GATE_SHIFT_RIGHT = 10


STATE_LIST = ['ready',
              'walking',
              'left wall',
              'right wall',
              'gate',
              'left border',
              'right border',
              'look left',
              'look right',
              'gate shift left',
              'gate shift right']

state = 0
encounter_wall = 0
encounter_right_border = 0
encounter_left_border = 0
encounter_gate = 0
left_wall_area = 0
right_wall_area = 0

mask_blue = 123
frame = 123
lava = 123

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
        global frame
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

    global mask_blue
    mask_blue = cv2.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
    mask_blue = cv2.erode(mask_blue, np.ones((3, 3)))
    mask_blue = cv2.dilate(mask_blue, np.ones((3, 3)))
    mask_blue = cv2.dilate(mask_blue, np.ones((3, 3)))
    mask_blue = cv2.erode(mask_blue, np.ones((3, 3)))

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


    roi_near = mask_blue[int(mask_blue.shape[0] * 0.625):, :]
    roi_left_yellow = mask_yellow[int(mask_yellow.shape[0] * 0.8):, :int(mask_yellow.shape[1] * 0.1)]
    roi_left = gray_img[int(frame.shape[0] * 0.8):, :int(frame.shape[1] * 0.1)] 
    roi_right_yellow = mask_yellow[int(mask_yellow.shape[0] * 0.8):, int(mask_yellow.shape[1] * 0.9):]
    roi_right = gray_img[int(frame.shape[0] * 0.8):, int(frame.shape[1] * 0.9):]

    roi_near_yellow = mask_yellow[int(mask_yellow.shape[0] * 0.8):, int(mask_yellow.shape[1] * 0.25):int(mask_yellow.shape[1] * 0.75)]


    roi_red = mask_red[int(mask_red.shape[0] * 0.05):int(mask_red.shape[0] * 0.09),\
int(mask_red.shape[1] * 0.485):int(mask_red.shape[1] * 0.515)]
    cv2.rectangle(frame, (int(mask_red.shape[1] * 0.485), int(mask_red.shape[0] * 0.05)),\
(int(mask_red.shape[1] * 0.515), int(mask_red.shape[0] * 0.09)), (0, 0, 255), 1)


    roi_yellow = np.concatenate((np.concatenate((roi_left,roi_right), axis = 1), np.concatenate(((roi_left_yellow,roi_right_yellow)),axis = 1)),axis = 0)
    cv2.imshow('blue', mask_blue)
    cv2.imshow('yellow',roi_yellow)

    cv2.imshow("near", roi_near)
    cv2.imshow("left", roi_left)
    cv2.imshow("right", roi_right)
    cv2.imshow("near yellow",roi_near_yellow)
    cv2.imshow("red",mask_red)
    gray_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    vis = np.concatenate((hsv_img, gray_img, frame), axis=1)
    cv2.imshow('vis', vis)

    pd_blue = pd.DataFrame(data=mask_blue[:, :])




    global state

    global encounter_wall
    global encounter_gate
    global left_wall_area
    global right_wall_area
    global encounter_left_border
    global encounter_right_border

    mask_blue = cv2.dilate(mask_blue, np.ones((200, 200))) 

    global lava
    lava = mask_blue[0:mask_blue.shape[0]:,:]
    cv2.imshow('lava',lava)

    if white_area(roi_near) > 0.2:
        encounter_wall = 1
    else:
        encounter_wall = 0
    if white_area(roi_left_yellow) > 0.03:
        encounter_left_border = 1
    else:
        encounter_left_border = 0

    if white_area(roi_right_yellow) > 0.03:
        encounter_right_border = 1
    else:
        encounter_right_border = 0
    if cv2.waitKey(1) == 27:
        rospy.signal_shutdown('ESC is pressed!')


def white_area(frame):
    '''Calculate white(value == 255) pixels in a "1-channel" image, in ratio.

    Args:
        frame: imput image
    Returns
        A float number between 0 and 1.0
    '''
    pd_img = pd.DataFrame(data=frame[:, :])
    return float(pd_img.sum().sum())/(255*frame.shape[0]*frame.shape[1])



if __name__ == '__main__':


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
    global state
    while not rospy.is_shutdown():

        rospy.loginfo('State: ' + STATE_LIST[state])
    
        # obstacle run mode
        if current_mode == 1:
            rospy.loginfo('Obstacle run mode.')
            if state == READY:
                rospy.loginfo('take a break.')
                rospy.Rate(1).sleep()
                state = WALKING

            elif state == WALKING:

                if encounter_gate == 1:
                    rospy.loginfo('Encounter a gate, stop!')
                    walk_command_pub.publish('stop')
                    rospy.Rate(2).sleep()
                    state = GATE
                elif encounter_right_border == 1:
                    rospy.loginfo('Encounter right border, stop!')
                    walk_command_pub.publish('stop')
                    rospy.Rate(2).sleep()
                    state = RIGHT_BORDER
                elif encounter_left_border == 1:
                    rospy.loginfo('Encounter left border, stop!')
                    walk_command_pub.publish('stop')
                    rospy.Rate(2).sleep()
                    state = LEFT_BORDER

                elif encounter_wall == 1:
                    rospy.loginfo('Encounter a wall, stop!')
                    walk_command_pub.publish('stop')
                    rospy.Rate(2).sleep()
                    ###############move head to left
                    move_joint('head_tilt',-0.5)
                    move_joint('head_pan',1)
                    rospy.Rate(1).sleep()

                    state = LOOK_LEFT

                else:
                    global WALKING_STRAIGHT
                    WALKING_STRAIGHT.x_move_amplitude = 0.01
                    WALKING_STRAIGHT.y_move_amplitude = 0
                    walk_set_param_pub.publish(WALKING_STRAIGHT)

                    walk_command_pub.publish('start')

            elif state == LOOK_LEFT:
                rospy.Rate(1).sleep()

                left_wall_area = white_area(mask_blue)
                rospy.loginfo('left area:' + str(left_wall_area))
                cv2.imwrite(PACKAGE_PATH+'left.png',frame)
                if encounter_left_border == 1:
                    state = LEFT_BORDER

                else:
                    state = LOOK_RIGHT
                     #################move head to right
                    move_joint('head_tilt',-0.5)
                    move_joint('head_pan',-1)
                    rospy.Rate(1).sleep()

            elif state == LOOK_RIGHT:
                rospy.Rate(1).sleep()
                right_wall_area = white_area(mask_blue)
                rospy.loginfo('left area:' + str(right_wall_area))
                cv2.imwrite(PACKAGE_PATH+'right.png',frame)
                ################move head to mid
                move_joint('head_tilt',-0.75)
                move_joint('head_pan',0)
                rospy.Rate(1).sleep()

                if encounter_right_border == 1:
                    state = RIGHT_BORDER
                elif left_wall_area > right_wall_area:
                    state = LEFT_WALL
                else:
                    state = RIGHT_WALL

                left_wall_area = 0
                right_wall_area = 0
            elif state == RIGHT_WALL:
                WALKING_STRAIGHT.x_move_amplitude = 0
                WALKING_STRAIGHT.y_move_amplitude = 0.02
                walk_set_param_pub.publish(WALKING_STRAIGHT)
                walk_command_pub.publish('start')

                lava_pd = pd.DataFrame(data=lava[:, :])
                rospy.loginfo('lava: ' + str(lava_pd.sum()[lava.shape[1]*0.5]))
                
                if (lava_pd.sum()[lava.shape[1]*0.5]) < (lava.shape[0] * 0.7):
                    state = READY


               #########danger
            elif state == LEFT_WALL:
                WALKING_STRAIGHT.x_move_amplitude = 0
                WALKING_STRAIGHT.y_move_amplitude = -0.02
                walk_set_param_pub.publish(WALKING_STRAIGHT)
                walk_command_pub.publish('start')
                
                lava_pd = pd.DataFrame(data=lava[:, :])
                rospy.loginfo('lava: ' + str(lava_pd.sum()[lava.shape[1]*0.5]/255))
                rospy.loginfo(lava.shape[0])
                if (lava_pd.sum()[lava.shape[1]*0.5]/255) < (lava.shape[0] * 0.7):
                    rospy.loginfo('test')
                    state = READY
                    rospy.loginfo(state)

            elif state == LEFT_BORDER:
                WALKING_STRAIGHT.x_move_amplitude = 0
                WALKING_STRAIGHT.a_move_amplitude = -0.1
                walk_set_param_pub.publish(WALKING_STRAIGHT)
                walk_command_pub.publish('start')
                rospy.Rate(0.5).sleep()
                state == WALK

            elif state == RIGHT_BORDER:
                WALKING_STRAIGHT.x_move_amplitude = 0
                WALKING_STRAIGHT.a_move_amplitude = 0.1
                walk_set_param_pub.publish(WALKING_STRAIGHT)
                walk_command_pub.publish('start')
                rospy.Rate(0.5).sleep()
                state == WALK


        else:
            rospy.loginfo('READY_MODE')
        rospy.Rate(3).sleep()



    rospy.spin()
