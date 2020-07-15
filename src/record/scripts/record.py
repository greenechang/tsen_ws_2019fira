#!/usr/bin/env python

import os
import numpy as np
import yaml
import pygame as pg
from tf.transformations import euler_from_quaternion

import csv
import time
from datetime import datetime

import rospy
import rospkg

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, JointState, Imu
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam
from robotis_controller_msgs.srv import SetModule, SetJointModule
from op3_action_module_msgs.srv import IsRunning

from op3_ros_function import *

DEGREE2RADIAN = np.pi / 180

# ros init 
rospy.init_node('record_node')
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('record') + '/'
 
# op3 paths
DATA_PATH = PACKAGE_PATH + 'data/'
CONFIG_PATH = PACKAGE_PATH + 'config/'

# walking setup
walk_set_param_pub = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=1)

# camera callback setup
BRIDGE = CvBridge()
_frame = np.ones((3,3),np.uint8)

# take name
TAKE_NAME = rospy.get_param('/take_name')
FOLDER_PATH = PACKAGE_PATH + TAKE_NAME + '/'

# csv files path
IMU_CSV = FOLDER_PATH  + 'imu.csv'
VIDEO_CSV = FOLDER_PATH + 'video.csv'
COMPASS_CSV = FOLDER_PATH + 'compass.csv'

# VideoWriter setup
VIDEO_PATH = FOLDER_PATH + 'output.avi'
_fourcc = cv2.VideoWriter_fourcc(*'MPEG')
_out = cv2.VideoWriter(VIDEO_PATH,_fourcc,30.0,(640,360))
_frame_stamp = 0
_image_msg = Image
_counter = 1


# op3
robot = Robot('record')


def camera_callback(msg):
    try:
        global _frame
        global _image_msg
        cv2_img = BRIDGE.imgmsg_to_cv2(msg,"bgr8")
        _frame = cv2.resize(cv2_img, (0, 0), fx = 1, fy = 1)
        _image_msg = msg
    except CvBridgeError, err:
        rospy.loginfo(err)
    else:
        # print info
        log = ' Got an camera_raw message! '
        stamp = 'At ' + str(msg.header.stamp)
        #rospy.loginfo(stamp+log)
        
        # write video and csv file
        _out.write(_frame)

        global _counter
        img_csv_writer(msg,_counter)
        _counter += 1


def img_csv_writer(msg,counter):
    row = [str(counter),str(msg.header.stamp)[-12:-5]]
    with open(VIDEO_CSV,'a') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerow(row)


def imu2strlink(msg):
    orientation = msg.orientation
    angular = msg.angular_velocity
    linear = msg.linear_acceleration

    link = [str(msg.header.stamp)[-12:-5],
            str(orientation.x),str(orientation.y),str(orientation.z),str(orientation.w),
            str(angular.x),str(angular.y),str(angular.z),
            str(linear.x),str(linear.y),str(linear.z)]
    return link


def imu_callback(msg):
    log = ' Got an Imu message! '
    stamp = 'At ' + str(msg.header.stamp)
    timestamp = str(msg.header.stamp)[-12:]
    row = imu2strlink(msg) 
    quaternion = (msg.orientation.x,msg.orientation.y,
                  msg.orientation.z,msg.orientation.w)
    euler = euler_from_quaternion(quaternion)
    with open(IMU_CSV,'a') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerow(row)
       #rospy.loginfo(stamp+log)

def status_callback(msg):
    if msg.status_msg == 'Finish Init Pose':
        robot.setControlModule('walking_module')
        robot.setJointControlModule(['head_pan','head_tilt'],['none','none'])

def compass_callback(msg):
    #timestamp = str(msg.header.stamp)[-12:]
    rospy.loginfo('got a compass topic')
    timestamp = str(rospy.get_rostime())[-12:-5]
    rospy.loginfo(msg.data)
    row = [timestamp, msg.data]

    with open(COMPASS_CSV,'a') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerow(row)

if __name__ == '__main__':
    
    if not os.path.exists(FOLDER_PATH):
        os.makedirs(FOLDER_PATH)

    # clean csv files
    imu_label = [['timestamp',
               'orien_x','orien_y','orien_z','orien_w',
               'ang_v_x','ang_v_y','ang_v_z',
               'lin_acc_x','lin_acc_y','lin_acc_z']]
    with open(IMU_CSV,'w') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(imu_label)
        pass

    frame_label = [['counter','timestamp']]
    with open(VIDEO_CSV,'w') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(frame_label)
    with open(COMPASS_CSV,'w') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows([['timestamp','compass']])

    # ROS Subscriber setup 
    rospy.Subscriber('/robotis/open_cr/imu', Imu, imu_callback)
    rospy.Rate(0.5).sleep()
    rospy.Subscriber('/cv_camera/image_raw', Image, camera_callback)
    rospy.Subscriber('robotis/status', StatusMsg, status_callback)
    rospy.Subscriber('/compass', String, compass_callback)

    # motions
    x_move_amp = 0.0
    a_move_amp = 0.0
    
    head_pan = 0.0
    head_tilt = 6.0
   
    walking = False

    # pygame
    pg.init()
    pg.font.init()

    screen = pg.display.set_mode((350,200))

    font = pg.font.SysFont('whatever',30) 
    textsurface = font.render('test text', False, (255,255,0))
    #pg.image.save(textsurface, '~/asdf.png')

    pg.display.set_caption('whatever')
    rospy.sleep(5)
    robot.move_head(float(60*DEGREE2RADIAN), float(0*DEGREE2RADIAN))
    rospy.sleep(3)
    robot.move_head(float(-60*DEGREE2RADIAN), float(0*DEGREE2RADIAN))
    rospy.sleep(3)
    robot.move_head(float(0*DEGREE2RADIAN), float(6*DEGREE2RADIAN))
  
 
    while not rospy.is_shutdown():
        screen.fill((0,0,0))
        head_pan_text = 'head_pan: ' + str(head_pan)
        head_tilt_text = 'head_tilt: '+ str(head_tilt)
        x_amp_text = 'x_move_amp: ' + str(x_move_amp)
        a_amp_text = 'a_move_amp: ' + str(a_move_amp)

        head_pan_sur = font.render(head_pan_text, False, (255,255,0))
        head_tilt_sur = font.render(head_tilt_text, False, (255,255,0))

        x_amp_sur = font.render(x_amp_text, False, (255,255,0))
        a_amp_sur = font.render(a_amp_text, False, (255,255,0))
        screen.blit(head_pan_sur,(0,0))
        screen.blit(head_tilt_sur,(0,30))
        screen.blit(x_amp_sur,(0,60))
        screen.blit(a_amp_sur,(0,90))
        pg.display.flip()
        for event in pg.event.get():
            
            keys = pg.key.get_pressed()
            
            if keys[pg.K_w]:
                # speed up
                x_move_amp += 3 
                pass

            if keys[pg.K_s]:
                # speed down
                x_move_amp -= 3
                pass
            
            if keys[pg.K_a]:
                # turn left
                a_move_amp += 1
                pass
            
            if keys[pg.K_d]:
                # turn right
                a_move_amp -= 1
                pass
            if keys[pg.K_x]:
                a_move_amp = 0

            if keys[pg.K_i]:
                # head up
                head_tilt += 3
                pass
            
            if keys[pg.K_k]:
                # head down
                head_tilt -= 3
                pass

            if keys[pg.K_j]:
                # head left
                head_pan += 3
                pass
            
            if keys[pg.K_l]:
                # head right
                head_pan -= 3
                pass

            if keys[pg.K_u]:
                # head init
                head_pan = 60
                pass
            if keys[pg.K_o]:
                head_pan = -60
                pass
            if keys[pg.K_m]:
                head_pan = 0
                head_tilt = 6



            if keys[pg.K_SPACE]:
                # walk or stop
                if walking == False:
                    walking = True
                else:
                    walking = False
                    x_move_amp = 0.0
                    a_move_amp = 0.0
        
        # set param
        
        if walking: 
            robot.walkStart()
            robot.walkVelocities(x_move_amp,0,a_move_amp)
        else: 
            robot.walkStop()
            pass
        robot.move_head(float(head_pan*DEGREE2RADIAN), float(head_tilt*DEGREE2RADIAN))
        cv2.imshow('frame',_frame)
        if cv2.waitKey(30) == 27:
            _out.release()
            break
