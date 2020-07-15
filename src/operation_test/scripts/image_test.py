#!/usr/bin/env python

import rospy
import cv2
import rosparam

import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


rospy.set_param("test_param",'It\'s a test parameter.\n')


def image_callback(msg):
#    rospy.loginfo('Got image!')

    try:
        cv2_img = bridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError, e:
        rospy.loginfo(e)
    else:
        cv2.imshow('cam',cv2_img)
        cv2.waitKey(1)


    
'''
def get_image():
    rospy.Subscriber("/cv_camera/image_raw",Image,image_callback);
    rospy.spin()
'''

if __name__ == '__main__':
    rospy.init_node('image_test')
    
    test_param = rospy.get_param("/image_test/test_param")
    rospy.loginfo(test_param)
#    get_image();
    rospy.Subscriber("/cv_camera/image_raw",Image,image_callback)
    rospy.spin()

