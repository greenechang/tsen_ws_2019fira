#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def image_raw_callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.resize(img,(img.shape[1]/2,img.shape[0]/2),0,0,3)
        cv2.imshow('frame',img)
        cv2.waitKey(1)
    except CvBridgeError, err:
        rospy.loginfo(err)

rospy.Subscriber('/cv_camera/image_raw',Image,image_raw_callback,queue_size=1)
if __name__ == '__main__':
    rospy.init_node('cam')
    rospy.spin()
    
