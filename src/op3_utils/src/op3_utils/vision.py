import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class VisionSystem:
    """
        To be used as a callback for a Subscriber to a camera topic, saves
        the images to a limited buffer. Can also run a sequence of functions
        on the image as soon as it is captured. Each function in the pipeline
        should return a tuple of its resulting value and success status. The
        first argument of the function should be an image.

        foo(img, *args) -> (result, success)

        Parameters:
            maxlen: The maximum size of the image buffer, old images are
                discarded.
            pipeline_funcs: A list of functions to be ran after reading a new
                image.
            pipeline_args: The arguments to each of the functions.

    """
    def __init__(self, pipeline_funcs=[], pipeline_args=[], maxlen=1):
        self.frame_count = 0
        self.img_buffer = deque(maxlen=maxlen)
        self.bridge = CvBridge()

        self.pipeline_funcs = pipeline_funcs
        self.pipeline_args = pipeline_args

        self.results = [None] * len(pipeline_funcs)
        self.status = [None] * len(pipeline_funcs)

    def read(self, ros_msg=None):
        """ Acquires a new frame from a ROS message. This function is intended to
            be passed as callback when subscribing to a camera topic.

            Parameters:
                ros_msg: A ros message containing the image
        
        """
        try:
            img = self.bridge.imgmsg_to_cv2(ros_msg, "bgr8")
            self.img_buffer.append(img)
            self.frame_count += 1
        except CvBridgeError, err:
            rospy.loginfo(err)

        i_foo = 0
        for func, args in zip(self.pipeline_funcs, self.pipeline_args):
            try:
                result, success = func(img, *args)

                self.status[i_foo] = success
                if success != False:
                    self.results[i_foo] = result

            except Exception as e:
                rospy.loginfo("Failed to run function %d in pipeline" % i_foo)
                rospy.loginfo(e)
                self.status[i_foo] = False

            i_foo += 1


