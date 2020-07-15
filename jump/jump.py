#! /usr/bin/env python


import rospy
from op3_ros_utils import *

rospy.init_node("mememem")
robot = Robot()

def init():
    robot.setGeneralControlModule("action_module")

    robot.setGrippersPos(left=100.0, right=0.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")

    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.8])

    rospy.sleep(1.0)

        

init()
raw_input()
print('remove the cable now')
rospy.sleep(8)

robot.setGeneralControlModule("walking_module")

robot.walkVelocities(x=5.0)
robot.walkStart()
rospy.sleep(5)
robot.walkStop()
rospy.sleep(5)

robot.setGeneralControlModule("action_module")
robot.playMotion(70, wait_for_end=True)
rospy.sleep(3)
robot.playMotion(70, wait_for_end=True)
rospy.sleep(3)
robot.playMotion(70, wait_for_end=True)
rospy.sleep(3)

