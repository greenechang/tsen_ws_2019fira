#! /usr/bin/env python
from op3_util.op3_util import Robot
import rospy


if __name__ == '__main__':
    rospy.init_node('test')
    robot = Robot('obstacle_run')
    rospy.loginfo(robot.walking_params)
    robot.walking_params.append(robot.loadWalkingParams())
    rospy.loginfo('')
    rospy.loginfo(robot.walking_params[1].dsp_ratio)
    
    rospy.spin()
