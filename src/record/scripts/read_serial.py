#!/usr/bin/env python

import rospy
import serial

from std_msgs.msg import String

mag_pub = rospy.Publisher('/compass', String, queue_size = 1)

ser = serial.Serial('/dev/ttyUSB2')
ser.flushInput()

rospy.init_node('read_serial')

if __name__ == '__main__':
    
    while True:
        try:
            ser_bytes = ser.readline()
            msg = String()
            msg.data = ser_bytes[:-2]
            #rospy.loginfo(msg)
            mag_pub.publish(msg)

        except:
            print('error!')
            break
        
