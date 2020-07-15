#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

import os
import sys, tty, termios



fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dynamixel_sdk import *

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting

RGRIPPER_ID                     = 21                 # Dynamixel#1 ID : 21
LGRIPPER_ID                     = 22                 # Dynamixel#1 ID : 22

BAUDRATE                    = 1000000        # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/u2d2-2'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


def torque(enable):
    # Enable Dynamixel#21 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, RGRIPPER_ID, ADDR_PRO_TORQUE_ENABLE, enable)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % RGRIPPER_ID)

    # Enable Dynamixel#22 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, LGRIPPER_ID, ADDR_PRO_TORQUE_ENABLE, enable)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % LGRIPPER_ID)


# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead.addParam(RGRIPPER_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % RGRIPPER_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead.addParam(LGRIPPER_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % LGRIPPER_ID)
    quit()

def gripper_torque_callback(msg):
    if msg.data == 'on':
        torque(1)
    elif msg.data == 'off':
        torque(0)

def gripper_action_callback(msg):
    
    r_goal = int(msg.x)
    l_goal = int(msg.y)
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(r_goal)), DXL_HIBYTE(DXL_LOWORD(r_goal)), DXL_LOBYTE(DXL_HIWORD(r_goal)), DXL_HIBYTE(DXL_HIWORD(r_goal))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(RGRIPPER_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % RGRIPPER_ID)
        quit()

    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(l_goal)), DXL_HIBYTE(DXL_LOWORD(l_goal)), DXL_LOBYTE(DXL_HIWORD(l_goal)), DXL_HIBYTE(DXL_HIWORD(l_goal))]

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(LGRIPPER_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % LGRIPPER_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()





rospy.init_node('gripper')
rospy.loginfo('Gripper node open.')

torque(1)
rospy.Subscriber("/gripper_action",Point,gripper_action_callback)
rospy.Subscriber("/gripper_torque",String,gripper_torque_callback)


rospy.spin()




