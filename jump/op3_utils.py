import yaml

from os import path
from copy import copy

import rospy
import rospkg

from std_msgs.msg import Int32, String, Float32, Float64, Bool, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from op3_online_walking_module_msgs.msg import FootStepCommand, JointPose
from op3_online_walking_module_msgs.msg import WalkingParam as OnlineWalkingParam
from op3_walking_module_msgs.msg import WalkingParam
from op3_action_module_msgs.srv import IsRunning
from robotis_controller_msgs.srv import SetModule, SetJointModule, SetJointModuleRequest
from op3_walking_module_msgs.msg import WalkingParam
from op3_gripper.msg import GripperPosition

import numpy as np

rospack = rospkg.RosPack()
DEGREE2RADIAN = np.pi / 180

HEAD = ['head_tilt','head_pan']
LEFT_HAND = ['l_el','l_sho_pitch','l_sho_roll']
RIGHT_HAND = ['r_el','r_sho_pitch','r_sho_roll']
LEFT_FOOT = ['r_hip_yaw','r_hip_roll','r_hip_pitch','r_knee','r_ank_pitch','r_ank_roll']
RIGHT_FOOT = ['l_hip_yaw','l_hip_roll','l_hip_pitch','l_knee','l_ank_pitch','l_ank_roll']
LOWER_BODY = LEFT_FOOT + RIGHT_FOOT
BOTH_HAND = LEFT_HAND + RIGHT_HAND
WHOLE_BODY_WITHOUT_HEAD = LOWER_BODY + BOTH_HAND
WHOLE_BODY = WHOLE_BODY_WITHOUT_HEAD + HEAD

class Robot:
    """ Represents the current robot state

    """

    def __init__(self):
        # Initialize publishers, subscribers and other needed variables


        
        self.joint_state_pub = rospy.Publisher("/robotis/set_joint_states", 
            JointState, queue_size=1)
        self.grippers_torque = rospy.Publisher('/grippers/torque', Bool, 
            queue_size=1)
        self.action_pub = rospy.Publisher('/robotis/action/page_num', Int32, 
            queue_size=1)
        self.walk_command_pub = rospy.Publisher('/robotis/walking/command', 
            String, queue_size=1)
        self.walk_set_param_pub = rospy.Publisher('/robotis/walking/set_params', 
            WalkingParam, queue_size=1)
        self.move_gripper_pub = rospy.Publisher('/move_gripper', GripperPosition,
            queue_size=1)
        self.move_right_gripper_pub = rospy.Publisher('/move_right_gripper', Int32,
            queue_size=1)
        self.move_left_gripper_pub = rospy.Publisher('/move_left_gripper', Int32,
            queue_size=1)
        self.set_gripper_torque_pub = rospy.Publisher('/set_gripper_torque', String,
            queue_size=1)

        
        self.online_walk_set_body_offset_pub = rospy.Publisher(
            '/robotis/online_walking/body_offset', Pose, queue_size=1)
        self.online_walk_foot_distance_pub = rospy.Publisher(
            '/robotis/online_walking/foot_distance', Float64, queue_size=1)
        self.online_walk_command_pub = rospy.Publisher(
            '/robotis/online_walking/foot_step_command', FootStepCommand, queue_size=1)
        self.online_walk_balance_pub = rospy.Publisher(
            '/robotis/online_walking/wholebody_balance_msg', String, queue_size=1)
        self.online_walk_param_pub = rospy.Publisher(
            '/robotis/online_walking/walking_param', OnlineWalkingParam, queue_size=1)
        self.online_walk_reset_pose_pub = rospy.Publisher(
            '/robotis/online_walking/reset_body', Bool, queue_size=1)
        self.online_walk_goal_pose_pub = rospy.Publisher(
            '/robotis/online_walking/goal_joint_pose', JointPose, queue_size=1)
        
        self.walking_params = [self.loadWalkingParams()]
        # put differnet walking params in a list, which the 0th is the default one.
        # add new params like this:
        # robot.walking_params.append(robot.loadWalkingParams(config_file)

        self.set_joint_ctrl_module = rospy.ServiceProxy(
            "/robotis/set_present_joint_ctrl_modules", SetJointModule)
        self.set_ctrl_module = rospy.ServiceProxy(
            "/robotis/set_present_ctrl_modules", SetModule)
        self.action_is_running = rospy.ServiceProxy("/robotis/action/is_running",
            IsRunning)

        rospy.Subscriber("/robotis/present_joint_states", JointState, 
                self.updateJointPos, queue_size=1)
        
        rospy.Subscriber('/robotis/open_cr/button', String,
                self.buttonCallBack, queue_size=1)
                
        self.pressed_button = None
        
        self.joint_pos = {}
        
    def buttonCallBack(self,msg):
        self.pressed_button = msg.data
        
    def get_pressed_button(self):
    
        if self.pressed_button == None:
            return None
        
        else:
            but = self.pressed_button
            self.pressed_button = None
            return but
        

    def onlineWalkSetup(self, x=0.0, y=0.0, z=0.0, foot_dist=0.070, foot_height=0.05, 
            dsp_ratio=0.20, zmp_offset_x=0.0, zmp_offset_y=0.0):
        # Set control module if it hasn't already
        self.setGeneralControlModule("online_walking_module")
        rospy.sleep(0.5)

        self.onlineWalkIniPose()
        self.online_walk_reset_pose_pub.publish(True)
        rospy.sleep(3)
        
        # Activate balance control, required for online walking
        # I had to dig for the specific message to send to this topic. 
        # Why don't they just use bool types for this kind of stuff?
        self.online_walk_balance_pub.publish("balance_on")
        rospy.sleep(1)

        ####### Important Note #######
        # For some reason it is needed to call the walk command once before
        # setting the body offsets, otherwise it generates some "different
        # control type error". So we call the function with 0 number of steps
        #self.onlineWalkCommand(direction="stop", start_leg="right", step_num=0, step_time=0.1)
        #rospy.sleep(5)

        # Set the walking parameters
        walking_params = OnlineWalkingParam()
        walking_params.foot_height_max = foot_height
        walking_params.dsp_ratio = dsp_ratio
        walking_params.zmp_offset_x = zmp_offset_x
        walking_params.zmp_offset_y = zmp_offset_y
        walking_params.lipm_height = 0.12 # This is fixed on demo gui
        self.online_walk_param_pub.publish(walking_params)

        # Set the body offsets
        body_offsets = Pose()
        body_offsets.position.x = x
        body_offsets.position.y = y
        body_offsets.position.z = z
        body_offsets.orientation.x = 0.0
        body_offsets.orientation.y = 0.0
        body_offsets.orientation.z = 0.0
        body_offsets.orientation.w = 0.0
        self.online_walk_set_body_offset_pub.publish(body_offsets)

        ## Publish messages
        self.online_walk_foot_distance_pub.publish(foot_dist)

        rospy.sleep(5)

    def onlineWalkCommand(self, direction, start_leg, step_num, 
            front_length=0.0, side_length=0.0, step_angle=0.0, step_time=0.5):
        # Create foot step message and publish it
        step_cmd = FootStepCommand()
        step_cmd.command = direction
        step_cmd.start_leg = start_leg
        step_cmd.step_num = step_num
        step_cmd.step_length = front_length
        step_cmd.side_length = side_length
        step_cmd.step_angle = step_angle
        step_cmd.step_time = step_time

        self.online_walk_command_pub.publish(step_cmd)

    def onlineWalkIniPose(self):
        ini_file = open(path.join(rospack.get_path("op3_gui_demo"), 
            "config/init_pose.yaml"))

        yf = yaml.load(ini_file)

        msg = JointPose()
        msg.mov_time = yf["mov_time"]
        
        joint_values = [DEGREE2RADIAN * v for v in yf["tar_pose"].values()]
        joint_names = yf["tar_pose"].keys()

        msg.pose.name = joint_names
        msg.pose.position = joint_values

        self.online_walk_goal_pose_pub.publish(msg)


    def setGeneralControlModule(self, module_name):
        self.set_ctrl_module(module_name)

    def setJointsControlModule(self, joint_names, module_names):
        req = SetJointModuleRequest()
        if len(module_names) == 1:
            module_names = module_names * len(joint_names)
            print(module_names)
        req.joint_name = joint_names
        req.module_name = module_names
        try: 
            res = self.set_joint_ctrl_module(req)
            rospy.loginfo(res)
            return res.result
        except rospy.ServiceException, err:
            rospy.loginfo(err)

    def setJointPos(self, joints, pos):
        msg = JointState()
        msg.name = joints
        msg.position = pos

        self.joint_state_pub.publish(msg)

    def playMotion(self, motion_number, wait_for_end=True):
        self.action_pub.publish(motion_number)
        if wait_for_end:
            while self.action_is_running().is_running: rospy.sleep(0.1)

    def updateJointPos(self, msg):
        for i, joint_name in enumerate(msg.name):
            self.joint_pos[joint_name] = msg.position[i]

    def walkStart(self):
        self.walk_command_pub.publish("start")

    def walkStop(self):
        self.walk_command_pub.publish("stop")

    def walkVelocities(self, x=0.0, y=0.0, th=0.0,params=0):
        mod_params = copy(self.walking_params[params])
        mod_params.x_move_amplitude = x / 1000.0
        mod_params.y_move_amplitude = y / 1000.0
        mod_params.angle_move_amplitude = th * (np.pi / 180)

        self.walk_set_param_pub.publish(mod_params)
    
    def moveHead(self,pan=None,tilt=None):
        if not pan is None and not tilt is None:
            self.setJointPos(['head_pan','head_tilt'],[pan,tilt])
        
        if tilt is None and not pan is None:
            self.setJointPos(['head_pan'],[pan])
        elif pan is None and not tilt is None:
            self.setJointPos(['head_tilt'],[tilt])
            
        
        

    def moveGripper(self,left=None,right=None):
        if left == None:
            msg = Int32()
            msg.data = right
            self.move_right_gripper(msg)
        elif right == None:
            msg = Int32()
            msg.data = left
            self.move_left_gripper(msg)
        else:
            msg = GripperPosition()
            msg.Left, msg.Right = left, right
            self.move_gripper_pub.publish(msg)

    def setGripperTorqueOn(self):
        msg = String()
        msg.data = 'on'
        self.set_gripper_torque_pub.publish(msg)

    def setGripperTorqueOff(self):
        msg = String()
        msg.data = 'off'
        self.set_gripper_torque_pub.publish(msg)

    def loadWalkingParams(self,file_name='param.yaml'):
        '''set walking param function
        Parameters besides x, y, a_move_amp are loaded from param.yaml file.
        '''
        CONFIG_PATH = path.join(rospack.get_path('op3_walking_module'), "config")
        yaml_file = open(path.join(CONFIG_PATH, file_name))

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
