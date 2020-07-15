#!/usr/bin/env python

import numpy as np
import pygame
import rospy
import cv2

from op3_utils.op3_utils import *
from op3_utils.vision import VisionSystem
from sensor_msgs.msg import Image

#DEGREE2RADIAN = np.pi / 180


def getImage(img):
    return img, True

# vision system setup
'''
func0 = getImage
arg0 = ()
pipeline_funcs = [func0]
pipeline_args = [arg0]

vision = VisionSystem(pipeline_funcs,pipeline_args)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
'''
# robot setup
robot = Robot('mini_DRC')
rospy.sleep(5)



if __name__ == '__main__':
    rospy.init_node('mini_DRC')
    
    pygame.init()
    pygame.font.init()
    pygame.mixer.init()
    pygame.mixer.music.load('/home/robotis/sound/rocky.mp3')
    #pygame.mixer.music.play(-1) 

    # motions
    x_move = 0
    y_move = 0
    a_move = 0
    
    head_pan = 0.0
    head_tilt = -60.0
 
    walking = False
    

    sho_roll_ini = -75
    sho_pitch_ini = -45
    sho_el = 15

    sho_roll = sho_roll_ini
    sho_pitch = sho_pitch_ini
    el = sho_el
    hand_control = False
    
    
    
    screen = pygame.display.set_mode((350,200))    
    
    #robot.setGeneralControlModule('action_module')
    #robot.playMotion(53,wait_for_end=True)
    
    robot.setJointsControlModule(['head_pan','head_tilt'],['none'])
    robot.setJointsControlModule(BOTH_HAND,['none'])

    robot.setJointsControlModule(LOWER_BODY,['walking_module'])
    
    pygame.display.set_caption('whatever')
    font = pygame.font.SysFont('whatever',30) 
    textsurface = font.render('test text', False, (255,255,0))
    
    while not rospy.is_shutdown():
        #frame, res = vision.results[0], vision.status[0]
        '''
        if res:
            cv2.imshow('frame',frame)
            cv2.waitKey(33)
        '''    
        cv2.waitKey(33)
        screen.fill((0,0,0))
        head_pan_text = 'head_pan: ' + str(head_pan)
        head_tilt_text = 'head_tilt: '+ str(head_tilt)
        x_amp_text = 'x_move_amp: ' + str(x_move)
        y_amp_text = 'y_move_amp: ' + str(y_move)
        a_amp_text = 'a_move_amp: ' + str(a_move)

        head_pan_sur = font.render(head_pan_text, False, (255,255,0))
        head_tilt_sur = font.render(head_tilt_text, False, (255,255,0))

        x_amp_sur = font.render(x_amp_text, False, (255,255,0))
        y_amp_sur = font.render(y_amp_text, False, (255,255,0))
        a_amp_sur = font.render(a_amp_text, False, (255,255,0))
        screen.blit(head_pan_sur,(0,0))
        screen.blit(head_tilt_sur,(0,30))
        screen.blit(x_amp_sur,(0,60))
        screen.blit(y_amp_sur,(0,75))
        screen.blit(a_amp_sur,(0,90))
        pygame.display.flip()
        for event in pygame.event.get():
            
            keys = pygame.key.get_pressed()
            
            #if keys[pygame.K_w]:
            #    robot.walkStart()
            #    robot.walkVelocities(10,0,0)
            #    rospy.sleep(1.5)
            #    robot.walkStop()
            #
            #if keys[pygame.K_s]:
            #    robot.walkStart()
            #    robot.walkVelocities(-10,0,0)
            #    rospy.sleep(1.5)
            #    robot.walkStop()
            # 
            #if keys[pygame.K_q]:
            #    robot.walkStart()
            #    robot.walkVelocities(-3,0,5)
            #    rospy.sleep(1.5)
            #    robot.walkStop()
            #    walking = False

            #if keys[pygame.K_e]:
            #    robot.walkStart()
            #    robot.walkVelocities(-3,0,-5)
            #    rospy.sleep(1.5)
            #    robot.walkStop()
            #    walking = False


            #if keys[pygame.K_a]:
            #    robot.walkStart()
            #    robot.walkVelocities(0,8,0)
            #    rospy.sleep(1.5)
            #    robot.walkStop()
            #    walking = False

            #if keys[pygame.K_d]:
            #    robot.walkStart()
            #    robot.walkVelocities(0,-8,0)
            #    rospy.sleep(1.5)
            #    robot.walkStop()
            #    walking = False
            unit = 1 
            if keys[pygame.K_w]:
                x_move += unit

            if keys[pygame.K_s]:
                x_move -= unit

            if keys[pygame.K_a]:
                y_move += unit

            if keys[pygame.K_d]:
                y_move -= unit
            
            if keys[pygame.K_q]:
                a_move += unit

            if keys[pygame.K_e]:
                a_move -= unit


            if keys[pygame.K_y]:
                # head forward
                head_pan = 0

            if keys[pygame.K_g]:
                # head left
                head_pan = 90
            
            if keys[pygame.K_j]:
                # head right
                head_pan = -90

            if keys[pygame.K_t]:
                head_pan = 45
                pass
                
            if keys[pygame.K_u]:
                head_pan = -45
                pass
                
            if keys[pygame.K_m]:
                head_pan = -135

            if keys[pygame.K_b]:
                head_pan = 135

            if keys[pygame.K_4]:
                head_pan += 10

            if keys[pygame.K_5]:
                head_pan -= 10


            if keys[pygame.K_r]:
                head_tilt += 10
            if keys[pygame.K_f]:
                head_tilt -= 10
            if keys[pygame.K_h]:
                head_pan = 0
                head_tilt = -60
                
            
            # right hand control
            if keys[pygame.K_8]:
                sho_pitch += 5
            if keys[pygame.K_i]:
                sho_pitch -= 5
                
            if keys[pygame.K_9]:
                sho_roll += 5
            if keys[pygame.K_o]:
                sho_roll -= 5
                
            if keys[pygame.K_0]:
                el += 5
            if keys[pygame.K_p]:
                el -= 5
            
            if keys[pygame.K_k]:
                hand_control ^= True


            if keys[pygame.K_SPACE]:
                # walk or stop
                if walking == False:
                    walking = True
                else:
                    walking = False
                    x_move = 0
                    y_move = 0
                    a_move = 0
        
        # set param
        if hand_control:
            robot.setJointsControlModule(RIGHT_HAND,['none'])
            robot.setJointsControlModule(LOWER_BODY,['walking_module'])
            pitch_rad = sho_pitch * DEGREE2RADIAN
            roll_rad = sho_roll * DEGREE2RADIAN
            el_rad = el * DEGREE2RADIAN

            robot.setJointPos(['r_sho_pitch','r_sho_roll','r_el'],[pitch_rad,roll_rad,el_rad])
            rospy.loginfo('sho: {}, pitch: {}, roll:{} '.format(sho_pitch,sho_roll,el))
        else:
            pass
            #robot.setJointsControlModule(HEAD,['none'])
            #robot.setJointsControlModule(LOWER_BODY,['walking_module'])
        
        if walking: 
            robot.walkStart()
            robot.walkVelocities(x=x_move,y=y_move,th=a_move)
        else: 
            robot.walkStop()
            pass
        robot.moveHead(float(head_pan*DEGREE2RADIAN), float(head_tilt*DEGREE2RADIAN))

        pass
    
    pass


