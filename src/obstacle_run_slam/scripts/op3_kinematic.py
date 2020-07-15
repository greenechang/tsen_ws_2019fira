#!/usr/bin/env python

import numpy as np
from numpy import sin,cos,tan
import rospy

def deg2rad(deg):
    return deg * np.pi / 180
   
def get_trans_matrix(alpha,a,d,theta):
    '''get a trans matrix from link describe parameters
    '''

#   alpha, theta = alpha * np.pi / 180, theta * np.pi / 180
    trans = np.array([[cos(theta),            -sin(theta),           0,            a],
                      [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                      [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d],
                      [0,                     0,                     0,           1]])
    return trans


class robot:
    '''robot
    '''
    link_num = 0
    link_params = np.array([np.identity(4)])
    trans_matrix = np.identity(4)
    T_u_c = np.identity(4)
    
    def __init__(self,link_num,link_params=np.array([np.identity(4)])):
        self.link_num = link_num
        self.link_params = link_params
        if link_params.shape[:] != (link_num,4):
            print('link param input error')
        else:
            self.calculate_kinematic()

        '''
            for links in self.link_params:
                new_link = get_trans_matrix(links[0],links[1],links[2],links[3])

                self.trans_matrix = np.dot(self.trans_matrix,new_link)
            self.T_u_c = np.linalg.inv(self.trans_matrix)
        '''

    def set_link_params(self,link_params):
        self.link_params = link_params
        self.calculate_kinematic()

    def calculate_kinematic(self):

        self.trans_matrix = np.identity(4)

        for links in self.link_params:

            new_link = get_trans_matrix(links[0],links[1],links[2],links[3])

            self.trans_matrix = np.dot(self.trans_matrix,new_link)

            
        self.T_u_c = np.linalg.inv(self.trans_matrix)




def cramer(T,point):
#   print('T:')
#   print(T)
    c = -T[:3,3:4]
#   print('c:')
#   print(c)
    
    point = point * np.pi / 180
    A = np.zeros((3,3))
    A[:,0:2] = T[:3,1:3] 
    A[:,2:] = -np.array([[ np.tan(point[0])] , [np.tan(point[1])] , [1]])
#   print('A:')
#   print(A)
    
    As,At,Au = A.copy(), A.copy(), A.copy()
    As[:,0:1] = c
#   print('As:')
#   print(As)
    At[:,1:2] = c
    Au[:,2:3] = c

#   print('At:')
#   print(At)
    
    
    detA = np.linalg.det(A)
    
#   print('detA: ' + str(detA))
#   print('detAs: '+ str(np.linalg.det(As)))
#   print('detAt: '+ str(np.linalg.det(At)))

    s = np.linalg.det(As) / detA
    t = np.linalg.det(At) / detA
    u = np.linalg.det(Au) / detA
    
    return s,t

