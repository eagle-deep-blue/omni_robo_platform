#!/usr/bin/env python

#Author: Solomon Jingchun YIN
#Contact Address: jingchun.yin@live.com
#Completed on 25th Apr., 2015

#*****************************************************************
#PACKAGE description:
#INPUT: Cartesian-space Velocity Command <- geometry_msgs/Twist: {m/s, m/s, radian/s}
#OUTPUT: Joint-space Velocity Command <- sensor_msgs/JointState {radian/s}
#*****************************************************************
#The mathematical motion model is derived from the book of 
#Introduction to Autonomous Mobile Robots,
#R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, 
#MIT Press, 2nd Revised edition, 2011
#*****************************************************************

import rospy
import geometry_msgs.msg
import sensor_msgs.msg

from numpy import *
from matplotlib import *
from pylab import *
from math import atan2

#*****************************************************************
#       parameter initialization
#*****************************************************************

flag_read_odometry = True 

flag_debug = False
flag_diagnosis = False

h = 0.285
b = 0.0625
r = 0.0625

v_max_x = 2
v_max_y = 2
v_min_x = 0
v_min_y = 0
omega_max = pi
omega_min = -pi

beta_max = 3 * pi
phi_max = 3 * pi

alpha_0 = -pi/2
alpha_1 = pi/2

threshold_stabilization = 1e-8

#   M(PHI) * P_dot = C*PHI_dot
#    PHI is computed assuming ideal condition, or is actually retrieved from wheel encoder

class differential_kinematics():
    def __init__(self):

        self.beta0 = pi
        self.beta1 = 0
        self.phi0 = 0
        self.phi1 = 0
        self.flag_old_time = False

        if flag_diagnosis:
            print('...kinematic data already recorded for diagnosis...')
            
            fs = open('diagnosis_inv_kinematics.txt','w')
            aux = 't'+'\t'+'v_x' + '\t' + 'v_y' + '\t' + 'omega'  + '\t' + \
                'beta_dot_0' + '\t' + 'beta_dot_1' + '\t' + 'phi_dot_0' + '\t' + 'phi_dot_1'+'\t' + \
                'beta0' + '\t' + 'beta1' + '\t' + 'phi0' + '\t' + 'phi1'+'\n'
            fs.write(aux)
            fs.close()
            
        rospy.init_node('differential_kinematics', anonymous=True)
        self.pub_jnt_cmd = rospy.Publisher('/jnt_ref_cmd', sensor_msgs.msg.JointState, queue_size=10)
        self.start_time = rospy.get_time()
        
        rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist, self.callback_cartesian_cmd)
        if flag_read_odometry:
            self.flag_odom_joint = False            
            rospy.Subscriber('/odom_joints', sensor_msgs.msg.JointState, self.callback_odom_joint)
        rospy.spin()

    def callback_cartesian_cmd(self, data):
        
#    retrieve P_ddot value        
        v_x = data.linear.x
        v_y = data.linear.y
        omega = data.angular.z
 
#        v_x = max(min(v_x, v_max_x), v_min_x)        
#        v_y = max(min(v_y, v_max_y), v_min_y)        
#        omega = max(min(omega, omega_max), omega_min)
        
#    update PHI value
#        based on ideal computation
        if not flag_read_odometry:
            
            if self.flag_old_time:
                dlt_time = rospy.get_time() - self.old_time
                if flag_debug:
                    rospy.loginfo('...{DELTA_T}=[%f] (ms) for joint command update...', dlt_time*1e3)        
                
                if not dlt_time > 1:
                    self.beta0 += self.beta_dot_0 * dlt_time
#                    self.beta0 = self.modulate_angle(self.beta0)
                    self.beta1 += self.beta_dot_1 * dlt_time
#                    self.beta1 = self.modulate_angle(self.beta1)
                    self.phi0 += self.phi_dot_0 * dlt_time
                    self.phi1 += self.phi_dot_1 * dlt_time      
                else:
                    rospy.loginfo('...robot has already stopped...')
                    
#      compute PHI_dot value            
            [self.beta_dot_0, self.phi_dot_0, self.beta_dot_1, self.phi_dot_1] = \
            self.differential_kinematics_model(v_x,v_y,omega,self.beta0, self.beta1)
            
#      publish PHI and PHI_dot value    
            aux = sensor_msgs.msg.JointState()
            aux.name = ['beta_0', 'phi_0', 'beta_1', 'phi_1']
            aux.position = [self.beta0, self.phi0, self.beta1, self.phi1]
            aux.velocity = [self.beta_dot_0, self.phi_dot_0, self.beta_dot_1, self.phi_dot_1]
            self.pub_jnt_cmd.publish(aux)

#       store data for diagnosis
            if flag_diagnosis:
                fs = open('diagnosis_inv_kinematics.txt','a')
                aux_time = rospy.get_time()-self.start_time
                aux = str(aux_time)+ '\t' +str(v_x) + '\t' + str(v_y) + '\t' + str(omega) + '\t' + \
                str(self.beta_dot_0) + '\t' + str(self.beta_dot_1) + '\t' + str(self.phi_dot_0) + '\t' + str(self.phi_dot_1) + '\t' + str(self.beta0) + '\t' + str(self.beta1) + '\t' + str(self.phi0) + '\t' + str(self.phi1)+'\n'
                fs.write(aux)  
                fs.close()

#      record time for PHI computation            
            self.old_time = rospy.get_time()
            self.flag_old_time = True       
            
        else:
            
#       based on wheel encoder odometry data retrieval
            if self.flag_odom_joint:
                if self.flag_old_time:
                    dlt_time = rospy.get_time() - self.old_time
                    if flag_debug:        
                        rospy.loginfo('...{DELTA_T}=[%f] (ms) for joint command update...', dlt_time*1e3)
                    
#      compute PHI_dot value            
                [self.beta_dot_0, self.phi_dot_0, self.beta_dot_1, self.phi_dot_1] = \
                    self.differential_kinematics_model(v_x,v_y,omega,self.beta0, self.beta1)
                    
#      publish PHI and PHI_dot value    
                aux = sensor_msgs.msg.JointState()
                aux.name = ['beta_0', 'phi_0', 'beta_1', 'phi_1']
                aux.position = [self.beta0, self.phi0, self.beta1, self.phi1]
                aux.velocity = [self.beta_dot_0, self.phi_dot_0, self.beta_dot_1, self.phi_dot_1]
                self.pub_jnt_cmd.publish(aux)

#       store data for diagnosis
                if flag_diagnosis:
                    fs = open('diagnosis_inv_kinematics.txt','a')
                    
                    aux_time = rospy.get_time()-self.start_time
                    aux = str(aux_time)+ '\t' + str(v_x) + '\t' + str(v_y) + '\t' + str(omega) + '\t' + \
                        str(self.beta_dot_0) + '\t' + str(self.beta_dot_1) + '\t' + str(self.phi_dot_0) + '\t' + str(self.phi_dot_1) + '\t' + str(self.beta0) + '\t' + str(self.beta1) + '\t' + str(self.phi0) + '\t' + str(self.phi1)+'\n'
                    fs.write(aux)  
                    fs.close()

#      record time for PHI computation            
                self.old_time = rospy.get_time()
                self.flag_old_time = True       

#    INSTANTANEOUS KINEMATICS {M(PHI)*P_dot = PHI_dot}
#           P_dot <- [v_x,v_y,omega] (Cartesian-Space velocity)
#           PHI_dot <- [beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1]
#           {beta_0, beta_1} -> angular displacement of steering motor from left to right

    def differential_kinematics_model(self, v_x,v_y,omega,beta_0, beta_1):
        M = array([  [cos(alpha_0+beta_0), sin(alpha_0+beta_0), b+h*sin(alpha_0+beta_0)],\
                     [-sin(alpha_0+beta_0), cos(alpha_0+beta_0), h*cos(alpha_0+beta_0)],\
                     [cos(alpha_1+beta_1), sin(alpha_1+beta_1), b+h*sin(alpha_1+beta_1)],\
                     [-sin(alpha_1+beta_1), cos(alpha_1+beta_1), h*cos(alpha_1+beta_1)]    ])
        P_dot = array([ v_x, v_y, omega])  
        aux = dot(M,P_dot)  
        C = array([ [-1/b, 0, 0, 0], \
                    [0, 1/r, 0, 0], \
                    [0, 0, -1/b, 0], \
                    [0, 0, 0, 1/r] ])
        
        PHI_dot = dot(C,aux)
        
        if abs(PHI_dot[0])<threshold_stabilization:
            PHI_dot[0]=0
        if abs(PHI_dot[2])<threshold_stabilization:
            PHI_dot[2]=0
            
        return PHI_dot
        
    def modulate_angle(self, x):
        x = atan2(sin(x),cos(x))
        return x

    def callback_odom_joint(self, data):
        self.beta0 = data.position[0]
        self.phi0 = data.position[1]
        self.beta1 = data.position[2]
        self.phi1 = data.position[3]
        
        self.flag_odom_joint = True

if __name__=='__main__':
    try:
        differential_kinematics()        
    except rospy.ROSInterruptException: pass
