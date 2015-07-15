#!/usr/bin/env python

#Author: Solomon Jingchun YIN
#Contact Address: jingchun.yin@live.com
#Completed on 28th Apr., 2015

#*****************************************************************
#PACKAGE description:
#INPUT: Joint-space Velocity Command <- sensor_msgs/JointState {radian/s}
#OUTPUT: robot pose <- <tf>
#*****************************************************************
#The mathematical motion model is derived from the book of
#Introduction to Autonomous Mobile Robots,
#R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza,
#MIT Press, 2nd Revised edition, 2011
#*****************************************************************

import rospy
import sensor_msgs.msg
import nav_msgs.msg
import tf

from geometry_msgs.msg import Point, Vector3

from numpy import *
from matplotlib import *
from pylab import *

#*****************************************************************
#       parameter initialization
#*****************************************************************

flag_read_odometry = True 

flag_debug = False
flag_diagnosis = False

h = 0.285
b = 0.0625
r = 0.0625

alpha_0 = -pi/2
alpha_1 = pi/2

inertial_frame = '/odom'
robot_frame = '/base_link'

#   P_dot = F(PHI, PHI_dot)
#   dlt_P = F(PHI, dlt_PHI)

class deduced_reckoning:
    def __init__(self):

        self.x = 0
        self.y = 0
        self.theta = 0

        self.flag_old_odom = False

        self.cnt = 0

        if flag_diagnosis:
            print('...kinematic data already recorded for diagnosis...')

            fs = open('diagnosis_fwd_kinematics.txt','w')
            aux = 't'+'\t'+ 'x' + '\t' + 'y'+ '\t' + 'theta' + '\n'
            fs.write(aux)
            fs.close()

        rospy.init_node('deduced_reckoning',anonymous=True)

        self.start_time = rospy.get_time()

        self.br = tf.TransformBroadcaster()
        self.pub_odom_robot = rospy.Publisher('/odom', nav_msgs.msg.Odometry, queue_size=10)

        if not flag_read_odometry:
            rospy.Subscriber('/jnt_ref_cmd', sensor_msgs.msg.JointState, self.callback_joints_odom)
        else:
            rospy.Subscriber('/odom_joints', sensor_msgs.msg.JointState, self.callback_joints_odom)

        rospy.spin()

    def callback_joints_odom(self, data):
        self.cnt += 1

        if self.cnt %5 == 3:

            if self.flag_old_odom:

                current_time = rospy.get_rostime()

                dlt_beta_0 = data.position[0]-self.beta0
                dlt_phi_0 = data.position[1]-self.phi0
                dlt_beta_1 = data.position[2]-self.beta1
                dlt_phi_1 = data.position[3]-self.phi1

                dlt_time = rospy.get_time() - self.old_time

                if flag_debug:
                    rospy.loginfo('... odom data retrieval update: %f', dlt_time)

                [dlt_x, dlt_y, dlt_theta] = self.dlt_P_from_dlt_PHI\
                        (dlt_beta_0, dlt_phi_0, dlt_beta_1, dlt_phi_1)

                self.x += dlt_x
                self.y += dlt_y
                self.theta += dlt_theta
                self.theta = (self.theta + pi)%(2*pi) - pi

                quaternion = tf.transformations.quaternion_from_euler(0,0,self.theta)
                self.br.sendTransform\
                    ((self.x, self.y, 0), quaternion, \
                    rospy.Time.now(), '/base_link', '/odom')
#
#                beta_dot_0 = dlt_beta_0 / dlt_time
#                phi_dot_0 = dlt_phi_0 / dlt_time
#                beta_dot_1 = dlt_beta_1 / dlt_time
#                phi_dot_1 = dlt_phi_1 / dlt_time
#
#                P_dot_r = self.P_dot_r_from_PHI_dot(beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1)
#                print(P_dot_r)
##
#                odom = nav_msgs.msg.Odometry()
#
#                odom.header.stamp = current_time
#                odom.header.frame_id = inertial_frame
#
#                odom.pose.pose.position = Point(self.x, self.y, 0.0)
#                odom.pose.pose.orientation = quaternion
#
#                odom.child_frame_id = robot_frame
#                odom.twist.twist.linear = Vector3(P_dot_r[0],P_dot_r[1], 0.0)
#                odom.twist.twist.angular = Vector3(0, 0, P_dot_r[2])
#
#                self.pub_odom_robot.publish(odom)

#   diagnosis
                if flag_diagnosis:
                    fs = open('diagnosis_fwd_kinematics.txt','a')

                    aux_time = rospy.get_time()-self.start_time
                    aux = str(aux_time)+ '\t' + str(self.x) + '\t' + str(self.y) + '\t' + str(self.theta) + '\n'
                    fs.write(aux)
                    fs.close()

    #   store data for comparison

            self.beta0 = data.position[0]
            self.phi0 = data.position[1]
            self.beta1 = data.position[2]
            self.phi1 = data.position[3]

            self.old_time = rospy.get_time()
            self.flag_old_odom = True

#   INSTANTANEOUS KINEMATICS   A(P,PHI)*dlt_P = M2(PHI)*dlt_PHI
#   input: dlt_PHI <- [beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1]
#   output: dlt_P <- [dlt_x, dlt_y, dlt_theta]
    def dlt_P_from_dlt_PHI\
        (self, dlt_beta_0, dlt_phi_0, dlt_beta_1, dlt_phi_1):
        A = array([  [cos(self.theta), sin(self.theta), -h*sin(alpha_0)+b*cos(alpha_0+self.beta0) ], \
                        [-sin(self.theta), cos(self.theta), h*cos(alpha_0)+b*sin(alpha_0+self.beta0)], \
                      [cos(self.theta), sin(self.theta), -h*sin(alpha_1)+b*cos(alpha_1+self.beta1) ], \
                      [ -sin(self.theta), cos(self.theta), h*cos(alpha_1)+b*sin(alpha_1+self.beta1)]   ])

        M_1 = array([   [cos(alpha_0+self.beta0), -sin(alpha_0+self.beta0), 0, 0], \
                        [sin(alpha_0+self.beta0), cos(alpha_0+self.beta0), 0, 0], \
                        [0, 0, cos(alpha_1+self.beta1), -sin(alpha_1+self.beta1)], \
                        [0, 0, sin(alpha_1+self.beta1), cos(alpha_1+self.beta1)]])
        M_2= array([  [-1*b, 0, 0, 0], \
                        [0, r, 0, 0], \
                        [0, 0, -1*b, 0], \
                        [0, 0, 0, r] ])

        dlt_PHI = array([   dlt_beta_0, dlt_phi_0, dlt_beta_1, dlt_phi_1 ])
        tmp = dot(M_2, dlt_PHI)
        aux = dot(M_1, tmp)

#        dlt_P = linalg.lstsq(A,aux)[0]

        M = dot(A.T,A)
        n = dot(A.T, aux)
        dlt_P = linalg.solve(M,n)

        return dlt_P

#   INSTANTANEOUS KINEMATICS   A(PHI)*P_dot_r = M2(PHI)*PHI_dot
#   input: PHI_dot <- [beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1]
#   output: P_dot_r <- [x_dot_r, y_dot_r, theta_dot_r]
    def P_dot_r_from_PHI_dot\
        (self, beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1):
        A = array([  [1, 0, -h*sin(alpha_0)+b*cos(alpha_0+self.beta0) ], \
                        [0, 1, h*cos(alpha_0)+b*sin(alpha_0+self.beta0)], \
                      [1, 0, -h*sin(alpha_1)+b*cos(alpha_1+self.beta1) ], \
                      [ 0, 1, h*cos(alpha_1)+b*sin(alpha_1+self.beta1)]   ])

        M_1 = array([   [cos(alpha_0+self.beta0), -sin(alpha_0+self.beta0), 0, 0], \
                        [sin(alpha_0+self.beta0), cos(alpha_0+self.beta0), 0, 0], \
                        [0, 0, cos(alpha_1+self.beta1), -sin(alpha_1+self.beta1)], \
                        [0, 0, sin(alpha_1+self.beta1), cos(alpha_1+self.beta1)]])
        M_2= array([  [-1*b, 0, 0, 0], \
                        [0, r, 0, 0], \
                        [0, 0, -1*b, 0], \
                        [0, 0, 0, r] ])

        PHI_dot = array([   beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1 ])
        tmp = dot(M_2, PHI_dot)
        aux = dot(M_1, tmp)

#        dlt_P = linalg.lstsq(A,aux)[0]

        M = dot(A.T,A)
        n = dot(A.T, aux)
        P_dot_r = linalg.solve(M,n)

        return P_dot_r

if __name__== '__main__':
    try:
        deduced_reckoning()
    except rospy.ROSInterruptException: pass

