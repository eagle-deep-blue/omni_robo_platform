#!/usr/bin/env python

#Author: Solomon Jingchun YIN,
#Contact Address: jingchun.yin@live.com
#Completed on 25th Apr., 2015

import rospy
import sensor_msgs.msg

from numpy import *
from matplotlib import *
from pylab import *

flag_debug = True

frequency = 80

acc_angular_limit = pi

class joint_cmd_publisher:
    def __init__(self,(b_d_0_old, p_d_0_old, b_d_1_old, p_d_1_old), \
                (beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1), flag_plot_cmds = True):

#*****************************************************************
#       PLANNING: velocity profile        
#*****************************************************************
        flag_last_cmd = False
        
        cmd_new = array([   beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1])
        cmd_old = array([   b_d_0_old, p_d_0_old, b_d_1_old, p_d_1_old])
        dlt_cmd = cmd_new - cmd_old
        dlt_time = max(abs(dlt_cmd/acc_angular_limit))
        
        if not dlt_time == 0:
            slope_b_0 = dlt_cmd[0]/dlt_time
            slope_ph_0 = dlt_cmd[1]/dlt_time        
            slope_b_1 = dlt_cmd[2]/dlt_time        
            slope_ph_1 = dlt_cmd[3]/dlt_time        
        else:
            slope_b_0 = 0
            slope_ph_0 = 0
            slope_b_1 = 0
            slope_ph_1 = 0
        SLOPE = array([ slope_b_0, slope_ph_0, slope_b_1, slope_ph_1])

#*****************************************************************
#       open-loop commands
#*****************************************************************       
#       ROS stuff
        rospy.init_node("joint_cmd_publisher", anonymous = True)
        pub_cmd_vel = rospy.Publisher("/jnt_ref_cmd", sensor_msgs.msg.JointState, queue_size=10)
        
        loop_rate = rospy.Rate(frequency)
        cnt = 0
        
        if flag_plot_cmds:
            cmds = []
        
        while not rospy.is_shutdown():
            if cnt == 0:
                time_initial = rospy.get_time()
                t_k = 0
            else:
                t_k = rospy.get_time() - time_initial
                
            cmd = sensor_msgs.msg.JointState()
            aux = cmd_old + SLOPE * t_k
            cmd.velocity = aux.tolist()
            pub_cmd_vel.publish(cmd)
            
            if flag_debug:
                if flag_last_cmd:
                        dlt_btw_cmds = rospy.get_time()-time_last_cmd
                        rospy.loginfo('...{DELTA_T}=[%f] (ms) for cmd update...', dlt_btw_cmds*1e3)
                time_last_cmd = rospy.get_time()
                flag_last_cmd = True
                
            if flag_plot_cmds:
                cmds.append([cmd.velocity[0],cmd.velocity[1],cmd.velocity[2],cmd.velocity[3],t_k])

            if t_k >= dlt_time:
                if flag_plot_cmds:
                    self.plot_cmds(cmds)
                return      
                
            cnt += 1                
            loop_rate.sleep()           
            
    def plot_cmds(self, cmds):
        aux = array(cmds)
        print(aux.shape)
        fig=figure()
        subplot(4,1,1)
        plot(aux[:,4],aux[:,0],'rx-')
        ylabel(r'$\dot{beta}_0$ (rad/s)', fontsize=15)
        subplot(4,1,2)
        plot(aux[:,4],aux[:,1],'rx-')
        ylabel(r'$\dot{\phi}_0$ (rad/s)', fontsize=15)            
        subplot(4,1,3)
        plot(aux[:,4],aux[:,2],'rx-')
        ylabel(r'$\dot{\beta}_1$ (rad/s)', fontsize=15)        
        subplot(4,1,4)
        plot(aux[:,4],aux[:,3],'rx-')
        ylabel(r'$\dot{\phi}_1$ (rad/s)', fontsize=15)        
        xlabel(r'$t$ (s)', fontsize=15)
        fig.suptitle('Cmds Delivered',fontsize=20,fontweight='bold')
        show()
            
if __name__=='__main__':
    if not len(sys.argv) == 9:
        print('...enter the arguments in sequence: [beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1]...')
        sys.exit()
    else:
        b_d_0_old  = float(sys.argv[1])
        p_d_0_old= float(sys.argv[2])
        b_d_1_old = float(sys.argv[3])
        p_d_1_old = float(sys.argv[4])  
        beta_dot_0= float(sys.argv[5]) 
        phi_dot_0  = float(sys.argv[6]) 
        beta_dot_1 = float(sys.argv[7]) 
        phi_dot_1 = float(sys.argv[8]) 
    try:
        joint_cmd_publisher((b_d_0_old, p_d_0_old, b_d_1_old, p_d_1_old), \
                (beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1))
    except rospy.ROSInterruptException: pass        
