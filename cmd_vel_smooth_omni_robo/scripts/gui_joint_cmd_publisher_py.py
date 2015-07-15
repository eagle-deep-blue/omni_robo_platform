#!/usr/bin/env python

#Author: Solomon Jingchun YIN,
#Contact Address: jingchun.yin@live.com
#Completed on 25th Apr., 2015

import Tkinter

from numpy import array, zeros, arange

from joint_cmd_publisher import *

angular_velocity_limit = int(pi/8*1000)

class gui_joint_cmd_publisher:
    def __init__(self, root):
        self.old_cmd = (0,0,0,0)
        
        root.title("JOINT-Space Velocy Commands")
    	frame = Tkinter.Frame(root,width=1200,height=500)
    	frame.pack() 
       
       
        
        self.sc_st_rt = Tkinter.Scale(frame, from_=-angular_velocity_limit, to=angular_velocity_limit, length=300, orient=Tkinter.HORIZONTAL)
        self.sc_st_rt.pack()
        Tkinter.Label(frame, text="Right STEERING Motor").pack()

        self.sc_sp_rt = Tkinter.Scale(frame, from_=-angular_velocity_limit, to=angular_velocity_limit, length=300, orient=Tkinter.HORIZONTAL)
        self.sc_sp_rt.pack()
        Tkinter.Label(frame, text="Right Spinning Motor").pack()

        self.sc_st_lf = Tkinter.Scale(frame, from_=-angular_velocity_limit, to=angular_velocity_limit, length=300, orient=Tkinter.HORIZONTAL)
        self.sc_st_lf.pack()
        Tkinter.Label(frame, text="Left STEERING Motor").pack() 
        
        self.sc_sp_lf = Tkinter.Scale(frame, from_=-angular_velocity_limit, to=angular_velocity_limit, length=300, orient=Tkinter.HORIZONTAL)
        self.sc_sp_lf.pack()
        Tkinter.Label(frame, text="Left Spinning Motor").pack()
        
        Tkinter.Label(frame, text="(micro_radian/s)").pack()
        Tkinter.Label(frame, text="range: +/- pi/8").pack()
        Tkinter.Button(frame, text="EXECUTE",justify=Tkinter.CENTER,command=self.callback_start).pack()
        Tkinter.Button(frame, text="RESET",justify=Tkinter.CENTER,command=self.callback_reset).pack()
        Tkinter.Button(frame, text="QUIT",justify=Tkinter.CENTER,command=root.quit).pack()
        
        self.old_cmd = zeros(4)
        self.threshold_cmd_variation = 1e-3
        
    def callback_start(self):
        beta_dot_0 = float(self.sc_st_rt.get())*1e-3
        phi_dot_0 = float(self.sc_sp_rt.get())*1e-3
        beta_dot_1 = float(self.sc_st_lf.get())*1e-3
        phi_dot_1 = float(self.sc_sp_lf.get())*1e-3

        try:
            joint_cmd_publisher(self.old_cmd, (beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1), False)
            self.old_cmd = (beta_dot_0, phi_dot_0, beta_dot_1, phi_dot_1)
        except rospy.ROSInterruptException: pass

    def callback_reset(self):
        self.sc_st_rt.set(0)
        self.sc_st_lf.set(0)
        self.sc_sp_rt.set(0)
        self.sc_sp_lf.set(0)
        self.callback_start() 
if __name__=='__main__':
    root=Tkinter.Tk()
    gui_joint_cmd_publisher(root)
    root.mainloop()
    root.destroy()
