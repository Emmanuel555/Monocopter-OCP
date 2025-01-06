import logging
import time

import Mocap
import DataSave
import Data_process_swarm

import math
from scipy.optimize import fsolve
from pyrr import quaternion
import numpy as np
import numpy.linalg as la


class att_ctrl(object):
    def __init__(self,p_gains,d_gains,i_gains,angle_gains,body_rate_gains,body_rate_rate_gains):
        ## feedback
        self.robot_pos = np.array([0,0,0]) # x y z
        self.robot_vel = np.array([0,0,0]) # x y z
        self.robot_acc = np.array([0,0,0]) # x y z
        self.robot_quat = np.array([0,0,0,0]) # x y z w
        self.robot_tpp_bod_rate = np.array([0,0,0]) # x y z
        self.robot_tpp_bod_raterate = np.array([0,0,0]) # x y z
        
        ## gains
        # pos
        self.kp = np.array(p_gains) 
        self.kd = np.array(d_gains) 
        self.ki = np.array(i_gains) 
        # attitude/angle
        # self.kpa = np.array([10, 10]) # abt x y
        self.kpa = np.array(angle_gains) # abt x y
        # body rates
        # self.kpr = np.array([50, 50]) # abt x y
        self.kpr = np.array(body_rate_gains) # abt x y
        # body rate rates
        # self.kprr = np.array([1000, 1000]) # abt x y
        self.kprr = np.array(body_rate_rate_gains) # abt x y
        
        ## sampling time
        self.dt = 0

        ## references
        self.ref_pos = np.array([0,0,0]) 
        self.ref_vel = np.array([0,0,0]) 
        self.ref_acc = np.array([0,0,0]) 
        self.ref_jer = np.array([0,0,0]) 
        self.ref_sna = np.array([0,0,0]) 
        
        ## physical parameters
        self.g = 9.81
        self.rho = 1.225
        self.wing_radius = 0
        self.chord_length = 0
        self.mass = 0
        self.cl = 0
        self.cd = 0

        ## bem
        self.drag_rotation_wo_rps = 0
        self.lift_rotation_wo_rps = 0


        ## control signals
        self.position_error_last = np.array([0, 0, 0])
        self.control_signal = np.array([0,0,0]) 
        self.z_offset = 0
        self.cmd_z = 0


    def physical_params(self,wing_radius,chord_length,mass,cl,cd):
        # cl = lift coefficient
        # cd = drag coefficient
        self.wing_radius = wing_radius
        self.chord_length = chord_length
        self.mass = mass
        self.cl = cl
        self.cd = cd


    def compute_bem_wo_rps(self,pitch):
        # pitch = pitch angle in degrees 
        # original formula for lift: (cl*pitch*(rps^2)*rho*chord_length*(wing_radius^3))/6
        self.drag_rotation_wo_rps = (self.cd*pitch*self.rho*self.chord_length*(self.wing_radius**3))/6 # has mass inside, not needed atm
        self.lift_rotation_wo_rps = (self.cl*pitch*self.rho*self.chord_length*(self.wing_radius**3))/6 # has mass inside


    def linear_ref(self,ref_pos,ref_vel,ref_acc,ref_jer,ref_sna):
        self.ref_pos = ref_pos
        self.ref_vel = ref_vel
        self.ref_acc = ref_acc    
        self.ref_jer = ref_jer
        self.ref_sna = ref_sna 


    def update(self, robot_locale, dt, ref_pos, z_offset):
        self.z_offset = z_offset
        self.robot_pos = np.array(robot_locale[0:3])
        self.robot_quat = np.array(robot_locale[3:7])
        self.dt = dt
        self.ref_pos = ref_pos
        self.ref_pos[2] = self.ref_pos[2] + self.z_offset


    def info_update_z(self):
        ref_pos = self.ref_pos
        ref_pos[2] = ref_pos[2] - self.z_offset 
        return ref_pos


    def attitude_loop(self, quat, control_input):
        kpa = self.kpa # abt x y
        qz = quaternion.create(quat[0], quat[1], quat[2], 1) # x y z w ## qw is always set to 1 even in optitrack itself
        qzi = quaternion.inverse(qz)
        ez = np.array([0, 0, 1]) # 3,:
        disk_vector = quaternion.apply_to_vector(qz, ez) # flattened array
        disk_vector = np.array([[disk_vector[0],disk_vector[1],disk_vector[2]]])  # 1 x 3 - row based simulated disk vector
        
        # pos control input here
        zd = control_input/la.norm(control_input,2)
        zd = np.array([[zd[0],zd[1],zd[2]]]) # 1 x 3 - row based simulated zd which is desired vector 
        num = np.dot(disk_vector,np.transpose(zd)) # 1 x 1
        den = la.norm(disk_vector,2)*la.norm(zd,2) # L2 norm of a and b
        angle = math.acos(num/den) # angle in radians

        n = np.cross(disk_vector,zd)/la.norm(np.cross(disk_vector,zd)) # cross product of a and b - 1 x 3
        n = list(n.flat) # flattened array
        B = quaternion.apply_to_vector(qzi, n) # inverse of qz applied to n
        error_quat = np.array([math.cos(angle/2), B[0]*math.sin(angle/2), B[1]*math.sin(angle/2), B[2]*math.sin(angle/2)]) # abt w x y z

        if error_quat[0] < 0:
            cmd_att = -2*error_quat[1:3]
        else:
            cmd_att = 2*error_quat[1:3] # bod_att[0] = abt x, bod_att[1] = abt y 
        cmd_att = kpa*(cmd_att) # abt x y z

        return (cmd_att)
    

    def control_input(self):
        # control gains
        # kpx = 12_000
        # kpy = 12_000
        # kpz = self.kpz
        # p_gains = np.array([kpx, kpy, kpz])
        p_gains = self.kp # p_gains = ([kpx, kpy, kpz])

        # kdx = 5_000
        # kdy = 5_000
        # kdz = self.kdz
        # d_gains = np.array([kdx, kdy, kdz])
        d_gains = self.kd # d_gains = ([kpx, kpy, kpz])

        # kix = 0
        # kiy = 0
        # kiz = self.kiz
        # i_gains = np.array([kix, kiy, kiz])
        i_gains = self.ki # i_gains = ([kpx, kpy, kpz])
        
        I_term_z_prior = 0
        I_term_prior = np.array([0, 0, I_term_z_prior])

        position_error = self.ref_pos - self.robot_pos # calculate position error
        rate_posiition_error = (position_error - self.position_error_last)/self.dt
        integral_error = (position_error*self.dt) + I_term_prior
        self.position_error_last = position_error
        
        # weight of the robot
        robot_mg = np.array([0,0,self.mass*self.g]) # robot weight, cf = 47500

        # position pid controller
        self.control_signal = (p_gains * position_error) + (d_gains * rate_posiition_error) + (i_gains * integral_error) + robot_mg

        # w acceleration references, velocity not needed as d term from position compensates that already
        self.control_signal = self.control_signal + self.ref_acc
        

    def body_rate_loop(self,cascaded_ref_bod_rates):
        # body rate gains
        # kpr = np.array([50, 50]) # abt x y
        kpr = self.kpr
        fb = np.array(self.robot_tpp_bod_rate[0:2]) # abt x y z

        # body rate controller
        cmd_bod_rates = kpr*(cascaded_ref_bod_rates - fb)
        return (cmd_bod_rates)
    

    def INDI_loop(self,cascaded_ref_bod_acc):
        # body rate gains
        # kpr = np.array([50, 50]) # abt x y
        kprr = self.kprr
        fb = np.array(self.robot_tpp_bod_raterate[0:2]) # abt x y z

        # body rate controller
        cmd_bod_acc = kprr*(cascaded_ref_bod_acc - fb)
        return (cmd_bod_acc)
    
    
    def get_angles_and_thrust(self,flatness_option):
        self.control_input()
        cmd_att = self.attitude_loop(self.robot_quat, self.control_signal)

        if flatness_option == 0:
            cascaded_ref_bod_rates = self.body_rate_loop(cmd_att)
            cmd_bod_acc = self.INDI_loop(cascaded_ref_bod_rates)
        else:
            cascaded_ref_bod_rates = self.body_rate_loop(cmd_att)
            cascaded_ref_bod_rates = self.include_jerk_bod_rates() + cascaded_ref_bod_rates
            cmd_bod_acc = self.INDI_loop(cascaded_ref_bod_rates)
            cmd_bod_acc = self.include_snap_bod_raterate() + cmd_bod_acc

        # in degrees
        #des_roll = int(cmd_att[0]*180/math.pi)
        #des_pitch = int(cmd_att[1]*180/math.pi)
        
        # in radians
        des_roll = int(cmd_bod_acc[0])
        des_pitch = int(cmd_bod_acc[1])

        # collective thrust - linearised
        des_rps = np.sqrt((int(self.control_signal[2]))/self.lift_rotation_wo_rps) # input to motor
        des_thrust = self.lift_rotation_wo_rps*(des_rps**2)

         # output saturation
        if des_roll > 25:
            des_roll = 25
        if des_roll < -25:
            des_roll = -25
        if des_pitch > 25:
            des_pitch = 25
        if des_pitch < -25:
            des_pitch = -25
        if des_thrust > 60_000:
            des_thrust = 60_000
        if des_thrust < 10:
            des_thrust = 10

        final_cmd = np.array([des_roll, des_pitch, des_rps]) # roll pitch rps
        self.cmd_z = des_thrust

        return (final_cmd)
    

    def include_jerk_bod_rates(self):
        if self.cmd_z == 0:
            wy = 0
            wx = 0
        else:    
            wy = self.ref_jer[0]/self.cmd_z
            wx = self.ref_jer[1]/(-1*self.cmd_z)
        
        ref_bod_rates = np.array([wx,wy]) # flattened array abt x y z
        
        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_rates
    

    def include_snap_bod_raterate(self):
        if self.cmd_z == 0:
            wy_dot = 0
            wx_dot = 0
        else:    
            wy_dot = self.ref_sna[0]/self.cmd_z
            wx_dot = self.ref_sna[1]/(-1*self.cmd_z)
        
        ref_bod_raterate = np.array([wx_dot,wy_dot]) # flattened array abt x y z
        
        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_raterate
    
    
"""     def INDI_loop(self):
        # INDI body rate rates
        self.kpang = np.array([1.0, 1.0, 1.0]) # 3,: flattened form
        self.kdang = np.array([0, 0, 0]) # 3,: flattened form
        self.kiang = np.array([0, 0, 0]) # 3,: flattened form

        # commanded inputs
        self.cmd_att = np.array([0, 0, 0]) # attitude
        self.cmd_bod_rates = np.array([0, 0, 0]) # body rates
        self.ff_snap = np.array([0, 0, 0]) # ff snap
        self.cmd_bod_rates_rates = np.array([0, 0, 0]) # body rate rates

        
        if self.mode == 1:
           self.cmd_bod_rates_rates = self.kpang*(self.cmd_att - self.last_angular_rate_rate)
        elif self.mode == 3:
            self.cmd_bod_rates_rates = self.kpang*(((self.cmd_att + self.cmd_bod_rates + self.ff_snap)/3) - self.last_angular_rate_rate) 
        self.cmd_bod_rates_rates = self.MOI*self.cmd_bod_rates_rates
        return self.cmd_bod_rates_rates """


