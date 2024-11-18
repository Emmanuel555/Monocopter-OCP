import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

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
    def __init__(self,z_gains):
        self.robot_pos = np.array([0,0,0]) # x y z
        self.robot_quat = np.array([0,0,0,0]) # x y z w
        self.kpz = z_gains[0]
        self.kdz = z_gains[1]
        self.kiz = z_gains[2]
        self.dt = 0
        self.ref_pos = np.array([0,0,0]) 
        self.position_error_last = np.array([0, 0, 0])
        self.control_signal = np.array([0,0,0]) 
        self.z_offset = 0
        

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
        kpa = np.array([1.0, 1.0]) # abt x y
        qz = quaternion.create(quat[0], quat[1], quat[2], 1) # x y z w
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
        kpx = 12_000
        kpy = 12_000
        kpz = self.kpz
        p_gains = np.array([kpx, kpy, kpz])

        kdx = 5_000
        kdy = 5_000
        kdz = self.kdz
        d_gains = np.array([kdx, kdy, kdz])

        kix = 0
        kiy = 0
        kiz = self.kiz
        i_gains = np.array([kix, kiy, kiz])
        I_term_z_prior = 0
        I_term_prior = np.array([0, 0, I_term_z_prior])

        position_error = self.ref_pos - self.robot_pos # calculate position error
        rate_posiition_error = (position_error - self.position_error_last)/self.dt
        integral_error = (position_error*self.dt) + I_term_prior
        self.position_error_last = position_error
        
        # pid controller
        robot_mg = np.array([0,0,47500]) # robot weight
        self.control_signal = (p_gains * position_error) + (d_gains * rate_posiition_error) + (i_gains * integral_error) + robot_mg
        

    def get_angles_and_thrust(self,enable):
        self.control_input()
        cmd_att = self.attitude_loop(self.robot_quat, self.control_signal)
        des_roll = int(cmd_att[0]*180/math.pi)
        des_pitch = int(cmd_att[1]*180/math.pi)
        des_thrust = int(self.control_signal[2])

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

        final_cmd = np.array([des_roll, des_pitch, 0, enable*des_thrust])

        return (final_cmd)

