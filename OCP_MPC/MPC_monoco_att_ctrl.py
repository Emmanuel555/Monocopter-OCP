import logging
import time

import Utils.Mocap as Mocap
import Utils.DataSave as DataSave

import math
from scipy.optimize import fsolve
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

import MPC_optimizer_monoco
import SAM


class att_ctrl(object):
    def __init__(self, body_rate_rate_gains, q_cost, r_cost, monoco):
        ## feedback
        self.robot_pos = np.array([0.0,0.0,0.0]) # x y z
        self.robot_vel = np.array([0.0,0.0,0.0]) # x y z
        self.robot_acc = np.array([0.0,0.0,0.0]) # x y z

        self.robot_quat = np.array([0.0,0.0,0.0,0.0]) # x y z w
        self.robot_quat_x = np.array([0.0,0.0,0.0,0.0]) # x y z w
        self.robot_quat_y = np.array([0.0,0.0,0.0,0.0]) # x y z w

        self.robot_tpp = np.array([0.0,0.0,0.0]) # x y z
        self.robot_tpp_bod_rate = np.array([0.0,0.0,0.0]) # x y z
        self.robot_tpp_bod_raterate = np.array([0.0,0.0,0.0]) # x y z

        self.yaw = 0.0
        self.yawrate = 0.0
        
        ## gains
        # body rate rates
        self.kprr = np.array(body_rate_rate_gains) # abt x y # control effectiveness
        ## sampling time
        self.dt = 0

        ## references
        self.ref_pos = np.array([0.0,0.0,0.0]) 
        self.ref_pos_z = 0.0
        self.ref_vel = np.array([0.0,0.0,0.0]) 
        self.ref_acc = np.array([0.0,0.0,0.0]) 
        self.ref_jer = np.array([0.0,0.0,0.0]) 
        self.ref_sna = np.array([0.0,0.0,0.0]) 
        
        
        ## control signals
        self.p_control_signal = np.array([0.0,0.0,0.0]) 
        self.v_control_signal = np.array([0.0,0.0,0.0]) 
        self.cmd_z = 0.0
        self.cmd_att = np.array([0.0,0.0])
        self.cmd_bod_rates_final = np.array([0.0,0.0])
        self.cmd_bod_raterates_final = np.array([0.0,0.0])
        self.cascaded_ref_bod_rates = np.array([0.0,0.0])

        # differential flatness
        self.ref_rates = np.array([0.0,0.0])
        self.ref_raterates = np.array([0.0,0.0])

        # attitude error tracking
        self.attitude_error = 0.0
        self.attitude_rate_error = 0.0
        self.attitude_raterate_error = 0.0

        # model
        self.monoco = monoco
        self.mpc_monoco = MPC_optimizer_monoco.Monoco_Optimizer(monoco_type=self.monoco, model_name=self.monoco.monoco_name+"_monoco_acados_mpc", q_cost=q_cost, r_cost=r_cost)


    def linear_ref(self,ref_pos,ref_vel,ref_acc,ref_jer,ref_sna,ref_pos_z):
        self.ref_pos = ref_pos
        self.ref_pos_z = ref_pos_z
        self.ref_vel = ref_vel
        self.ref_acc = ref_acc    
        self.ref_jer = ref_jer
        self.ref_sna = ref_sna 


    def update(self, linear_pos, rotational_pos, rotational_quat, dt, z_offset, yaw, quat_x, quat_y, yawrate):
        self.z_offset = z_offset
        self.robot_pos = np.array(linear_pos[0:3])
        self.robot_vel = np.array(linear_pos[3:6])
        self.robot_acc = np.array(linear_pos[6:9])
        self.robot_quat = rotational_quat
        self.robot_tpp = np.array(rotational_pos[0])
        self.robot_tpp_bod_rate = np.array(rotational_pos[1])
        self.robot_tpp_bod_raterate = np.array(rotational_pos[2])
        self.dt = dt
        self.yaw = yaw
        self.robot_quat_x = quat_x
        self.robot_quat_y = quat_y
        self.yawrate = yawrate
        

    def update_ref_pos(self, ref_pos):
        self.ref_pos = ref_pos 
    

    def p_control_input_manual(self,manual_input):
        self.p_control_signal = manual_input
    

    def body_rate_loop(self,cascaded_ref_bod_rates):
        kpr = self.kpr
        fb = np.array(self.robot_tpp_bod_rate[0:2]) # abt x y z

        # body rate controller
        cmd_bod_rates_error = cascaded_ref_bod_rates - fb
        self.attitude_rate_error = cmd_bod_rates_error
        self.cmd_bod_rates_final = kpr*(cmd_bod_rates_error)
        return (self.cmd_bod_rates_final)
    

    def INDI_loop(self,cascaded_ref_bod_acc):
       
        kprr = self.kprr
       
        fb = np.array(self.robot_tpp_bod_raterate[0:2]) # abt x y z
        cmd_bod_acc_error = cascaded_ref_bod_acc - fb
        self.attitude_raterate_error = cmd_bod_acc_error
        cmd_bod_acc_final = kprr*(cmd_bod_acc_error) 
        self.cmd_bod_raterates_final = cmd_bod_acc_final # for logging purposes

        ## NDI
        #cmd_bod_acc_final = kprr*(cascaded_ref_bod_acc)
        return (cmd_bod_acc_final)
    
    
    def get_angle(self): # corrected
        control_input = self.p_control_signal #+ self.v_control_signal
        cmd_att = self.attitude_loop(self.robot_quat, control_input)
        self.cmd_att = cmd_att #rpy
        return (self.cmd_att)
    

    def get_body_rate(self,flatness_option):
        if flatness_option == 0:
            self.cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att)
        else:
            self.cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att) + self.include_jerk_bod_rates()
        return (self.cascaded_ref_bod_rates)
    

    def CF_SAM_get_angles_and_thrust(self,enable,flatness_option):
        # output saturation/normalisation
        des_rps = self.des_rps
        #cmd_bod_acc = self.cmd_att
        
        ## icra method
        #cmd_bod_acc = self.cmd_att + self.include_snap_bod_raterate() + self.include_jerk_bod_rates()


        if flatness_option == 0:
            #cascaded_ref_bod_rates = self.body_rate_loop(self.cmd_att)
            cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
        else:
            #cascaded_ref_bod_rates = self.body_rate_loop(cmd_att)
            #cascaded_ref_bod_rates = self.include_jerk_bod_rates() + cascaded_ref_bod_rates
            cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
            cmd_bod_acc = self.include_snap_bod_raterate() + cmd_bod_acc
            #cmd_att = cmd_att + self.include_snap_bod_raterate() + self.include_jerk_bod_rates()
        

        ## to account for phase delay
        x_sign = math.sin(self.yaw)
        y_sign = math.cos(self.yaw)

        cmd_bod_acc[0] = cmd_bod_acc[0] * y_sign * -1 
        cmd_bod_acc[1] = cmd_bod_acc[1] * x_sign * -1
        

        # output saturation (cmd_att)
        if abs(cmd_bod_acc[0]) > 10000:
            cmd_bod_acc[0] = 10000*(cmd_bod_acc[0]/abs(cmd_bod_acc[0]))        
        if abs(cmd_bod_acc[1]) > 10000:
             cmd_bod_acc[1] = 10000*(cmd_bod_acc[1]/abs(cmd_bod_acc[1]))


        # motor saturation
        # if final_motor_output > 65500:
        #     final_motor_output = 65500
        # elif final_motor_output < 10:
        #     final_motor_output = 10

        #final_cmd = np.array([final_motor_output, final_motor_output, final_motor_output, final_motor_output])
        
        #self.cmd_z = enable*des_thrust

        #return (final_cmd)
        return (cmd_bod_acc)


    def include_jerk_bod_rates(self):
        wy = self.ref_jer[0]/self.g
        wx = self.ref_jer[1]/(-1*self.g)
        
        ref_bod_rates = np.array([wy,wx]) # flattened array abt x y z
        self.ref_rates = ref_bod_rates 
        
        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_rates
    

    def include_snap_bod_raterate(self):
        wy_dot = self.ref_sna[0]/self.g
        wx_dot = self.ref_sna[1]/(-1*self.g)
        
        ref_bod_raterate = np.array([wy_dot,wx_dot]) # flattened array abt x y z
        self.ref_raterates = ref_bod_raterate

        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_raterate
    
    