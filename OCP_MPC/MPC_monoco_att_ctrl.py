import logging
import time

import Utils.Mocap as Mocap
import Utils.DataSave as DataSave

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

import MPC_optimizer_monoco
import SAM


class att_ctrl(object):
    def __init__(self, body_rate_rate_gains, thrust_rate, q_cost, r_cost, monoco, time_horizon, nodes):
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

        # differential flatness - MPC reference states
        self.ref_att = np.array([0.0,0.0,0.0])
        self.ref_rates = np.array([0.0,0.0,0.0])
        self.ref_raterates = np.array([0.0,0.0,0.0])

        # attitude error tracking
        self.attitude_error = 0.0
        self.attitude_rate_error = 0.0
        self.attitude_raterate_error = 0.0
        self.position_error_last = np.array([0.0, 0.0, 0.0])

        # thrust rate
        self.previous_motor_cmd = 0.0
        self.previous_motor_error = 0.0
        self.kup = thrust_rate[0]
        self.kud = thrust_rate[1]
        self.kui = thrust_rate[2]


        # model
        self.g = 9.81
        self.monoco = monoco
        self.mpc_monoco = MPC_optimizer_monoco.Monoco_Optimizer(monoco_type=self.monoco, 
                                                                model_name=self.monoco.monoco_name, q_cost=q_cost, 
                                                                r_cost=r_cost, t_horizon=time_horizon, n_nodes=nodes)


    def ocp_stats(self):
        self.mpc_monoco.ocp_stats()
        

    def linear_ref(self,ref_pos,ref_vel,ref_acc,ref_jer,ref_sna):
        self.ref_pos = ref_pos
        self.ref_vel = ref_vel
        self.ref_acc = ref_acc    
        self.ref_jer = ref_jer
        self.ref_sna = ref_sna 


    def update(self, linear_pos, rotational_pos, rotational_quat, dt, z_offset, yaw, quat_x, quat_y, yawrate, bod_pitch):
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
        self.bod_pitch = bod_pitch
        

    def p_control_input_manual(self,manual_input):
        self.p_control_signal = manual_input
        self.opti_ref_states()

        
    def opti_ref_states(self):
        self.ref_acc_att()
        self.ref_jerk_bod_rates()
        self.ref_snap_bod_raterate()
        self.mpc_monoco.set_reference_state(x_target=[self.p_control_signal,self.ref_att,self.ref_vel,self.ref_rates])


    def rotational_drag(self):
        self.mpc_monoco.rotational_drag(self.bod_pitch)

    
    def opti_output_control(self): # change robot pos[2] to rotational rate tmr...
        self.robot_tpp_bod_rate[2] = self.yawrate
        initial_state_taken = np.concatenate((self.robot_pos,self.robot_tpp,self.robot_vel,self.robot_tpp_bod_rate))
        opt_output = self.mpc_monoco.run_optimization(initial_state=initial_state_taken)
        control_inputs = opt_output[0]
        state_outputs = opt_output[1]
        return (control_inputs, state_outputs)


    def INDI_loop(self,cascaded_ref_bod_acc):
        kprr = self.kprr
        fb = np.array(self.robot_tpp_bod_raterate[0:2]) # towards x, y - abt y x
        cmd_bod_acc_error = cascaded_ref_bod_acc - fb
        self.attitude_raterate_error = cmd_bod_acc_error
        cmd_bod_acc_final = kprr*(cmd_bod_acc_error) 
        self.cmd_bod_raterates_final = cmd_bod_acc_final # for logging purposes

        ## NDI
        cmd_bod_acc_final = kprr*(cascaded_ref_bod_acc)
        return (cmd_bod_acc_final)
    

    def manual_collective_thrust(self,kpz,kdz,kiz): 
        # weight of the robot
        robot_mg = np.array([0.0,0.0,1000*self.g]) # robot weight, cf = 47500
        self.z_error = self.p_control_signal[2] - self.robot_pos[2]
        rate_position_error_z = (self.z_error - self.position_error_last[2])/self.dt
        integral_error_z = (self.z_error*self.dt)
        self.position_error_last[2] = self.z_error 

        p_error_z = kpz*(self.z_error) + kdz*(rate_position_error_z) + robot_mg[2] + kiz*(integral_error_z) + self.ref_acc[2] # z error
        
        # self.des_rps = p_error_z + manual_thrust
        self.des_rps = p_error_z

        # motor saturation
        if self.des_rps > 55500:
            self.des_rps = 55500
        elif self.des_rps < 10:
            self.des_rps = 10
        
        return (self.des_rps)
    
    
    def MPC_SAM_get_angles_and_thrust(self):
        # run entire MPC-SAM loop
        opti_outputs = self.opti_output_control()
        control_inputs = opti_outputs[0]
        state_outputs = opti_outputs[1]

        # collective thrust
        #des_rps = control_inputs[2] * self.monoco.max_thrust_collective
        #des_rps = des_rps * self.monoco.cf_max

        des_rps = control_inputs[2] * pow(self.monoco.max_thrust_collective,2)
        if des_rps < 0.0:
            des_rps = 0.0
        des_rps = np.sqrt(des_rps) * self.monoco.cf_max

        # cyclic
        cyclic = np.array(control_inputs[0:2]) * self.monoco.max_thrust_cyclic
        cyclic = cyclic * self.monoco.cf_max
        cmd_bod_acc = np.array([cyclic[1], cyclic[0]]) # abt y x only, roll, pitch
        raw_cmd_bod_acc = cmd_bod_acc

        self.cascaded_ref_bod_rates = cmd_bod_acc/self.monoco.J[0]         
        cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
        
    
        ## to account for phase delay
        x_sign = math.sin(self.yaw)
        y_sign = math.cos(self.yaw)

        cmd_bod_acc[0] = cmd_bod_acc[0] * y_sign * -1 
        cmd_bod_acc[1] = cmd_bod_acc[1] * x_sign * -1
        
        # # output saturation (cmd_att)
        # if abs(cmd_bod_acc[0]) > 10000:
        #     cmd_bod_acc[0] = 10000*(cmd_bod_acc[0]/abs(cmd_bod_acc[0]))        
        # if abs(cmd_bod_acc[1]) > 10000:
        #      cmd_bod_acc[1] = 10000*(cmd_bod_acc[1]/abs(cmd_bod_acc[1]))

        #final_motor_output = des_rps + cmd_bod_acc[0] + cmd_bod_acc[1]  # collective thrust + cyclic
        motor_error = des_rps - self.previous_motor_cmd
        motor_error_error = motor_error-self.previous_motor_error
        final_motor_output = self.previous_motor_cmd + self.kup*(motor_error) # collective thrust + cyclic
        
        # motor saturation
        if final_motor_output > 65500:
            final_motor_output = 65500
        elif final_motor_output < 10:
            final_motor_output = 10

        self.previous_motor_cmd = final_motor_output
        self.previous_motor_error = motor_error_error

        # return (final_motor_output, cmd_bod_acc, des_rps, raw_cmd_bod_acc)
        return (cmd_bod_acc, des_rps, raw_cmd_bod_acc, final_motor_output, state_outputs)
    

    def test_MPC_SIM_get_angles_and_thrust(self):
        # run entire MPC-SAM loop
        opti_outputs = self.opti_output_control()
        control_inputs = opti_outputs[0]
        state_outputs = opti_outputs[1]

        # collective thrust
        des_rps = control_inputs[2] * self.monoco.max_thrust_collective
        des_rps = des_rps * self.monoco.cf_max

        # cyclic
        cyclic = np.array(control_inputs[0:2]) * self.monoco.max_thrust_cyclic
        cyclic = cyclic * self.monoco.cf_max
        cmd_bod_acc = np.array([cyclic[1], cyclic[0]]) # abt y x only, roll, pitch
        raw_cmd_bod_acc = cmd_bod_acc
        
        self.cascaded_ref_bod_rates = cmd_bod_acc/self.monoco.J[0]         
        cmd_bod_acc = self.INDI_loop(self.cascaded_ref_bod_rates)
       
    
        ## to account for phase delay
        x_sign = math.sin(self.yaw)
        y_sign = math.cos(self.yaw)

        cmd_bod_acc[0] = cmd_bod_acc[0] * y_sign * -1 
        cmd_bod_acc[1] = cmd_bod_acc[1] * x_sign * -1
        
        # # output saturation (cmd_att)
        # if abs(cmd_bod_acc[0]) > 10000:
        #     cmd_bod_acc[0] = 10000*(cmd_bod_acc[0]/abs(cmd_bod_acc[0]))        
        # if abs(cmd_bod_acc[1]) > 10000:
        #      cmd_bod_acc[1] = 10000*(cmd_bod_acc[1]/abs(cmd_bod_acc[1]))

        #final_motor_output = des_rps + cmd_bod_acc[0] + cmd_bod_acc[1]  # collective thrust + cyclic
        final_motor_output = des_rps 

        # motor saturation
        if final_motor_output > 65500:
            final_motor_output = 65500
        elif final_motor_output < 10:
            final_motor_output = 10

        # return (final_motor_output, cmd_bod_acc, des_rps, raw_cmd_bod_acc)
        return (cmd_bod_acc, des_rps, raw_cmd_bod_acc, final_motor_output)


    def ref_acc_att(self):
        ay = self.ref_acc[0]/self.g
        ax = self.ref_acc[1]/(-1*self.g)
        
        ref_att = np.array([ay,ax,0]) # flattened array abt x y z
        self.ref_att = ref_att 
        
        return ref_att    


    def ref_jerk_bod_rates(self):
        wy = self.ref_jer[0]/self.g
        wx = self.ref_jer[1]/(-1*self.g)
        
        ref_bod_rates = np.array([wy,wx,0]) # flattened array abt x y z
        self.ref_rates = ref_bod_rates 
        
        return ref_bod_rates
    

    def ref_snap_bod_raterate(self):
        wy_dot = self.ref_sna[0]/self.g
        wx_dot = self.ref_sna[1]/(-1*self.g)
        
        ref_bod_raterate = np.array([wy_dot,wx_dot,0]) # flattened array abt x y z
        self.ref_raterates = ref_bod_raterate

        #self.cmd_bod_rates = self.kpr*(ref_bod_rates - self.last_angular_rate)
        return ref_bod_raterate
    
    