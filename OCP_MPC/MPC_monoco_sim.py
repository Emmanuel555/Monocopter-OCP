import time
import pygame
import Utils.Mocap as Mocap
import Utils.DataSave as DataSave
import Localization.Data_process as Data_process
import numpy as np
import timeit
import math
import numpy.linalg as la
from pyrr import quaternion

# specific to MPC 
import SAM
import MPC_monoco_att_ctrl

if __name__ == '__main__':

    time_start = time.time()
    time_end = time_start + 1000
    last_time = 0
    count = 0

    # position
    kp = np.array([1.3,1.3,1.3])

    # angle
    ka = np.array([0.0,0.0,0.0])
   
    # velocity
    kv = np.array([0.0,0.0,0.0])
    
    # bodyrates
    kr = np.array([0.0,0.0,0.0])

    # INDI Loop
    krr = [1.0, 1.0] # 1.0

    # MPC gains
    q_cost = np.concatenate((kp,ka,kv,kr))
    r_cost = np.array([0.01, 0.01, 0.01])

    # MPC Monoco Model
    monoco_name = "short"
    monoco_type = SAM.SAM(monoco_name)
    # MPC Monoco INDI Control & Optimizer
    monoco = MPC_monoco_att_ctrl.att_ctrl(krr, q_cost, r_cost, monoco_type)

    
    try:
        while True:  

            # print("Starting the MPC Monoco simulation...")

            # update positions
            linear_state_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            rotational_state_vector = [[0,0,0],[0,0,0],[0,0,0]]
            tpp_quat = [[0,0,0,1],[0,0,0,1],[0,0,0,1]]
            dt = 1/250
            z_offset = 0.0
            body_yaw = 0.0 # hold on to this first
            yawrate = 11.2 # 11.2 hz
            monoco.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)

            # update references for manual control
            manual_cyclic = np.array([0.0, 1.0, 1.0])  # ref pos to xyz

            # Simulated control via MPC
            ff = np.array([0.0, 0.0, 0.0])
            ref_pos = manual_cyclic
            ref_vel = ff
            ref_acc = ff
            ref_jerk = ff
            ref_snap = ff
            ref_msg = 'Manual_flight'
            monoco.linear_ref(ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap)
            monoco.p_control_input_manual(ref_pos) # update the ref states

            # from att ctrl
            control_outputs = monoco.MPC_SAM_get_angles_and_thrust()
            raw_cmd_bod_acc = control_outputs[3]
            
            # att_raterate_error = monoco.attitude_raterate_error
            
            print(f"control inputs are: {raw_cmd_bod_acc} and size is: {raw_cmd_bod_acc.shape}")
            
            #monoco.ocp_stats()

    except KeyboardInterrupt: 
        print("Exiting the program...")

        

