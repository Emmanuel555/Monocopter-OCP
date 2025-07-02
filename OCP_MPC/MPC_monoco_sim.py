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
    kp = np.array([2.5,2.5,2.5])

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
    r_cost = np.array([0.1, 0.1, 0.05])

    # thrust rate
    ku = 1.0

    # Solver terms
    t_horizon = 1/20 # 20 Hz
    Nodes = 20 

    # MPC Monoco Model
    monoco_name = "short"
    monoco_type = SAM.SAM(monoco_name)

    # MPC Monoco INDI Control & Optimizer
    monoco = MPC_monoco_att_ctrl.att_ctrl(krr, ku, q_cost, r_cost, monoco_type, time_horizon=t_horizon, nodes=Nodes)
    
    time_last = time.time()
    
    try:
        while True:  
            start = timeit.default_timer() 
            abs_time = time.time() - time_last

            # update positions
            linear_state_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            rotational_state_vector = [[0,0,0],[0,0,0],[0,0,0]]
            tpp_quat = [[0,0,0,1],[0,0,0,1],[0,0,0,1]]
            dt = 1/20
            z_offset = 0.0
            body_yaw = 0.0 # hold on to this first
            yawrate = 11.2 # 11.2 hz
            monoco.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)

            # update references for manual control
            manual_cyclic = np.array([1.0, 0.0, 1.0])  # ref pos to xyz

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
            control_outputs = monoco.test_MPC_SIM_get_angles_and_thrust() # roll and pitch torque requirements into motor values 
            cmd_bod_acc = control_outputs[0]
            des_rps = control_outputs[1]
            cyclic = control_outputs[2]
            motor_soln = control_outputs[3]

            print(f"Roll pitch cyclic inputs are: {cyclic}, and size is: {cyclic.shape}, des_rps is {des_rps}, motor_soln is {motor_soln}")
            stop = timeit.default_timer()
            time_last = time.time()
            print(f"Program Runtime: {stop - start}, Program running at: {1/abs_time} Hz") 

            time.sleep(0.0495)
            #monoco.ocp_stats()

    except KeyboardInterrupt: 
        print("Exiting the program...")

        

