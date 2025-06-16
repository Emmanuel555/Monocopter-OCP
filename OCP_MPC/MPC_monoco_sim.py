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
    ka = np.array([0.01,0.01,0.01])
   
    # velocity
    kv = np.array([0.01,0.01,0.01])
    
    # bodyrates
    kr = np.array([0.01,0.01,0.01])

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
            print("Starting the MPC Monoco simulation...")

    except KeyboardInterrupt: 
        print("Exiting the program...")

        

