import logging
import time

import Mocap
import DataSave
import Data_process

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la


import monoco_att_ctrl
import trajectory_generator


# robot address
# Change uris and sequences according to your setup

# radio 1
#URI1 = 'radio://0/20/2M/E7E7E7E702'
#URI1 = 'radio://0/30/2M/E7E7E7E703'
URI1 = 'radio://0/30/2M/E7E7E7E703'



uris = {
    URI1,
}


def swarm_exe(cmd_att):
    seq_args = {
        URI1: [cmd_att[0]],
    }
    return seq_args


def init_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(0.1)
        cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        print("Initialisation swarming....set ur sticks to mid and down")
        time.sleep(0.1)
    except Exception as e:
        print("Initialisation swarming error: ", e)


def arm_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        #print('arming w thrust val....', cmds[3])
    except Exception as e:
        print("swarming error: ", e)


if __name__ == '__main__':

    data_receiver = Mocap.Udp()
    sample_rate = data_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'robot_1','ref_position')
                                   
    logging.basicConfig(level=logging.ERROR)
    
    time_last = 0
    count = 0

    # reference offset for z
    z_offset = 0.0

    # rmse terms
    rmse_num_x = 0
    final_rmse_x = 0
    rmse_num_y = 0
    final_rmse_y = 0
    rmse_num_z = 0
    final_rmse_z = 0


    # omega and omega_dot terms for robot
    last_robot = [0, 0, 0, 0, 0, 0, 0, 0]
    last_robot_dot = [0, 0, 0, 0, 0, 0, 0, 0]


    # trajectory generator
    traj_gen = trajectory_generator.trajectory_generator()


    # traj generator for min snap circle
    # pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle(0, 0.5, 1)
    ref_pos_1 = traj_gen.hover_test(0)
    pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(0, 2.0, 50, 5)  # (0, 1.2, 30) - 3 m/s, (0 , 1.4, 50) - 5m/s


    # collective
    kpz = 20
    kdz = 10
    kiz = 1
    
    # cyclic
    kp = [20,20,kpz]
    kd = [10,10,kdz]
    ki = [1,1,kiz]
    ka = [28.2, 28.2]
    kad = [0.116, 0.116]
    kr = [31.3, 31.3]
    krd = [0.799, 0.799]
    krr = [1.72, 1.72]
    
    att_robot = monoco_att_ctrl.att_ctrl(kp, kd, ki, ka, kad, kr, krd, krr)
            
    time_start = time.time()
    time_end = time.time() + 6000

    while time_end > time.time():
        abs_time = time.time() - time_start
        
        
        lowest = 0.9
        highest = -1
        range_js = -(highest - lowest)
        range_motor = 65535
        rate = range_motor / range_js
    

        # require data from Mocap
        data = data_receiver.get_data()

        # data unpack
        data_processor.data_unpack_filtered(data)

        # processed tpp data/feedback
        state_vector = data_processor.get_Omega_dot_dotdot_filt()
        tpp_angle = data_processor.tpp
        tpp_omega = data_processor.Omega
        tpp_omega_dot = data_processor.Omega_dot
        tpp_quat = data_processor.tpp_eulerAnglesToQuaternion


        # calculate velocity
        dt = time.time() - time_last  #  time difference
        time_last = time.time()


        # reference position
        #ref_pos_1 = traj_gen.simple_rectangle(0, abs_time)
        #ref_pos_1 = traj_gen.simple_circle(0, 0.25, count, 5)
        #ref_pos_1 = traj_gen.elevated_circle(0, 0.6, count)
        #ref_pos_1 = traj_gen.hover_test(0)


        #ref_pos_1 = traj_gen.helix(0, 0.4, count, 5)
        #ref_pos = ref_pos_1[0]


        ref_derivatives = traj_gen.jerk_snap_circle(pva,num_pts,count,0.25)
        ref_pos = ref_derivatives[0]
        ref_vel = ref_derivatives[1]
        ref_acc = ref_derivatives[2]
        ref_jerk = ref_derivatives[3]
        ref_snap = ref_derivatives[4]
        ref_msg = ref_derivatives[5]

        # update positions etc.
        att_robot_1.update(robot, dt, ref_pos, z_offset)

        """ # control input (arming test)
        cmd_att_arm = np.array([0, 0, 0, conPad * 1]) # optitrack control [roll,  pitch ,  yawrate, thrust]
        cmd_att = np.array([cmd_att_arm, cmd_att_arm, cmd_att_arm]) """
        
        # control input (traj execution)
        cmd_att_1 = att_robot_1.get_angles_and_thrust(enable)

        # feedforward terms
        ff_jerk_1 = att_robot_1.include_jerk_bod_rates(ref_jerk)
        ff_snap_1 = att_robot_1.include_snap_bod_raterate(ref_snap)
        
        # control input w js ff
        cmd_att_js = cmd_att_1[0:3] + ff_jerk_1 + ff_snap_1

        # concatenate w js ff
        cmd_att_1[0:3] = cmd_att_js

        # send to single js
        cmd_att = np.array([cmd_att_1])
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(arm_throttle, args_dict=seq_args)

        count = count + 1
        if count % 10 == 0:
            
            #print(abs_time) # updating at 120 hz
            print(ref_msg) 
            print('shapes: ', np.shape(cmd_att_js), np.shape(ff_jerk_1), np.shape(ff_snap_1))
            print('robot_position', robot[0], robot[1], robot[2])
            print('ref robot_position', ref_pos[0], ref_pos[1], ref_pos[2])
            print('pos_error', ref_pos[0]-robot[0], ref_pos[1]-robot[1], ref_pos[2]-robot[2])

        # rmse accumulation
        rmse_num_x = rmse_num_x + (ref_pos[0]-robot[0])**2
        rmse_num_y = rmse_num_y + (ref_pos[1]-robot[1])**2
        rmse_num_z = rmse_num_z + (ref_pos[2]-robot[2])**2
        
        # save data
        data_saver.add_item(abs_time,
                            robot,ref_pos
                            )

        if button0 == 1:
            """ # for hovering test
            ref_pos[2] = 0.15
            # descend
            att_robot_1.update(robot, dt, ref_pos, z_offset)
            cmd_att_1 = att_robot_1.get_angles_and_thrust(enable)
            cmd_att = np.array([cmd_att_1])
            seq_args = swarm_exe(cmd_att)
            swarm.parallel(arm_throttle, args_dict=seq_args) """

            # for traj 
            cmd_att_cut = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
            cmd_att = np.array([cmd_att_cut])
            seq_args = swarm_exe(cmd_att)
            swarm.parallel(init_throttle, args_dict=seq_args)
            final_rmse_x = math.sqrt(rmse_num_x/count)
            final_rmse_y = math.sqrt(rmse_num_y/count)
            final_rmse_z = math.sqrt(rmse_num_z/count)
            final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)
            print('Emergency Stopped and final rmse produced: ', final_rmse)
            break

# save data
path = '/home/emmanuel/AFC_Optitrack/jerk_snap/'
data_saver.save_data(path)

