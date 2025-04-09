import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

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
import timeit
from pynput import keyboard

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


# robot address
# Change uris and sequences according to your setup

# monoco radio 1
URI1 = 'radio://0/80/2M/E7E7E7E701'


uris = {
    URI1,
}


def swarm_exe(cmd_att):
    seq_args = {
        URI1: [cmd_att[0]],
    }
    return seq_args


def swarm_logging(swarm_log):
    seq_args = {
        URI1: [swarm_log[0]],
    }
    return seq_args


def init_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(0.1)

        # initialise the controller
        cf.param.add_update_callback(group='stabilizer', name='estimator',
                                 cb=param_stab_est_callback)
        cf.param.set_value('stabilizer.controller', '3')  # 3: INDI controller
        time.sleep(0.1)

        #cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) # roll, pitch, yawrate, thrust 
        cf.commander.send_position_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        print("Initialisation monoco....set ur sticks to mid and down")
        time.sleep(0.1)
    except Exception as e:
        print("Initialisation error: ", e)


def arm_throttle(scf, cmds):
    try:
        cf = scf.cf
        cf.param.set_value('stabilizer.controller', '3')  # 3: INDI controller
        cf.commander.send_position_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3]))  
        #print('arming w thrust val....', cmds[3])
    except Exception as e:
        print("swarming error: ", e)


def transmitter_calibration():
    # where hand control comes
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    lowest = 0.97
    highest = -1
    range_js = -(highest - lowest)
    range_motor = 65535
    rate = range_motor / range_js
    a0 = joystick.get_axis(0)  # x axis right hand <- ->
    a1 = joystick.get_axis(1)  # y axis right hand up down
    a2 = joystick.get_axis(2)  # thrust
    a3 = joystick.get_axis(3)

    button0 = joystick.get_button(0)
    button1 = joystick.get_button(1)

    # thrust from control pad
    conPad = int((a2 - highest) * rate)

    # throttle joystick saturation
    if conPad < 10:
        conPad = 10
    if conPad > 65500:
        conPad = 65500

    # takeoff sign
    if conPad < 2000:
        enable = 0
    else:
        enable = 1

    cmd = conPad*enable
    return (cmd, button0, button1, a0, a1, enable)


def att_manual_ctrl(a0,a1,af):
    pad_x = a0
    pad_y = a1
    trigger = math.sqrt(pad_x ** 2 + pad_y ** 2)

    # direction of pad
    pad_dir = math.atan2(pad_y, pad_x)

    k = math.cos(af * math.pi / 180) # aggression factor means lowering the number
    d_x_pad = math.cos(pad_dir) * k
    d_y_pad = math.sin(pad_dir) * k
    d_z_pad = math.sqrt(1 - (d_x_pad ** 2 + d_y_pad ** 2))

    cmd = np.array([d_x_pad, d_y_pad, d_z_pad])  # roll, pitch, yawrate, thrust
    return (cmd) 

    
def logging_config():
    # Create the log config for the position
    lg_stab = LogConfig(name='gyro_tracking', period_in_ms=10) # limited to 100 hz, period = 10/1000
    #lg_stab.add_variable('gyro.x', 'float') # deg/s
    #lg_stab.add_variable('gyro.y', 'float')
    #lg_stab.add_variable('gyro.z', 'float')
    
    # attitude rate
    lg_stab.add_variable('ctrlINDI.Omega_f_p', 'float') # rad/s
    lg_stab.add_variable('ctrlINDI.Omega_f_q', 'float')
    lg_stab.add_variable('ctrlINDI.Omega_f_r', 'float')
    
    # attitude rate rate
    lg_stab.add_variable('ctrlINDI.rate_d_roll', 'float') # rad/s^2
    lg_stab.add_variable('ctrlINDI.rate_d_pitch', 'float')
    lg_stab.add_variable('ctrlINDI.rate_d_yaw', 'float')

    return (lg_stab)


def param_stab_est_callback(name, value):
    print('The crazyflie monocopter has parameter ' + name + ' set at number: ' + value)


def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    #gyro_x = data.get('gyro.x') # roll float
    #gyro_y = data.get('gyro.y') # pitch
    #gyro_z = data.get('gyro.z') # yaw

    # attitude rate (rad/s)
    omega_roll = data.get('ctrlINDI.Omega_f_p') # r 
    omega_pitch = data.get('ctrlINDI.Omega_f_q') # p
    omega_yaw = data.get('ctrlINDI.Omega_f_r') # y

    # attitude rate rate (rad/s^2)
    omega_roll_dot = data.get('ctrlINDI.rate_d_roll', 'float') # r 
    omega_pitch_dot = data.get('ctrlINDI.rate_d_pitch', 'float') # p
    omega_yaw_dot = data.get('ctrlINDI.rate_d_yaw', 'float') # y
    print('omega_roll_dot:', omega_roll_dot)


def log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    #time.sleep(5)
    #logconf.stop()


if __name__ == '__main__':

    # Setup keyboard listener
    stage = 'hover'
    # def on_press(key):
    #     global stage
    #     try:
    #         if key.char == 'g':  # Check if 'g' is pressed
    #             stage = 'trajectory on'
    #     except AttributeError:
    #         pass  # Ignore special keys

    # # Start listening for key presses
    # listener = keyboard.Listener(on_press=on_press)
    # listener.start()

    data_receiver_sender = Mocap.Udp()
    max_sample_rate = 360
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(5, [64], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'Monocopter_XYZ','ref_position','rmse_num_xyz','final_rmse','ref_msg','status','cmd','tpp_angle')
                      
                                   
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1

    # Status
    status = "pending for state estimation"

    # rmse terms
    rmse_num_x = 0.0
    final_rmse_x = 0.0
    rmse_num_y = 0.0
    final_rmse_y = 0.0
    rmse_num_z = 0.0
    final_rmse_z = 0.0


    # must always start from the ground
    # loop rates
    loop_counter = 1
    att_loop = 1
    pid_loop = 1
    
    
    time_last = 0
    count = 0        
    time_start = time.time()
    minutes = 1 # no.of mins to run this loop
    time_end = time.time() + (60*100*minutes) 


    # collective z 
    kpz = 9.6 # 7.5, 30.5 |  9.6
    kdz = 5.0 # 2.5, 10.5 | 5.0
    kiz = 128 # | 128


    # cyclic xyz (position)
    kp = [0.8,0.8,0.0] # 0.45 - 1.5 * 0.1m/s 0.02     0.8
    kd = [0.03,0.03,0.0] # 0.2 - 1.5 * 0.1m/s 0.032  0.025
    ki = [0.0,0.0,0.0] # 0.0015   0.003


    # cyclic xyz (velocity) - monocopter doesnt like lol
    kvp = [0.0,0.0,0.0] 
    

    # cyclic xy (attitude)
    ka = [1.0, 1.0]  # 0.08 - 1.5 * 0.1m/s
    kr = [1.0, 1.0]
    krr = [0.001, 0.001] # 0.00005, sim = 0.0091, 0.001
   

    # physical params
    wing_radius = 200/1000 # change to 700 next round
    chord_length = 0.12
    mass = 75/1000
    cl = 0.5
    cd = 0.052
    J = np.array([1/100000,1/100000,1/1000000]) # moment of inertia
    

    monoco = monoco_att_ctrl.att_ctrl(kp, kd, ki, kvp, ka, kr, krr)
    monoco.physical_params(wing_radius, chord_length, mass, cl, cd, J) 


     # Initialize references
    ref_pos_circle = np.array([0.0,0.0,0.0])
    ref_pos = np.array([0.0,0.0,0.0])
    ref_pos_z = 0.0
    ref_vel = np.array([0.0,0.0,0.0])
    ref_acc = np.array([0.0,0.0,0.0])
    ref_jerk = np.array([0.0,0.0,0.0])
    ref_snap = np.array([0.0,0.0,0.0])
    ref_msg = "havent computed yet" 


    # reference offset for xyz
    x_offset = 0.0
    y_offset = 0.0
    z_offset = 0.0
      
    
    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        #swarm.reset_estimators()
        cmd_att_startup = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
        cmd_att = np.array([cmd_att_startup])
        data_log = logging_config()
        swarm_log = np.array([data_log])
        seq_args_log = swarm_logging(swarm_log)
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(init_throttle, args_dict=seq_args)
        #swarm.parallel(log_async, args_dict=seq_args_log) # only can log up to six items at a time

        try:
            #while time_end > time.time():
            while True:
                start = timeit.default_timer() 
                abs_time = time.time() - time_start

                # require data from Mocap
                data = data_receiver_sender.get_data()

                # data unpack
                data_processor.data_unpack_filtered(data)

                # processed tpp data/feedback
                # rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_finite_diff()
                rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_central_diff()
                linear_state_vector = data_processor.pos_vel_acc_filtered()
                body_pitch = data_processor.body_pitch
                tpp_angle = data_processor.tpp
                tpp_omega = data_processor.Omega
                tpp_omega_dot = data_processor.Omega_dot
                body_yaw = data_processor.yaw
                yawrate = data_processor.get_yawrate()
                # tpp_omega = data_processor.Omega
                # tpp_omega_dot = data_processor.Omega_dot
                tpp_quat = data_processor.tpp_eulerAnglesToQuaternion()

                # time difference needed to calculate velocity
                dt = time.time() - time_last  #  time step/period
                time_last = time.time()

                # update positions etc.
                monoco.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)

                tx_cmds = transmitter_calibration()  # get the joystick commands
                manual_thrust = tx_cmds[0]  # thrust command
                button0 = tx_cmds[1]
                button1 = tx_cmds[2]
                enable = tx_cmds[5]

                # update references for PID position loop
                if loop_counter % pid_loop == 0:
                
                    #if button0 == 0:                    
                    a0 = tx_cmds[3]     
                    a1 = tx_cmds[4]
                    af = 80 # aggression factor
                    manual_cyclic = att_manual_ctrl(a0, a1, af) # manual position control
                    monoco.p_control_input_manual(manual_cyclic) # update the manual cyclic inputs
                    monoco.v_control_input()


                if loop_counter % att_loop == 0:

                    # get angle
                    cmd_att = monoco.get_angle(1/(max_sample_rate/att_loop))

                


                # control input (traj execution)
                final_cmd = np.array([cmd, cmd, cmd, cmd]) 
                final_cmd = np.array([final_cmd])
                seq_args = swarm_exe(final_cmd)
                swarm.parallel(arm_throttle, args_dict=seq_args)

                if count % 10 == 0:
                    print('cmd and button commands: ', cmd, button0, button1)
                    print('tx commands: ', a0, a1)

                    if dt > 0.0:
                        print('frequency (Hz) = ', 1/dt)
                        #print('time step: ', dt, 'abs time: ', abs_time)

                count += 1


        except KeyboardInterrupt:    
            print('cmd and button commands: ', cmd, button0, button1)
            print('tx commands: ', a0, a1)
                    

# save data
#path = '/home/emmanuel/AFC_Optitrack/robot_solo/'
#data_saver.save_data(path)

