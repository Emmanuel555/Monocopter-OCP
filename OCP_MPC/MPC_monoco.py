import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import logging
import time

import Utils.Mocap as Mocap
import Utils.DataSave as DataSave
import Localization.Data_process as Data_process
import Localization.Foam_Data_process as Foam_data_process

import math
from pyrr import quaternion
import numpy as np
import numpy.linalg as la

import MPC_monoco_att_ctrl
import Utils.trajectory_generator as trajectory_generator
import timeit
from pynput import keyboard

import stab_monoco_att_ctrl

import SAM
import MPC_optimizer_monoco

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


# robot address
# Change uris and sequences according to your setup

# monoco radio 1
URI1 = 'radio://0/80/2M/E7E7E7E701' # long, short
#URI1 = 'radio://0/80/2M/E7E7E7E7E7' # ultralight


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
        time.sleep(0.01)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(0.01)

        # initialise the controller
        cf.param.add_update_callback(group='stabilizer', name='estimator',
                                 cb=param_stab_est_callback)
        cf.param.set_value('stabilizer.controller', '3')  # 3: INDI controller
        time.sleep(0.01)

        #cf.commander.send_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) # roll, pitch, yawrate, thrust 
        cf.commander.send_position_setpoint(int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3])) 
        print("Initialisation monoco....set ur sticks to mid and down")
        time.sleep(0.01)
    except Exception as e:
        print("Initialisation error: ", e)


def arm_throttle(scf, cmds):
    try:
        cf = scf.cf
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

    button0 = round(joystick.get_axis(5))
    button1 = round(joystick.get_axis(6))
    button2 = round(joystick.get_axis(4))
    
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

    cmd = conPad*enable*button0 # thrust
    manual_thrust = conPad*enable*button0

    if cmd != 0:
        cmd = cmd/(65500/2)
    #print(f"Joystick_axes available: {joystick.get_numaxes()}")

    return (cmd, button0, button1, a0, a1, enable, conPad, button2, manual_thrust)


def p_control_input(linear_pos,kpn,kvn,kin,nom_pos,dt):
    """ position control, nom = {xyz rpy (vx,vy,vz) bodyrates} """
    # error
    control_x = kpn[0]*(linear_pos[0] - nom_pos[0]) + kvn[0]*(linear_pos[3] - nom_pos[6]) + kin[0]*(linear_pos[0] - nom_pos[0])*dt
    control_y = kpn[1]*(linear_pos[1] - nom_pos[1]) + kvn[1]*(linear_pos[4] - nom_pos[7]) + kin[1]*(linear_pos[1] - nom_pos[1])*dt
    control_z = 1.0

    cmd = np.array([control_x, control_y, control_z])  # roll, pitch, yawrate, thrust
    return (cmd) 
    

def ref_manual_ctrl(a0,a1,alt):
    control_x = a0
    control_y = a1
    #control_z = 1.0
    control_z = alt # to include in MPC?

    #cmd = np.array([d_x_pad, d_y_pad, d_z_pad])  # roll, pitch, yawrate, thrust
    cmd = np.array([control_x, control_y, control_z])  # roll, pitch, yawrate, thrust
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

    stage = 'hover'

    # Setup keyboard listener
    """ def on_press(key):
        global stage
        try:
            if key.char == 't':  # Check if 't' is pressed
                stage = 'trajectory on'
            if key.char == 'h':  # Check if 'h' is pressed
                stage = 'hover'
            if key.char == 'l':  # Check if 'l' is pressed
                stage = 'land'          
        except AttributeError:
            pass  # Ignore special keys

    # Start listening for key presses
    listener = keyboard.Listener(on_press=on_press)
    listener.start() """


    data_receiver_sender = Mocap.Udp()
    max_sample_rate = 250 # 360 at 65 - tested fully
    mpc_rate = 100 # best - 100 - tested fully
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / max_sample_rate
    mpc_sample_time = 1 / mpc_rate
    
    monoco_name = "long" # long, short, ultralight

    if monoco_name == "ultralight":
        data_processor = Foam_data_process.RealTimeProcessor(4, [100], 'lowpass', 'cheby2', 85, sample_rate)
    else:
        data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    # data_saver = DataSave.SaveData('Data_time',
    #                                'Monocopter_XYZ','motor_cmd','ref_position','tpp_roll','tpp_pitch','body_yaw_deg','tpp_omega','tpp_omega_dot','body_angle_roll',
    #                                'rmse_num_xyz','att_raterate_error','yawrate')   

    data_saver = DataSave.SaveData('Data_time',
                                   'Monocopter_XYZ','rotational_state_vector','motor_cmd','ref_position','ref_velocity','motor_actual_cmd','cmd_bod_acc','yawrate') 
              
                                   
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
    x_error = 0.0
    y_error = 0.0
    z_error = 0.0
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
    rate_loop = 1
    
    
    time_last = time.time()
    count = 0        
    time_start = time.time()
    minutes = 1 # no.of mins to run this loop
    time_end = time.time() + (60*100*minutes) 
    now = time.time()
    dt_sum = []

    ## old gains from INDI Diff flatness
    # collective z 
    kpz = 5000 
    kdz = 6000 
    kiz = 1200 # | 128

    if monoco_name == "short":
        apz = 320000 
        adz = 50000 
        aiz = 1000 # | 128
    elif monoco_name == "long":
        apz = 320000 
        adz = 50000 
        aiz = 1000 # | 128
    elif monoco_name == "ultralight":
        apz = 120000 
        adz = 25000 
        aiz = 1000 # | 128

    # cyclic xyz (stab position)
    kpn = [1.3,1.3,0.0] # 0.04
    kdn = [0.0005,0.0005,0.0] # not in use
    kin = [10.0,10.0,0.0] 

    # cyclic xyz (stab velocity)
    kvn = [0.01,0.01,0.0] 
    
    # cyclic xy (stab attitude) - heuristic gains thus far
    kan = [6000, 6000]  # 6000
    krn = [10.0, 10.0] # 10

    # null
    kn = np.array([0.0,0.0,0.0])

    # position
    kp = np.array([1000,1000,1200])

    # angle
    ka = np.array([0.0,0.0,0.0])
   
    # velocity - try this
    kv = np.array([60.0,60.0,0.0]) # 20 for 0.5 m/s
    
    # bodyrates
    kr = np.array([250.0,250.0,0.0])

    # INDI Loop
    krr = [1.0, 1.0] # 1.0

    # MPC gains
    # hover
    h_q_cost = np.concatenate((kp,kn,kn,kn))
    h_r_cost = np.array([0.0, 0.0, 0.0])

    # trajectory
    t_q_cost = np.concatenate((kp,ka,kv,kr))
    t_r_cost = np.array([0.0, 0.0, 0.0]) # dun tune U, not good in general


    # thrust rate
    ku = np.array([1.2,0.0,0.0]) #1.5

    # Initialize references
    ref_pos_circle = np.array([0.0,0.0,0.0])
    ref_pos = np.array([1.0,0.0,1.0]) # 0,0 fked up for some reason
    land_pos = np.array([0.0,0.0,0.4])
    x_hover_offset = ref_pos[0]
    y_hover_offset = ref_pos[1]
    z_hover_offset = ref_pos[2]
    x_land_offset = land_pos[0]
    y_land_offset = land_pos[1]
    z_land_offset = land_pos[2]
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

    # flatness option
    flatness_option = 1  # this is a must!
    amplitude = 1  

    manual_cyclic = np.array([0.0, 0.0, 0.0]) 
    auto_cyclic = np.array([0.0, 0.0, 0.0]) 


    # circle parameters
    radius = 0.75 # 0.75 for normal, 0.5 for ultralight 
    speedX = 10.0 # 0.5 m/s the best thus far, start from 1, move up to 1.5, 2 bit hard cos very erratic
    laps = 5
    leminiscate_laps = 4
    leminiscate_radius = 1.5
    helix_laps = 9
    alt = ref_pos[2]
    elevated_alt = 1.0

    if monoco_name == 'short':
        reverse_cw = 1 # 1 for clockwise, 0 for counterclockwise
    elif monoco_name == 'long':
        reverse_cw = 0 # doesnt look like circle but prefer this 
    elif monoco_name == 'ultralight':
        reverse_cw = 1


    # trajectory generator
    traj_gen = trajectory_generator.trajectory_generator()
    ## traj generator for min snap circle, ####### pre computed points
    ## 2 pt line
    #chosen_traj = "_2_pt_line_"
    #pva,num_pts = traj_gen.two_pt_line(speedX, max_sample_rate/pid_loop, alt)
    ## circle
    chosen_traj = "_circle_"
    pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop, laps, reverse_cw, alt) # mechanical limit for monocopter is 0.5m/s
    ## lemniscate
    #chosen_traj = "_lemniscate_"
    #pva,num_pts = traj_gen.lemniscate(x_offset, y_offset, leminiscate_laps, leminiscate_radius, max_sample_rate/pid_loop, reverse_cw, speedX, alt)
    ## helix
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_helix_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop,helix_laps,reverse_cw,alt)
    ## elevated circle
    #chosen_traj = "_elevated_circle_"
    #pva,num_pts = traj_gen.compute_jerk_snap_9pt_elevated_circle_x_laps(x_offset, y_offset, radius, speedX, max_sample_rate/pid_loop,laps,reverse_cw,elevated_alt)


    # Solver terms
    t_horizon = mpc_sample_time # 100 Hz
    Nodes = 20 # tested fully, 15 - pushing it


    # MPC Monoco Model
    monoco_type = SAM.SAM(monoco_name)
    

    # Set constraints on
    set_constraints = True


    # MPC Monoco Control & Optimizer
    monoco = MPC_monoco_att_ctrl.att_ctrl(krr, ku, h_q_cost, h_r_cost, monoco_type, time_horizon=t_horizon, nodes=Nodes, set_constraints=set_constraints)
    

    # MPC Monoco Stab INDI Control
    monoco_stab = stab_monoco_att_ctrl.stab_att_ctrl(kpn, kdn, kin, kvn, kan, krn, krr)


    with Swarm(uris, factory= CachedCfFactory(rw_cache='./cache')) as swarm:
        #swarm.reset_estimators()
        cmd_att_startup = np.array([0, 0, 0, 0]) # init setpt to 0 0 0 0
        cmd_att = np.array([cmd_att_startup])
        #data_log = logging_config()
        #swarm_log = np.array([data_log])
        #seq_args_log = swarm_logging(swarm_log)
        seq_args = swarm_exe(cmd_att)
        swarm.parallel(init_throttle, args_dict=seq_args)
        
        #swarm.parallel(log_async, args_dict=seq_args_log) # only can log up to six items at a time

        try:
            while True:
                start = timeit.default_timer() 
                abs_time = time.time() - time_start

                # looprate tracker
                dt = time.time() - now
                now = time.time() 

                # Match loop time
                dt_sum.append(dt)

                # dk why need this for python 3.10
                joystick = pygame.joystick.Joystick(0) # added here to speed up loop

                # require data from Mocap
                data = data_receiver_sender.get_data()

                # data unpack
                data_processor.data_unpack_filtered(data)

                # processed tpp data/feedback
                # rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_finite_diff()
                rotational_state_vector = data_processor.get_Omega_dot_dotdot_filt_eul_central_diff()
                pos_raw = data_processor.raw_data
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
                bod_angle_roll = data_processor.body_angle_roll
                ## time difference needed to calculate velocity
                # dt = time.time() - time_last  #  time step/period
                
                # update from transmitter
                tx_cmds = transmitter_calibration()  # get the joystick commands
                manual_alt = tx_cmds[0]  # thrust command
                button0 = tx_cmds[1]
                button1 = tx_cmds[2]
                enable = tx_cmds[5]
                conPad = tx_cmds[6]
                button2 = tx_cmds[7]
                manual_thrust = tx_cmds[8]

                a0 = tx_cmds[3] # x     
                a1 = tx_cmds[4] # y
                
                ## update references for manual control
                manual_cyclic = ref_manual_ctrl(a0, a1, manual_alt) # manual position control 

                # update positions etc.
                monoco.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate, body_pitch)

                if button2 == 1:

                    ## hovering test
                    if button1 == 0:
                        stage = 'hover'
                        ref = traj_gen.hover_test(x_hover_offset,y_hover_offset,z_hover_offset)
                        hovering_ff = np.array([0.0, 0.0, 0.0])
                        ref_pos = ref[0]
                        ref_vel = hovering_ff
                        ref_acc = hovering_ff
                        ref_jerk = hovering_ff
                        ref_snap = hovering_ff
                        ref_msg = ref[1]
                        q = h_q_cost
                        r = h_r_cost
                        count = 0


                    ## trajectory inputs
                    elif button1 == 1:
                        stage = 'trajectory on'
                        ref_derivatives = traj_gen.jerk_snap_circle(pva,num_pts,count,alt)
                        ref_pos = ref_derivatives[0]
                        ref_vel = ref_derivatives[1]
                        ref_acc = ref_derivatives[2]
                        ref_jerk = ref_derivatives[3]
                        ref_snap = ref_derivatives[4]
                        ref_msg = ref_derivatives[5]  
                        q = t_q_cost
                        r = t_r_cost
                        if sum(dt_sum) >= sample_time: 
                            count += 1
                            dt_sum.clear() 


                    ## landing 
                    elif button1 == -1:
                        stage = 'land'
                        ref = traj_gen.hover_test(x_land_offset,y_land_offset,z_land_offset)
                        hovering_ff = np.array([0.0, 0.0, 0.0])
                        ref_pos = ref[0]
                        ref_vel = hovering_ff
                        ref_acc = hovering_ff
                        ref_jerk = hovering_ff
                        ref_snap = hovering_ff
                        ref_msg = ref[1]
                        q = h_q_cost
                        r = h_r_cost
                        count = 0

                    # ff references
                    # Manual thrust control
                    # stage == 'TX_Manual_flight'
                    # ff = np.array([0.0, 0.0, 0.0])
                    # ref_pos = manual_cyclic
                    # ref_vel = ff
                    # ref_acc = ff
                    # ref_jerk = ff
                    # ref_snap = ff
                    # ref_msg = 'Manual_flight'

                    monoco.linear_ref(ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap)
                    # mpc p control
                    monoco.p_control_input_manual(ref_pos)
                    
                    # from mpc att ctrl
                    control_outputs = monoco.MPC_SAM_get_angles_and_thrust(q,r) # roll and pitch torque requirements into motor values 
                    cmd_bod_acc = control_outputs[0]
                    nom_state = control_outputs[3]


                    # # stab control
                    # monoco_stab.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)
                    # monoco_stab.linear_ref(nom_state)
                    # stab_cyclic = p_control_input(linear_state_vector, kpn, kvn, kin, nom_state, sample_time) # stab position term
                    # monoco_stab.p_control_input_manual(stab_cyclic)
                    # # get angle
                    # stab_att = monoco_stab.get_angle()
                    # # get body rate
                    # stab_rates = monoco_stab.get_body_rate()
                    # # get torque and motor cmds after INDI
                    # stab_act_limit = 2000
                    # stab_bod_acc = monoco_stab.CF_SAM_get_angles_and_thrust(stab_act_limit)
                    # stab_bod_acc = stab_bod_acc[0] + stab_bod_acc[1]


                    # alt control with input from TX - des rps doesnt work lol, cfm plus chop 
                    motor_soln = monoco.manual_collective_thrust(apz,adz,aiz)
                    motor_soln = motor_soln + cmd_bod_acc[0] + cmd_bod_acc[1]  # collective thrust + cyclic
                    #motor_soln = motor_soln + stab_bod_acc # add stab body acceleration to motor solution

                else:

                    # Manual control via MPC
                    stage == 'MPC_Manual_flight'
                    ff = np.array([0.0, 0.0, 0.0])
                    ref_pos = manual_cyclic
                    ref_vel = ff
                    ref_acc = ff
                    ref_jerk = ff
                    ref_snap = ff
                    ref_msg = 'Manual_flight'
                    q = h_q_cost
                    r = h_r_cost
                    monoco.linear_ref(ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap)
                    monoco.p_control_input_manual(ref_pos) # update the ref states
                    #monoco.rotational_drag()

                    # from mpc att ctrl
                    control_outputs = monoco.MPC_SAM_get_angles_and_thrust(q,r) # roll and pitch torque requirements into motor values    
                    cmd_bod_acc = control_outputs[0]
                    nom_state = control_outputs[3]

                    ## stab control
                    # monoco_stab.update(linear_state_vector, rotational_state_vector, tpp_quat[0], dt, z_offset, body_yaw, tpp_quat[1], tpp_quat[2], yawrate)
                    # monoco_stab.linear_ref(nom_state)
                    # stab_cyclic = p_control_input(linear_state_vector, kpn, kvn, kin, nom_state, sample_time) # stab position term
                    # monoco_stab.p_control_input_manual(stab_cyclic)
                    # # get angle
                    # stab_att = monoco_stab.get_angle()
                    # # get body rate
                    # stab_rates = monoco_stab.get_body_rate()
                    # # get torque and motor cmds after INDI
                    # stab_act_limit = 2000
                    # stab_bod_acc = monoco_stab.CF_SAM_get_angles_and_thrust(stab_act_limit)
                    # stab_bod_acc = stab_bod_acc[0] + stab_bod_acc[1]                   


                    ## alt control with input from TX - des rps doesnt work lol, cfm plus chop
                    motor_soln = monoco.manual_collective_thrust(apz,adz,aiz)
                    motor_soln = motor_soln + cmd_bod_acc[0] + cmd_bod_acc[1]  # collective thrust + cyclic
                    #motor_soln = motor_soln + stab_bod_acc # add stab body acceleration to motor solution

                att_raterate_error = monoco.attitude_raterate_error
            
                # motor output
                motor_cmd = int(motor_soln)*button0*enable

                if motor_cmd > 65500:
                    motor_cmd = 65500
                elif motor_cmd < 10:
                    motor_cmd = 10
                
                final_cmd = np.array([motor_cmd, motor_cmd, motor_cmd, motor_cmd]) # e.g                
                final_cmd = np.array([final_cmd])
                seq_args = swarm_exe(final_cmd) 
                swarm.parallel(arm_throttle, args_dict=seq_args)


                stop = timeit.default_timer()
                time_last = time.time()       


                if loop_counter % 20 == 0:
                    """ print(f"Thrust: {manual_alt}, X: {a0}, Y: {a1}, Enable: {enable}, Button0: {button0}, Button1: {button1}, ConPad: {conPad}, Button2: {button2}")     
                    print(f"Cmd_bod_acc are: {cmd_bod_acc}, roll pitch cyclic inputs are: {cyclic}, des_rps is {des_rps}, motor_soln is {motor_soln}")
                    print(ref_msg) 
                    #print('tpp_position', linear_state_vector[0], linear_state_vector[1], linear_state_vector[2]) """
                    #print('altitude error: ', manual_alt - linear_state_vector[2])
                    #print('x error: ', ref_pos[0] - linear_state_vector[0])
                    #print('y error: ', ref_pos[1] - linear_state_vector[1])

                    print(f'x ref: {ref_pos[0]}, y_ref: {ref_pos[1]}')

                    #print('ref_alt: ', manual_alt) 
                    print('motor_soln: ', motor_soln)
                    #print('actual_motor_cmd: ', motor_cmd)
                    print(f'button2: {button2}, button1: {button1}')
                    print(f'Stage: {stage}, Ref Msg: {ref_msg}')
                    #print('actual_altitude: ', linear_state_vector[2])

                    #print('manual_cyclic_xyz: ', manual_cyclic)
                    #print('p_cyclic_xyz: ', monoco.p_control_signal)

                    #print('raw att_cmds: ', control_outputs[2])
                    #print('monoco.rates comparison: ', monoco.cmd_bod_rates_final, monoco.ref_rates)
                    #print('monoco.raterates comparison: ', monoco.cmd_bod_raterates_final, monoco.ref_raterates)
                    #print('yawrate: ', yawrate)

                    if dt > 0.0:
                        print(f"Program Runtime: {stop - start}, Program running at: {1/dt} Hz") 
                        #print('time step: ', dt, 'abs time: ', abs_time)


                # control loop counter
                loop_counter += 1


                ## auto & collect data
                if button2 == 1: 
                    if stage == 'trajectory on':
                #         x_error = ref_pos[0]-x_offset-linear_state_vector[0]
                #         y_error = ref_pos[1]-y_offset-linear_state_vector[1]
                #         z_error = ref_pos[2]-z_offset-linear_state_vector[2]
                #         rmse_num_x = rmse_num_x + (x_error)**2
                #         rmse_num_y = rmse_num_y + (y_error)**2
                #         rmse_num_z = rmse_num_z + (x_error )**2
                #         rmse_num = [x_error,y_error,z_error,rmse_num_x,rmse_num_y,rmse_num_z]
                        
                #         data_saver.add_item(abs_time,
                #                     linear_state_vector[0:3],motor_cmd,ref_pos,round((tpp_angle[0]*(180/np.pi)),3),round((tpp_angle[1]*(180/np.pi)),3),round(body_yaw*(180/np.pi),2),tpp_omega,tpp_omega_dot,bod_angle_roll,
                #                     rmse_num,att_raterate_error,yawrate)   

                        data_saver.add_item(abs_time,linear_state_vector[0:6],rotational_state_vector,motor_soln,ref_pos,ref_vel,motor_cmd,cmd_bod_acc,yawrate) 


                ## Dun bother with enforcing loop rate anymore, doesnt work!
                ## testing:
                #print(f"dt: {dt}")
                #sleep_time = max(0.0, (1/max_sample_rate) - (time.time() - now))
                #print (f"sleep_time: {sleep_time}")
                #time.sleep(sleep_time)


                    

        except KeyboardInterrupt:  
            # final rmse calculation
            status = "Emergency stop"
            ref_msg = "flight ended..."
            # final_rmse_x = math.sqrt(rmse_num_x/(count))
            # final_rmse_y = math.sqrt(rmse_num_y/(count))
            # final_rmse_z = math.sqrt(rmse_num_z/(count))
            # final_rmse = la.norm([final_rmse_x, final_rmse_y, final_rmse_z], 2)
            # rmse_num = [final_rmse_x, final_rmse_y, final_rmse_z]  

            #print('Emergency Stopped and final rmse produced: ', rmse_num )
        
                    

# save data
path = "/home/emmanuel/Monocopter-OCP/OCP_MPC/MPC_robot/MPC_" + monoco_name + chosen_traj + str(speedX*0.1) + '_ms'
data_saver.save_data(path)

