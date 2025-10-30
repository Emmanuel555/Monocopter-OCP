# general libs
import math
import time
import numpy as np

# crazyflie libs
import cflib.crtp
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import logging

# customized libs
import DataSave
import Data_process
import Mocap
from cflog import LoggingCore2


if __name__ == '__main__':
    # robot list
    URI = 'radio://0/80/2M/E7E7E7E7E7'

    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    data_receiver = Mocap.Udp()
    sample_rate = data_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(3, [20], 'lowpass', 'cheby2', 85, sample_rate)

    # reading onboard data
    logging_list0 = {'pm.vbat': 'float'
                     }

    logging_list_array = [logging_list0]

    lc = LoggingCore2(URI, 20, logging_list_array)
    time.sleep(1)

    lc.cf.commander.send_position_setpoint(0, 0, 0, 0)

    data_saver = DataSave.SaveData('Data_time',
                                   'data_raw',
                                   'data_filt',
                                   'motor_signal',
                                   'err_x',
                                   'err_y',
                                   'battery_voltage'
                                   )

    # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1

    # parameter
    count = 0
    time_last = -1

    I_term_z = 0
    circle = 1
    body_x_last = 0
    pz_last = 0

    time_start = time.time()
    time_end = time.time() + 2000

    while lc.is_connected and time.time() < time_end:
        abs_time = time.time() - time_start

        # where hand control comes
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        highest = 0.97
        lowest = -1
        range_js = (highest - lowest)
        range_motor = 65535
        rate = range_motor / range_js
        a0 = joystick.get_axis(0)  # x axis right hand <- ->
        a1 = joystick.get_axis(1)  # y axis right hand up down
        a2 = joystick.get_axis(2)  # thrust
        a3 = joystick.get_axis(3)

        SA = joystick.get_axis(4)
        SD = joystick.get_axis(7)

        # thrust from control pad
        conPad = int((a2 - lowest) * rate)

        # Get onboard data
        logged_data = lc.get_logged_data()
        battery_vol = logged_data[0].get('pm.vbat', 0)

        if abs_time < 0.1 and SD > 0:
            print('SD < 0')
            break

        if conPad > 2000:
            enable = 1
        else:
            enable = 0

        # trigger
        pad_x = a0
        pad_y = a1
        trigger = math.sqrt(pad_x ** 2 + pad_y ** 2)

        if trigger > 0.5:
            flag = 1
        else:
            flag = 0

        # thrust from control pad
        conPad = int((a2 - lowest) * rate)

        # direction of pad
        pad_dir = math.atan2(pad_y, pad_x)

        # require data from Mocap
        data = data_receiver.get_data()
        # data unpack
        data_processor.data_unpack(data)
        # raw data
        raw_data = data_processor.raw_data
        # filt data
        filt_data = data_processor.get_data_filted()
        # rotation matrix
        rotm = data_processor.get_rotm()
        # roll angle
        roll_raw = data_processor.get_roll_x()
        # robot orientation from mocap
        body_x = data_processor.get_heading_x()

        # setpoint
        set_px = -1
        set_py = 0
        set_pz = 1.2

        # read measurement
        mea_px = data_processor.px_filted
        mea_py = data_processor.py_filted
        mea_pz = data_processor.pz

        dt = abs_time - time_last
        time_last = abs_time

        # position error

        err_px = set_px - mea_px
        err_py = set_py - mea_py

        err_pz = set_pz - mea_pz

        # velocity
        vz = (mea_pz - pz_last) / dt
        pz_last = mea_pz  # update pz

        # height control

        take_off = conPad

        I_term_z = enable * 2000 * err_pz * dt / 2 + I_term_z

        z_command = 3000 * err_pz + I_term_z - 6000 * vz + take_off

        # desired direction
        k = math.cos(80 * math.pi / 180)

        d_x_pad = math.cos(pad_dir) * k
        d_y_pad = math.sin(pad_dir) * k
        d_z_pad = math.sqrt(1 - (d_x_pad ** 2 + d_y_pad ** 2))

        alpha = math.atan2(err_py, err_px)

        err_mag = math.sqrt(err_px ** 2 + err_py ** 2)

        phi_err = 0.07 * err_mag

        # saturation is set to be 10 degrees
        if phi_err > 0.10:
            phi_err = 0.10
        elif phi_err < -0.10:
            phi_err = -0.10

        d_x_pos = math.sin(phi_err) * math.cos(alpha)
        d_y_pos = math.sin(phi_err) * math.sin(alpha)
        d_z_pos = math.cos(phi_err)

        if flag == 0:
            xi_x_d = d_x_pos
            xi_y_d = d_y_pos
            xi_z_d = d_z_pos
        else:
            xi_x_d = d_x_pad
            xi_y_d = d_y_pad
            xi_z_d = d_z_pad

        d_z = [xi_x_d, xi_y_d, xi_z_d]

        # measure direction
        x_m = [data_processor.R11, data_processor.R21, data_processor.R31]
        y_m = [data_processor.R12, data_processor.R22, data_processor.R32]

        # attitude error (definition)
        off_x = 0.21
        off_y = 0.07

        err_x = np.dot(y_m, d_z) - off_x
        err_y = -np.dot(x_m, d_z) + off_y

        kp_x = 60000
        kp_y = 60000

        tau_x = -err_x * kp_x
        tau_y = -err_y * kp_y

        theta = -math.pi / 2 - math.pi / 6

        tau_x_test = math.cos(theta) * tau_x - math.sin(theta) * tau_y
        tau_y_test = math.sin(theta) * tau_x + math.cos(theta) * tau_y

        if tau_x_test > 10000:
            tau_x_test = 10000
        elif tau_x_test < -10000:
            tau_x_test = -10000

        if tau_y_test > 10000:
            tau_y_test = 10000
        elif tau_y_test < -10000:
            tau_y_test = -10000

        if data_processor.pz > 0.1:
            controller_enable = 1
        else:
            controller_enable = 0

        off_set = math.pi / 2

        if flag == 1:
            # thrust = conPad + 7000 * controller_enable * flag * math.sin(body_x + pad_dir + off_set)
            thrust = tau_x_test * controller_enable * 1 + z_command * enable
        else:
            thrust = tau_x_test * controller_enable * 1 + z_command * enable

        if thrust < 10:
            thrust = 10
        if thrust > 65500:
            thrust = 65500

        # thrust = conPad

        lc.cf.commander.send_position_setpoint(int(thrust), int(thrust), int(thrust), int(thrust))

        if count % 10 == 0:
            print("time:", abs_time)
            print('err_x', err_x)
            print('err_y', err_y)
            print('m1', thrust)
            print('battery_voltage', battery_vol)
            print('measured z', mea_pz)

        count = count + 1

        # save data
        data_saver.add_item(abs_time,
                            raw_data,
                            filt_data,
                            thrust,
                            err_x,
                            err_y,
                            battery_vol
                            )

        if SD > 0:
            if abs_time < 0.1:
                print('pull up SD button an try')
            else:
                print('program stoped')
            break

    # save data
    path = '/Users/tsaixinyu/Desktop/Solar_monocopter/Data/'
    data_saver.save_data(path)