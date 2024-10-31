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


def equations(p):
    theta, phi, f = p
    R13 = math.cos(psi) * math.sin(theta) * math.cos(phi) + math.sin(psi) * math.sin(phi)
    R23 = math.sin(psi) * math.sin(theta) * math.cos(phi) - math.cos(psi) * math.sin(phi)
    R33 = math.cos(phi) * math.cos(theta)
    return [R13 * f - fx, R23 * f - fy, R33 * f - fz]


if __name__ == '__main__':

    data_receiver = Mocap.Udp()
    sample_rate = data_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process_swarm.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'data'
                                   )

    # robot address

    URI = 'radio://0/30/2M/E7E7E7E704'
    #URI = 'radio://0/114/2M/E7E7E7E706'

    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

  # Initialize the joysticks
    pygame.init()
    pygame.joystick.init()
    done = False
    controllerEnable = False
    pad_speed = 1

    # control gains
    kpx = 13_000
    kpy = 13_000
    kpz = 10_000

    kdx = 0 #12_000
    kdy = 0 #12_000
    kdz = 0 #15_000

    kix = 0
    kiy = 0
    kiz = 10000

    # other parameters
    count = 0

    psi = 0
    fx = 0
    fy = 0
    fz = 0

    px_last = 0
    py_last = 0
    pz_last = 0
    time_last = -100
    I_term_z = 0


    # input parameters
    # robot_num = input('input your robot number (end with enter): ')
    # print('recorded  robot number is:', robot_num)

    with SyncCrazyflie(URI, cf = Crazyflie(rw_cache = './cache')) as scf:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        time_start = time.time()
        time_end = time.time() + 6000

        while time_end > time.time():
            abs_time = time.time() - time_start
            # where hand control comes
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            lowest = 0.787
            highest = -0.89
            range_js = -(highest - lowest)
            range_motor = 65535
            rate = range_motor / range_js
            a0 = joystick.get_axis(0)  # x axis right hand <- ->
            a1 = joystick.get_axis(1)  # y axis right hand up down
            a2 = joystick.get_axis(2)  # thrust
            a3 = joystick.get_axis(3)

            L1 = joystick.get_button(0)
            R1 = joystick.get_button(1)

            # thrust from control pad
            conPad = int((a2 - highest) * rate)

            # joystick saturation

            if conPad < 10:
                conPad = 10
            if conPad > 65500:
                conPad = 65500

            # takeoff sign
            if conPad < 2000:
                enable = 0
            else:
                enable = 1

            # require data from Mocap
            data = data_receiver.get_data()
            # data unpack
            data_processor.data_unpack(data)
            # raw data
            raw_data = data_processor.raw_data
            # filt data
            filt_data = data_processor.get_data_filted()
            # rotation matrix
            rotm_1 = data_processor.get_rotm_1()
            rotm_2 = data_processor.get_rotm_2()
            rotm_3 = data_processor.get_rotm_3()
            # yaw
            yaw_1 = data_processor.get_heading_x1()
            yaw_2 = data_processor.get_heading_x2()
            yaw_3 = data_processor.get_heading_x3()

            # position feedback
            robot_1 = [data_processor.px1, data_processor.py1, data_processor.pz1, yaw_1]
            robot_2 = [data_processor.px2, data_processor.py2, data_processor.pz2, yaw_2]
            robot_3 = [data_processor.px3, data_processor.py3, data_processor.pz3, yaw_3]

            # select robot number

            robot = robot_2

            # if robot_num == '1':
            #     robot = robot_1
            # elif robot_num == '2':
            #     robot = robot_2
            # elif robot_num == '3':
            #     robot = robot_3
            # else:
            #     print('input issue, program ends')
            #     break

            # desired trajectory - hovering
            px_s = 0
            py_s = 0
            pz_s = 0.1

            # landing sign or hovering at 1m hgt
            if R1 == 0: # right switch all the way up
                px_s = 0
                py_s = 0
                pz_s = 0.5

            # update position
            px_m = robot[0]

            py_m = robot[1]

            pz_m = robot[2]

            # px_m = data_processor.px
            # py_m = data_processor.py
            # pz_m = data_processor.pz


            # calculate velocity
            dt = time.time() - time_last  #  time difference
            time_last = time.time()

            vx = (px_m - px_last) / dt
            vy = (py_m - py_last) / dt
            vz = (pz_m - pz_last) / dt

            px_last = px_m
            py_last = py_m
            pz_last = pz_m

            px_err = px_s - px_m
            py_err = py_s - py_m
            pz_err = pz_s - pz_m   # calculate position error

            # pid controller
            fx_d = kpx * px_err - kdx * vx
            fy_d = kpy * py_err - kdy * vy

            # integration term for z position
            I_term_z = enable * kiz * pz_err * dt / 2 + I_term_z
            fz_d = kpz * pz_err - kdz * vz + I_term_z + 22_000

            fx = fx_d
            fy = fy_d
            fz = fz_d * enable + 10_000

            psi = robot[3]

            des_pitch, des_roll, des_thrust = fsolve(equations, [0, 0, 30000])

            des_roll = int(des_roll * 180 / math.pi)
            des_pitch = int(des_pitch * 180 / math.pi)

            # output saturation
            if des_roll > 50:
                des_roll = 50
            if des_roll < -50:
                des_roll = -50
            if des_pitch > 50:
                des_pitch = 50
            if des_pitch < -50:
                des_pitch = -50
            if des_thrust > 60_000:
                des_thrust = 60_000
            if des_thrust < 10:
                des_thrust = 10

            thrust = int(des_thrust)  # for optitrack control

            roll_set = -a0 * 20 / 0.835  # degree
            pitch_set = -a1 * 20 / 0.835  # degree
            yaw_rate_set = -a3 * 20 / 0.835  # degree/s

            cf.commander.send_setpoint(0*des_roll, 0*des_pitch, 0, thrust * 1 + conPad * 0)   # optitrack control [roll,  pitch ,  yawrate, thrust]

            print("desired inputs: ", des_roll, des_pitch, 0, thrust * 1 + conPad * 0) 
            count = count + 1

            print('robot_position', robot)
            print('conPad', conPad)
            print('R1', R1)
            print("ref: ", px_s, py_s, pz_s)

            # save data
            data_saver.add_item(abs_time,
                                robot,
                                )

            if L1 == 1: # emergency stop cut throttle left hand switch all the way up
                cf.commander.send_setpoint(0, 0, 0, 10)
                print('Emergency Stopped')
                break

        # save data
        #path = '/Users/airlab/PycharmProjects/AFC/data/'
        #data_saver.save_data(path)

