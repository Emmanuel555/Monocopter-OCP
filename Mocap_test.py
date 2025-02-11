import time

import Mocap
import DataSave
import Data_process
import numpy as np
import timeit

if __name__ == '__main__':

    data_receiver_sender = Mocap.Udp()
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    ##data_processor = Data_process_swarm.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'raw_data'
                                   )

    time_start = time.time()
    time_end = time_start + 1000
    last_time = 0

    UDP_IP = "192.168.65.221"
    UDP_PORT = 1234
    test_cmd_1 = float(3.423)
    test_cmd_2 = float(4.534)
    test_cmd_3 = float(5.645)
    test_cmd_4 = float(6.756)
    final_cmd = np.array([[test_cmd_1,test_cmd_2,test_cmd_3,test_cmd_4]])

    count = 0
    try:
        #while time_end > time.time():
        while True:  
            start = timeit.default_timer()  
            abs_time = time.time() - time_start

            # require data from Mocap
            data = data_receiver_sender.get_data()
            
            # data unpack
            # data_processor.data_unpack(data)
            data_processor.data_unpack_filtered(data)

            # cartesian position
            pos = data_processor.filted_data
            
            # processed tpp data/feedback
            state_vector = data_processor.get_Omega_dot_dotdot_filt_eul()
            
            # needa find pitch angle of the body during hover
            # data_processor.get_rotm_filtered()
            tpp_angle = data_processor.tpp # test it again and w r11 & r22
            tpp_omega = data_processor.Omega
            tpp_omega_dot = data_processor.Omega_dot
            body_pitch = data_processor.body_pitch
            tpp_quat = data_processor.tpp_eulerAnglesToQuaternion()

            # testing sending information over udp to wj esp32
            #print ("sending info: ", final_cmd)
            data_receiver_sender.send_data(UDP_IP, UDP_PORT, final_cmd)

            count = count + 1

            if count % 10 == 0:
                #print('-----', 'time: ', abs_time, '-----')
                #print(raw_data)

                # save data
                
                #data_saver.add_item(abs_time,
                #                    raw_data
                #                    )
                
                t_diff = abs_time - last_time
                last_time = abs_time

                """ r11 = data_processor.R11
                r12 = data_processor.R12
                r13 = data_processor.R13
                r21 = data_processor.R21
                r22 = data_processor.R22
                r23 = data_processor.R23
                r31 = data_processor.R31
                r32 = data_processor.R32
                r33 = data_processor.R33 """
                
                #print("sampling period and freq: ", t_diff, 1/t_diff) 
                print("tpp angles:", tpp_angle) # rpy
                #print("tpp bodyrates:", tpp_omega) # rpy
                #print("tpp bodyraterates:", tpp_omega_dot) # rpy
                print("position: ", pos[0:3])
                #print("tpp quaternion: ", tpp_quat)

                #n = 100000
                #print("r11, r12, r13: ", r11/n, r12/n, r13/n)
                #print("r21, r22, r23: ", r21/n, r22/n, r23/n)
                #print("r31: ", r31/n)
                #print("r32: ", r32/n)
                #print("r33: ", -1*r33/n) # seems to be correct 
                #print("body pitch: ", body_pitch)
                
                #time.sleep(0.05) # impt to pause and see information
            stop = timeit.default_timer()
            print('Program Runtime: ', stop - start)  

    except KeyboardInterrupt:
        print('Keyboard interrupt')
        
    # save data
    #path = '/Users/airlab/PycharmProjects/AFC/data/'
    #path = '/home/emmanuel/AFC_Optitrack/linux_data/'
    #data_saver.save_data(path)

