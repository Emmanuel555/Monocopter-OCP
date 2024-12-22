

import time

import Mocap
import DataSave
import Data_process_swarm
import Data_process

if __name__ == '__main__':

    data_receiver_sender = Mocap.Udp()
    sample_rate = data_receiver_sender.get_sample_rate()
    sample_time = 1 / sample_rate
    #data_processor = Data_process_swarm.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'raw_data'
                                   )

    time_start = time.time()
    time_end = time_start + 1000
    last_time = 0

    count = 0
    while time_end > time.time():
        abs_time = time.time() - time_start

        # require data from Mocap
        data = data_receiver_sender.get_data()
        
        # data unpack
        # data_processor.data_unpack(data)
        data_processor.data_unpack_filtered(data)
        
        # processed tpp data/feedback
        state_vector = data_processor.get_Omega_dot_dotdot_filt()
        tpp_angle = data_processor.tpp
        tpp_omega = data_processor.Omega
        tpp_omega_dot = data_processor.Omega_dot

        # testing sending information over udp to wj esp32
        UDP_IP = "127.0.0.1"
        UDP_PORT = 5005
        MESSAGE = b"Msg sent from python"
        data_receiver_sender.send_data(UDP_IP, UDP_PORT, MESSAGE)

        count = count + 1

        """ if count % 20 == 0:
            print('-----', 'time: ', abs_time, '-----')
            print(raw_data) """

        # save data
        
        #data_saver.add_item(abs_time,
        #                    raw_data
        #                    )
        
        t_diff = abs_time - last_time
        last_time = abs_time
        
        print("sampling period and freq: ", t_diff, 1/t_diff) 
        print("tpp angle:", tpp_angle)
        #print("rpy: ", data_processor.get_RPY())

        
        time.sleep(0.05) # impt to pause and see information
        # save data
    #path = '/Users/airlab/PycharmProjects/AFC/data/'
    #path = '/home/emmanuel/AFC_Optitrack/linux_data/'
    #data_saver.save_data(path)

