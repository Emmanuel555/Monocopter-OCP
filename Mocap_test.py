

import time

import Mocap
import DataSave
#import Data_process_swarm
import Data_process

if __name__ == '__main__':

    data_receiver = Mocap.Udp()
    sample_rate = data_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    data_processor = Data_process.RealTimeProcessor(5, [16], 'lowpass', 'cheby2', 85, sample_rate)

    data_saver = DataSave.SaveData('Data_time',
                                   'raw_data'
                                   )

    time_start = time.time()
    time_end = time_start + 1000

    count = 0
    while time_end > time.time():
        abs_time = time.time() - time_start

        # require data from Mocap
        data = data_receiver.get_data()
        # data unpack
        data_processor.data_unpack(data)
        # raw data
        raw_data = data_processor.raw_data

        count = count + 1

        if count % 20 == 0:
            print('-----', 'time: ', abs_time, '-----')
            print(raw_data)

        # save data
        
        data_saver.add_item(abs_time,
                            raw_data
                            )
        
       #print("rpy: ", data_processor.get_RPY())
        

        time.sleep(0.05)
        # save data
    #path = '/Users/airlab/PycharmProjects/AFC/data/'
    #data_saver.save_data(path)

