import socket
import time
import numpy as np


class Udp(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222, num_bodies=1):
        self.udpStop = False
        self.udp_data = None
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.num_bodies=num_bodies
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sample_rate_flag = -1  # flag
        self.sample_rate = self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate

    def get_sample_rate(self):
        if self.sample_rate_flag == -1:
            print('computing sample rate..')
            time_list = []
            for i in range(100): # get 1000 sample
                time_list.append(time.time())
                data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes, if nothing it would be stuck here
            dtime = np.diff(time_list)
            sample_time = np.mean(dtime)
            print('Sample rate: ', '%.2f' % (1/sample_time), 'Hz')
            return 1/sample_time
        else:
            return self.sample_rate

    # def udp_step(self):
    #     udp_data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
    #     return udp_data

    def get_data(self):
        udp_data, addr = self.sock.recvfrom(100)
        return udp_data
