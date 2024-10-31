# from Library.IIR2Filter import IIR2Filter
import struct
from Filter import IIR2Filter
import math
import numpy as np


class RealTimeProcessor(object):
    def __init__(self, order, cutoff, ftype, design, rs, sample_rate):
        self.flag = 0
        self.sample_time = 1 / sample_rate
        self.sample_rate = sample_rate
        self.qx_last = 0
        self.qy_last = 0
        self.qz_last = 0
        self.qw_last = 0

        # raw data from motion capture
        self.px = 0
        self.py = 0
        self.pz = 0
        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.quat_w = 0

        # filted data with IIR2Filter
        self.px_filted = 0
        self.py_filted = 0
        self.pz_filted = 0
        self.quat_x_filted = 0
        self.quat_y_filted = 0
        self.quat_z_filted = 0
        self.quat_w_filted = 0

        # component of rotation matrix
        self.R11 = 0
        self.R12 = 0
        self.R13 = 0
        self.R21 = 0
        self.R22 = 0
        self.R23 = 0
        self.R31 = 0
        self.R32 = 0
        self.R33 = 0

        # filter setup
        self.FilterX = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)

        self.FilterOmega_x = IIR2Filter(5, [50], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterOmega_y = IIR2Filter(5, [50], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterOmega_z = IIR2Filter(5, [50], ftype, design=design, rs=rs, fs=sample_rate)

    def data_unpack(self, udp_data):
        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data)
        self.px = x * 0.0005  # position px
        self.py = y * 0.0005  # position py
        self.pz = z * 0.0005  # position pz

        self.quat_x = float(qx * 0.001)
        self.quat_y = float(qy * 0.001)
        self.quat_z = float(qz * 0.001)
        self.quat_w = float(qw * 0.001)
        self.raw_data = [self.px, self.py, self.pz, self.quat_x, self.quat_y, self.quat_z, self.quat_w]


    def get_data_filted(self):
        self.px_filted = self.FilterX.filter(self.px)
        self.py_filted = self.FilterY.filter(self.py)
        self.pz_filted = self.FilterZ.filter(self.pz)
        filted_data = [self.px_filted, self.py_filted, self.pz_filted]

        return filted_data


    def get_rotm(self):

        xx = self.quat_x * self.quat_x
        yy = self.quat_y * self.quat_y
        zz = self.quat_z * self.quat_z
        xy = self.quat_x * self.quat_y
        xz = self.quat_x * self.quat_z
        yz = self.quat_y * self.quat_z
        wx = self.quat_w * self.quat_x
        wy = self.quat_w * self.quat_y
        wz = self.quat_w * self.quat_z

        self.R11 = 1 - 2 * (yy + zz)
        self.R12 = 2 * (xy - wz)
        self.R13 = 2 * (xz + wy)
        self.R21 = 2 * (xy + wz)
        self.R22 = 1 - 2 * (xx + zz)
        self.R23 = 2 * (yz - wx)
        self.R31 = 2 * (xz - wy)
        self.R32 = 2 * (yz + wx)
        self.R33 = 1 - 2 * (xx + yy)

        rotm = [self.R11, self.R12, self.R13, self.R21, self.R22, self.R23, self.R31, self.R32, self.R33]

        return rotm

    def get_heading_x(self):
        # # rotation matrix
        # RotM = np.array([[self.R11, self.R12, self.R13], [self.R21, self.R22, self.R23], [self.R31, self.R32, self.R33]])
        # Axis_x_b = np.array([[1], [0], [0]])
        # proj_x = np.dot(RotM,Axis_x_b)
        # proj_x_x = proj_x[0][0]
        # proj_x_y = proj_x[1][0]
        # if proj_x_x >= 0:
        #     if proj_x_y >= 0:
        #         heading = math.atan2(abs(proj_x_y),abs(proj_x_x))*180/math.pi   # +x +x
        #     else:
        #         heading = math.atan2(abs(proj_x_y),abs(proj_x_x))*180/math.pi + 270  # +x -y
        #
        # else:
        #     if proj_x_y >= 0:
        #         heading = math.atan2(abs(proj_x_y),abs(proj_x_x))*180/math.pi + 90  # -x +y
        #     else:
        #         heading = math.atan2(abs(proj_x_y),abs(proj_x_x))*180/math.pi + 180  #-x -y

        # atan2(y,x)
        # heading is the angle where x axis in body frame points to
        #heading = math.atan2(2 * self.quat_x*self.quat_z+2*self.quat_y*self.quat_w, 2 * self.quat_y * self.quat_z - 2 * self.quat_x * self.quat_w)
        # angle x
        heading = math.atan2(self.R21, self.R11)

        return heading

    def get_roll_x(self):
        roll_x = math.atan2(2*self.quat_w*self.quat_x + 2*self.quat_y*self.quat_z, 1 - 2*(self.quat_y*self.quat_y + self.quat_x*self.quat_x))
        return roll_x

    def get_tpp(self):
        xi_x = math.atan2(self.R13, self.R33)
        xi_y = math.atan2(self.R23, self.R33)
        # xi_x = self.R13
        # xi_y = self.R23
        tpp = [xi_x, xi_y]
        return tpp

    def get_z_x(self):
        z_x = [self.R11, self.R21, self.R31]
        return z_x

    def get_z_y(self):
        z_y = [self.R12, self.R22, self.R32]
        return z_y

    def get_quat_dot(self):
        diff_qx = self.quat_x - self.qx_last
        diff_qy = self.quat_y - self.qy_last
        diff_qz = self.quat_z - self.qz_last
        diff_qw = self.quat_w - self.qw_last
        self.qx_last = self.quat_x
        self.qy_last = self.quat_y
        self.qz_last = self.quat_z
        self.qw_last = self.quat_w

        dot_quat = [diff_qw / self.sample_time, diff_qx / self.sample_time, diff_qy / self.sample_time,
                    diff_qz / self.sample_time]

        return dot_quat

    def get_Omega(self):

        diff_qx = self.quat_x - self.qx_last
        diff_qy = self.quat_y - self.qy_last
        diff_qz = self.quat_z - self.qz_last
        diff_qw = self.quat_w - self.qw_last
        self.qx_last = self.quat_x
        self.qy_last = self.quat_y
        self.qz_last = self.quat_z
        self.qw_last = self.quat_w

        dot_quat = [diff_qw/self.sample_time, diff_qx/self.sample_time, diff_qy/self.sample_time, diff_qz/self.sample_time]

        E_q_trans = [[-self.quat_x, self.quat_w, self.quat_z, -self.quat_y],
                     [-self.quat_y, -self.quat_z, self.quat_w, self.quat_x],
                     [-self.quat_z, self.quat_y, -self.quat_x, self.quat_w]]

        self.Omega = 2 * np.dot(E_q_trans, dot_quat)

        return self.Omega

    def get_Omega_filt(self):
        Omega_x = self.Omega[0]
        Omega_y = self.Omega[1]
        Omega_z = self.Omega[2]

        self.Omega_x_f = self.FilterOmega_x.filter(Omega_x)
        self.Omega_y_f = self.FilterOmega_y.filter(Omega_y)
        self.Omega_z_f = self.FilterOmega_z.filter(Omega_z)

        self.Omega_f = [self.Omega_x_f, self.Omega_y_f, self.Omega_z_f]

        return self.Omega_f

    def get_RPY(self):
        # roll - rotating about x axis
        roll_a = 2 * (self.quat_w * self.quat_x + self.quat_y * self.quat_z)
        roll_b = 1 - 2 * (self.quat_x * self.quat_x + self.quat_y * self.quat_y)
        angle_roll = math.atan2(roll_a, roll_b)

        # pitch - rotating about y axis
        pitch_a = 2 * (self.quat_w * self.quat_y - self.quat_z * self.quat_x)
        angle_pitch = math.asin(pitch_a)

        # yaw - rotating about z axis
        yaw_a = 2 * (self.quat_w * self.quat_z + self.quat_x * self.quat_y)
        yaw_b = 1 - 2 * (self.quat_y * self.quat_y + self.quat_z * self.quat_z)
        angle_yaw = math.atan2(yaw_a, yaw_b)

        RPY = [angle_roll, angle_pitch, angle_yaw]
        return RPY



