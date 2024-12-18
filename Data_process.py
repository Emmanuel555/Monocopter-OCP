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
        self.diff_qx_last = 0
        self.diff_qy_last = 0
        self.diff_qz_last = 0
        self.diff_qw_last = 0

        # raw data from motion capture
        self.px = 0
        self.py = 0
        self.pz = 0
        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.quat_w = 0  

        # tpp angle
        self.tpp = np.array([0, 0, 0])  # flat array 

        # filted data with IIR2Filter
        self.px_filted = 0
        self.py_filted = 0
        self.pz_filted = 0
        self.quat_x_filted = 0
        self.quat_y_filted = 0
        self.quat_z_filted = 0
        self.quat_w_filted = 0

        # omega and omega_dot
        self.Omega = np.array([0, 0, 0])  # flat array
        self.Omega_dot = np.array([0, 0, 0])  # flat array

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

        self.FilterQX = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQY = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQZ = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterQW = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)

        """ self.FilterOmega_x = IIR2Filter(5, [50], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterOmega_y = IIR2Filter(5, [50], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterOmega_z = IIR2Filter(5, [50], ftype, design=design, rs=rs, fs=sample_rate)

        self.FilterOmega_dot_x = IIR2Filter(5, [70], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterOmega_dot_y = IIR2Filter(5, [70], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterOmega_dot_z = IIR2Filter(5, [70], ftype, design=design, rs=rs, fs=sample_rate) """

        
    def data_unpack(self, udp_data): ## qns abt this
        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data) # h refers to python type integer of byte size 2
        
        self.px = x * 0.0005  # position px 
        self.py = y * 0.0005  # position py  
        self.pz = z * 0.0005  # position pz 

        #self.px = float(x)   # position px 
        #self.py = float(y)   # position py  
        #self.pz = float(z)   # position pz 

        self.quat_x = float(qx * 0.001)
        self.quat_y = float(qy * 0.001)
        self.quat_z = float(qz * 0.001)
        self.quat_w = float(qw * 0.001)

        #self.quat_x = float(qx)
        #self.quat_y = float(qy)
        #self.quat_z = float(qz)
        #self.quat_w = float(qw)/np.abs(float(qw)) # normalizing the quaternion

        raw_data = [self.px, self.py, self.pz, self.quat_x, self.quat_y, self.quat_z, self.quat_w]
        #return raw_data

    def data_unpack_filtered(self,udp_data):
        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data) # h refers to python type integer of byte size 2

        self.px = x * 0.0005  # position px 
        self.py = y * 0.0005  # position py  
        self.pz = z * 0.0005  # position pz 

        self.quat_x = float(qx * 0.001)
        self.quat_y = float(qy * 0.001)
        self.quat_z = float(qz * 0.001)
        self.quat_w = float(qw) # needa check if this can always be left as 1 when spinning 

        self.px_filted = self.FilterX.filter(self.px)
        self.py_filted = self.FilterY.filter(self.py)
        self.pz_filted = self.FilterZ.filter(self.pz)

        self.quat_x_filted = self.FilterQX.filter(self.quat_x)
        self.quat_y_filted = self.FilterQY.filter(self.quat_y)
        self.quat_z_filted = self.FilterQZ.filter(self.quat_z)
        self.quat_w_filted = self.FilterQW.filter(self.quat_w)

        self.filted_data = [self.px_filted, self.py_filted, self.pz_filted, self.quat_x_filted, self.quat_y_filted, self.quat_z_filted, self.quat_w_filted]

        #return filted_data


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
        self.R13 = 2 * (xz + wy) # z cross x or x cross z = y vector
        self.R21 = 2 * (xy + wz)
        self.R22 = 1 - 2 * (xx + zz)
        self.R23 = 2 * (yz - wx) # z cross y or y cross z = x vector
        self.R31 = 2 * (xz - wy)
        self.R32 = 2 * (yz + wx)
        self.R33 = 1 - 2 * (xx + yy) # x cross y or y cross x = z vector

        rotm = [self.R11, self.R12, self.R13, self.R21, self.R22, self.R23, self.R31, self.R32, self.R33]

        #return rotm

    
    def get_rotm_filtered(self):
        xx = self.quat_x_filted * self.quat_x_filted
        yy = self.quat_y_filted * self.quat_y_filted
        zz = self.quat_z_filted * self.quat_z_filted
        xy = self.quat_x_filted * self.quat_y_filted
        xz = self.quat_x_filted * self.quat_z_filted
        yz = self.quat_y_filted * self.quat_z_filted
        wx = self.quat_w_filted * self.quat_x_filted
        wy = self.quat_w_filted * self.quat_y_filted
        wz = self.quat_w_filted * self.quat_z_filted

        self.R11 = 1 - 2 * (yy + zz)
        self.R12 = 2 * (xy - wz)
        self.R13 = 2 * (xz + wy) # z cross x or x cross z = y vector
        self.R21 = 2 * (xy + wz)
        self.R22 = 1 - 2 * (xx + zz)
        self.R23 = 2 * (yz - wx) # z cross y or y cross z = x vector
        self.R31 = 2 * (xz - wy)
        self.R32 = 2 * (yz + wx)
        self.R33 = 1 - 2 * (xx + yy) # x cross y or y cross x = z vector

        rotm = [self.R11, self.R12, self.R13, self.R21, self.R22, self.R23, self.R31, self.R32, self.R33]

        #return rotm
        

    def get_tpp_angle(self): # qns abt this
        xi_x = math.atan2(self.R13, self.R33) # atan2(opp,adj) \| where opp is y vector and adj is z vector
        xi_y = math.atan2(self.R23, self.R33) # atan2(opp,adj) \| where opp is x vector and adj is z vector
        xi_z = 0
        # xi_x = self.R13
        # xi_y = self.R23
        # tpp = [xi_x, xi_y, xi_z] # disk euler angle vector about (x, y, z)
        
        self.tpp[0] = xi_x # roll
        self.tpp[1] = xi_y # pitch
        self.tpp[2] = xi_z # yaw

        #return self.tpp


    def get_Omega_dot_dotdot(self): # taken from shane
        self.get_rotm()
        self.get_tpp_angle()

        diff_qx = self.quat_x - self.qx_last
        diff_qy = self.quat_y - self.qy_last
        diff_qz = self.quat_z - self.qz_last
        diff_qw = self.quat_w - self.qw_last

        diff_diff_qx = diff_qx - self.diff_qx_last
        diff_diff_qy = diff_qy - self.diff_qy_last
        diff_diff_qz = diff_qz - self.diff_qz_last
        diff_diff_qw = diff_qw - self.diff_qz_last

        self.qx_last = self.quat_x
        self.qy_last = self.quat_y
        self.qz_last = self.quat_z
        self.qw_last = self.quat_w

        self.diff_qx_last = diff_qx
        self.diff_qy_last = diff_qy
        self.diff_qz_last = diff_qz
        self.diff_qw_last = diff_qw

        dot_quat = [diff_qw/self.sample_time, diff_qx/self.sample_time, diff_qy/self.sample_time, diff_qz/self.sample_time]
        dot_dot_quat = [diff_diff_qw/self.sample_time, diff_diff_qx/self.sample_time, diff_diff_qy/self.sample_time, diff_diff_qz/self.sample_time]
        # the convention for quat here is quat rot mat * qw qx qy qz 
        E_q_trans = [[-self.quat_x, self.quat_w, self.quat_z, -self.quat_y],
                     [-self.quat_y, -self.quat_z, self.quat_w, self.quat_x],
                     [-self.quat_z, self.quat_y, -self.quat_x, self.quat_w]]

        self.Omega = 2 * np.dot(E_q_trans, dot_quat) # 3 x 1 - about x, y, z
        self.Omega_dot = 2 * np.dot(E_q_trans, dot_dot_quat) # 3 x 1 - about x, y, z

        roll = self.tpp[0]
        pitch = self.tpp[1]

        rot_mat_world2tpp = [[1, 0, -math.sin(pitch)],
                             [0, math.cos(roll), math.cos(pitch)*math.sin(roll)],
                             [0, -math.sin(roll), math.cos(pitch)*math.cos(roll)]]
        
        self.Omega = np.dot(rot_mat_world2tpp, self.Omega) # 3 x 1 - about x, y, z
        self.Omega_dot = np.dot(rot_mat_world2tpp, self.Omega_dot) # 3 x 1 - about x, y, z

        return (self.tpp, self.Omega, self.Omega_dot) # convention is abt x, y, z - rpy world frame w heading zero facing x


    def get_Omega_dot_dotdot_filt(self):
        self.get_rotm_filtered()
        self.get_tpp_angle()

        diff_qx = self.quat_x_filted - self.qx_last
        diff_qy = self.quat_y_filted - self.qy_last
        diff_qz = self.quat_z_filted - self.qz_last
        diff_qw = self.quat_w_filted - self.qw_last

        diff_diff_qx = diff_qx - self.diff_qx_last
        diff_diff_qy = diff_qy - self.diff_qy_last
        diff_diff_qz = diff_qz - self.diff_qz_last
        diff_diff_qw = diff_qw - self.diff_qz_last

        self.qx_last = self.quat_x_filted
        self.qy_last = self.quat_y_filted
        self.qz_last = self.quat_z_filted
        self.qw_last = self.quat_w_filted

        self.diff_qx_last = diff_qx
        self.diff_qy_last = diff_qy
        self.diff_qz_last = diff_qz
        self.diff_qw_last = diff_qw

        dot_quat = [diff_qw/self.sample_time, diff_qx/self.sample_time, diff_qy/self.sample_time, diff_qz/self.sample_time]
        dot_dot_quat = [diff_diff_qw/self.sample_time, diff_diff_qx/self.sample_time, diff_diff_qy/self.sample_time, diff_diff_qz/self.sample_time]
        # the convention for quat here is quat rot mat * qw qx qy qz 
        E_q_trans = [[-self.quat_x, self.quat_w, self.quat_z, -self.quat_y],
                     [-self.quat_y, -self.quat_z, self.quat_w, self.quat_x],
                     [-self.quat_z, self.quat_y, -self.quat_x, self.quat_w]]
        
        self.Omega = 2 * np.dot(E_q_trans, dot_quat) # 3 x 1 - about x, y, z
        self.Omega_dot = 2 * np.dot(E_q_trans, dot_dot_quat) # 3 x 1 - about x, y, z

        roll = self.tpp[0]
        pitch = self.tpp[1]

        rot_mat_world2tpp = [[1, 0, -math.sin(pitch)],
                             [0, math.cos(roll), math.cos(pitch)*math.sin(roll)],
                             [0, -math.sin(roll), math.cos(pitch)*math.cos(roll)]]
        
        self.Omega = np.dot(rot_mat_world2tpp, self.Omega) # 3 x 1 - about x, y, z
        self.Omega_dot = np.dot(rot_mat_world2tpp, self.Omega_dot) # 3 x 1 - about x, y, z
    
        return (self.tpp, self.Omega, self.Omega_dot) # convention is abt x, y, z - rpy world frame w heading zero facing x


    def get_RPY(self): # solely for quad
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

        RPY = [angle_roll, angle_pitch, angle_yaw] # pry
        return RPY
    

    def tpp_eulerAnglesToQuaternion(self):
        """
        Convert an Euler angle to a quaternion.
        
        We have used the following definition of Euler angles.

        - Tait-Bryan variant of Euler Angles
        - Yaw-pitch-roll rotation order (ZYX convention), rotating around the z, y and x axes respectively
        - Intrinsic rotation (the axes move with each rotation)
        - Active (otherwise known as alibi) rotation (the point is rotated, not the coordinate system)
        - Right-handed coordinate system with right-handed rotations
        
        Parameters
        ----------
        eulerAngles : 
            [3x1] np.ndarray  
            [roll, pitch, yaw] angles in radians 
                
        Returns
        -------
        p : [4x1] np.ndarray
            quaternion defining a given orientation
    """
        """ if isinstance(eulerAngles, list) and len(eulerAngles)==3:
            eulerAngles = np.array(eulerAngles) 
        elif isinstance(eulerAngles, np.ndarray) and eulerAngles.size==3:
            pass
        else:
            raise TypeError("The eulerAngles must be given as [3x1] np.ndarray vector or a python list of 3 elements")
        
        
        roll = eulerAngles[0]
        pitch = eulerAngles[1]
        yaw = eulerAngles[2]
        """
        
        roll = self.tpp[0]
        pitch = self.tpp[1]
        yaw = self.tpp[2]
        
        q0 = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q1 = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q2 = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q3 = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        
        p = np.r_[q0, q1, q2, q3]
        
        return p
    


