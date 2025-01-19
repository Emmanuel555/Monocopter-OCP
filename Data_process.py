# from Library.IIR2Filter import IIR2Filter
import struct
from Filter import IIR2Filter
import math
import numpy as np
from pyrr import quaternion


class RealTimeProcessor(object):
    def __init__(self, order, cutoff, ftype, design, rs, sample_rate):
        self.flag = 0
        self.sample_time = 1/sample_rate
        self.sample_rate = sample_rate
        self.roll_x_last = 0
        self.pitch_y_last = 0
        self.rollrate_x_last = 0
        self.pitchrate_y_last = 0
        self.qx_last = 0
        self.qy_last = 0
        self.qz_last = 0
        self.qw_last = 0
        self.diff_qx_last = 0
        self.diff_qy_last = 0
        self.diff_qz_last = 0
        self.diff_qw_last = 0
        self.px_last = 0
        self.py_last = 0
        self.pz_last = 0
        self.vx_last = 0
        self.vy_last = 0
        self.vz_last = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        
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

        # filtered data with IIR2Filter
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
        
        # 0.0005 is the scale factor for the position data
        self.px = x * 0.0005 # position px 
        self.py = y * 0.0005  # position py  
        self.pz = z * 0.0005  # position pz 

        #self.px = float(x)   # position px 
        #self.py = float(y)   # position py  
        #self.pz = float(z)   # position pz 

        # 1 seems to work better than 0.001
        self.quat_x = float(qx)
        self.quat_y = float(qy)
        self.quat_z = float(qz)
        self.quat_w = float(qw)

        #self.quat_x = float(qx)
        #self.quat_y = float(qy)
        #self.quat_z = float(qz)
        #self.quat_w = float(qw)/np.abs(float(qw)) # normalizing the quaternion

        self.raw_data = [self.px, self.py, self.pz, self.quat_x, self.quat_y, self.quat_z, self.quat_w]
        #return raw_data

    def data_unpack_filtered(self,udp_data):
        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data) # h refers to python type integer of byte size 2

        # 0.0005 is the scale factor for the position data
        self.px = x * 0.0005  # position px 
        self.py = y * 0.0005  # position py  
        self.pz = z * 0.0005  # position pz 

        # 1 seems to work better than 0.001
        self.quat_x = float(qx)
        self.quat_y = float(qy)
        self.quat_z = float(qz)
        self.quat_w = float(qw) # needa check if this can always be left as 1 when spinning 

        self.px_filted = self.FilterX.filter(self.px)
        self.py_filted = self.FilterY.filter(self.py)
        self.pz_filted = self.FilterZ.filter(self.pz)

        self.quat_x_filted = self.FilterQX.filter(self.quat_x)
        self.quat_y_filted = self.FilterQY.filter(self.quat_y)
        self.quat_z_filted = self.FilterQZ.filter(self.quat_z)
        self.quat_w_filted = self.FilterQW.filter(self.quat_w)

        #return filted_data
        self.filted_data = [self.px_filted, self.py_filted, self.pz_filted, self.quat_x_filted, self.quat_y_filted, self.quat_z_filted, self.quat_w_filted]

        
    def pos_vel_acc_filtered(self):
        self.vx = (self.px_filted - self.px_last)/self.sample_time
        self.vy = (self.py_filted - self.py_last)/self.sample_time
        self.vz = (self.pz_filted - self.pz_last)/self.sample_time

        self.ax = (self.vx - self.vx_last)/self.sample_time
        self.ay = (self.vy - self.vy_last)/self.sample_time
        self.az = (self.vz - self.vz_last)/self.sample_time

        self.px_last = self.px_filted
        self.py_last = self.py_filted
        self.pz_last = self.pz_filted

        self.vx_last = self.vx
        self.vy_last = self.vy
        self.vz_last = self.vz

        pos_vel_acc = np.array([self.px_filted, self.py_filted, self.pz_filted, self.vx, self.vy, self.vz, self.ax, self.ay, self.az])
        
        #return vel_acc
        return pos_vel_acc


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
        self.R13 = 2 * (xz + wy) # abt x (Contribution of the rotating z-axis to the rotated x-axis)
        self.R21 = 2 * (xy + wz)
        self.R22 = 1 - 2 * (xx + zz)
        self.R23 = 2 * (yz - wx) # abt y (Contribution of the rotating z-axis to the rotated y-axis)
        self.R31 = 2 * (xz - wy)
        self.R32 = 2 * (yz + wx)
        self.R33 = 1 - 2 * (xx + yy) # How much of the z-axis aligns with itself after the rotation

        rotm = [self.R11, self.R12, self.R13, self.R21, self.R22, self.R23, self.R31, self.R32, self.R33]

        #return rotm
        

    def get_tpp_angle_xy(self): # solved during testing
        """ Finding the TPP Angle
        To calculate the TPP angle using the rotation matrix:

        Step 1: Identify the TPP Normal Vector
        The TPP is typically perpendicular to the rotational velocity vector (𝜔D) in the disk's body frame. 
        Assuming the blade motion traces a circle, the TPP normal in the body frame is:

        𝑛TPP=[0,0,1]T
        
        This vector is aligned with the disk's z-axis in the body frame.

        Step 2: Transform the Normal to the Inertial Frame
        Use the rotation matrix rotm to transform 𝑛TPP into the inertial frame:

        𝑛TPP inertial = rotm⋅𝑛TPP

        This gives the orientation of the TPP in the inertial frame. """
                
        abt_y = math.atan2(self.R13, self.R33) # atan2(opp,adj) \| where opp is x vector and adj is z vector
        abt_x = math.atan2(self.R23, self.R33) # atan2(opp,adj) \| where opp is y vector and adj is z vector     
        abt_z = 0
         
        # self.tpp is in radians 

        # needa multiply with R22 to get the correct roll angle
        self.tpp[0] = -1*abt_x # disk roll - need to negate to match the convention of the tpp
        # needa multiply with R11 to get the correct pitch angle
        self.tpp[1] = abt_y # disk pitch
        self.tpp[2] = abt_z # disk yaw

        return self.tpp


    def get_Omega_dot_dotdot(self): # taken from shane
        self.get_rotm()
        self.get_tpp_angle_xy() # in radians

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
        self.get_tpp_angle_xy()

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
        E_q_trans_filted = [[-self.quat_x_filted, self.quat_w_filted, self.quat_z_filted, -self.quat_y_filted],
                     [-self.quat_y_filted, -self.quat_z_filted, self.quat_w_filted, self.quat_x_filted],
                     [-self.quat_z_filted, self.quat_y_filted, -self.quat_x_filted, self.quat_w_filted]]
        
        self.Omega = 2 * np.dot(E_q_trans_filted, dot_quat) # 3 x 1 - about x, y, z
        self.Omega_dot = 2 * np.dot(E_q_trans_filted, dot_dot_quat) # 3 x 1 - about x, y, z

        roll = self.tpp[0]
        pitch = self.tpp[1]

        rot_mat_world2tpp = [[1, 0, -math.sin(pitch)],
                             [0, math.cos(roll), math.cos(pitch)*math.sin(roll)],
                             [0, -math.sin(roll), math.cos(pitch)*math.cos(roll)]]
        
        self.Omega = np.dot(rot_mat_world2tpp, self.Omega) # 3 x 1 - about x, y, z
        self.Omega_dot = np.dot(rot_mat_world2tpp, self.Omega_dot) # 3 x 1 - about x, y, z
    
        return (self.tpp, self.Omega, self.Omega_dot) # convention is abt x, y, z - rpy world frame w heading zero facing x


    def get_Omega_dot_dotdot_filt_eul(self):
        self.get_rotm_filtered()
        self.get_tpp_angle_xy()

        roll = self.tpp[0]
        pitch = self.tpp[1]

        rollrate_x = (roll - self.roll_x_last)/self.sample_time
        pitchrate_y = (pitch - self.pitch_y_last)/self.sample_time

        rollraterate_x = (rollrate_x - self.rollrate_x_last)/self.sample_time
        pitchraterate_y = (pitchrate_y - self.pitchrate_y_last)/self.sample_time

        self.roll_x_last = roll
        self.pitch_y_last = pitch
        
        self.rollrate_x_last = rollrate_x 
        self.pitchrate_y_last = pitchrate_y
         
        self.Omega = [rollrate_x, pitchrate_y, 0] # 3 x 1 - about x, y, z
        self.Omega_dot = [rollraterate_x, pitchraterate_y, 0] # 3 x 1 - about x, y, z

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
        Converts the TPP Euler angle to a TPP quaternion angle to prepare quaternion rotation of vector 001
        
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
        
        tpp_qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        tpp_qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        tpp_qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        tpp_qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        
        tpp_quaternion = np.array([tpp_qx, tpp_qy, tpp_qz, tpp_qw])

        # alternative from pyrr same formula as above 
        ## formula:
        ##  
        ##        halfRoll = roll * 0.5
        ##        sR = np.sin(halfRoll)
        ##        cR = np.cos(halfRoll)

        ##        halfPitch = pitch * 0.5
        ##        sP = np.sin(halfPitch)
        ##        cP = np.cos(halfPitch)

        ##        halfYaw = yaw * 0.5
        ##        sY = np.sin(halfYaw)
        ##        cY = np.cos(halfYaw)
        ##        
        ##    tpp_qx = (sR * cP * cY) + (cR * sP * sY),
        ##    tpp_qy = (cR * sP * cY) - (sR * cP * sY),
        ##    tpp_qz = (cR * cP * sY) + (sR * sP * cY),
        ##    tpp_qw = (cR * cP * cY) - (sR * sP * sY),
        ##

        # tpp_quaternion = quaternion.create_from_eulers(roll, pitch, yaw) # actual line of code to use
        
        return tpp_quaternion
    


