import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import statistics
  
class all_trajectory(object):
    def __init__(self):
        pass

    def traj_data_compiler(self,file_path):
        #file_path = '/home/emmanuel/Monocopter-OCP/circle/0.3/'
        files = os.listdir(file_path)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)

        last_file = files[0]
        mat_data = loadmat(os.path.join(file_path, last_file)) ##

        #print(mat_data)
        time = mat_data['Data_time']
        #position = mat_data['data']
        position = mat_data['Monocopter_XYZ']
        ref_position = mat_data['ref_position']
        ref_velocity = mat_data['ref_velocity']
        att_feedback = mat_data['rotational_state_vector']
        yawrate = mat_data['yawrate']
        motor_cmd = mat_data['motor_cmd']

        # ideal
        ## circle
        #start = int(np.size(time[0])*1.5/5)
        #end = int(np.size(time[0])*3.5/5) #- int(np.size(time[0])

        ## lem-short
        #start = int(np.size(time[0])*2.83/5)
        #end = int(np.size(time[0])*5/5) #- int(np.size(time[0])

        ## lem-long
        #start = int(np.size(time[0])*2/5)
        #end = int(np.size(time[0])*4.47/5) #- int(np.size(time[0])

        start = 0
        end = int(np.size(time[0])*5/5) #- int(np.size(time[0])

        px = [row[0] for row in position] # column vector
        py = [row[1] for row in position]
        pz = [row[2] for row in position]
        vx = [row[3] for row in position] # column vector
        vy = [row[4] for row in position]
        vz = [row[5] for row in position]

        px = np.array([px])
        py = np.array([py])
        pz = np.array([pz])

        vx = np.array([vx])
        vy = np.array([vy])
        vz = np.array([vz])

        px_r = [row[0] for row in ref_position] # column vector
        py_r = [row[1] for row in ref_position]
        pz_r = [row[2] for row in ref_position]

        px_r = np.array([px_r])
        py_r = np.array([py_r])
        pz_r = np.array([pz_r])

        vx_r = [row[0] for row in ref_velocity] # column vector
        vy_r = [row[1] for row in ref_velocity]
        vz_r = [row[2] for row in ref_velocity]

        vx_r = np.array([vx_r])
        vy_r = np.array([vy_r])
        vz_r = np.array([vz_r])

        roll_raterate = [row[2][0] for row in att_feedback] # column vector
        pitch_raterate = [row[2][1] for row in att_feedback] 

        roll_raterate = np.array([roll_raterate])
        pitch_raterate = np.array([pitch_raterate])

        traj_time = time[0][start:end]
        px_r = px_r[0][start:end]
        py_r = py_r[0][start:end]
        pz_r = pz_r[0][start:end]
        vx_r = vx_r[0][start:end]
        vy_r = vy_r[0][start:end]
        vz_r = vz_r[0][start:end]
        #mf_px = ndimage.median_filter(px[0][start:end], size=1300)
        #mf_py = ndimage.median_filter(py[0][start:end], size=1300)
        #mf_pz = ndimage.median_filter(pz[0][start:end], size=1300)
        mf_px = ndimage.gaussian_filter1d(px[0][start:end], 200) #200
        mf_py = ndimage.gaussian_filter1d(py[0][start:end], 200)
        mf_pz = ndimage.gaussian_filter1d(pz[0][start:end], 200)
        mf_vx = ndimage.median_filter(vx[0][start:end], size=300)
        mf_vy = ndimage.median_filter(vy[0][start:end], size=300)
        mf_vz = ndimage.median_filter(vz[0][start:end], size=300)
        roll_raterate = ndimage.gaussian_filter1d(roll_raterate[0][start:end], 500) # could be 1500
        pitch_raterate = ndimage.gaussian_filter1d(pitch_raterate[0][start:end], 500) # could be 1500
        #roll_raterate = ndimage.median_filter(roll_raterate, size=1200) # could be 1500
        #pitch_raterate = ndimage.median_filter(pitch_raterate, size=1200) # could be 1500
        
        yawrate = ndimage.gaussian_filter1d(yawrate[0], 120)
        #yawrate = ndimage.median_filter(yawrate[0], size=300)
        yawrate = np.round((yawrate[start:end]/(2*math.pi)),2) # in Hz

        #motor_cmd = ndimage.median_filter(motor_cmd[0][start:end], size=30)
        motor_cmd = ndimage.gaussian_filter1d(motor_cmd[0][start:end], 120)

        xyz_error_norm = []
        velocity_xyz_error_norm = []
        rollpitch_raterate_norm = []  
        yawrate_ls = []     

        for i in range(len(traj_time)):
            x_error_squared = (px_r[i] - mf_px[i]) ** 2
            y_error_squared = (py_r[i] - mf_py[i]) ** 2    
            z_error_squared = (pz_r[i] - mf_pz[i]) ** 2
            xyz_error_norm.append(math.sqrt(x_error_squared + y_error_squared + z_error_squared))

            vx_error_squared = (vx_r[i] - mf_vx[i]) ** 2
            vy_error_squared = (vy_r[i] - mf_vy[i]) ** 2    
            vz_error_squared = (vz_r[i] - mf_vz[i]) ** 2
            velocity_xyz_error_norm.append(math.sqrt(vx_error_squared + vy_error_squared + vz_error_squared))

            #roll_raterate_error_squared = (att_raterate_error_roll[i]/10000) ** 2
            #pitch_raterate_error_squared = (att_raterate_error_pitch[i]/10000) ** 2
            roll_raterate_sq = (roll_raterate[i]) ** 2
            pitch_raterate_sq = (pitch_raterate[i]) ** 2
            #roll_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared))
            #pitch_raterate_error_norm.append(math.sqrt(pitch_raterate_error_squared))
            rollpitch_raterate_norm.append(math.sqrt(roll_raterate_sq + pitch_raterate_sq))
            #norm = norm*(180/math.pi)
            yawrate_ls.append(yawrate[i])

            
        return (
            traj_time,
            mf_px,
            mf_py,
            mf_pz,
            px_r,
            py_r,
            pz_r,
            xyz_error_norm,
            rollpitch_raterate_norm, #8
            yawrate_ls, #9
            mf_vx,
            mf_vy,
            mf_vz,
            vx_r,
            vy_r,
            vz_r,
            velocity_xyz_error_norm,
            motor_cmd #17
        )
    

    def att_data_compiler(self,file_path):
        #file_path = '/home/emmanuel/Monocopter-OCP/circle/0.3/'
        files = os.listdir(file_path)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)

        last_file = files[0]
        mat_data = loadmat(os.path.join(file_path, last_file)) ##

        #print(mat_data)
        time = mat_data['Data_time']
        #position = mat_data['data']
        position = mat_data['Monocopter_XYZ']
        ref_position = mat_data['ref_position']
        att_raterate_error = mat_data['att_raterate_error']
        yawrate = mat_data['yawrate']

        start = 0
        end = int(np.size(time[0])) # int(np.size(time[0])

        px = [row[0] for row in position] # column vector
        py = [row[1] for row in position]
        pz = [row[2] for row in position]

        px = np.array([px])
        py = np.array([py])
        pz = np.array([pz])

        px_r = [row[0] for row in ref_position] # column vector
        py_r = [row[1] for row in ref_position]
        pz_r = [row[2] for row in ref_position]

        px_r = np.array([px_r])
        py_r = np.array([py_r])
        pz_r = np.array([pz_r])

        att_raterate_error_roll = [row[0] for row in att_raterate_error] # column vector
        att_raterate_error_pitch = [row[1] for row in att_raterate_error] 

        att_raterate_error_roll = np.array([att_raterate_error_roll])
        att_raterate_error_pitch = np.array([att_raterate_error_pitch])

        traj_time = time[0][start:end]
        px_r = px_r[0][start:end]
        py_r = py_r[0][start:end]
        pz_r = pz_r[0][start:end]
        mf_px = ndimage.median_filter(px[0][start:end], size=700)
        mf_py = ndimage.median_filter(py[0][start:end], size=700)
        mf_pz = ndimage.median_filter(pz[0][start:end], size=700)
        att_raterate_error_roll = ndimage.median_filter(att_raterate_error_roll[0][start:end], size=1500)
        att_raterate_error_pitch = ndimage.median_filter(att_raterate_error_pitch[0][start:end], size=1500)
        
        yawrate = ndimage.median_filter(yawrate[0], size=300)
        yawrate = np.round((yawrate[start:end]/(2*math.pi)),2) # in Hz

        xyz_error_norm = []
        rollpitch_raterate_error_norm = []
        roll_raterate_error_norm  = []
        pitch_raterate_error_norm = []  
        yawrate_ls = []  
        roll_raterate_error_sq = []
        pitch_raterate_error_sq = []   

        for i in range(len(mf_px)):
            x_error_squared = (px_r[i] - mf_px[i]) ** 2
            y_error_squared = (py_r[i] - mf_py[i]) ** 2    
            z_error_squared = (pz_r[i] - mf_pz[i]) ** 2
            xyz_error_norm.append(math.sqrt(x_error_squared + y_error_squared + z_error_squared))

            roll_raterate_error_squared = (att_raterate_error_roll[i]/10000) ** 2
            pitch_raterate_error_squared = (att_raterate_error_pitch[i]/10000) ** 2
            roll_raterate_error_sq.append(roll_raterate_error_squared)
            pitch_raterate_error_sq.append(pitch_raterate_error_squared)
            roll_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared))
            pitch_raterate_error_norm.append(math.sqrt(pitch_raterate_error_squared))
            rollpitch_raterate_error_norm.append(math.sqrt(roll_raterate_error_squared + pitch_raterate_error_squared))
            
            yawrate_ls.append(yawrate[i])

        roll_raterate_rmse = math.sqrt(sum(roll_raterate_error_sq)/len(mf_px))
        pitch_raterate_rmse = math.sqrt(sum(pitch_raterate_error_sq)/len(mf_px))

        rmse_raterate_list = [roll_raterate_rmse,pitch_raterate_rmse]
        final_rmse_raterate = la.norm(rmse_raterate_list, 2)
            
        return (
            traj_time,
            mf_px,
            mf_py,
            mf_pz,
            px_r,
            py_r,
            pz_r,
            xyz_error_norm,
            roll_raterate_error_norm,
            pitch_raterate_error_norm,
            rollpitch_raterate_error_norm,
            yawrate_ls,
            final_rmse_raterate
        )


    def normal_data_compiler(self,file_path,type):
        #file_path = '/home/emmanuel/Monocopter-OCP/circle/0.3/'
        files = os.listdir(file_path)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)

        last_file = files[0]
        mat_data = loadmat(os.path.join(file_path, last_file)) ##

        #print(mat_data)
        time = mat_data['Data_time']
        position = mat_data['Monocopter_XYZ']
        ref_position = mat_data['ref_position']
        yawrate = mat_data['yawrate']
        motor_cmd = mat_data['motor_cmd']
        ref_velocity = mat_data['ref_velocity']

        # short
        if type == 'short_traj_data':
            start = 0 #int(np.size(time[0])/10)
            end = int(np.size(time[0])) #- int(np.size(time[0])
        elif type == 'long_traj_data':
        #long
            start = int(np.size(time[0])*2/5)
            end = int(np.size(time[0])*4/5)
        else: # ultralight
            start = 0 #int(np.size(time[0])/10)
            end = int(np.size(time[0])) #- int(np.size(time[0])    

        px = [row[0] for row in position] # column vector
        py = [row[1] for row in position]
        pz = [row[2] for row in position]
        vx = [row[3] for row in position] # column vector
        vy = [row[4] for row in position]
        vz = [row[5] for row in position]

        px = np.array([px])
        py = np.array([py])
        pz = np.array([pz])

        vx = np.array([vx])
        vy = np.array([vy])
        vz = np.array([vz])

        px_r = [row[0] for row in ref_position] # column vector
        py_r = [row[1] for row in ref_position]
        pz_r = [row[2] for row in ref_position]

        px_r = np.array([px_r])
        py_r = np.array([py_r])
        pz_r = np.array([pz_r])

        vx_r = [row[0] for row in ref_velocity] # column vector
        vy_r = [row[1] for row in ref_velocity]
        vz_r = [row[2] for row in ref_velocity]

        vx_r = np.array([vx_r])
        vy_r = np.array([vy_r])
        vz_r = np.array([vz_r])

        traj_time = time[0][start:end]
        motor_cmd = ndimage.median_filter(motor_cmd[0][start:end], size=700)
        px_r = px_r[0][start:end]
        py_r = py_r[0][start:end]
        pz_r = pz_r[0][start:end]
        vx_r = vx_r[0][start:end]
        vy_r = vy_r[0][start:end]
        vz_r = vz_r[0][start:end]
        #mf_px = ndimage.gaussian_filter1d(px[0][start:end], 500) #200
        #mf_py = ndimage.gaussian_filter1d(py[0][start:end], 500)
        #mf_pz = ndimage.gaussian_filter1d(pz[0][start:end], 500)

        if type == 'short_traj_data':
            mf_px = ndimage.median_filter(px[0][start:end], size=700)
            mf_py = ndimage.median_filter(py[0][start:end], size=700)
            mf_pz = ndimage.median_filter(pz[0][start:end], size=700)
        elif type == 'long_traj_data':
            mf_px = ndimage.median_filter(px[0][start:end], size=300)
            mf_py = ndimage.median_filter(py[0][start:end], size=300)
            mf_pz = ndimage.median_filter(pz[0][start:end], size=300)
        else: # ultralight
            mf_px = ndimage.median_filter(px[0][start:end], size=700)
            mf_py = ndimage.median_filter(py[0][start:end], size=700)
            mf_pz = ndimage.median_filter(pz[0][start:end], size=700)

        mf_vx = ndimage.median_filter(vx[0][start:end], size=300)
        mf_vy = ndimage.median_filter(vy[0][start:end], size=300)
        mf_vz = ndimage.median_filter(vz[0][start:end], size=300)
        
        yawrate = ndimage.median_filter(yawrate[0], size=700)
        yawrate = np.round((yawrate[start:end]/(2*math.pi)),2) # in Hz

        ## removing duplicate reference points
        ## to test if updating at 250Hz, and yes even after filtering, the data is quite consistent
        new_px = []
        new_py = []
        new_pz = []
        new_vx = []
        new_vy = []
        new_vz = []
        new_rx = []
        new_ry = []
        new_rz = []
        new_vrx = []
        new_vry = []
        new_vrz = []
        new_traj_time = []
        traj_time = traj_time - traj_time[0]
        old_rx = 0.0
        for v in range(traj_time.size):
            #if traj_time[v] < (7500/250):
            if type == 'short_traj_data':
                if px_r[v] != old_rx: # if the reference position has changed, then we take that data point
                    new_px.append(mf_px[v])
                    new_py.append(mf_py[v])
                    new_pz.append(mf_pz[v])
                    new_vx.append(mf_vx[v])
                    new_vy.append(mf_vy[v])
                    new_vz.append(mf_vz[v])
                    new_rx.append(px_r[v])
                    new_ry.append(py_r[v])
                    new_rz.append(pz_r[v])
                    new_vrx.append(vx_r[v])
                    new_vry.append(vy_r[v])
                    new_vrz.append(vz_r[v])
                    new_traj_time.append(traj_time[v])
                    old_rx = px_r[v]
            else:
                new_px.append(mf_px[v])
                new_py.append(mf_py[v])
                new_pz.append(mf_pz[v])
                new_vx.append(mf_vx[v])
                new_vy.append(mf_vy[v])
                new_vz.append(mf_vz[v])
                new_rx.append(px_r[v])
                new_ry.append(py_r[v])
                new_rz.append(pz_r[v])
                new_vrx.append(vx_r[v])
                new_vry.append(vy_r[v])
                new_vrz.append(vz_r[v])
                new_traj_time.append(traj_time[v])
                
        traj_time = np.array(new_traj_time)
        mf_px = new_px
        mf_py = new_py
        mf_pz = new_pz
        mf_vx = new_vx
        mf_vy = new_vy
        mf_vz = new_vz
        px_r = new_rx
        py_r = new_ry
        pz_r = new_rz
        vx_r = new_vrx
        vy_r = new_vry
        vz_r = new_vrz   
        ##

        yawrate_ls = []
        xyz_error_norm = []
        velocity_xyz_error_norm = []      

        for i in range(len(mf_px)):
            x_error_squared = (px_r[i] - mf_px[i]) ** 2
            y_error_squared = (py_r[i] - mf_py[i]) ** 2    
            z_error_squared = (pz_r[i] - mf_pz[i]) ** 2
            xyz_error_norm.append(math.sqrt(x_error_squared + y_error_squared + z_error_squared))
            #yawrate_ls.append(yawrate[i])

            vx_error_squared = (vx_r[i] - mf_vx[i]) ** 2
            vy_error_squared = (vy_r[i] - mf_vy[i]) ** 2    
            vz_error_squared = (vz_r[i] - mf_vz[i]) ** 2
            velocity_xyz_error_norm.append(math.sqrt(vx_error_squared + vy_error_squared + vz_error_squared))

        return (
            traj_time,
            mf_px,
            mf_py,
            mf_pz,
            px_r,
            py_r,
            pz_r,
            xyz_error_norm,
            yawrate_ls,
            motor_cmd,
            mf_vx,
            mf_vy,
            mf_vz,
            vx_r,
            vy_r,
            vz_r,
            velocity_xyz_error_norm
        )


    def data_whisker_plot(self,file_path,type):
        data = self.normal_data_compiler(file_path,type)
        mf_px = data[1]
        mf_py = data[2]
        mf_pz = data[3]
        px_r = data[4]
        py_r = data[5]
        pz_r = data[6]
        mf_vx = data[10]
        mf_vy = data[11]
        mf_vz = data[12]
        vx_r = data[13]
        vy_r = data[14]
        vz_r = data[15]
        yawrate_ls = data[8]
        motor_cmd = data[9]
        traj_time = data[0]

        x_error = [] 
        y_error = []
        z_error = []
        x_error_squared = [] 
        y_error_squared = []
        z_error_squared = []
        x_error_norm = []
        y_error_norm = []
        z_error_norm = []
        
        vx_error = [] 
        vy_error = []
        vz_error = []
        vx_error_squared = [] 
        vy_error_squared = []
        vz_error_squared = []
        vx_error_norm = []
        vy_error_norm = []
        vz_error_norm = []

        for i in range(len(mf_px)):
            x_error_squared.append((px_r[i] - mf_px[i]) ** 2)
            y_error_squared.append((py_r[i] - mf_py[i]) ** 2)    
            z_error_squared.append((pz_r[i] - mf_pz[i]) ** 2)
            x_error_norm.append(math.sqrt(x_error_squared[i]))
            y_error_norm.append(math.sqrt(y_error_squared[i]))
            z_error_norm.append(math.sqrt(z_error_squared[i]))
            x_error.append((px_r[i] - mf_px[i]))
            y_error.append((py_r[i] - mf_py[i]))
            z_error.append((pz_r[i] - mf_pz[i]))

            vx_error_squared.append((vx_r[i] - mf_vx[i]) ** 2)
            vy_error_squared.append((vy_r[i] - mf_vy[i]) ** 2)    
            vz_error_squared.append((vz_r[i] - mf_vz[i]) ** 2)
            vx_error_norm.append(math.sqrt(vx_error_squared[i]))
            vy_error_norm.append(math.sqrt(vy_error_squared[i]))
            vz_error_norm.append(math.sqrt(vz_error_squared[i]))
            vx_error.append((vx_r[i] - mf_vx[i]))
            vy_error.append((vy_r[i] - mf_vy[i]))
            vz_error.append((vz_r[i] - mf_vz[i]))

        final_rmse_x = math.sqrt(sum(x_error_squared)/len(mf_px))
        final_rmse_y = math.sqrt(sum(y_error_squared)/len(mf_px))
        final_rmse_z = math.sqrt(sum(z_error_squared)/len(mf_px))
        rmse_xyz_list = [final_rmse_x,final_rmse_y,final_rmse_z]
        final_rmse_xyz = la.norm(rmse_xyz_list, 2)

        final_rmse_vx = math.sqrt(sum(vx_error_squared)/len(mf_px))
        final_rmse_vy = math.sqrt(sum(vy_error_squared)/len(mf_px))
        final_rmse_vz = math.sqrt(sum(vz_error_squared)/len(mf_px))
        vel_rmse_xyz_list = [final_rmse_vx,final_rmse_vy,final_rmse_vz]
        vel_final_rmse_xyz = la.norm(vel_rmse_xyz_list, 2)

        return (
            x_error,
            y_error,
            z_error,
            x_error_squared,
            y_error_squared,
            z_error_squared,
            x_error_norm, # 6
            y_error_norm, # 7
            z_error_norm, # 8
            rmse_xyz_list,
            final_rmse_xyz,
            traj_time,
            yawrate_ls,
            motor_cmd,
            vx_error,
            vy_error,
            vz_error,
            vx_error_squared,
            vy_error_squared,
            vz_error_squared,
            vx_error_norm, # 20
            vy_error_norm, # 21
            vz_error_norm, # 22
            vel_rmse_xyz_list,
            vel_final_rmse_xyz
        )

            




