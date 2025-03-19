import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage
from numpy import linalg as la

file_path = '/home/emmanuel/Monocopter-OCP/sim_data/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


#print(mat_data)
time = mat_data['Data_time']
#position = mat_data['data']
cmd = mat_data['cmd']
tpp = mat_data['tpp_angle']
print(tpp[1])

cmds = [row for row in cmd] # column vector

cmd_x = [row[0][0] for row in cmds] # column vector
cmd_y = [row[0][1] for row in cmds]

tpp_roll = [row[0] for row in tpp] # column vector
tpp_pitch = [row[0] for row in tpp]

cmd_x = np.array([cmd_x])
cmd_y = np.array([cmd_y])

tpp_roll = np.array([tpp_roll])
tpp_pitch = np.array([tpp_pitch])

print(type(time))
print(type(cmd_x))

#plt.figure(figsize=(10, 5))
fig, ((ax1), (ax2)) = plt.subplots(2, 1, figsize=(40, 10))
#print(cmd_x[0])

ax1.plot(time[0], cmd_x[0], label='cmd_x', color='blue',linewidth=2)
ax1.plot(time[0], cmd_y[0], label='cmd_y', color='red',linewidth=2)
ax1.legend()
ax1.set_title('Control signal vs time', fontsize=20)
ax1.set_xlabel('Time(s)')
ax1.set_ylabel('Cmd')

plt.subplots_adjust(hspace=0.34, wspace=0.7)

ax2.plot(time[0], np.round((tpp_roll[0]*(180/np.pi)),3), label='tpp_roll', color='blue')
ax2.plot(time[0], np.round((tpp_pitch[0]*(180/np.pi)),3), label='tpp_pitch', color='red')
ax2.legend()
ax2.set_title('TPP angles(deg) vs time', fontsize=20)
ax2.set_xlabel('Time(s)')
ax2.set_ylabel('Angles(deg)')


# Show the figure
plt.show()

