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
print((cmd[100][0]))

cmds = [row for row in cmd] # column vector

cmd_x = [row[0][0] for row in cmds] # column vector
cmd_y = [row[0][1] for row in cmds]

cmd_x = np.array([cmd_x])

print(np.size(time))
print(np.size(cmd_x))

plt.figure(figsize=(10, 5))

plt.plot(time[0], cmd_x[0], label='measurements', color='orange')
#plt.plot(time[0]*10, pz_des, label='reference', linestyle='dashed')
#plt.plot(time[0], altitude_error, label='pz_des', linestyle='dashed')

plt.title('Control signal x vs time', fontsize=20)
plt.legend()
#plt.xlim(5, 40)
#plt.ylim(-1.0, 2.0)
plt.xlabel('time(s)')
plt.ylabel('cmd x')

# Show the figure
plt.show()

