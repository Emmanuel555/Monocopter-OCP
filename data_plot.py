import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

file_path = '/home/emmanuel/Monocopter-OCP/data_selected/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


#print(mat_data)
time = mat_data['Data_time']
#position = mat_data['data']
position = mat_data['Monocopter_XYZ']
px = [row[0] for row in position] # column vector
py = [row[1] for row in position]
pz = [row[2] for row in position]


#gains = mat_data['gains']

ref_position = mat_data['ref_position']
px_des = [row[0] for row in ref_position]
py_des = [row[1] for row in ref_position]
pz_des = [row[2] for row in ref_position]

# altitude error
altitude_error_squared = [(pz_des - pz) ** 2 for pz_des, pz in zip(pz_des, pz)]

time_flat_list = [item for sublist in time for item in sublist]

time_index_5 = min(range(len(time_flat_list)), key=lambda i: abs(time_flat_list[i] - 5))

trim_error_squared = altitude_error_squared[time_index_5:len(altitude_error_squared)]
mean_error_squared = sum(trim_error_squared)/len(trim_error_squared)
rmse = math.sqrt(mean_error_squared)
print(np.shape(px))



## plotting out

""" plt.figure(figsize=(10, 5))

rmse = round(rmse, 4)

text_label = 'RMSE (robot_1): {} (m)'.format(rmse)


plt.plot(time[0]*10, pz, label='measurements', color='orange')
plt.plot(time[0]*10, pz_des, label='reference', linestyle='dashed')
#plt.plot(time[0], altitude_error, label='pz_des', linestyle='dashed')
# plt.title('Altitude Plot')
plt.legend()
#plt.xlim(5, 40)
plt.ylim(-1.0, 2.0)
plt.xlabel('time (s)')
plt.ylabel('altitude (m)') """


plt.figure(figsize=(10, 5))

rmse = round(rmse, 4)

#text_label = 'RMSE: {} (m)'.format(rmse)

median_filtered_px = ndimage.median_filter(px, size=1000)
median_filtered_py = ndimage.median_filter(py, size=1000)

start = 5000
end = 30000

print(np.shape(median_filtered_px))
print('ref pos: ', px_des[0], py_des[0], pz_des[0])
plt.plot(median_filtered_px[start:end], median_filtered_py[start:end], label='measurements', color='orange')
#plt.plot(time[0]*10, pz_des, label='reference', linestyle='dashed')
#plt.plot(time[0], altitude_error, label='pz_des', linestyle='dashed')

plt.title('median filter: X vs Y')
plt.legend()
#plt.xlim(5, 40)
#plt.ylim(-1.0, 2.0)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')

#plt.text(15, 1.6, text_label, fontsize=20, ha='left', va='bottom', color='blue')


# Show the figure
plt.show()

