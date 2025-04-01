import numpy as np
import matplotlib.pyplot as plt
import minsnap_trajectories as ms
import trajectory_generator
from pyrr import quaternion
import timeit
import copy
from random import randrange
import matplotlib.animation as animation
from matplotlib import style
from mpl_toolkits.mplot3d import Axes3D

alt = 1.2
start = timeit.default_timer() 

x = [0.0,0.0,0.0,0.0,0.0]
y = [1.0,0.5,0.0,-0.5,-1.0]
parts = len(x)
distance = abs(y[0])*2
speed = 0.1
total_time = distance/speed
sampling_freq = 360
num_pts = int(total_time*sampling_freq)
refs = []

for i in range(parts):
    refs.append(ms.Waypoint(
        time=(total_time/(parts-1))*i,
        position=np.array([x[i], y[i], alt]), # altitude used to be 1.0
    ))


polys = ms.generate_trajectory(
        refs,
        degree=8,  # Polynomial degree
        idx_minimized_orders=(3, 4),  
        num_continuous_orders=3,  
        algorithm="closed-form",  # Or "constrained"
    )


# t = np.linspace(0, total_time, num_pts)
# ## Sample up to the 3rd order (Jerk) -----v
# pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6

# ## available order in derivatives in pva is one lesser than computed meaning up to one derivative less  
# print(np.shape(pva)) # up to order of derivatives available, waypoints, axis
# pos_x = pva[0,:,0] # position
# vel_x = pva[1,:,0] # velocity
# acc_x = pva[2,:,0] # accleleration
# jerk_x = pva[3,:,0] # jerk
# snap_x = pva[4,:,0] # snap

# pos_y = pva[0,:,1] # position
# vel_y = pva[1,:,1] # velocity
# acc_y = pva[2,:,1] # acceleration
# jerk_y = pva[3,:,1] # jerk
# snap_y = pva[4,:,1] # snap


traj_gen = trajectory_generator.trajectory_generator()
#derivatives = traj_gen.jerk_snap_9pt_circle(0, 0.4, 0, 1)
pva,num_pts = traj_gen.two_pt_line(1,36,alt)
#pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(1.16,-0.29,0.9,1,5,1)

#derivatives = traj_gen.jerk_snap_circle(0.5,0,0,1) .....
pos_x = pva[0,:,0] # position
vel_x = pva[1,:,0] # velocity
acc_x = pva[2,:,0] # accleleration
jerk_x = pva[3,:,0] # jerk
snap_x = pva[4,:,0] # snap

pos_y = pva[0,:,1] # position
vel_y = pva[1,:,1] # velocity
acc_y = pva[2,:,1] # acceleration
jerk_y = pva[3,:,1] # jerk
snap_y = pva[4,:,1] # snap

t = np.linspace(0, total_time, num_pts)

#print (len(derivatives))
#print(np.shape(pos))
fig, ax = plt.subplots(1)
ax.plot(t, pos_y, label='pos_y', color='blue')
#ax.plot(t, jerk_y, label='jerk_y', color='red')
#ax.plot(t, snap_y, label='snap_y', color='green')
ax.set_title('Derivatives gen', fontsize=20)
ax.legend()
ax.set_xlabel('Time(1/360s)')
ax.set_ylabel('Y axis input')
plt.show()





    
        

#stop = timeit.default_timer()
#print('Program Runtime: ', stop - start)  