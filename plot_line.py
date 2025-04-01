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

x = [0.5,0.0,0.0,0.0,0.0]
y = [1.0,0.5,0.0,-0.5,-1.0]
parts = len(x)
distance = abs(y[0])*2
speed = 0.5
total_time = distance/speed 

refs = [
    ms.Waypoint(
        time=(total_time/(parts-1))*0,
        position=np.array([x[0], y[0], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*1,
        position=np.array([x[1], y[1], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*2,
        position=np.array([x[2], y[2], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*3,
        position=np.array([x[3], y[3], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*4,
        position=np.array([x[4], y[4], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*5,
        position=np.array([x[5], y[5], 1.0]),
    ),
   ms.Waypoint(
        time=(total_time/(parts-1))*6,
        position=np.array([x[6], y[6], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*7,
        position=np.array([x[7], y[7], 1.0]),
    ),
    ms.Waypoint(
        time=(total_time/(parts-1))*8,
        position=np.array([x[8], y[8], 1.0]),
    ),
]


polys = ms.generate_trajectory(
        refs,
        degree=8,  # Polynomial degree
        idx_minimized_orders=(3, 4),  
        num_continuous_orders=3,  
        algorithm="closed-form",  # Or "constrained"
    )


t = np.linspace(0, total_time, num_points)
# Sample up to the 3rd order (Jerk) -----v
pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6


# available order in derivatives in pva is one lesser than computed meaning up to one derivative less  


print(np.shape(pva)) # up to order of derivatives available, waypoints, axis
pos_x = pva[0,:,0] # position
vel_x = pva[1,:,0] # velocity
acc_x = pva[2,:,0] # accleleration
jerk_x = pva[3,:,0] # jerk
snap_x = pva[4,:,0] # snap

pos_y = pva[0,:,1] # position
vel_y = pva[1,:,1] # velocity
acc_y = pva[2,:,1] # accleleration
jerk_y = pva[3,:,1] # jerk
snap_y = pva[4,:,1] # snap


traj_gen = trajectory_generator.trajectory_generator()
#derivatives = traj_gen.jerk_snap_9pt_circle(0, 0.4, 0, 1)
pva,num_pts = traj_gen.compute_jerk_snap_9pt_circle_x_laps(1.16,-0.29,0.9,1,5,1)

#derivatives = traj_gen.jerk_snap_circle(0.5,0,0,1) .....
#pos = derivatives[0]

#print (len(derivatives))
#print(np.shape(pos))
fig, ax = plt.subplots(1)
ax.plot(pva[0,0:2000,0], pva[0,0:2000,1])
ax.set_aspect(1)
plt.show()

test = traj_gen.elevated_circle(0, 0.6, 0, 1)




    
        

#stop = timeit.default_timer()
#print('Program Runtime: ', stop - start)  