import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.axes3d import get_test_data


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'
foam_wing = 'ultralight_traj_data'

att = 0

wing_short_circle = data_sorter.plot_fan_circle(short_wing,att)
wing_long_circle = data_sorter.plot_fan_circle(long_wing,att)
wing_foam_circle = data_sorter.plot_foam_circle(foam_wing,att)


wing = [wing_short_circle,wing_long_circle,wing_foam_circle]
rows = 2
## generate the plot
fig = plt.figure(figsize=(20, 12))
graphs = [[None]*3 for _ in range(rows)]

# Create 2x3 3D subplots manually
for i in range(rows):
    for j in range(3):
        #ax = fig.add_subplot(1, 3, i * 3 + j + 1, projection='3d')
        ax = fig.add_subplot(2, 3, i * 3 + j + 1, projection='3d')
        graphs[i][j] = ax

colors = ['#254abe','#96400b','#254abe','#96400b']
fig.subplots_adjust(hspace=0.272, wspace=0.037, 
                    left=0.03, right=1, 
                    top =0.955, bottom =0.059)
#fig.suptitle(title, fontsize=40, fontweight='bold')

for i in range(rows):
    for a in range(len(graphs[1])): # 3D plot for trajs
        placement = 0.0 # default position in each graph plot
        if i == 0:
            if a == 0:
                traj_label = 'Short '
            elif a == 1:
                traj_label = 'Long '
            elif a == 2:
                traj_label = 'Ultralight '   
        else:
            if a == 0:
                traj_label = 'Short-Fan '
            elif a == 1:
                traj_label = 'Long-Fan '
            elif a == 2:
                traj_label = 'Ultralight-Payload ' 

        for m in range(rows): # 1.0 NMPC+INDI and DFBC+INDI
            num = i*2
            m = m + num
            if m == num:
                method_label = 'NMPC+INDI'
                style = 'solid'
            elif m == num+1:
                method_label = 'DFBC+INDI'
                style = 'solid'

            if m == num:
                #print ('num',num)
                rx = wing[a][m][4] 
                ry = wing[a][m][5] 
                rz = wing[a][m][6] 
                graphs[i][a].plot3D(rx,ry,rz,label='Ref',color='red',linewidth=7.0,linestyle='solid') 
        
            x = wing[a][m][1] 
            y = wing[a][m][2] 
            z = wing[a][m][3]
            graphs[i][a].plot3D(x,y,z,label=method_label,linewidth=7.0,color=colors[m-num],linestyle=style) #label=['NMPC']
                    
        graphs[i][a].set_zlim(0,2)
        graphs[i][a].set_xlabel('Xw [m]', fontsize=20)
        graphs[i][a].set_ylabel('Yw [m]', fontsize=20)
        graphs[i][a].set_zlabel('Zw [m]', fontsize=20)
        # legend        
        graphs[i][a].legend(loc='upper right', fontsize=15) # font used to be 23
        # add title
        graphs[i][a].set_title(traj_label, fontsize=25, fontweight='bold') # font used to be 35
        graphs[i][a].tick_params(axis='both', labelsize=20)
        graphs[i][a].tick_params(axis='x', pad=40)
        graphs[i][a].tick_params(axis='y', pad=40)
        graphs[i][a].tick_params(axis='z', pad=40)   

        if i == 1 and a < 2: # only for the second row of graphs for short and long
            # fan parameters
            x0, y0, z0 = -2.0, 1.5, 0.0  # fan center
            r = 0.5                      # radius of the plate
            alpha = np.deg2rad(20)       # tilt angle about the x-axis

            # 1) generate the flat circle in local coords (centered at origin)
            θ = np.linspace(0, 2*np.pi, 100)
            x_local = r * np.cos(θ)
            y_local = r * np.sin(θ)
            z_local = np.zeros_like(θ)

            # 2) build the rotation matrix about the x-axis
            R_x = np.array([
                [1,              0,               0],
                [0,  np.cos(alpha), -np.sin(alpha)],
                [0,  np.sin(alpha),  np.cos(alpha)]
            ])

            R_y = np.array([
                [ np.cos(alpha),  0, np.sin(alpha)],
                [             0,  1,             0],
                [-np.sin(alpha),  0, np.cos(alpha)]
            ])

            R = R_y @ R_x

            # 3) apply rotation + translate back to (x0,y0,z0)
            pts = np.vstack((x_local, y_local, z_local))      # shape (3,100)
            rotated = R @ pts                               # still (3,100)
            x_c = x0 + rotated[0, :]
            y_c = y0 + rotated[1, :]
            z_c = z0 + rotated[2, :]

            # 4) plot the tilted circle
            graphs[i][a].plot3D(x_c, y_c, z_c, color='dimgrey', linewidth=2)

            # 5) blades: you can either leave them tilted as before
            blade_angles = np.linspace(0, 2*np.pi, 5, endpoint=False)
            for φ in blade_angles:
                # original blade tip in local coords
                tip_local = np.array([
                    r * np.cos(φ),
                    r * np.sin(φ),
                    0
                ])
                # rotate that tip
                tip_rot = R @ tip_local
                x_end, y_end, z_end = x0 + tip_rot[0], y0 + tip_rot[1], z0 + tip_rot[2]
                # plot line from center to tip
                graphs[i][a].plot3D(
                    [x0, x_end],
                    [y0, y_end],
                    [z0, z_end],
                    color='black',
                    linewidth=2
                )

                # 3) label “fan” beside it
                # place the text just outside the rim in the blade-0 direction
                φ0 = blade_angles[0]
                offset = 0.5
                # compute a point a bit beyond the plate edge
                label_point_local = np.array([(r + offset)*np.cos(φ0),
                                            (r + offset)*np.sin(φ0),
                                            0.0])
                label_rot = R @ label_point_local
                x_lbl = x0 + label_rot[0]
                y_lbl = y0 + label_rot[1]
                z_lbl = z0 + label_rot[2]
                graphs[i][a].text(x_lbl, y_lbl, z_lbl, 'FAN', fontsize=23, color='black')


#plt.savefig(title+'.pdf')   
plt.savefig('Fan_3D.png', dpi=300, bbox_inches='tight')  
plt.show()


# data_saver = DataSave.SaveData('Data_time',
#                                   'Monocopter_XYZ','rotational_state_vector','motor_cmd','ref_position','ref_velocity','motor_actual_cmd','cmd_bod_acc','yawrate') 
      