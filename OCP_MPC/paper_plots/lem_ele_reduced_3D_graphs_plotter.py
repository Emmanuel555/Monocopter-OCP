import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.axes3d import get_test_data
import matplotlib.gridspec as gridspec


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'

att = 0

wing_short_lem = data_sorter.plot_lem(short_wing,att)
wing_long_lem = data_sorter.plot_lem(long_wing,att)
wing_short_ele = data_sorter.plot_elevated_circle(short_wing,att)
wing_long_ele = data_sorter.plot_elevated_circle(long_wing,att)

wing = [wing_short_ele,wing_short_lem,wing_long_ele,wing_long_lem]
rows = 2
## generate the plot
fig = plt.figure(figsize=(24, 12))
gs  = gridspec.GridSpec(2, 2,
       width_ratios=[1.5, 1],   # left col is 1.5× the width of right
       height_ratios=[1, 1],
       figure=fig)
graphs = [[None]*3 for _ in range(rows)]

# Create 2x3 3D subplots manually
for i in range(rows):
    for j in range(2):
        #ax = fig.add_subplot(2, 2, i * 2 + j + 1, projection='3d')
        ax = fig.add_subplot(gs[i, j], projection="3d")
        graphs[i][j] = ax

colors = ['#254abe','#96400b','#254abe','#96400b']
fig.subplots_adjust(hspace=0.28, wspace=0.0, 
                    left=0.0, right=0.63, 
                    top =0.955, bottom =0.059)
#fig.suptitle(title, fontsize=40, fontweight='bold')

for i in range(rows):
    for a in range(rows): # 3D plot for trajs
        placement = 0.0 # default position in each graph plot
        if i == 0:
            if a == 0:
                traj_label = 'Short-Elevated Circle '
            elif a == 1:
                traj_label = 'Short-Lemniscate '
             
        else:
            if a == 0:
                traj_label = 'Long-Elevated Circle '
            elif a == 1:
                traj_label = 'Long-Lemniscate '

        num = i*2
        m = a + num

        for n in range(rows):                
            if n == 0:
                method_label = 'NMPC+INDI'
                style = 'solid'
            elif n == 1:
                method_label = 'DFBC+INDI'
                style = 'solid'

            if n == 0:
                #print ('num',num)
                rx = wing[m][n][4] 
                ry = wing[m][n][5] 
                rz = wing[m][n][6] 
                graphs[i][a].plot3D(rx,ry,rz,label='Ref',color='red',linewidth=7.0,linestyle='solid') 
        
            x = wing[m][n][1] 
            y = wing[m][n][2] 
            z = wing[m][n][3]
            graphs[i][a].plot3D(x,y,z,label=method_label,linewidth=7.0,color=colors[n],linestyle=style) #label=['NMPC']
                    
        graphs[i][a].set_zlim(0,2)
        graphs[i][a].set_xlabel('Xw [m]', fontsize=20)
        graphs[i][a].set_ylabel('Yw [m]', fontsize=20)
        graphs[i][a].set_zlabel('Zw [m]', fontsize=20)
        # legend        
        graphs[i][a].legend(loc='upper right', fontsize=20) # font used to be 23
        # add title
        graphs[i][a].set_title(traj_label, fontsize=25, fontweight='bold') # font used to be 35
        graphs[i][a].tick_params(axis='both', labelsize=20)
        graphs[i][a].tick_params(axis='x', pad=40)
        graphs[i][a].tick_params(axis='y', pad=40)
        graphs[i][a].tick_params(axis='z', pad=40)   

        
#plt.savefig(title+'.pdf')   
plt.savefig('lem_ele.png', dpi=300, bbox_inches='tight')  
plt.show()


# data_saver = DataSave.SaveData('Data_time',
#                                   'Monocopter_XYZ','rotational_state_vector','motor_cmd','ref_position','ref_velocity','motor_actual_cmd','cmd_bod_acc','yawrate') 
      