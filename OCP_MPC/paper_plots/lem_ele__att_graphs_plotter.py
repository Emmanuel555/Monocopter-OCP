import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import gridspec


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'

type = 'ele'

att = 1

wing_short_lem = data_sorter.plot_lem(short_wing,att)
wing_long_lem = data_sorter.plot_lem(long_wing,att)
wing_short_ele = data_sorter.plot_elevated_circle(short_wing,att)
wing_long_ele = data_sorter.plot_elevated_circle(long_wing,att)

if type == 'lem':
    title = 'Lemniscate_att.png'
    wing = [wing_short_lem,wing_long_lem]
elif type == 'ele':
    title = 'Elevated_circle_att.png'
    wing = [wing_short_ele,wing_long_ele]    


rows = 3
cols = 2
## generate the plot
fig = plt.figure(figsize=(40, 20))

gs = gridspec.GridSpec(rows, cols,
        width_ratios=[1.5, 1],
        height_ratios=[1, 1, 1],
        figure=fig)


graphs = [[None]*cols for _ in range(rows)]

# Create 2x3 3D subplots manually
for i in range(rows):
    for j in range(cols):
        ax = fig.add_subplot(gs[i, j])
        graphs[i][j] = ax

colors = ['#254abe','#96400b','#254abe','#96400b']
fig.subplots_adjust(hspace=0.248, wspace=0.078, 
                    left=0.03, right=0.98, 
                    top =0.9, bottom =0.075)
#fig.suptitle(title, fontsize=40, fontweight='bold')


for i in range(rows):
    for a in range(len(graphs[1])): # 2D plot for trajs
        placement = 0.0 # default position in each graph plot
        if i == 0:
            att_label = '∥˙ωD∥ (rad/s^2)' 
            if a == 0:
                traj_label = 'Short-'
            elif a == 1:
                traj_label = 'Long-'   
        elif i == 1:
            att_label = 'Body yaw rotational rate (Hz)'
            if a == 0:
                traj_label = 'Short-'
            elif a == 1:
                traj_label = 'Long-'
        elif i == 2:
            att_label = 'Motor throttle (%)'
            if a == 0:
                traj_label = 'Short-'
            elif a == 1:
                traj_label = 'Long-'      
 

        if i == 1:
            for m in range((rows-1)*2): # 1.0 NMPC+INDI and DFBC+INDI
                if m == 0:
                    method_label = 'NMPC+INDI'
                    style = 'solid'
                elif m == 1:
                    method_label = 'DFBC+INDI'
                    style = 'solid'
                elif m == 2:
                    method_label = '(FAN) NMPC+INDI'
                    style = 'dashed'
                elif m == 3:
                    method_label = '(FAN) DFBC+INDI'
                    style = 'dashed'
                

                start = wing[a][m][0][0]
                traj_time = wing[a][m][0] - start 
                yawrate = wing[a][m][9] 
                mean = statistics.mean(yawrate)
                mean = np.round(mean,1)
                graphs[i][a].set_xlabel('Time(s)',fontsize=20) 
                #graphs[i][a].plot(traj_time,yawrate,label=method_label+"(Mean:{})".format(mean),linewidth=7.0,color=colors[m],linestyle=style) #label=['INDI']
                graphs[i][a].plot(traj_time,yawrate,label=method_label,linewidth=7.0,color=colors[m],linestyle=style) 
                 
        elif i == 0:
            for m in range((rows-1)*2): # 1.0 NMPC+INDI and DFBC+INDI
                if m == 0:
                    method_label = 'NMPC+INDI'
                    style = 'solid'
                elif m == 1:
                    method_label = 'DFBC+INDI'
                    style = 'solid'
                elif m == 2:
                    method_label = '(FAN) NMPC+INDI'
                    style = 'dashed'
                elif m == 3:
                    method_label = '(FAN) DFBC+INDI'
                    style = 'dashed'
                
                start = wing[a][m][0][0]
                traj_time = wing[a][m][0] - start  
                att_raterate = wing[a][m][8] 
                mean = statistics.mean(att_raterate)
                mean = np.round(mean,2)
                graphs[i][a].set_xlabel('Time(s)',fontsize=20) 
                #graphs[i][a].plot(traj_time,att_track,label=method_label+"(Mean:{}, RMS:{})".format(mean, rms),linewidth=7.0,color=colors[m],linestyle=style) #label=['INDI']
                graphs[i][a].plot(traj_time,att_raterate,label=method_label,linewidth=7.0,color=colors[m],linestyle=style) 
                 
        else:
            for m in range((rows-1)*2): # 1.0 NMPC+INDI and DFBC+INDI
                if m == 0:
                    method_label = 'NMPC+INDI'
                    style = 'solid'
                elif m == 1:
                    method_label = 'DFBC+INDI'
                    style = 'solid'
                elif m == 2:
                    method_label = '(FAN) NMPC+INDI'
                    style = 'dashed'
                elif m == 3:
                    method_label = '(FAN) DFBC+INDI'
                    style = 'dashed'

                start = wing[a][m][0][0]
                traj_time = wing[a][m][0] - start  
                motor_cmd = ((wing[a][m][17])/65500)*100 
                mean = statistics.mean(motor_cmd)
                mean = np.round(mean,2)
                graphs[i][a].set_xlabel('Time(s)',fontsize=20) 
                #graphs[i][a].plot(traj_time,att_track,label=method_label+"(Mean:{}, RMS:{})".format(mean, rms),linewidth=7.0,color=colors[m],linestyle=style) #label=['INDI']
                graphs[i][a].plot(traj_time,motor_cmd,label=method_label,linewidth=7.0,color=colors[m],linestyle=style) 
                 

        # set limits
        if type == 'lem':
            graphs[i][a].set_xlim(0,28)
        elif type == 'ele':
            graphs[i][a].set_xlim(0,15)
        
        if i == 2:
            if a == 1:
                graphs[i][a].set_ylim(0,90)

        # legend        
        graphs[i][a].legend(loc='upper right', fontsize=20)
        # axes
        graphs[i][a].tick_params(axis='both', labelsize=20)
        # add title
        graphs[i][a].set_title(traj_label+att_label, fontsize=25, fontweight='bold')
        graphs[i][a].grid(linewidth=4.0)



#plt.savefig(title+'.pdf')   
plt.savefig(title, dpi=300, bbox_inches='tight')  
plt.show()
