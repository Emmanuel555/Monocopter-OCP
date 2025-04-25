import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'
foam_wing = 'foam_traj_data'


# test short wing first 11 12 13
wing_circle_long = data_sorter.whisker_circle(long_wing)
wing_circle_short = data_sorter.whisker_circle(short_wing)
wing_circle_foam = data_sorter.foam_whisker_circle(foam_wing)

wing = [wing_circle_long,wing_circle_short,wing_circle_foam]

## generate the plot
fig, ((ax2,ax3)) = plt.subplots(1, 2, figsize=(20, 8))
graphs = [ax2,ax3]
colors = ['#254abe','#96400b','#a725be']
fig.subplots_adjust(hspace=0.3, wspace=0.2, 
                    left=0.048, right=0.97, 
                    top = 0.89, bottom = 0.042)
fig.suptitle('Long wing motor commands for circle trajectory', fontsize=40, fontweight='bold')


for a in range(len(wing)): # 3 trajs
    placement = 0.0 # default position in each graph plot
    if a == 0:
        wing_label = 'Long '
        method_label = 'INDI '
    elif a == 1:
        wing_label = 'Short '
        method_label = 'NDI '
    elif a == 2:
        wing_label = 'Foam ' 
        method_label = 'ATT '

    # Only for Circle 0.3m/s

    # yawrate
    traj_time_yawrate = wing[a][0][11]
    yawrate = wing[a][0][12]

    # motor
    traj_time_motor_3 = wing[0][a][11]
    motor_cmd_3 = wing[0][a][13] # only for long wing
    traj_time_motor_5 = wing[0][a+3][11]
    motor_cmd_5 = wing[0][a+3][13] # only for long wing   
    
    ax2.plot(traj_time_motor_3, motor_cmd_3, color=colors[a], linestyle='solid', linewidth=3.0, label=method_label+'(0.3m/s)')                         
    ax3.plot(traj_time_motor_5, motor_cmd_5, color=colors[a], linestyle='solid', linewidth=3.0, label=method_label+'(0.5m/s)') 
  
ax2.set_xlabel('Time(s)', fontsize=25)
ax2.set_ylabel('pwm', fontsize=25)
ax3.set_xlabel('Time(s)', fontsize=25)
ax3.set_ylabel('pwm', fontsize=25)
ax2.tick_params(axis='both', labelsize=25)
ax3.tick_params(axis='both', labelsize=25)
ax2.legend(loc='upper right', fontsize=25)
ax3.legend(loc='upper right', fontsize=25)
#ax2.set_title('Long wing motor commands @ cir 0.3m/s', fontsize=30, fontweight='bold')
#ax3.set_title('Long wing motor commands @ cir 0.5m/s', fontsize=30, fontweight='bold')
ax2.grid(linewidth=2.0)
ax3.grid(linewidth=2.0)    
  
plt.savefig('motor_cmd.png', dpi=300, bbox_inches='tight')  
plt.show()
