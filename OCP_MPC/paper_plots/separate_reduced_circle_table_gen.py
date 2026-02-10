import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics
import json


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'
foam_wing = 'ultralight_traj_data'

display_mean = False

#condition = '(Nominal) '
condition = '(Perturbed) '

#order = '-Position Error'
order = '-Velocity Error' 

title = 'Tracking performances along a Circle (○) '
wing_foam_circle = data_sorter.foam_whisker_circle(foam_wing,foam_wing)
wing_short_circle = data_sorter.fan_whisker_circle(short_wing,short_wing)
wing_long_circle = data_sorter.fan_whisker_circle(long_wing,long_wing)
wing = [wing_short_circle,wing_long_circle,wing_foam_circle]


fig, ((ax1,ax3,ax5),(ax2,ax4,ax6)) = plt.subplots(2, 3, figsize=(70, 50))
graphs = [[ax1,ax2],[ax3,ax4],[ax5,ax6]]
#colors = ['#254abe','#96400b','#a725be','#254abe','#96400b','#a725be']
colors = ['#254abe','#96400b']
#med_colors = ['#34be25','#34be25','red','#34be25','#34be25','red']
med_colors = ['#34be25','red']
aggregate = np.zeros((2, 3, 3))
aggregate = np.array(aggregate, dtype=list) # to store mean and stdev
fig.subplots_adjust(hspace=0.3, wspace=0.25, 
                    left=0.048, right=0.97, 
                    top = 0.87, bottom = 0.042)


if order == '-Position Error':
    fig.suptitle(condition + title + ' (L2 position norm error(m))', fontsize=118, fontweight='bold')
else:
    fig.suptitle(condition + title + ' (L2 velocity norm error(m/s))', fontsize=118, fontweight='bold')


for a in range(len(graphs)): # 3 monocos
    placement = 0.0 # default position in each graph plot
    if a == 0:
        drone_label = 'Short-'
    elif a == 1:
        drone_label = 'Long-'        
    elif a == 2:
        drone_label = 'Ultralight-' 
        

    for m in range(len(graphs[a])): # 1.0 m/s circle w and w/o payload, NMPC+INDI and DFBC+INDI 
        if condition == '(Nominal) ':
            if m == 0:
                if a > 1:
                    method_label = '0.4m/s \n(NMPC+INDI)'
                else:
                    method_label = '1.0m/s \n(NMPC+INDI)'
            elif m == 1:
                if a > 1:
                    method_label = '0.4m/s \n(DFBC+INDI)'
                else:
                    method_label = '1.0m/s \n(DFBC+INDI)'
        elif condition == '(Perturbed) ':
            if m == 0:    
                if a > 1:
                    method_label = '0.4m/s-Payload \n(NMPC+INDI)'
                else:
                    method_label = '1.0m/s-Fan \n(NMPC+INDI)'      
            elif m == 1:
                if a > 1:
                    method_label = '0.4m/s-Payload \n(DFBC+INDI)'
                else:
                    method_label = '1.0m/s-Fan \n(DFBC+INDI)' 
        

    
        if order == '-Position Error':

            if condition == '(Nominal) ':
                f = 0
            else:
                f = 2

            for i in range(3): #xyz
                if i == 0:
                    label = '∥Px∥(m)'
                elif i == 1:
                    label = '∥Py∥(m)'
                elif i == 2:
                    label = '∥Pz∥(m)'
                method = graphs[a][m].boxplot(wing[a][m+f][6+i],
                                    #patch_artist = True,
                                    boxprops={'color':colors[m]},
                                    capprops={'color':'orange'},
                                    medianprops={'color':med_colors[m]},
                                    whiskerprops={'color':'orange'},
                                    showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                # rmse
                graphs[a][m].plot(placement+i*0.05, wing[a][m+f][9][0+i], 'X', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 70, label='RMS') # marker type
                # mean
                graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m+f][6+i]), 'o', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 70, label='Mean') # marker type
                
                method_mean = statistics.mean(wing[a][m+f][6+i])
                method_stdev = statistics.stdev(wing[a][m+f][6+i])
                rmse = wing[a][m+f][9][0+i]

                #aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
            
                # display mean and stdev
                if display_mean == True:
                    # Add the text label above the median line
                    graphs[a][m].text(0+i, 1,  # x, y coordinates for the text
                    f'{statistics.mean(wing[a][m+f][6+i]):.2f},{statistics.stdev(wing[a][m+f][6+i]):.2f}',  # The text to display (formatted to 2 decimal places)
                    ha='center',  # Horizontal alignment
                    va='bottom',  # Vertical alignment
                    fontsize=15,  # Adjust fontsize as needed
                    color='black')  # Adjust color as needed

                # rotate x axis labels
                graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=100)
                graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=100)
                graphs[a][m].set_xlim(-0.02, 0.12)
                # add title
                graphs[a][m].set_title(drone_label + method_label + order, fontsize=100, fontweight='bold')
                graphs[a][m].grid(linewidth=7.0)
                # add legend
                if i == 0:
                    graphs[a][m].legend(loc='upper right', fontsize=70)

                for whisker in method['whiskers']:
                    whisker.set(linewidth=35.0) # Set the thickness 

                for box in method['boxes']:
                    box.set(linewidth=35.0) # Set the thickness 

                for cap in method['caps']:
                    cap.set(linewidth=35.0) # Set the thickness
                    y = cap.get_ydata()[0]

                for median in method['medians']:
                    median.set(linewidth=35.0) # Set the thickness


        elif order == '-Velocity Error':

            if condition == '(Nominal) ':
                f = 0
            else:
                f = 2

            for i in range(3): #xyz
                if i == 0:
                    label = '∥Vx∥(m/s)'
                elif i == 1:
                    label = '∥Vy∥(m/s)'
                elif i == 2:
                    label = '∥Vz∥(m/s)'
                method = graphs[a][m].boxplot(wing[a][m+f][20+i],
                                    #patch_artist = True,
                                    boxprops={'color':colors[m]},
                                    capprops={'color':'orange'},
                                    medianprops={'color':med_colors[m]},
                                    whiskerprops={'color':'orange'},
                                    showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                # rmse
                graphs[a][m].plot(placement+i*0.05, wing[a][m+f][23][0+i], 'X', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 70, label='RMS') # marker type
                # mean
                graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m+f][20+i]), 'o', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 70, label='Mean') # marker type
                
                method_mean = statistics.mean(wing[a][m+f][20+i])
                method_stdev = statistics.stdev(wing[a][m+f][20+i])
                rmse = wing[a][m+f][23][0+i]

                #aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
            
                # display mean and stdev
                if display_mean == True:
                    # Add the text label above the median line
                    graphs[a][m].text(0+i, 1,  # x, y coordinates for the text
                    f'{statistics.mean(wing[a][m+f][20+i]):.2f},{statistics.stdev(wing[a][m+f][20+i]):.2f}',  # The text to display (formatted to 2 decimal places)
                    ha='center',  # Horizontal alignment
                    va='bottom',  # Vertical alignment
                    fontsize=15,  # Adjust fontsize as needed
                    color='black')  # Adjust color as needed

                # rotate x axis labels
                graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=100)
                graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=100)
                graphs[a][m].set_xlim(-0.02, 0.12)
                # add title
                graphs[a][m].set_title(drone_label + method_label + order, fontsize=100, fontweight='bold')
                graphs[a][m].grid(linewidth=7.0)
                # add legend
                if i == 0:
                    graphs[a][m].legend(loc='upper right', fontsize=70)

                for whisker in method['whiskers']:
                    whisker.set(linewidth=35.0) # Set the thickness 

                for box in method['boxes']:
                    box.set(linewidth=35.0) # Set the thickness 

                for cap in method['caps']:
                    cap.set(linewidth=35.0) # Set the thickness
                    y = cap.get_ydata()[0]

                for median in method['medians']:
                    median.set(linewidth=35.0) # Set the thickness
    

""" # Iterate through each median line and its corresponding data
for i, line in enumerate(indi['medians']):
    # Get the median value
    median_value = line.get_ydata()[0]  # Both y-data points are the same on a median line

    # Get the x position of the box (boxes are evenly spaced)
    x_position = 0

    # Add the text label above the median line
    ax1.text(x_position, median_value,  # x, y coordinates for the text
            f'{median_value:.2f}',  # The text to display (formatted to 2 decimal places)
            ha='center',  # Horizontal alignment
            va='bottom',  # Vertical alignment
            fontsize=10,  # Adjust fontsize as needed
            color='black')  # Adjust color as needed
 """


# save figure
plt.savefig(condition+title+order+'.pdf', dpi=300, bbox_inches='tight')  
#plt.show()
