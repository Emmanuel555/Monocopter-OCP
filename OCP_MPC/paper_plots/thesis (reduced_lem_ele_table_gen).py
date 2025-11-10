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

display_mean = False
type = 'lem'


wing_short_lem = data_sorter.fan_whisker_lem(short_wing,short_wing)
wing_long_lem = data_sorter.fan_whisker_lem(long_wing,long_wing)
wing_short_ele = data_sorter.fan_whisker_elevated_circle(short_wing,short_wing)
wing_long_ele = data_sorter.fan_whisker_elevated_circle(long_wing,long_wing)


if type == 'lem':
    title = 'Tracking performances along a Lemniscate (∞) '
    wing = [wing_short_lem,wing_short_lem,wing_long_lem,wing_long_lem]
elif type == 'ele':
    title = 'Tracking performances along an Elevated Circle (○) '
    wing = [wing_short_ele,wing_short_ele,wing_long_ele,wing_long_ele]    


fig, ((ax1,ax5,ax9,ax13),(ax2,ax6,ax10,ax14),
    (ax3,ax7,ax11,ax15),(ax4,ax8,ax12,ax16)) = plt.subplots(4, 4, figsize=(40, 60))
graphs = [[ax1,ax2,ax3,ax4],[ax5,ax6,ax7,ax8],[ax9,ax10,ax11,ax12],
        [ax13,ax14,ax15,ax16]]
#colors = ['#254abe','#96400b','#a725be','#254abe','#96400b','#a725be']
colors = ['#254abe','#96400b','#254abe','#96400b']
#med_colors = ['#34be25','#34be25','red','#34be25','#34be25','red']
med_colors = ['#34be25','red','#34be25','red']
aggregate = np.zeros((4, 4, 3))
aggregate = np.array(aggregate, dtype=list) # to store mean and stdev
fig.subplots_adjust(hspace=0.5, wspace=0.38, 
                    left=0.048, right=0.97, 
                    top = 0.92, bottom = 0.042)
fig.suptitle(title + ' (L2 norm error(m,m/s))', fontsize=73, fontweight='bold')



for a in range(len(graphs)): # 3 monocos
    placement = 0.0 # default position in each graph plot
    if a < 2:
        drone_label = 'Short-'
        if a == 0:
            order = '\nPos Error (m)'
        else: 
            order = '\nVel Error (m/s)'    
    elif a > 1 and a < 4:
        drone_label = 'Long-'
        if a == 2:
            order = '\nPos Error (m)'
        else:
            order = '\nVel Error (m/s)'       

   
    for m in range(len(graphs[a])): # 1.0 m/s circle w and w/o payload, NMPC+INDI and DFBC+INDI 
        if m == 0:
            method_label = '1.0m/s \n(NMPC+INDI)'
        elif m == 1:
            method_label = '1.0m/s \n(DFBC+INDI)'
        elif m == 2:            
            method_label = '1.0m/s-Fan \n(NMPC+INDI)'      
        elif m == 3:
            method_label = '1.0m/s-Fan \n(DFBC+INDI)' 
        

    
        if order == '\nPos Error (m)':

            for i in range(3): #xyz
                if i == 0:
                    label = '∥Px∥'
                elif i == 1:
                    label = '∥Py∥'
                elif i == 2:
                    label = '∥Pz∥'
                method = graphs[a][m].boxplot(wing[a][m][6+i],
                                    #patch_artist = True,
                                    boxprops={'color':colors[m]},
                                    capprops={'color':'orange'},
                                    medianprops={'color':med_colors[m]},
                                    whiskerprops={'color':'orange'},
                                    showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                # rmse
                graphs[a][m].plot(placement+i*0.05, wing[a][m][9][0+i], 'X', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 30, label='RMS') # marker type
                # mean
                graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][6+i]), 'o', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 30, label='Mean') # marker type
                
                method_mean = statistics.mean(wing[a][m][6+i])
                method_stdev = statistics.stdev(wing[a][m][6+i])
                rmse = wing[a][m][9][0+i]

                #aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
            
                # display mean and stdev
                if display_mean == True:
                    # Add the text label above the median line
                    graphs[a][m].text(0+i, 1,  # x, y coordinates for the text
                    f'{statistics.mean(wing[a][m][6+i]):.2f},{statistics.stdev(wing[a][m][6+i]):.2f}',  # The text to display (formatted to 2 decimal places)
                    ha='center',  # Horizontal alignment
                    va='bottom',  # Vertical alignment
                    fontsize=15,  # Adjust fontsize as needed
                    color='black')  # Adjust color as needed

                # rotate x axis labels
                graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=53)
                graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=53)
                graphs[a][m].set_xlim(-0.02, 0.12)
                # add title
                graphs[a][m].set_title(drone_label + method_label + order, fontsize=43, fontweight='bold')
                graphs[a][m].grid(linewidth=4.0)
                # add legend
                if i == 0:
                    graphs[a][m].legend(loc='upper right', fontsize=43)

                for whisker in method['whiskers']:
                    whisker.set(linewidth=7.0) # Set the thickness 

                for box in method['boxes']:
                    box.set(linewidth=7.0) # Set the thickness 

                for cap in method['caps']:
                    cap.set(linewidth=7.0) # Set the thickness
                    y = cap.get_ydata()[0]

                for median in method['medians']:
                    median.set(linewidth=7.0) # Set the thickness


        elif order == '\nVel Error (m/s)':

            for i in range(3): #xyz
                if i == 0:
                    label = '∥Vx∥'
                elif i == 1:
                    label = '∥Vy∥'
                elif i == 2:
                    label = '∥Vz∥'
                method = graphs[a][m].boxplot(wing[a][m][20+i],
                                    #patch_artist = True,
                                    boxprops={'color':colors[m]},
                                    capprops={'color':'orange'},
                                    medianprops={'color':med_colors[m]},
                                    whiskerprops={'color':'orange'},
                                    showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                # rmse
                graphs[a][m].plot(placement+i*0.05, wing[a][m][23][0+i], 'X', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 30, label='RMS') # marker type
                # mean
                graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][20+i]), 'o', 
                                mfc = med_colors[m], mec = med_colors[m], ms = 30, label='Mean') # marker type
                
                method_mean = statistics.mean(wing[a][m][20+i])
                method_stdev = statistics.stdev(wing[a][m][20+i])
                rmse = wing[a][m][23][0+i]

                #aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
            
                # display mean and stdev
                if display_mean == True:
                    # Add the text label above the median line
                    graphs[a][m].text(0+i, 1,  # x, y coordinates for the text
                    f'{statistics.mean(wing[a][m][20+i]):.2f},{statistics.stdev(wing[a][m][20+i]):.2f}',  # The text to display (formatted to 2 decimal places)
                    ha='center',  # Horizontal alignment
                    va='bottom',  # Vertical alignment
                    fontsize=15,  # Adjust fontsize as needed
                    color='black')  # Adjust color as needed

                # rotate x axis labels
                graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=53)
                graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=53)
                graphs[a][m].set_xlim(-0.02, 0.12)
                # add title
                graphs[a][m].set_title(drone_label + method_label + order, fontsize=43, fontweight='bold')
                graphs[a][m].grid(linewidth=4.0)
                # add legend
                if i == 0:
                    graphs[a][m].legend(loc='upper right', fontsize=43)

                for whisker in method['whiskers']:
                    whisker.set(linewidth=7.0) # Set the thickness 

                for box in method['boxes']:
                    box.set(linewidth=7.0) # Set the thickness 

                for cap in method['caps']:
                    cap.set(linewidth=7.0) # Set the thickness
                    y = cap.get_ydata()[0]

                for median in method['medians']:
                    median.set(linewidth=7.0) # Set the thickness
    

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
plt.savefig(title+'.pdf', dpi=300, bbox_inches='tight')  
#plt.show()
