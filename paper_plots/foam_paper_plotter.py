import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf
import statistics


data_sorter = cf.sort_traj_data()

foam_wing = 'foam_traj_data'

selected_wing = foam_wing


if selected_wing == foam_wing:
    title = 'Controller Tracking Performance using Ultralight Wing'

# test short wing first
wing_circle = data_sorter.foam_whisker_circle(selected_wing)
wing_elevated = data_sorter.foam_whisker_elevated_circle(selected_wing)

wing = [wing_circle,wing_elevated]

## generate the plot
fig, ((ax1,ax3,ax5,ax7),(ax2,ax4,ax6,ax8)) = plt.subplots(2, 4, figsize=(40, 20))
graphs = [[ax1,ax2,ax3,ax4],[ax5,ax6,ax7,ax8]]
colors = ['#254abe','#96400b','#254abe','#96400b']
med_colors = ['#34be25','#34be25','#34be25','#34be25']
fig.subplots_adjust(hspace=0.2, wspace=0.17, 
                    left=0.048, right=0.97, 
                    top = 0.915, bottom = 0.042)
fig.suptitle(title, fontsize=40, fontweight='bold')


for a in range(len(graphs)): # 3 trajs
    placement = 0.0 # default position in each graph plot
    if a == 0:
        traj_label = 'Cir '
    elif a == 1:
        traj_label = 'Ele '
    
    for m in range(len(graphs[a])): # 0.3 and 0.5 all methods  
        if m == 0:
            method_label = 'INDI 0.3m/s '
        elif m == 1:
            method_label = 'NDI 0.3m/s '
        elif m == 2:
            method_label = 'INDI 0.5m/s '
        elif m == 3:
            method_label = 'NDI 0.5m/s '         

        for i in range(3): #xyz
            if i == 0:
                label = 'Xw'
            elif i == 1:
                label = 'Yw'
            elif i == 2:
                label = 'Zw'
            indi = graphs[a][m].boxplot(wing[a][m][6+i],
                                #patch_artist = True,
                                boxprops={'color':colors[m]},
                                capprops={'color':'orange'},
                                medianprops={'color':med_colors[m]},
                                whiskerprops={'color':'orange'},
                                showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

            # rmse
            graphs[a][m].plot(placement+i*0.05, wing[a][m][9][0+i], 'X', 
                              mfc = med_colors[m], mec = med_colors[m], ms = 15, label='RMS') # marker type
            # mean
            graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][6+i]), 'o', 
                              mfc = med_colors[m], mec = med_colors[m], ms = 15, label='Mean') # marker type
            # rotate x axis labels
            graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=25)
            graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=25)
            graphs[a][m].set_xlim(-0.02, 0.12)
            # add title
            graphs[a][m].set_title(traj_label + method_label + 'Norm error(m)', fontsize=25)
            # add legend
            if i == 0:
                graphs[a][m].legend(loc='upper right', fontsize=20)


            for whisker in indi['whiskers']:
                whisker.set(linewidth=5.0) # Set the thickness 

            for box in indi['boxes']:
                box.set(linewidth=5.0) # Set the thickness 

            for cap in indi['caps']:
                cap.set(linewidth=5.0) # Set the thickness
                y = cap.get_ydata()[0]

            for median in indi['medians']:
                median.set(linewidth=5.0) # Set the thickness
    

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
plt.savefig(title+'.pdf')   
plt.savefig(title+'.png', dpi=300, bbox_inches='tight')  
plt.show()
