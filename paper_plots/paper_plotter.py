import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import matplotlib.pyplot as plt
import CF_folder_traj_data_sort as cf


data_sorter = cf.sort_traj_data()

long_wing = 'long_traj_data'
short_wing = 'short_traj_data'
foam_wing = 'foam_traj_data'

short_wing_circle = data_sorter.whisker_circle(short_wing)
short_wing_elevated = data_sorter.whisker_elevated_circle(short_wing)
short_wing_lem = data_sorter.whisker_lem(short_wing)

## generate the plot

#fig, ((ax1,ax2), (ax3,ax4)) = plt.subplots(2, 2, figsize=(40, 10))
#fig, ((ax1,ax2,ax3)) = plt.subplots(1, 3, figsize=(40, 5))
fig, ((ax1)) = plt.subplots(1, 1, figsize=(20, 5))
#fig.subplots_adjust(hspace=0.5, wspace=0.5)
indi = ax1.boxplot(short_wing_circle[0][6],
                    #patch_artist = True,
                    boxprops={'color':'blue'},
                    capprops={'color':'orange'},
                    medianprops={'color':'red'},
                    whiskerprops={'color':'orange'},
                    showfliers=False,widths=0.05,positions=[0],whis=(5, 95),labels=['INDI'])

for whisker in indi['whiskers']:
    whisker.set(linewidth=2.0) # Set the thickness 

for box in indi['boxes']:
    box.set(linewidth=2.0) # Set the thickness 

for cap in indi['caps']:
    cap.set(linewidth=3.0) # Set the thickness

for median in indi['medians']:
    median.set(linewidth=3.0) # Set the thickness

ndi = ax1.boxplot(short_wing_circle[1][6],
                    boxprops={'color':'green'},
                    capprops={'color':'orange'},
                    medianprops={'color':'red'},
                    whiskerprops={'color':'orange'},
                    showfliers=False,widths=0.05,positions=[0.07],whis=(5, 95),labels=['NDI'])

for whisker in ndi['whiskers']:
    whisker.set(linewidth=2.0) # Set the thickness 

for box in ndi['boxes']:
    box.set(linewidth=2.0) # Set the thickness 

for cap in ndi['caps']:
    cap.set(linewidth=3.0) # Set the thickness

for median in ndi['medians']:
    median.set(linewidth=3.0) # Set the thickness

att = ax1.boxplot(short_wing_circle[2][6],
                    boxprops={'color':'pink'},
                    capprops={'color':'orange'},
                    medianprops={'color':'red'},
                    whiskerprops={'color':'orange'},
                    showfliers=False,widths=0.05,positions=[0.14],whis=(5, 95),labels=['att'])

for whisker in att['whiskers']:
    whisker.set(linewidth=2.0) # Set the thickness 

for box in att['boxes']:
    box.set(linewidth=2.0) # Set the thickness 

for cap in att['caps']:
    cap.set(linewidth=3.0) # Set the thickness

for median in att['medians']:
    median.set(linewidth=3.0) # Set the thickness
    
plt.show()



#plt.savefig('test.pdf')