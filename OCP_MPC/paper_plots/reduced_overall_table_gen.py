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

selected_wing = long_wing
type = selected_wing
display_mean = False

if selected_wing == short_wing:
    title = 'reduced_MPC tracking performance using short-wing'
elif selected_wing == long_wing:
    title = 'reduced_MPC tracking performance using long-wing'
elif selected_wing == foam_wing:
    title = 'reduced_MPC tracking performance using ultralight-wing'

# test short wing first
if selected_wing == foam_wing:
    wing_circle = data_sorter.red_whisker_foam_circle(selected_wing)
    wing = [wing_circle,wing_circle]

else:
    wing_circle = data_sorter.red_whisker_circle(selected_wing)
    wing_elevated = data_sorter.red_whisker_ele(selected_wing,type,'ele')
    wing_lem = data_sorter.red_whisker_lem(selected_wing,type,'lem')
    wing = [wing_circle,wing_circle,wing_elevated,wing_elevated,wing_lem,wing_lem]


## to test if updating at 250Hz, and yes even after filtering, the data is quite consistent
""" test_long = data_sorter.norm_circle()
traj_time = test_long[0]
o = 0
for v in range(30):
    if traj_time[v] % (1/250) < 1/2050:
        #print ('fk off')
        o = o + 1
print ('test:', traj_time[-1]) """


if selected_wing == foam_wing:
    ## generate the plot
    fig, ((ax1,ax5),(ax2,ax6),
        (ax3,ax7),(ax4,ax8)) = plt.subplots(4, 2, figsize=(40, 20))
    graphs = [[ax1,ax2,ax3,ax4],[ax5,ax6,ax7,ax8]]
    #colors = ['#254abe','#96400b','#a725be','#254abe','#96400b','#a725be']
    colors = ['#254abe','#96400b','#254abe','#96400b']
    #med_colors = ['#34be25','#34be25','red','#34be25','#34be25','red']
    med_colors = ['#34be25','red','#34be25','red']
    aggregate = np.zeros((2, 4, 3))
    aggregate = np.array(aggregate, dtype=list) # to store mean and stdev
    fig.subplots_adjust(hspace=0.3, wspace=0.2, 
                        left=0.048, right=0.97, 
                        top = 0.92, bottom = 0.042)
    fig.suptitle(title + ' (L2 norm error(m))', fontsize=40, fontweight='bold')


else:
    fig, ((ax1,ax7,ax13,ax19,ax25,ax31),(ax2,ax8,ax14,ax20,ax26,ax32),
        (ax3,ax9,ax15,ax21,ax27,ax33),(ax4,ax10,ax16,ax22,ax28,ax34),
        (ax5,ax11,ax17,ax23,ax29,ax35),(ax6,ax12,ax18,ax24,ax30,ax36)) = plt.subplots(6, 6, figsize=(50, 30))
    graphs = [[ax1,ax2,ax3,ax4,ax5,ax6],[ax7,ax8,ax9,ax10,ax11,ax12],[ax13,ax14,ax15,ax16,ax17,ax18],
            [ax19,ax20,ax21,ax22,ax23,ax24],[ax25,ax26,ax27,ax28,ax29,ax30],[ax31,ax32,ax33,ax34,ax35,ax36]]
    #colors = ['#254abe','#96400b','#a725be','#254abe','#96400b','#a725be']
    colors = ['#254abe','#96400b','#254abe','#96400b','#254abe','#96400b']
    #med_colors = ['#34be25','#34be25','red','#34be25','#34be25','red']
    med_colors = ['#34be25','red','#34be25','red','#34be25','red']
    aggregate = np.zeros((6, 6, 3))
    aggregate = np.array(aggregate, dtype=list) # to store mean and stdev
    fig.subplots_adjust(hspace=0.3, wspace=0.2, 
                        left=0.048, right=0.97, 
                        top = 0.92, bottom = 0.042)
    fig.suptitle(title + ' (L2 norm error(m))', fontsize=40, fontweight='bold')



for a in range(len(graphs)): # 3 trajs
    placement = 0.0 # default position in each graph plot
    if a < 2:
        traj_label = 'Cir '
        if a == 0:
            order = '(pos)'
        else: 
            order = '(vel)'    
    elif a > 1 and a < 4:
        traj_label = 'Ele '
        if a == 2:
            order = '(pos)'
        else:
            order = '(vel)'    
    elif a > 3:
        traj_label = 'Lem ' 
        if a == 4:
            order = '(pos)'
        else:
            order = '(vel)'  


    if selected_wing == foam_wing:
        for m in range(len(graphs[a])): # 0.4 all methods  
            if m == 0:
                method_label = '0.4m/s (NMPC)  '
            elif m == 1:
                method_label = '0.4m/s (DFBC) '
            elif m == 2:
                method_label = '0.4m/s payload (NMPC) ' 
            elif m == 3:
                method_label = '0.4m/s payload (DFBC) '


            if order == '(pos)':
                for i in range(3): #xyz
                    if i == 0:
                        label = 'Xw'
                    elif i == 1:
                        label = 'Yw'
                    elif i == 2:
                        label = 'Zw'
                    method = graphs[a][m].boxplot(wing[a][m][6+i],
                                        #patch_artist = True,
                                        boxprops={'color':colors[m]},
                                        capprops={'color':'orange'},
                                        medianprops={'color':med_colors[m]},
                                        whiskerprops={'color':'orange'},
                                        showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                    # rmse
                    graphs[a][m].plot(placement+i*0.05, wing[a][m][9][0+i], 'X', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='RMS') # marker type
                    # mean
                    graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][6+i]), 'o', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='Mean') # marker type
                    
                    method_mean = statistics.mean(wing[a][m][6+i])
                    method_stdev = statistics.stdev(wing[a][m][6+i])
                    rmse = wing[a][m][9][0+i]

                    aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
                
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
                    graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=27)
                    graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=27)
                    graphs[a][m].set_xlim(-0.02, 0.12)
                    # add title
                    graphs[a][m].set_title(traj_label + method_label + order, fontsize=25, fontweight='bold')
                    graphs[a][m].grid(linewidth=4.0)
                    # add legend
                    if i == 0:
                        graphs[a][m].legend(loc='upper right', fontsize=20)

                    for whisker in method['whiskers']:
                        whisker.set(linewidth=7.0) # Set the thickness 

                    for box in method['boxes']:
                        box.set(linewidth=7.0) # Set the thickness 

                    for cap in method['caps']:
                        cap.set(linewidth=7.0) # Set the thickness
                        y = cap.get_ydata()[0]

                    for median in method['medians']:
                        median.set(linewidth=7.0) # Set the thickness


            elif order == '(vel)':

                for i in range(3): #xyz
                    if i == 0:
                        label = 'Xw'
                    elif i == 1:
                        label = 'Yw'
                    elif i == 2:
                        label = 'Zw'
                    method = graphs[a][m].boxplot(wing[a][m][20+i],
                                        #patch_artist = True,
                                        boxprops={'color':colors[m]},
                                        capprops={'color':'orange'},
                                        medianprops={'color':med_colors[m]},
                                        whiskerprops={'color':'orange'},
                                        showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                    # rmse
                    graphs[a][m].plot(placement+i*0.05, wing[a][m][23][0+i], 'X', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='RMS') # marker type
                    # mean
                    graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][20+i]), 'o', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='Mean') # marker type
                    
                    method_mean = statistics.mean(wing[a][m][20+i])
                    method_stdev = statistics.stdev(wing[a][m][20+i])
                    rmse = wing[a][m][23][0+i]

                    aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
                
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
                    graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=27)
                    graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=27)
                    graphs[a][m].set_xlim(-0.02, 0.12)
                    # add title
                    graphs[a][m].set_title(traj_label + method_label + order, fontsize=25, fontweight='bold')
                    graphs[a][m].grid(linewidth=4.0)
                    # add legend
                    if i == 0:
                        graphs[a][m].legend(loc='upper right', fontsize=20)

                    for whisker in method['whiskers']:
                        whisker.set(linewidth=7.0) # Set the thickness 

                    for box in method['boxes']:
                        box.set(linewidth=7.0) # Set the thickness 

                    for cap in method['caps']:
                        cap.set(linewidth=7.0) # Set the thickness
                        y = cap.get_ydata()[0]

                    for median in method['medians']:
                        median.set(linewidth=7.0) # Set the thickness

                  
    else:
        for m in range(len(graphs[a])): # 1 and 1.5 all methods  
            if m == 0:
                method_label = '1.0m/s (NMPC)  '
            elif m == 1:
                method_label = '1.0m/s (DFBC) '
            elif m == 2:
                method_label = '1.5m/s (NMPC) ' 
            elif m == 3:
                method_label = '1.5m/s (DFBC) '
            elif m == 4:
                method_label = '1.0m/s fan (NMPC) '
            elif m == 5:
                method_label = '1.0m/s fan (DFBC)'

        
            if order == '(pos)':

                for i in range(3): #xyz
                    if i == 0:
                        label = 'Xw'
                    elif i == 1:
                        label = 'Yw'
                    elif i == 2:
                        label = 'Zw'
                    method = graphs[a][m].boxplot(wing[a][m][6+i],
                                        #patch_artist = True,
                                        boxprops={'color':colors[m]},
                                        capprops={'color':'orange'},
                                        medianprops={'color':med_colors[m]},
                                        whiskerprops={'color':'orange'},
                                        showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                    # rmse
                    graphs[a][m].plot(placement+i*0.05, wing[a][m][9][0+i], 'X', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='RMS') # marker type
                    # mean
                    graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][6+i]), 'o', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='Mean') # marker type
                    
                    method_mean = statistics.mean(wing[a][m][6+i])
                    method_stdev = statistics.stdev(wing[a][m][6+i])
                    rmse = wing[a][m][9][0+i]

                    aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
                
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
                    graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=27)
                    graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=27)
                    graphs[a][m].set_xlim(-0.02, 0.12)
                    # add title
                    graphs[a][m].set_title(traj_label + method_label + order, fontsize=25, fontweight='bold')
                    graphs[a][m].grid(linewidth=4.0)
                    # add legend
                    if i == 0:
                        graphs[a][m].legend(loc='upper right', fontsize=20)

                    for whisker in method['whiskers']:
                        whisker.set(linewidth=7.0) # Set the thickness 

                    for box in method['boxes']:
                        box.set(linewidth=7.0) # Set the thickness 

                    for cap in method['caps']:
                        cap.set(linewidth=7.0) # Set the thickness
                        y = cap.get_ydata()[0]

                    for median in method['medians']:
                        median.set(linewidth=7.0) # Set the thickness


            elif order == '(vel)':

                for i in range(3): #xyz
                    if i == 0:
                        label = 'Xw'
                    elif i == 1:
                        label = 'Yw'
                    elif i == 2:
                        label = 'Zw'
                    method = graphs[a][m].boxplot(wing[a][m][20+i],
                                        #patch_artist = True,
                                        boxprops={'color':colors[m]},
                                        capprops={'color':'orange'},
                                        medianprops={'color':med_colors[m]},
                                        whiskerprops={'color':'orange'},
                                        showfliers=False,widths=0.03,positions=[placement+i*0.05],whis=(5, 95),labels=[label]) #labels=['INDI']

                    # rmse
                    graphs[a][m].plot(placement+i*0.05, wing[a][m][23][0+i], 'X', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='RMS') # marker type
                    # mean
                    graphs[a][m].plot(placement+i*0.05, statistics.mean(wing[a][m][20+i]), 'o', 
                                    mfc = med_colors[m], mec = med_colors[m], ms = 17, label='Mean') # marker type
                    
                    method_mean = statistics.mean(wing[a][m][20+i])
                    method_stdev = statistics.stdev(wing[a][m][20+i])
                    rmse = wing[a][m][23][0+i]

                    aggregate[a][m][i] = [np.round(method_mean,4), np.round(method_stdev,4), np.round(rmse,4)]
                
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
                    graphs[a][m].tick_params(axis='x', labelrotation=0, labelsize=27)
                    graphs[a][m].tick_params(axis='y', labelrotation=0, labelsize=27)
                    graphs[a][m].set_xlim(-0.02, 0.12)
                    # add title
                    graphs[a][m].set_title(traj_label + method_label + order, fontsize=25, fontweight='bold')
                    graphs[a][m].grid(linewidth=4.0)
                    # add legend
                    if i == 0:
                        graphs[a][m].legend(loc='upper right', fontsize=20)

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


if selected_wing == foam_wing:
    xyz_hold_mean = np.zeros((4, 2, 3)) #rows: x,y,z taken from all traj; columns: NMPC1.0, DFBC1.0, NMPC1.5, DFBC1.5, NMPC_FAN1.0, DFBC_FAN1.0 
    xyz_hold_stddev = np.zeros((4, 2, 3))
    xyz_rmse = np.zeros((4, 2, 3))
    agg_size = np.shape(aggregate)
    for j in range(agg_size[0]): # row 
        for k in range(agg_size[1]): # cols
            for i in range(agg_size[2]): # xyz
                xyz_hold_mean[k][j][i] = aggregate[j][k][i][0] 
                xyz_hold_stddev[k][j][i] = aggregate[j][k][i][1]  # from xyz per element to xxx
                xyz_rmse[k][j][i] = aggregate[j][k][i][2]
                
    paper_mean = np.zeros((4, 2)) 
    paper_stddev = np.zeros((4, 2))
    paper_rmse = np.zeros((4, 2))
    paper_size = np.shape(paper_mean)
    for i in range(paper_size[0]):
        for j in range(paper_size[1]):
            paper_mean[i][j] = statistics.mean(xyz_hold_mean[i][j])
            paper_stddev[i][j] = statistics.stdev(xyz_hold_stddev[i][j])
            paper_rmse[i][j] = statistics.mean(xyz_rmse[i][j]) 

    file = open(title, "w+")
    file.write(selected_wing+'\n')
    file.write('Aggregated XYZ L2 norm error over circle (pos+vel): \n')
    file.write('Paper mean:\n')
    for i in range(paper_size[0]):
        if i == 0:
            label = 'NMPC 0.4m/s '
        elif i == 1:
            label = 'DBFC 0.4m/s '
        elif i == 2:
            label = 'NMPC_PAYLOAD 0.4m/s '
        elif i == 3:
            label = 'DFBC_PAYLOAD 0.4m/s '
            
        content_mean = label + f'{paper_mean[i][0]:.2f},{paper_mean[i][1]:.2f}\n'
        file.write(content_mean)

    file.write('\n')
    file.write('Paper stddev:\n')
    for i in range(paper_size[0]):
        if i == 0:
            label = 'NMPC 0.4m/s '
        elif i == 1:
            label = 'DBFC 0.4m/s '
        elif i == 2:
            label = 'NMPC_PAYLOAD 0.4m/s '
        elif i == 3:
            label = 'DFBC_PAYLOAD 0.4m/s '
            
        content_stddev = label + f'{paper_stddev[i][0]:.2f},{paper_stddev[i][1]:.2f}\n'
        file.write(content_stddev)

    file.write('\n')
    file.write('Paper rmse:\n')
    for i in range(paper_size[0]):
        if i == 0:
            label = 'NMPC 0.4m/s '
        elif i == 1:
            label = 'DBFC 0.4m/s '
        elif i == 2:
            label = 'NMPC_PAYLOAD 0.4m/s '
        elif i == 3:
            label = 'DFBC_PAYLOAD 0.4m/s '
            
        content_rmse = label + f'{paper_rmse[i][0]:.2f},{paper_rmse[i][1]:.2f}\n'
        file.write(content_rmse)        

else:    
    xyz_hold_mean = np.zeros((6, 6, 3)) #rows: x,y,z taken from all traj; columns: NMPC1.0, DFBC1.0, NMPC1.5, DFBC1.5, NMPC_FAN1.0, DFBC_FAN1.0 
    xyz_hold_stddev = np.zeros((6, 6, 3))
    xyz_rmse = np.zeros((6, 6, 3))
    agg_size = np.shape(aggregate)
    for j in range(agg_size[0]): # row 
        for k in range(agg_size[1]): # cols
            for i in range(agg_size[2]): # xyz
                xyz_hold_mean[k][j][i] = aggregate[j][k][i][0] 
                xyz_hold_stddev[k][j][i] = aggregate[j][k][i][1]  # from xyz per element to xxx
                xyz_rmse[k][j][i] = aggregate[j][k][i][2]
                
    paper_mean = np.zeros((6, 6)) 
    paper_stddev = np.zeros((6, 6))
    paper_rmse = np.zeros((6, 6))
    paper_size = np.shape(paper_mean)
    for i in range(paper_size[0]):
        for j in range(paper_size[1]):
            paper_mean[i][j] = statistics.mean(xyz_hold_mean[i][j])
            paper_stddev[i][j] = statistics.stdev(xyz_hold_stddev[i][j])
            paper_rmse[i][j] = statistics.mean(xyz_rmse[i][j])        

    file = open(title, "w+")
    file.write(selected_wing+'\n')
    file.write('Aggregated XYZ L2 norm error over circle (pos+vel), elevated (pos+vel), and lemniscate (pos+vel): \n')
    file.write('Paper mean:\n')
    for i in range(paper_size[0]):
        if i == 0:
            label = 'NMPC 1.0m/s '
        elif i == 1:
            label = 'DBFC 1.0m/s '
        elif i == 2:
            label = 'NMPC 1.5m/s '
        elif i == 3:
            label = 'DFBC 1.5m/s '
        elif i == 4:
            label = 'NMPC_FAN 1.0m/s '
        elif i == 5:
            label = 'DFBC_FAN 1.0m/s '
            
        content_mean = label + f'{paper_mean[i][0]:.2f},{paper_mean[i][1]:.2f},{paper_mean[i][2]:.2f},{paper_mean[i][3]:.2f},{paper_mean[i][4]:.2f},{paper_mean[i][5]:.2f}\n'
        file.write(content_mean)

    file.write('\n')
    file.write('Paper stddev:\n')
    for i in range(paper_size[0]):
        if i == 0:
            label = 'NMPC 1.0m/s '
        elif i == 1:
            label = 'DBFC 1.0m/s '
        elif i == 2:
            label = 'NMPC 1.5m/s '
        elif i == 3:
            label = 'DFBC 1.5m/s '
        elif i == 4:
            label = 'NMPC_FAN 1.0m/s '
        elif i == 5:
            label = 'DFBC_FAN 1.0m/s '
            
        content_stddev = label + f'{paper_stddev[i][0]:.2f},{paper_stddev[i][1]:.2f},{paper_stddev[i][2]:.2f},{paper_stddev[i][3]:.2f},{paper_stddev[i][4]:.2f},{paper_stddev[i][5]:.2f}\n'
        file.write(content_stddev)

    file.write('\n')
    file.write('Paper rmse:\n')
    for i in range(paper_size[0]):
        if i == 0:
            label = 'NMPC 1.0m/s '
        elif i == 1:
            label = 'DBFC 1.0m/s '
        elif i == 2:
            label = 'NMPC 1.5m/s '
        elif i == 3:
            label = 'DFBC 1.5m/s '
        elif i == 4:
            label = 'NMPC_FAN 1.0m/s '
        elif i == 5:
            label = 'DFBC_FAN 1.0m/s '
            
        content_rmse = label + f'{paper_rmse[i][0]:.2f},{paper_rmse[i][1]:.2f},{paper_rmse[i][2]:.2f},{paper_rmse[i][3]:.2f},{paper_rmse[i][4]:.2f},{paper_rmse[i][5]:.2f}\n'
        file.write(content_rmse)




#print ('Paper mean:', paper_mean)
#print ('Paper stddev:', paper_stddev)

file.close()

#plt.savefig(title+'.png', dpi=300, bbox_inches='tight')  
#plt.show()
