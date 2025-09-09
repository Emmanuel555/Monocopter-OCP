import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import CF_all_traj_data_compile as cf


class sort_traj_data(object):
    def __init__(self):
        self.traj_compile = cf.all_trajectory()

    
    def plot_circle(self,folder,att): # only for indi
        # circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/DFBC/'

        
        if att == 1:
            nmpc1 = self.traj_compile.att_data_compiler(nmpc1)
            dfbc1 = self.traj_compile.att_data_compiler(dfbc1)

            nmpc1_fan = self.traj_compile.att_data_compiler(nmpc1_fan)
            dfbc1_fan = self.traj_compile.att_data_compiler(dfbc1_fan)
            
            nmpc1_5 = self.traj_compile.att_data_compiler(nmpc1_5)
            dfbc1_5 = self.traj_compile.att_data_compiler(dfbc1_5)
        else:
            nmpc1 = self.traj_compile.traj_data_compiler(nmpc1)
            dfbc1 = self.traj_compile.traj_data_compiler(dfbc1)

            nmpc1_fan = self.traj_compile.traj_data_compiler(nmpc1_fan)
            dfbc1_fan = self.traj_compile.traj_data_compiler(dfbc1_fan)
            
            nmpc1_5 = self.traj_compile.traj_data_compiler(nmpc1_5)
            dfbc1_5 = self.traj_compile.traj_data_compiler(dfbc1_5)
        
    
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    

    def whisker_circle(self,folder):
        # circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan)
        
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    

    def foam_whisker_circle(self,folder):
        # circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/ndi/'
        
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/ndi/'
        

        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)

        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
        
        return (indi3, ndi3, indi5, ndi5)


    
    def plot_elevated_circle(self,folder,att):
        # elevated_circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_elevated_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_elevated_circle1/DFBC/'
     
        if att == 1:
            nmpc1 = self.traj_compile.att_data_compiler(nmpc1)
            dfbc1 = self.traj_compile.att_data_compiler(dfbc1)

            nmpc1_fan = self.traj_compile.att_data_compiler(nmpc1_fan)
            dfbc1_fan = self.traj_compile.att_data_compiler(dfbc1_fan)
            
            nmpc1_5 = self.traj_compile.att_data_compiler(nmpc1_5)
            dfbc1_5 = self.traj_compile.att_data_compiler(dfbc1_5)
        else:
            nmpc1 = self.traj_compile.traj_data_compiler(nmpc1)
            dfbc1 = self.traj_compile.traj_data_compiler(dfbc1)

            nmpc1_fan = self.traj_compile.traj_data_compiler(nmpc1_fan)
            dfbc1_fan = self.traj_compile.traj_data_compiler(dfbc1_fan)
            
            nmpc1_5 = self.traj_compile.traj_data_compiler(nmpc1_5)
            dfbc1_5 = self.traj_compile.traj_data_compiler(dfbc1_5)
        
    
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)


    def whisker_elevated_circle(self,folder):
        # elevated_circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_elevated_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_elevated_circle1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan)
        
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    

    def plot_lem(self,folder,att):
        # lem 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_lem1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_lem1/DFBC/'
        
        if att == 1:
            nmpc1 = self.traj_compile.att_data_compiler(nmpc1)
            dfbc1 = self.traj_compile.att_data_compiler(dfbc1)

            nmpc1_fan = self.traj_compile.att_data_compiler(nmpc1_fan)
            dfbc1_fan = self.traj_compile.att_data_compiler(dfbc1_fan)
            
            nmpc1_5 = self.traj_compile.att_data_compiler(nmpc1_5)
            dfbc1_5 = self.traj_compile.att_data_compiler(dfbc1_5)
        else:
            nmpc1 = self.traj_compile.traj_data_compiler(nmpc1)
            dfbc1 = self.traj_compile.traj_data_compiler(dfbc1)

            nmpc1_fan = self.traj_compile.traj_data_compiler(nmpc1_fan)
            dfbc1_fan = self.traj_compile.traj_data_compiler(dfbc1_fan)
            
            nmpc1_5 = self.traj_compile.traj_data_compiler(nmpc1_5)
            dfbc1_5 = self.traj_compile.traj_data_compiler(dfbc1_5)
        
    
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    

    def whisker_lem(self,folder):
        # lem 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_lem1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_lem1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan)
        
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    
    
    