import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import CF_all_traj_data_compile as cf


class sort_traj_data(object):
    def __init__(self):
        self.traj_compile = cf.all_trajectory()

    
    def plot_circle(self,folder,att): 
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
    

    def plot_fan_circle(self,folder,att): 
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
        
    
        return (nmpc1, dfbc1, nmpc1_fan, dfbc1_fan)


    def whisker_circle(self,folder,type):
        # circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1,type)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1,type)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5,type)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5,type)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan,type)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan,type)
        
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    

    def norm_circle(self):
        # circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/long_traj_data/circle1/NMPC/'
        nmpc1 = self.traj_compile.normal_data_compiler(nmpc1)
        return (nmpc1)
    

    def fan_whisker_circle(self,folder,type):
        # circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_circle1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1,type)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1,type)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5,type)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5,type)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan,type)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan,type)
        
        return (nmpc1, dfbc1, nmpc1_fan, dfbc1_fan)
    

    def plot_foam_circle(self,folder,att):
        # circle 0.4
        nmpc4 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle4/NMPC/'
        dfbc4 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle4/DFBC/'
        
        nmpc4_payload = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/payload_circle4/NMPC/'
        dfbc4_payload = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/payload_circle4/DFBC/'
        

        nmpc4 = self.traj_compile.traj_data_compiler(nmpc4)
        dfbc4 = self.traj_compile.traj_data_compiler(dfbc4)

        nmpc4_payload = self.traj_compile.traj_data_compiler(nmpc4_payload)
        dfbc4_payload = self.traj_compile.traj_data_compiler(dfbc4_payload)
        
        return (nmpc4, dfbc4, nmpc4_payload, dfbc4_payload)


    def foam_whisker_circle(self,folder,type):
        # circle 0.4
        nmpc4 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle4/NMPC/'
        dfbc4 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/circle4/DFBC/'
        
        nmpc4_payload = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/payload_circle4/NMPC/'
        dfbc4_payload = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/payload_circle4/DFBC/'
        

        nmpc4 = self.traj_compile.data_whisker_plot(nmpc4,type)
        dfbc4 = self.traj_compile.data_whisker_plot(dfbc4,type)

        nmpc4_payload = self.traj_compile.data_whisker_plot(nmpc4_payload,type)
        dfbc4_payload = self.traj_compile.data_whisker_plot(dfbc4_payload,type)
        
        return (nmpc4, dfbc4, nmpc4_payload, dfbc4_payload)


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


    def whisker_elevated_circle(self,folder,type):
        # elevated_circle 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/elevated_circle1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_elevated_circle1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_elevated_circle1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1,type)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1,type)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5,type)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5,type)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan,type)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan,type)
        
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
    

    def whisker_lem(self,folder,type):
        # lem 1 and 1.5
        nmpc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1/NMPC/'
        dfbc1 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1/DFBC/'

        nmpc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1.5/NMPC/'
        dfbc1_5 = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/lem1.5/DFBC/'

        nmpc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_lem1/NMPC/'
        dfbc1_fan = '/home/emmanuel/Monocopter-OCP/OCP_MPC/paper_plots/' + folder + '/fan_lem1/DFBC/'

        nmpc1 = self.traj_compile.data_whisker_plot(nmpc1,type)
        dfbc1 = self.traj_compile.data_whisker_plot(dfbc1,type)
        
        nmpc1_5 = self.traj_compile.data_whisker_plot(nmpc1_5,type)
        dfbc1_5 = self.traj_compile.data_whisker_plot(dfbc1_5,type)

        nmpc1_fan = self.traj_compile.data_whisker_plot(nmpc1_fan,type)
        dfbc1_fan = self.traj_compile.data_whisker_plot(dfbc1_fan,type)
        
        return (nmpc1, dfbc1, nmpc1_5, dfbc1_5, nmpc1_fan, dfbc1_fan)
    
    
    