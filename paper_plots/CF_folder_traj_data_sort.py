import math
import numpy as np
from scipy import ndimage
from numpy import linalg as la
import CF_all_traj_data_compile as cf


class sort_traj_data(object):
    def __init__(self):
        self.traj_compile = cf.all_trajectory()

    
    def plot_circle(self,folder): # only for indi
        # circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/ndi/'
       
        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/ndi/'
        
        indi3 = self.traj_compile.att_data_compiler(indi3)
        ndi3 = self.traj_compile.att_data_compiler(ndi3)
        
        indi5 = self.traj_compile.att_data_compiler(indi5)
        ndi5 = self.traj_compile.att_data_compiler(ndi5)
        
        
        return (indi3, ndi3, indi5, ndi5)
    

    def whisker_circle(self,folder):
        # circle 0.3
        indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/indi/'
        ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/ndi/'
        att3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle3/att/'

        indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/indi/'
        ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/ndi/'
        att5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/circle5/att/'

        indi3 = self.traj_compile.data_whisker_plot(indi3)
        ndi3 = self.traj_compile.data_whisker_plot(ndi3)
        att3 = self.traj_compile.data_whisker_plot(att3)

        indi5 = self.traj_compile.data_whisker_plot(indi5)
        ndi5 = self.traj_compile.data_whisker_plot(ndi5)
        att5 = self.traj_compile.data_whisker_plot(att5)
        
        return (indi3, ndi3, att3, indi5, ndi5, att5)

    
    def plot_elevated_circle(self,folder):
        # elevated circle 0.3
        elevated_indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/indi/'
        elevated_ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/ndi/'
        
        elevated_indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/indi/'
        elevated_ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/ndi/'
     
        elevated_indi3 = self.traj_compile.att_data_compiler(elevated_indi3)
        elevated_ndi3 = self.traj_compile.att_data_compiler(elevated_ndi3)
  
        elevated_indi5 = self.traj_compile.att_data_compiler(elevated_indi5)
        elevated_ndi5 = self.traj_compile.att_data_compiler(elevated_ndi5)
   
        return (elevated_indi3, elevated_ndi3, elevated_indi5, elevated_ndi5)


    def whisker_elevated_circle(self,folder):
        # elevated circle 0.3
        elevated_indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/indi/'
        elevated_ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/ndi/'
        elevated_att3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle3/att/'

        elevated_indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/indi/'
        elevated_ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/ndi/'
        elevated_att5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/elevated_circle5/att/'


        elevated_indi3 = self.traj_compile.data_whisker_plot(elevated_indi3)
        elevated_ndi3 = self.traj_compile.data_whisker_plot(elevated_ndi3)
        elevated_att3 = self.traj_compile.data_whisker_plot(elevated_att3)

        elevated_indi5 = self.traj_compile.data_whisker_plot(elevated_indi5)
        elevated_ndi5 = self.traj_compile.data_whisker_plot(elevated_ndi5)
        elevated_att5 = self.traj_compile.data_whisker_plot(elevated_att5)

        return (elevated_indi3, elevated_ndi3, elevated_att3, elevated_indi5, elevated_ndi5, elevated_att5)
    

    def plot_lem(self,folder):
        # lem 0.3
        lem_indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/indi/'
        lem_ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/ndi/' 
        
        lem_indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/indi/'
        lem_ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/ndi/'
        
        lem_indi3 = self.traj_compile.att_data_compiler(lem_indi3)
        lem_ndi3 = self.traj_compile.att_data_compiler(lem_ndi3)

        lem_indi5 = self.traj_compile.att_data_compiler(lem_indi5)
        lem_ndi5 = self.traj_compile.att_data_compiler(lem_ndi5)
       

        return (lem_indi3, lem_ndi3, lem_indi5, lem_ndi5)
    

    def whisker_lem(self,folder):
        # lem 0.3
        lem_indi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/indi/'
        lem_ndi3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/ndi/' 
        lem_att3 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem3/att/'

        lem_indi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/indi/'
        lem_ndi5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/ndi/'
        lem_att5 = '/home/emmanuel/Monocopter-OCP/paper_plots/' + folder + '/lem5/att/'


        lem_indi3 = self.traj_compile.data_whisker_plot(lem_indi3)
        lem_ndi3 = self.traj_compile.data_whisker_plot(lem_ndi3)
        lem_att3 = self.traj_compile.data_whisker_plot(lem_att3)

        lem_indi5 = self.traj_compile.data_whisker_plot(lem_indi5)
        lem_ndi5 = self.traj_compile.data_whisker_plot(lem_ndi5)
        lem_att5 = self.traj_compile.data_whisker_plot(lem_att5)

        return (lem_indi3, lem_ndi3, lem_att3, lem_indi5, lem_ndi5, lem_att5)
    
    
    