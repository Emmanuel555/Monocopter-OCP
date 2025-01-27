import numpy as np
import timeit


class drag_estimator(object):
    def __init__(self, step_size, array_size, order_duration, sampling_time):
        self.step_size = step_size
        self.drag_terms = np.zeros((order_duration/sampling_time,array_size))
        self.rmse = np.zeros((order_duration/sampling_time))
        self.drag_rmse_terms = [[self.drag_terms,self.rmse]]
        self.order_ready = False

        # phases
        self.p1 = True
        self.p2 = False
        self.p3 = False
        self.p4 = False


    def update_rmse(self, drag_terms, rmse, count):
        if count > len(self.rmse)-1:
            self.order_ready = True
        else:
            self.drag_rmse_terms[0][count] = drag_terms
            self.drag_rmse_terms[1][count] = rmse
            self.order_ready = False


    def best_centroid(self):
        self.drag_rmse_terms.sort(key=lambda each_element: each_element[1])
        self.lowest_rmse = self.drag_rmse_terms[0][1]
        cent_list = []
        for a in self.drag_rmse_terms[:-1]:
            cent_list.append(a[0])
        self.centroid = np.mean(cent_list, axis=0) 
        #print ("centroid: ", np.shape(centroid)) 
        return self.centroid


    def run_NM(self,alpha,sigma):
        self.sigma = sigma
        self.alpha = alpha
        if self.order_ready == True:
            self.best_centroid()
            if self.p1 == True:
                self.reflection_f(self)
            elif self.p3 == True:
                self.expansion_f(self)
                

    def reflection_f(self):
        self.reflection = self.centroid + self.alpha*(self.centroid - self.drag_rmse_terms[-1][0])
        self.p2 == True
        return self.reflection
    
    
    def expansion_f(self):
        self.expansion = self.centroid + self.sigma*(self.reflection - self.centroid)
        self.p4 == True
        return self.expansion


    def centroid_test(self,rmse):
        if self.p2 == True:
            if rmse > self.lowest_rmse and rmse < self.drag_rmse_terms[-2][1]:
                self.drag_rmse_terms[-1][1] = rmse
                self.drag_rmse_terms[-1][0] = self.reflection
                self.p2 = False
            elif rmse < self.lowest_rmse:
                self.old_reflection_rmse = rmse
                self.p3 = True
                self.p1 = False
        elif self.p4 == True:
            if rmse < self.old_reflection_rmse:
                self.drag_rmse_terms[-1][1] = rmse
                self.drag_rmse_terms[-1][0] = self.expansion
                self.p4 = False
                self.p1 = True
            elif rmse > self.old_reflection_rmse:
                self.drag_rmse_terms[-1][1] = self.old_reflection_rmse
                self.drag_rmse_terms[-1][0] = self.reflection
                self.p4 = False
                self.p1 = True
        # cont tmr...
            
        else:
            pass

    




