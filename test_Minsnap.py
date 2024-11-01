import minsnap_trajectories as ms
import numpy as np
import numpy.linalg as la
import math
from pyquaternion import Quaternion
from pyrr import Quaternion, Matrix33, Matrix44, Vector3


if __name__ == '__main__':
    refs = [
    ms.Waypoint(
        time=0.0,
        position=np.array([0.0, 0.0, 10.0]),
    ),
    ms.Waypoint(  # Any higher-order derivatives
        time=8.0,
        position=np.array([10.0, 0.0, 10.0]),
        velocity=np.array([0.0, 5.0, 0.0]),
        acceleration=np.array([0.1, 0.0, 0.0]),
    ),
    ms.Waypoint(  # Potentially leave intermediate-order derivatives unspecified
        time=16.0,
        position=np.array([20.0, 0.0, 10.0]),
        jerk=np.array([0.1, 0.0, 0.2]),
    ),
    ]

    polys = ms.generate_trajectory(
        refs,
        degree=8,  # Polynomial degree
        idx_minimized_orders=(3, 4),  
        num_continuous_orders=3,  
        algorithm="closed-form",  # Or "constrained"
    )

    t = np.linspace(0, 16, 100)
    #  Sample up to the 3rd order (Jerk) -----v
    pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6

    # available order in derivatives in pva is one lesser than computed meaning up to one derivative less  

    print(pva[5,99,2]) # up to order of derivatives available, waypoints, axis

    # print(np.shape(np.array([[1,2,3],[2,3,1]]))) # size is 2 x 3

    z = np.array([2,3,1]) # 3 x 1
    z = np.array([[2,3,1]]) # 1 x 3

    z = np.transpose(z)
    print(np.shape(z))

    a = np.array([[0,0,1]]) # 1 x 3 - row based simulated disk vector 
    b = np.array([[1,0,1]]) # 1 x 3 - row based simulated zd which is desired vector 
    c = np.dot(a,np.transpose(b))
    d = la.norm(a,2)*la.norm(b,2) # L2 norm of a and b
    angle = math.acos(c/d)

    n = np.cross(a,b)/la.norm(np.cross(a,b)) # cross product of a and b
    
    print(np.shape(n))

    #print(math.degrees(angle))
