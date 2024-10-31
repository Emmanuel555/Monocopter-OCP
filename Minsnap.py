import minsnap_trajectories as ms
import numpy as np


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