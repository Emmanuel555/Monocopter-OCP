import math
import numpy as np
import numpy.linalg as la
import minsnap_trajectories as ms


class trajectory_generator(object):
    def __init__(self):
        pass


    def hover_test(self, x_offset):
        ref_x = 0
        ref_y = 1.0
        ref_z = 0.15
        ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
        msg = "hovering test..."
        return (ref_pos,msg)
    

    def low_alt_rectangle(self, x_offset, abs_time):
        if abs_time < 3:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 3 <= abs_time < 7:
            ref_x = 0
            ref_y = 0.9
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 7 <= abs_time < 11:
            ref_x = 0
            ref_y = 1.2
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..." 
        elif 11 <= abs_time < 15:
            ref_x = 0.5
            ref_y = 1.2
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 15 <= abs_time < 19:
            ref_x = 0.5
            ref_y = 0.9
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 19 <= abs_time < 23:
            ref_x = 0.5
            ref_y = 0.6
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 23 <= abs_time < 27:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 27 <= abs_time:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.02
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "traj ended..."

        return (ref_pos,msg)
    

    def simple_rectangle(self, x_offset, abs_time):
        if abs_time < 3:
            ref_x = 0
            ref_y = 0.6
            ref_z = 1.0
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 3 <= abs_time < 7:
            ref_x = 0
            ref_y = 0.9
            ref_z = 1.0
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 7 <= abs_time < 11:
            ref_x = 0
            ref_y = 1.2
            ref_z = 1.0
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..." 
        elif 11 <= abs_time < 15:
            ref_x = 0.5
            ref_y = 1.2
            ref_z = 1.0
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 15 <= abs_time < 19:
            ref_x = 0.5
            ref_y = 0.9
            ref_z = 1.0
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 19 <= abs_time < 23:
            ref_x = 0.5
            ref_y = 0.6
            ref_z = 1.0
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 23 <= abs_time < 27:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 27 <= abs_time:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.01
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "traj ended..."

        return (ref_pos,msg)
    

    def elevated_rectangle(self, x_offset, abs_time):
        if abs_time < 3:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.3
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 3 <= abs_time < 7:
            ref_x = 0
            ref_y = 0.9
            ref_z = 0.7
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 7 <= abs_time < 11:
            ref_x = 0
            ref_y = 1.2
            ref_z = 1.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..." 
        elif 11 <= abs_time < 15:
            ref_x = 0.5
            ref_y = 1.2
            ref_z = 1.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 15 <= abs_time < 19:
            ref_x = 0.5
            ref_y = 0.9
            ref_z = 0.7
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 19 <= abs_time < 23:
            ref_x = 0.5
            ref_y = 0.6
            ref_z = 0.3
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 23 <= abs_time < 27:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.2
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "still flying..."
        elif 27 <= abs_time:
            ref_x = 0
            ref_y = 0.6
            ref_z = 0.02
            ref_pos = np.array([ref_x+x_offset, ref_y, ref_z])
            msg = "traj ended..."

        return (ref_pos,msg)
    

    def compute_jerk_snap_9pt_circle(self, x_offset, radius, speedX):
        # theta goes from 0 to 2pi
        parts = 9 # octagon
        theta = np.linspace(0, 2*np.pi, parts)

        # the radius of the circle
        r = radius
        circumference = 2*np.pi*r
        total_time = (circumference/0.1)/speedX
        num_points = int((circumference/0.1)*100) # 0.1 m/s baseline 
        num_points = int(num_points/speedX) # 0.1 m/s baseline


        # compute x1 and x2
        x = r*np.cos(theta) + x_offset
        y = r*np.sin(theta) + 1.2


        refs = [
            ms.Waypoint(
                time=(total_time/(parts-1))*0,
                position=np.array([x[0], y[0], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*1,
                position=np.array([x[1], y[1], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*2,
                position=np.array([x[2], y[2], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*3,
                position=np.array([x[3], y[3], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*4,
                position=np.array([x[4], y[4], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*5,
                position=np.array([x[5], y[5], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*6,
                position=np.array([x[6], y[6], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*7,
                position=np.array([x[7], y[7], 1.0]),
            ),
            ms.Waypoint(
                time=(total_time/(parts-1))*8,
                position=np.array([x[8], y[8], 1.0]),
            ),
        ]

        polys = ms.generate_trajectory(
                refs,
                degree=8,  # Polynomial degree
                idx_minimized_orders=(3, 4),  
                num_continuous_orders=3,  
                algorithm="closed-form",  # Or "constrained"
            )

        t = np.linspace(0, total_time, num_points)
        # Sample up to the 3rd order (Jerk) -----v
        pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6
        return (pva,num_points)
    

    def compute_jerk_snap_9pt_circle_x_laps(self, x_offset, radius, speedX, laps):
        # theta goes from 0 to 2pi
        parts = 9 # octagon lap x 5
        theta = np.linspace(0, 2*np.pi, parts)
        total_parts = parts + ((parts -1)*(laps-1)) 

        # the radius of the circle
        r = radius
        circumference = 2*np.pi*r
        total_time = laps*((circumference/0.1)/speedX)
        num_points = int(laps*((circumference/0.1)*100)) # 0.1 m/s baseline 
        num_points = int(num_points/speedX) # 0.1 m/s baseline

        # compute x1 and x2
        x_coordinates = r*np.cos(theta) + x_offset
        y_coordinates = r*np.sin(theta) + 1.2

        x = np.array([x_coordinates[0]])
        y = np.array([y_coordinates[0]])
        refs = []

        for i in range(laps):
            x = np.append(x,x_coordinates[1:])
            y = np.append(y,y_coordinates[1:])

        for i in range(total_parts):
            refs.append(ms.Waypoint(
                time=(total_time/(total_parts-1))*i,
                position=np.array([x[i], y[i], 1.0]),
            ))

        polys = ms.generate_trajectory(
                refs,
                degree=8,  # Polynomial degree
                idx_minimized_orders=(3, 4),  
                num_continuous_orders=3,  
                algorithm="closed-form",  # Or "constrained"
            )

        t = np.linspace(0, total_time, num_points)
        # Sample up to the 3rd order (Jerk) -----v
        pva = ms.compute_trajectory_derivatives(polys, t, 6) # up to order of derivatives is 6
        return (pva,num_points)


    def jerk_snap_circle(self, pva, num_points, count, landing_hgt):
        all_pos = np.array([pva[0,:,0],pva[0,:,1],pva[0,:,2]]) # position
        all_vel = np.array([pva[1,:,0],pva[1,:,1],pva[1,:,2]]) # velocity
        all_acc = np.array([pva[2,:,0],pva[2,:,1],pva[2,:,2]]) # acceleration
        all_jer = np.array([pva[3,:,0],pva[3,:,1],pva[3,:,2]]) # jerk
        all_sna = np.array([pva[4,:,0],pva[4,:,1],pva[4,:,2]]) # snap

        if count >= num_points:
            ref_pos = np.array([0,1.0,landing_hgt]) # position
            ref_vel = np.array([0,0,0]) # velocity
            ref_acc = np.array([0,0,0]) # acceleration
            ref_jer = np.array([0,0,0]) # jerk
            ref_sna = np.array([0,0,0]) # snap
            msg = "traj ended..."
        else:
            ref_pos = np.array([pva[0,count,0],pva[0,count,1],pva[0,count,2]]) # ref position
            ref_vel = np.array([pva[1,count,0],pva[1,count,1],pva[1,count,2]]) # ref velocity
            ref_acc = np.array([pva[2,count,0],pva[2,count,1],pva[2,count,2]]) # ref acceleration
            ref_jer = np.array([pva[3,count,0],pva[3,count,1],pva[3,count,2]]) # ref jerk
            ref_sna = np.array([pva[4,count,0],pva[4,count,1],pva[4,count,2]]) # ref snap
            msg = "still flying..."
        
        ref_pos = list(ref_pos.flat)
        ref_vel = list(ref_vel.flat)
        ref_acc = list(ref_acc.flat)    
        ref_jer = list(ref_jer.flat)
        ref_sna = list(ref_sna.flat)

        return (ref_pos,ref_vel,ref_acc,ref_jer,ref_sna,msg)
        
 
    def simple_circle(self, x_offset, radius, count, speedX):
        circumference = 2*np.pi*radius
        num_points = int((circumference/0.1)*100) # 0.1 m/s baseline over 1
        num_points = int(num_points/speedX) # 0.1 m/s baseline over 1
        theta = np.linspace(0, 2*np.pi, num_points) 

        # the radius of the circle
        r = radius

        # compute x and y, starts from bottom facing positive x
        x = r*np.cos(theta) + x_offset 
        y = r*np.sin(theta) + 1.2
        
        if count >= num_points:
            ref_x = x[-1]
            ref_y = y[-1]
            ref_z = 0.2
            msg = "traj ended..."
        else:
            ref_x = x[count]
            ref_y = y[count]
            ref_z = 1.0
            msg = "still flying..."

        ref_pos = np.array([ref_x, ref_y, ref_z])
        
        return (ref_pos,msg)


    def elevated_circle(self, x_offset, radius, count, speedX):
        circumference = 2*np.pi*radius
        num_points = int((circumference/0.1)*100) # 0.1 m/s baseline over 1
        num_points = int(num_points/speedX) # 0.1 m/s baseline over 1
        theta = np.linspace(0, 2*np.pi, num_points) 

        # the radius of the circle
        r = radius

        # compute x and y, starts from bottom facing positive x
        x = r*np.cos(theta) + x_offset 
        y = r*np.sin(theta) + 1.2
        z = y

        if count >= num_points:
            ref_x = x[-1]
            ref_y = y[-1]
            ref_z = 0.10
            msg = "traj ended..."
        else:
            ref_x = x[count]
            ref_y = y[count]
            ref_z = z[count]
            msg = "still flying..."

        ref_pos = np.array([ref_x, ref_y, ref_z])

        return (ref_pos,msg)
    

    def helix(self, x_offset, radius, count, speedX):
        circumference = 2*np.pi*radius
        num_points = int((circumference/0.1)*100) # 0.1 m/s baseline over 1
        num_points = int(num_points/speedX) # 0.1 m/s baseline over 1
        theta = np.linspace(0, 2*np.pi, num_points) 
        z_1 = np.linspace(0.7, 1, num_points)
        z_2 = np.linspace(1, 1.5, num_points)
        z_3 = np.linspace(1.5, 1, num_points)
        z_4 = np.linspace(1, 0.7, num_points)


        # the radius of the circle
        r = radius

        # compute x and y, starts from bottom facing positive x
        x = r*np.cos(theta) + x_offset 
        y = r*np.sin(theta) + 1.2
        z = y

        helix_array_x = np.array([x,x,x,x])
        helix_array_y = np.array([y,y,y,y]) 
        helix_array_z = np.array([z_1,z_2,z_3,z_4]) 

        helix_array_x = helix_array_x.flat
        helix_array_y = helix_array_y.flat
        helix_array_z = helix_array_z.flat

        if count >= num_points*4:
            ref_x = helix_array_x[-1]
            ref_y = helix_array_y[-1]
            ref_z = 0.10
            msg = "traj ended..."
        else:
            ref_x = helix_array_x[count]
            ref_y = helix_array_y[count]
            ref_z = helix_array_z[count]
            msg = "still flying..."

        ref_pos = np.array([ref_x, ref_y, ref_z])

        return (ref_pos,msg)


    