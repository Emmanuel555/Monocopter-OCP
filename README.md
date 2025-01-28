# Monocopter-OCP
Installation: https://docs.acados.org/installation/index.html

Link to problems with installation: https://discourse.acados.org/t/acados-installation-in-pycharm/103

Before running code:

1. Check that ethernet cable is connected from optitrack server rack to ur computer

2. Run native program in optitrack com to poll info from optitrack (track your desired rigid body). The ip 169.254.255.255 should poll to all who are connected. Tracking info is x,y,z,qx,qy,qz,qw, press 1 to initialise running and 0 to refresh in the terminal from the program.

3. Run Mocap_test.py to verify tracking or wtv info that's needed (eg. controls etc...)

4. Run monoco_js_ctrl.py to run controls.

5. 
