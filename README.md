# pid_control_ardrone
PID controller for AR,Drone. Developed as part of Second-Semester Mini-Project for MSc Robotics at the University of Birmingham. 

The controller uses ROS and depends on the ardrone_autonomy package (http://wiki.ros.org/ardrone_autonomy).
This can be used on the actual drone or the simulator (for ros indigo: https://github.com/dougvk/tum_simulator).

ON THE AR.DRONE:
The controller requires a motion capture system.
The data from the motion capture system has to be sent from another computer wirelessly for the mocap_odometry.py file to read.

Steps to run PID control on actual drone.
1. catkin_make the three packages.
2. Run mocap_odometry.py code.
3. Run controller.py code in pid_ardrone package.
4. Run ardrone_joy_keyboard.py in the ardrone_joy package.
5. Control instructions will be shown.

Steps to run in simulation. 
1. catkin_make all packages, except mocap_odometry (package not required).
2. Run ardrone simulator.
3. Run sim_controller.py code in pid_ardrone package.
4. Run ardrone_joy_keyboard.py in the ardrone_joy package.
5. Control instructions will be shown.
