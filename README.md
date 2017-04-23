# pid_control_ardrone
PID controller for AR,Drone. Developed as part of Second-Semester Mini-Project for MSc Robotics at University of Birmingham. 

The controller uses ROS and depends on the ardrone_autonomy package (http://wiki.ros.org/ardrone_autonomy).
This can be used on the actual drone or the simulator (http://wiki.ros.org/action/fullsearch/tum_simulator?action=fullsearch&context=180&value=linkto%3A%22tum_simulator%22).

ON THE AR.DRONE:
The controller requires a motion capture system.
The data from the motion capture system has to be sent from another computer wirelessly for the mocap_odometry.py file to read.

Steps to run PID control on actual drone.
1. Build the three packages.
2. Run mocap_odometry.py code.
3. Run controller.py code in pid_ardrone package.
4. Run ardrone_joy_keyboard.py in the ardrone_joy package.
5. Control instructions will be shown.

Steps to run in simulator. 
1. Build all packages, except mocap_odometry (package not required).
2. Run sim_controller.py code in pid_ardrone package.
3. Run ardrone_joy_keyboard.py in the ardrone_joy package.
4. Control instructions will be shown.
