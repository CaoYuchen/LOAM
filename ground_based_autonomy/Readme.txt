Copy ground_based_autonomy folder to Home and catkin_make, then use the .run files in scripts folder for mode switch

For online processing, use the start button on InkWash, for offline reprocessing of logged data

  roslaunch online_pose_graph online_pose_graph_offline.launch

For Gazebo simulation

  roslaunch vehicle_simulator vehicle_simulator.launch

Install required packages for iSAM:

  sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libsdl1.2-dev doxygen graphviz

To use joystick, install ROS joystick driver

  sudo apt-get install ros-noetic-joystick-drivers

To run in full autonomy mode, set autonomyMode = true and launch waypoint_example to send waypoint, speed and navigation boundary
