#!/bin/bash  

#export ROS_MASTER_URI=http://master_computer:11311
#export ROS_IP=10.1.1.100

trap "rosrun realearth_notification notify_user.bash -r" SIGHUP SIGINT SIGTERM

rosrun realearth_notification notify_user.bash -p
rosnode kill velodyneCloudStacker stack_gps driver_nodelet velodyne_nodelet_manager cloud_nodelet receive_xsens watchdog_all
sleep 2

rosrun clay_launch generate_map.sh
rosnode kill -a
sleep 2

rosrun realearth_notification notify_user.bash -r

