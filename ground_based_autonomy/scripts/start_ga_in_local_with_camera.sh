#!/bin/bash  

#export ROS_MASTER_URI=http://master_computer:11311
#export ROS_IP=10.1.1.100

trap "rosrun realearth_notification notify_user.bash -r" SIGHUP SIGINT SIGTERM
rosrun realearth_notification notify_user.bash -lw

roslaunch receive_gps receive_gps.launch &
sleep 1
#roslaunch receive_xsens receive_xsens.launch &
#sleep 2
#rosnode kill receive_xsens
#sleep 1

cp $(rospack find localization_wrapper)/map_localization.rviz.bak $(rospack find localization_wrapper)/map_localization.rviz
cp ~/.data/last_pose.txt ~/.data/last_pose.txt.bak
rm ~/.data/pointcloud.bin ~/.data/pointcloud.num ~/.data/trajectory.ply ~/.data/GPS.ply
roslaunch clay_launch start_localization_with_camera.launch &
sleep 1
source ~/ground_based_autonomy/devel/setup.sh --extend
roslaunch online_pose_graph online_pose_graph_in_local.launch
