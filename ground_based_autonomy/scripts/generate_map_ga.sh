#!/bin/bash
echo ""
echo "Generating map."
echo ""

#export ROS_MASTER_URI=http://master_computer:11311
#export ROS_IP=10.1.1.100

source ~/ground_based_autonomy/devel/setup.sh --extend

read -p "pointcloud_in: " pointcloud_in
pointcloud_in="${pointcloud_in//[\'\ ]/}"
pointcloud_out="${pointcloud_in//"pointcloud"/"pointcloud_loop_closed"}"

echo ""
read -p "trajectory_in: " trajectory_in
trajectory_in="${trajectory_in//[\'\ ]/}"
trajectory_out="${trajectory_in//"trajectory"/"trajectory_loop_closed"}"

echo ""
read -p "key_pose_in: " key_pose_in
key_pose_in="${key_pose_in//[\'\ ]/}"
echo ""

roslaunch point_cloud_stacker point_cloud_generator.launch pointcloud_in:=$pointcloud_in trajectory_in:=$trajectory_in key_pose_in:=$key_pose_in pointcloud_out:=$pointcloud_out trajectory_out:=$trajectory_out
