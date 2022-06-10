#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

string save_data_dir;
string file_name = "keypose";
string key_pose_topic_name = "/key_pose_to_map";
string key_pose_path_topic_name = "/key_pose_path";

struct PosePoint {
     float x, y, z;
     float roll, pitch, yaw;
     float x_org, y_org, z_org;
     float roll_org, pitch_org, yaw_org;
     double time;
     int ind;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PosePoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, roll, roll)
                                   (float, pitch, pitch)
                                   (float, yaw, yaw)
                                   (float, x_org, x_org)
                                   (float, y_org, y_org)
                                   (float, z_org, z_org)
                                   (float, roll_org, roll_org)
                                   (float, pitch_org, pitch_org)
                                   (float, yaw_org, yaw_org)
                                   (double, time, time)
                                   (int, ind, ind))

pcl::PointCloud<PosePoint>::Ptr keyPose(new pcl::PointCloud<PosePoint>());

bool systemInited = false;
bool systemFinished = false;
double odometryTime = 0;

void keyPoseHandler(const nav_msgs::Odometry::ConstPtr& odometry)
{
  systemInited = true;

  if (systemFinished) {
    return;
  }

  odometryTime = ros::Time::now().toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  PosePoint keyPoseCur;
  keyPoseCur.x = odometry->pose.pose.position.z;
  keyPoseCur.y = odometry->pose.pose.position.x;
  keyPoseCur.z = odometry->pose.pose.position.y;
  keyPoseCur.roll = roll;
  keyPoseCur.pitch = -pitch;
  keyPoseCur.yaw = -yaw;
  keyPoseCur.x_org = odometry->twist.twist.linear.z;
  keyPoseCur.y_org = odometry->twist.twist.linear.x;
  keyPoseCur.z_org = odometry->twist.twist.linear.y;
  keyPoseCur.roll_org = odometry->twist.twist.angular.z;
  keyPoseCur.pitch_org = odometry->twist.twist.angular.x;
  keyPoseCur.yaw_org = odometry->twist.twist.angular.y;
  keyPoseCur.time = 100.0 * (int(odometry->pose.covariance[1]) + double(odometry->pose.covariance[2]));
  keyPoseCur.ind = odometry->pose.covariance[0];

  keyPose->push_back(keyPoseCur);

  printf("%s received: %d\n", file_name.c_str(), int(keyPose->points.size()));
}

void keyPosePathHandler(const nav_msgs::Path::ConstPtr& path)
{
  if (systemFinished) {
    return;
  }

  int keyPoseNum = keyPose->points.size();
  for (int i = 0; i < keyPoseNum; i++) {
    int ind = keyPose->points[i].ind;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = path->poses[ind].pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    keyPose->points[i].x = path->poses[ind].pose.position.z;
    keyPose->points[i].y = path->poses[ind].pose.position.x;
    keyPose->points[i].z = path->poses[ind].pose.position.y;
    keyPose->points[i].roll = roll;
    keyPose->points[i].pitch = -pitch;
    keyPose->points[i].yaw = -yaw;
  }
}

void saveData()
{
  int keyPoseNum = keyPose->points.size();
  if (keyPoseNum <= 0) {
    return;
  }

  string name = save_data_dir + "/" + file_name + ".ply";
  FILE *fileVar = fopen(name.c_str(), "w");

  fprintf(fileVar, "ply\n");
  fprintf(fileVar, "format ascii 1.0\n");
  fprintf(fileVar, "element vertex %d\n", keyPoseNum);
  fprintf(fileVar, "property float x\n");
  fprintf(fileVar, "property float y\n");
  fprintf(fileVar, "property float z\n");
  fprintf(fileVar, "property float roll\n");
  fprintf(fileVar, "property float pitch\n");
  fprintf(fileVar, "property float yaw\n");
  fprintf(fileVar, "property float x_org\n");
  fprintf(fileVar, "property float y_org\n");
  fprintf(fileVar, "property float z_org\n");
  fprintf(fileVar, "property float roll_org\n");
  fprintf(fileVar, "property float pitch_org\n");
  fprintf(fileVar, "property float yaw_org\n");
  fprintf(fileVar, "property double time\n");
  fprintf(fileVar, "end_header\n");

  for (int i = 0; i < keyPoseNum; i++) {
    fprintf(fileVar, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n", keyPose->points[i].x, keyPose->points[i].y, 
            keyPose->points[i].z, keyPose->points[i].roll, keyPose->points[i].pitch, keyPose->points[i].yaw, 
            keyPose->points[i].x_org, keyPose->points[i].y_org, keyPose->points[i].z_org, keyPose->points[i].roll_org, 
            keyPose->points[i].pitch_org, keyPose->points[i].yaw_org, keyPose->points[i].time);
  }

  fclose(fileVar);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyPoseReceiver");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("save_data_dir", save_data_dir);
  nhPrivate.getParam("file_name", file_name);
  nhPrivate.getParam("key_pose_topic_name", key_pose_topic_name);
  nhPrivate.getParam("key_pose_path_topic_name", key_pose_path_topic_name);

  ros::Subscriber SubKeyPose = nh.subscribe<nav_msgs::Odometry>
                               (key_pose_topic_name, 50, keyPoseHandler);

  ros::Subscriber SubKeyPosePath = nh.subscribe<nav_msgs::Path>
                                   (key_pose_path_topic_name, 5, keyPosePathHandler);

  //printf("\nAwaiting for data...\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    double curTime = ros::Time::now().toSec();
    if (systemInited && curTime - odometryTime > 3600.0) {
      break;
    }

    status = ros::ok();
    rate.sleep();
  }

  systemFinished = true;
  saveData();

  //printf("\nKeypose files saved.\n\n");

  return 0;
}
