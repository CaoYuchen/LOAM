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

int pointCloudNum = 2;
string pointcloud_in;
string trajectory_in;
string key_pose_in;
string pointcloud_out;
string trajectory_out;

FILE *ptcd_in_file;
FILE *traj_in_file;
FILE *ptcd_out_file;
FILE *traj_out_file;

int readTrajHeader()
{
  if (traj_in_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, trajPointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(traj_in_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(traj_in_file, "%d", &trajPointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return trajPointNum;
}

int readPtcdHeader()
{
  if (ptcd_in_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, ptcdPointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(ptcd_in_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(ptcd_in_file, "%d", &ptcdPointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return ptcdPointNum;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LoopClosureStacker");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pointCloudNum", pointCloudNum);
  nhPrivate.getParam("pointcloud_in", pointcloud_in);
  nhPrivate.getParam("trajectory_in", trajectory_in);
  nhPrivate.getParam("pointcloud_out", pointcloud_out);
  nhPrivate.getParam("trajectory_out", trajectory_out);

  char id_name[10];
  int ptcdPointNumAll = 0;
  int trajPointNumAll = 0;
  for (int pointCloudCount = 0; pointCloudCount < pointCloudNum; pointCloudCount++) {
    sprintf(id_name, "%d", pointCloudCount + 1);

    string ptcd_in_name = pointcloud_in + id_name + ".ply";
    ptcd_in_file = fopen(ptcd_in_name.c_str(), "r");
    ptcdPointNumAll += readPtcdHeader();
    fclose(ptcd_in_file);

    string traj_in_name = trajectory_in + id_name + ".ply";
    traj_in_file = fopen(traj_in_name.c_str(), "r");
    trajPointNumAll += readTrajHeader();
    fclose(traj_in_file);
  }

  ptcd_out_file = fopen(pointcloud_out.c_str(), "w");
  traj_out_file = fopen(trajectory_out.c_str(), "w");

  printf ("\nSaving files...\n\n");

  fprintf(ptcd_out_file, "ply\n");
  fprintf(ptcd_out_file, "format ascii 1.0\n");
  fprintf(ptcd_out_file, "element vertex %d\n", ptcdPointNumAll);
  fprintf(ptcd_out_file, "property float x\n");
  fprintf(ptcd_out_file, "property float y\n");
  fprintf(ptcd_out_file, "property float z\n");
  fprintf(ptcd_out_file, "property float intensity\n");
  fprintf(ptcd_out_file, "property double time\n");
  fprintf(ptcd_out_file, "end_header\n");

  fprintf(traj_out_file, "ply\n");
  fprintf(traj_out_file, "format ascii 1.0\n");
  fprintf(traj_out_file, "element vertex %d\n", trajPointNumAll + 100 * (pointCloudNum - 1));
  fprintf(traj_out_file, "property float x\n");
  fprintf(traj_out_file, "property float y\n");
  fprintf(traj_out_file, "property float z\n");
  fprintf(traj_out_file, "property float roll\n");
  fprintf(traj_out_file, "property float pitch\n");
  fprintf(traj_out_file, "property float yaw\n");
  fprintf(traj_out_file, "property double time\n");
  fprintf(traj_out_file, "end_header\n");

  double time;
  float x, y, z, roll, pitch, yaw, intensity;
  int val1, val2, val3, val4, val5, val6, val7;
  for (int pointCloudCount = 0; pointCloudCount < pointCloudNum; pointCloudCount++) {
    sprintf(id_name, "%d", pointCloudCount + 1);

    string ptcd_in_name = pointcloud_in + id_name + ".ply";
    ptcd_in_file = fopen(ptcd_in_name.c_str(), "r");
    int ptcdPointNum = readPtcdHeader();
    for (int i = 0; i < ptcdPointNum; i++) {
      val1 = fscanf(ptcd_in_file, "%f", &x);
      val2 = fscanf(ptcd_in_file, "%f", &y);
      val3 = fscanf(ptcd_in_file, "%f", &z);
      val4 = fscanf(ptcd_in_file, "%f", &intensity);
      val5 = fscanf(ptcd_in_file, "%lf", &time);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      fprintf(ptcd_out_file, "%f %f %f %f %f\n", x, y, z, intensity, time + 1000000000.0 * pointCloudCount);
    }
    fclose(ptcd_in_file);

    if (pointCloudCount > 0) {
      for (int i = 0; i < 50; i++) {
        fprintf(traj_out_file, "%f %f %f %f %f %f %f\n", 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 1000000000.0 * (pointCloudCount + 1) + 2 * i);
        fprintf(traj_out_file, "%f %f %f %f %f %f %f\n", 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1000000000.0 * (pointCloudCount + 1) + 2 * i + 1.0);
      }
    }

    string traj_in_name = trajectory_in + id_name + ".ply";
    traj_in_file = fopen(traj_in_name.c_str(), "r");
    int trajPointNum = readTrajHeader();
    for (int i = 0; i < trajPointNum; i++) {
      val1 = fscanf(traj_in_file, "%f", &x);
      val2 = fscanf(traj_in_file, "%f", &y);
      val3 = fscanf(traj_in_file, "%f", &z);
      val4 = fscanf(traj_in_file, "%f", &roll);
      val5 = fscanf(traj_in_file, "%f", &pitch);
      val6 = fscanf(traj_in_file, "%f", &yaw);
      val7 = fscanf(traj_in_file, "%lf", &time);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      fprintf(traj_out_file, "%f %f %f %f %f %f %f\n", x, y, z, roll, pitch, yaw, time + 1000000000.0 * pointCloudCount);
    }
    fclose(traj_in_file);

    printf ("Stacked pointcloud %d/%d\n", pointCloudCount + 1, pointCloudNum);
  }

  fclose(ptcd_out_file);
  fclose(traj_out_file);

  printf ("\nComplete.\n\n");

  return 0;
}
