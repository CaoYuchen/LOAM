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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

string pointcloud_in;
string trajectory_in;
string key_pose_in;
string pointcloud_out;
string trajectory_out;

struct PosePoint {
     float x, y, z;
     float roll, pitch, yaw;
     double time;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PosePoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, roll, roll)
                                   (float, pitch, pitch)
                                   (float, yaw, yaw)
                                   (double, time, time))

struct keyPosePoint {
     float x, y, z;
     float roll, pitch, yaw;
     float x_org, y_org, z_org;
     float roll_org, pitch_org, yaw_org;
     double time;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (keyPosePoint,
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
                                   (double, time, time))

pcl::PointCloud<pcl::PointXYZ>::Ptr stackMap(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<PosePoint>::Ptr stackPose(new pcl::PointCloud<PosePoint>());
pcl::PointCloud<keyPosePoint>::Ptr keyPose(new pcl::PointCloud<keyPosePoint>());

std::vector<float> stackMapIntensity;
std::vector<double> stackMapTime;

FILE *ptcd_in_file;
FILE *traj_in_file;
FILE *key_pose_in_file;
FILE *ptcd_out_file;
FILE *traj_out_file;

pcl::PointXYZ mapPt;
float mapPtIntensity = 0;
double mapPtTime = 0;
PosePoint posePt;
keyPosePoint keyPosePt;

float transformSum[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

void transformAssociateToMap()
{
  float pitch = transformSum[0];
  float yaw = transformSum[1];
  float roll = transformSum[2];
  float tx = transformSum[3];
  float ty = transformSum[4];
  float tz = transformSum[5];

  cv::Mat mat_prior(4, 4, CV_64F, cv::Scalar::all(0));
  mat_prior.at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
  mat_prior.at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
  mat_prior.at<double>(0, 2) = cos(pitch)*sin(yaw);
  mat_prior.at<double>(1, 0) = cos(pitch)*sin(roll);
  mat_prior.at<double>(1, 1) = cos(pitch)*cos(roll);
  mat_prior.at<double>(1, 2) = -sin(pitch);
  mat_prior.at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
  mat_prior.at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
  mat_prior.at<double>(2, 2) = cos(pitch)*cos(yaw);
  mat_prior.at<double>(0, 3) = tx;
  mat_prior.at<double>(1, 3) = ty;
  mat_prior.at<double>(2, 3) = tz;
  mat_prior.at<double>(3, 3) = 1.0;

  pitch = transformBefMapped[0];
  yaw = transformBefMapped[1];
  roll = transformBefMapped[2];
  tx = transformBefMapped[3];
  ty = transformBefMapped[4];
  tz = transformBefMapped[5];

  cv::Mat mat_prior_last(4, 4, CV_64F, cv::Scalar::all(0));
  mat_prior_last.at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
  mat_prior_last.at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
  mat_prior_last.at<double>(0, 2) = cos(pitch)*sin(yaw);
  mat_prior_last.at<double>(1, 0) = cos(pitch)*sin(roll);
  mat_prior_last.at<double>(1, 1) = cos(pitch)*cos(roll);
  mat_prior_last.at<double>(1, 2) = -sin(pitch);
  mat_prior_last.at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
  mat_prior_last.at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
  mat_prior_last.at<double>(2, 2) = cos(pitch)*cos(yaw);
  mat_prior_last.at<double>(0, 3) = tx;
  mat_prior_last.at<double>(1, 3) = ty;
  mat_prior_last.at<double>(2, 3) = tz;
  mat_prior_last.at<double>(3, 3) = 1.0;

  pitch = transformAftMapped[0];
  yaw = transformAftMapped[1];
  roll = transformAftMapped[2];
  tx = transformAftMapped[3];
  ty = transformAftMapped[4];
  tz = transformAftMapped[5];

  cv::Mat mat_post_last(4, 4, CV_64F, cv::Scalar::all(0));
  mat_post_last.at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
  mat_post_last.at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
  mat_post_last.at<double>(0, 2) = cos(pitch)*sin(yaw);
  mat_post_last.at<double>(1, 0) = cos(pitch)*sin(roll);
  mat_post_last.at<double>(1, 1) = cos(pitch)*cos(roll);
  mat_post_last.at<double>(1, 2) = -sin(pitch);
  mat_post_last.at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
  mat_post_last.at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
  mat_post_last.at<double>(2, 2) = cos(pitch)*cos(yaw);
  mat_post_last.at<double>(0, 3) = tx;
  mat_post_last.at<double>(1, 3) = ty;
  mat_post_last.at<double>(2, 3) = tz;
  mat_post_last.at<double>(3, 3) = 1.0;

  cv::Mat mat_frame = mat_prior_last.inv() * mat_prior;
  cv::Mat mat_post = mat_post_last * mat_frame;

  pitch = -asin(mat_post.at<double>(1, 2));
  roll = atan2(mat_post.at<double>(1, 0) / cos(pitch), mat_post.at<double>(1, 1) / cos(pitch));
  yaw = atan2(mat_post.at<double>(0, 2) / cos(pitch), mat_post.at<double>(2, 2) / cos(pitch));
  tx = mat_post.at<double>(0, 3);
  ty = mat_post.at<double>(1, 3);
  tz = mat_post.at<double>(2, 3);

  transformTobeMapped[0] = pitch;
  transformTobeMapped[1] = yaw;
  transformTobeMapped[2] = roll;
  transformTobeMapped[3] = tx;
  transformTobeMapped[4] = ty;
  transformTobeMapped[5] = tz;
}

void pointAssociateTobeMapped(pcl::PointXYZ *pi, pcl::PointXYZ *po, float srx, float crx, float sry, 
                              float cry, float srz, float crz, float tx, float ty, float tz)
{
  float x1 = cry * (pi->x - tx) - sry * (pi->z - tz);
  float y1 = pi->y - ty;
  float z1 = sry * (pi->x - tx) + cry * (pi->z - tz);

  float x2 = x1;
  float y2 = crx * y1 + srx * z1;
  float z2 = -srx * y1 + crx * z1;

  po->x = crz * x2 + srz * y2;
  po->y = -srz * x2 + crz * y2;
  po->z = z2;
}

void pointAssociateToMap(pcl::PointXYZ *pi, pcl::PointXYZ *po, float srx, float crx, float sry, 
                         float cry, float srz, float crz, float tx, float ty, float tz)
{
  float x1 = crz * pi->x - srz * pi->y;
  float y1 = srz * pi->x + crz * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = crx * y1 - srx * z1;
  float z2 = srx * y1 + crx * z1;

  po->x = cry * x2 + sry * z2 + tx;
  po->y = y2 + ty;
  po->z = -sry * x2 + cry * z2 + tz;
}

int readPlyHeader(FILE *file)
{
  if (file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readKeyPose()
{
  key_pose_in_file = fopen(key_pose_in.c_str(), "r");
  int keyPoseNum = readPlyHeader(key_pose_in_file);

  int val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11, val12, val13;
  for (int i = 0; i < keyPoseNum; i++) {
    val1 = fscanf(key_pose_in_file, "%f", &keyPosePt.z);
    val2 = fscanf(key_pose_in_file, "%f", &keyPosePt.x);
    val3 = fscanf(key_pose_in_file, "%f", &keyPosePt.y);
    val4 = fscanf(key_pose_in_file, "%f", &keyPosePt.roll);
    val5 = fscanf(key_pose_in_file, "%f", &keyPosePt.pitch);
    val6 = fscanf(key_pose_in_file, "%f", &keyPosePt.yaw);
    val7 = fscanf(key_pose_in_file, "%f", &keyPosePt.z_org);
    val8 = fscanf(key_pose_in_file, "%f", &keyPosePt.x_org);
    val9 = fscanf(key_pose_in_file, "%f", &keyPosePt.y_org);
    val10 = fscanf(key_pose_in_file, "%f", &keyPosePt.roll_org);
    val11 = fscanf(key_pose_in_file, "%f", &keyPosePt.pitch_org);
    val12 = fscanf(key_pose_in_file, "%f", &keyPosePt.yaw_org);
    val13 = fscanf(key_pose_in_file, "%lf", &keyPosePt.time);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1 || 
        val8 != 1 || val9 != 1 || val10 != 1 || val11 != 1 || val12 != 1 || val13 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    keyPose->push_back(keyPosePt);
  }

  fclose(key_pose_in_file);
}

void readPtcdStackMap(int keyPoseID, int ptcdPointNum, int &ptcdPointCount)
{
  stackMap->clear();
  stackMapIntensity.clear();
  stackMapTime.clear();
  if (keyPoseID > 0) {
    stackMap->push_back(mapPt);
    stackMapIntensity.push_back(mapPtIntensity);
    stackMapTime.push_back(mapPtTime);
    ptcdPointCount++;
  }

  int val1, val2, val3, val4, val5;
  int keyPoseNum = keyPose->points.size();
  if (keyPoseID == keyPoseNum - 1) {
    while (ptcdPointCount < ptcdPointNum) {
      val1 = fscanf(ptcd_in_file, "%f", &mapPt.z);
      val2 = fscanf(ptcd_in_file, "%f", &mapPt.x);
      val3 = fscanf(ptcd_in_file, "%f", &mapPt.y);
      val4 = fscanf(ptcd_in_file, "%f", &mapPtIntensity);
      val5 = fscanf(ptcd_in_file, "%lf", &mapPtTime);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      stackMap->push_back(mapPt);
      stackMapIntensity.push_back(mapPtIntensity);
      stackMapTime.push_back(mapPtTime);
      ptcdPointCount++;
    }
  } else {
    while (ptcdPointCount < ptcdPointNum) {
      val1 = fscanf(ptcd_in_file, "%f", &mapPt.z);
      val2 = fscanf(ptcd_in_file, "%f", &mapPt.x);
      val3 = fscanf(ptcd_in_file, "%f", &mapPt.y);
      val4 = fscanf(ptcd_in_file, "%f", &mapPtIntensity);
      val5 = fscanf(ptcd_in_file, "%lf", &mapPtTime);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      if (mapPtTime <= keyPose->points[keyPoseID].time) {
        stackMap->push_back(mapPt);
        stackMapIntensity.push_back(mapPtIntensity);
        stackMapTime.push_back(mapPtTime);
        ptcdPointCount++;
      } else {
        break;
      }
    }
  }
}

void readTrajStackPose(int keyPoseID, int trajPointNum, int &trajPointCount)
{
  stackPose->clear();
  if (keyPoseID > 0) {
    stackPose->push_back(posePt);
    trajPointCount++;
  }

  int val1, val2, val3, val4, val5, val6, val7;
  int keyPoseNum = keyPose->points.size();
  if (keyPoseID == keyPoseNum - 1) {
    while (trajPointCount < trajPointNum) {
      val1 = fscanf(traj_in_file, "%f", &posePt.z);
      val2 = fscanf(traj_in_file, "%f", &posePt.x);
      val3 = fscanf(traj_in_file, "%f", &posePt.y);
      val4 = fscanf(traj_in_file, "%f", &posePt.roll);
      val5 = fscanf(traj_in_file, "%f", &posePt.pitch);
      val6 = fscanf(traj_in_file, "%f", &posePt.yaw);
      val7 = fscanf(traj_in_file, "%lf", &posePt.time);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      stackPose->push_back(posePt);
      trajPointCount++;
    }
  } else {
    while (trajPointCount < trajPointNum) {
      val1 = fscanf(traj_in_file, "%f", &posePt.z);
      val2 = fscanf(traj_in_file, "%f", &posePt.x);
      val3 = fscanf(traj_in_file, "%f", &posePt.y);
      val4 = fscanf(traj_in_file, "%f", &posePt.roll);
      val5 = fscanf(traj_in_file, "%f", &posePt.pitch);
      val6 = fscanf(traj_in_file, "%f", &posePt.yaw);
      val7 = fscanf(traj_in_file, "%lf", &posePt.time);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }

      if (posePt.time <= keyPose->points[keyPoseID].time) {
        stackPose->push_back(posePt);
        trajPointCount++;
      } else {
        break;
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointCloudGenerator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pointcloud_in", pointcloud_in);
  nhPrivate.getParam("trajectory_in", trajectory_in);
  nhPrivate.getParam("key_pose_in", key_pose_in);
  nhPrivate.getParam("pointcloud_out", pointcloud_out);
  nhPrivate.getParam("trajectory_out", trajectory_out);

  readKeyPose();
  int keyPoseNum = keyPose->points.size();

  ptcd_in_file = fopen(pointcloud_in.c_str(), "r");
  int ptcdPointNum = readPlyHeader(ptcd_in_file);
  int ptcdPointCount = 0;

  traj_in_file = fopen(trajectory_in.c_str(), "r");
  int trajPointNum = readPlyHeader(traj_in_file);
  int trajPointCount = 0;

  ptcd_out_file = fopen(pointcloud_out.c_str(), "w");
  traj_out_file = fopen(trajectory_out.c_str(), "w");

  printf ("\nSaving files...\n");

  fprintf(ptcd_out_file, "ply\n");
  fprintf(ptcd_out_file, "format ascii 1.0\n");
  fprintf(ptcd_out_file, "element vertex %d\n", ptcdPointNum);
  fprintf(ptcd_out_file, "property float x\n");
  fprintf(ptcd_out_file, "property float y\n");
  fprintf(ptcd_out_file, "property float z\n");
  fprintf(ptcd_out_file, "property float intensity\n");
  fprintf(ptcd_out_file, "property double time\n");
  fprintf(ptcd_out_file, "end_header\n");

  fprintf(traj_out_file, "ply\n");
  fprintf(traj_out_file, "format ascii 1.0\n");
  fprintf(traj_out_file, "element vertex %d\n", trajPointNum);
  fprintf(traj_out_file, "property float x\n");
  fprintf(traj_out_file, "property float y\n");
  fprintf(traj_out_file, "property float z\n");
  fprintf(traj_out_file, "property float roll\n");
  fprintf(traj_out_file, "property float pitch\n");
  fprintf(traj_out_file, "property float yaw\n");
  fprintf(traj_out_file, "property double time\n");
  fprintf(traj_out_file, "end_header\n");

  for (int keyPoseID = 0; keyPoseID < keyPoseNum; keyPoseID++) {
    readPtcdStackMap(keyPoseID, ptcdPointNum, ptcdPointCount);

    float srx1 = sin(keyPose->points[keyPoseID].pitch_org);
    float crx1 = cos(keyPose->points[keyPoseID].pitch_org);
    float sry1 = sin(keyPose->points[keyPoseID].yaw_org);
    float cry1 = cos(keyPose->points[keyPoseID].yaw_org);
    float srz1 = sin(keyPose->points[keyPoseID].roll_org);
    float crz1 = cos(keyPose->points[keyPoseID].roll_org);
    float tx1 = keyPose->points[keyPoseID].x_org;
    float ty1 = keyPose->points[keyPoseID].y_org;
    float tz1 = keyPose->points[keyPoseID].z_org;

    float srx2 = sin(keyPose->points[keyPoseID].pitch);
    float crx2 = cos(keyPose->points[keyPoseID].pitch);
    float sry2 = sin(keyPose->points[keyPoseID].yaw);
    float cry2 = cos(keyPose->points[keyPoseID].yaw);
    float srz2 = sin(keyPose->points[keyPoseID].roll);
    float crz2 = cos(keyPose->points[keyPoseID].roll);
    float tx2 = keyPose->points[keyPoseID].x;
    float ty2 = keyPose->points[keyPoseID].y;
    float tz2 = keyPose->points[keyPoseID].z;

    int stackMapNum = stackMap->points.size();
    for (int i = 0; i < stackMapNum; i++) {
      pointAssociateTobeMapped(&stackMap->points[i], &stackMap->points[i], 
                               srx1, crx1, sry1, cry1, srz1, crz1, tx1, ty1, tz1);
      pointAssociateToMap(&stackMap->points[i], &stackMap->points[i], 
                          srx2, crx2, sry2, cry2, srz2, crz2, tx2, ty2, tz2);

      fprintf(ptcd_out_file, "%f %f %f %f %f\n", stackMap->points[i].z, stackMap->points[i].x, 
                                                 stackMap->points[i].y, stackMapIntensity[i],
                                                 stackMapTime[i]);
    }

    readTrajStackPose(keyPoseID, trajPointNum, trajPointCount);

    transformBefMapped[0] = keyPose->points[keyPoseID].pitch_org;
    transformBefMapped[1] = keyPose->points[keyPoseID].yaw_org;
    transformBefMapped[2] = keyPose->points[keyPoseID].roll_org;
    transformBefMapped[3] = keyPose->points[keyPoseID].x_org;
    transformBefMapped[4] = keyPose->points[keyPoseID].y_org;
    transformBefMapped[5] = keyPose->points[keyPoseID].z_org;

    transformAftMapped[0] = keyPose->points[keyPoseID].pitch;
    transformAftMapped[1] = keyPose->points[keyPoseID].yaw;
    transformAftMapped[2] = keyPose->points[keyPoseID].roll;
    transformAftMapped[3] = keyPose->points[keyPoseID].x;
    transformAftMapped[4] = keyPose->points[keyPoseID].y;
    transformAftMapped[5] = keyPose->points[keyPoseID].z;

    int stackPoseNum = stackPose->points.size();
    for (int i = 0; i < stackPoseNum; i++) {
      transformSum[0] = stackPose->points[i].pitch;
      transformSum[1] = stackPose->points[i].yaw;
      transformSum[2] = stackPose->points[i].roll;
      transformSum[3] = stackPose->points[i].x;
      transformSum[4] = stackPose->points[i].y;
      transformSum[5] = stackPose->points[i].z;

      transformAssociateToMap();

      stackPose->points[i].pitch = transformTobeMapped[0];
      stackPose->points[i].yaw = transformTobeMapped[1];
      stackPose->points[i].roll = transformTobeMapped[2];
      stackPose->points[i].x = transformTobeMapped[3];
      stackPose->points[i].y = transformTobeMapped[4];
      stackPose->points[i].z = transformTobeMapped[5];

      fprintf(traj_out_file, "%f %f %f %f %f %f %f\n", stackPose->points[i].z, stackPose->points[i].x, 
              stackPose->points[i].y, stackPose->points[i].roll, stackPose->points[i].pitch,
              stackPose->points[i].yaw, stackPose->points[i].time);
    }
  }

  fclose(ptcd_in_file);
  fclose(traj_in_file);

  fclose(ptcd_out_file);
  fclose(traj_out_file);

  printf ("\nComplete.\n\n");

  return 0;
}
