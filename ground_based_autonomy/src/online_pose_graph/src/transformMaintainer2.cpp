#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

float transformSum[6] = {0};
float transformMapped[6] = {0};
float transformMappedRec[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

cv::Mat mappingCov(6, 6, CV_32F, cv::Scalar::all(0));
float localizationCovArray[36] = {0};

ros::Publisher *pubOdomIntegratedPointer = NULL;
ros::Publisher *pubOdomAftMappedPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
nav_msgs::Odometry odometry;
tf::Transform odometryTrans;

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

  transformMapped[0] = pitch;
  transformMapped[1] = yaw;
  transformMapped[2] = roll;
  transformMapped[3] = tx;
  transformMapped[4] = ty;
  transformMapped[5] = tz;

  cv::Mat mat_prior_rot(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat mat_post_rot(6, 6, CV_32F, cv::Scalar::all(0));
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat_prior_rot.at<float>(i, j) = mat_prior_last.at<double>(i, j);
      mat_prior_rot.at<float>(i, j + 3) = mat_prior_last.at<double>(i, j);
      mat_prior_rot.at<float>(i + 3, j) = mat_prior_last.at<double>(i, j);
      mat_prior_rot.at<float>(i + 3, j + 3) = mat_prior_last.at<double>(i, j);

      mat_post_rot.at<float>(i, j) = mat_post_last.at<double>(i, j);
      mat_post_rot.at<float>(i, j + 3) = mat_post_last.at<double>(i, j);
      mat_post_rot.at<float>(i + 3, j) = mat_post_last.at<double>(i, j);
      mat_post_rot.at<float>(i + 3, j + 3) = mat_post_last.at<double>(i, j);
    }
  }

  cv::Mat mat_prior_rot_t(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat mat_post_rot_t(6, 6, CV_32F, cv::Scalar::all(0));
  cv::transpose(mat_prior_rot, mat_prior_rot_t);
  cv::transpose(mat_post_rot, mat_post_rot_t);

  cv::Mat localizationCov = mat_post_rot * mat_prior_rot_t * mappingCov * mat_prior_rot * mat_post_rot_t;

  localizationCovArray[0] = localizationCov.at<float>(2, 2);
  localizationCovArray[1] = localizationCov.at<float>(2, 0);
  localizationCovArray[2] = localizationCov.at<float>(2, 1);
  localizationCovArray[3] = localizationCov.at<float>(2, 5);
  localizationCovArray[4] = localizationCov.at<float>(2, 3);
  localizationCovArray[5] = localizationCov.at<float>(2, 4);
  localizationCovArray[6] = localizationCov.at<float>(0, 2);
  localizationCovArray[7] = localizationCov.at<float>(0, 0);
  localizationCovArray[8] = localizationCov.at<float>(0, 1);
  localizationCovArray[9] = localizationCov.at<float>(0, 5);
  localizationCovArray[10] = localizationCov.at<float>(0, 3);
  localizationCovArray[11] = localizationCov.at<float>(0, 4);
  localizationCovArray[12] = localizationCov.at<float>(1, 2);
  localizationCovArray[13] = localizationCov.at<float>(1, 0);
  localizationCovArray[14] = localizationCov.at<float>(1, 1);
  localizationCovArray[15] = localizationCov.at<float>(1, 5);
  localizationCovArray[16] = localizationCov.at<float>(1, 3);
  localizationCovArray[17] = localizationCov.at<float>(1, 4);
  localizationCovArray[18] = localizationCov.at<float>(5, 2);
  localizationCovArray[19] = localizationCov.at<float>(5, 0);
  localizationCovArray[20] = localizationCov.at<float>(5, 1);
  localizationCovArray[21] = localizationCov.at<float>(5, 5);
  localizationCovArray[22] = localizationCov.at<float>(5, 3);
  localizationCovArray[23] = localizationCov.at<float>(5, 4);
  localizationCovArray[24] = localizationCov.at<float>(3, 2);
  localizationCovArray[25] = localizationCov.at<float>(3, 0);
  localizationCovArray[26] = localizationCov.at<float>(3, 1);
  localizationCovArray[27] = localizationCov.at<float>(3, 5);
  localizationCovArray[28] = localizationCov.at<float>(3, 3);
  localizationCovArray[29] = localizationCov.at<float>(3, 4);
  localizationCovArray[30] = localizationCov.at<float>(4, 2);
  localizationCovArray[31] = localizationCov.at<float>(4, 0);
  localizationCovArray[32] = localizationCov.at<float>(4, 1);
  localizationCovArray[33] = localizationCov.at<float>(4, 5);
  localizationCovArray[34] = localizationCov.at<float>(4, 3);
  localizationCovArray[35] = localizationCov.at<float>(4, 4);
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryIn)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometryIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformSum[0] = pitch;
  transformSum[1] = yaw;
  transformSum[2] = roll;

  transformSum[3] = odometryIn->pose.pose.position.y;
  transformSum[4] = odometryIn->pose.pose.position.z;
  transformSum[5] = odometryIn->pose.pose.position.x;

  mappingCov.at<float>(2, 2) = odometryIn->pose.covariance[0];
  mappingCov.at<float>(2, 0) = odometryIn->pose.covariance[1];
  mappingCov.at<float>(2, 1) = odometryIn->pose.covariance[2];
  mappingCov.at<float>(2, 5) = odometryIn->pose.covariance[3];
  mappingCov.at<float>(2, 3) = odometryIn->pose.covariance[4];
  mappingCov.at<float>(2, 4) = odometryIn->pose.covariance[5];
  mappingCov.at<float>(0, 2) = odometryIn->pose.covariance[6];
  mappingCov.at<float>(0, 0) = odometryIn->pose.covariance[7];
  mappingCov.at<float>(0, 1) = odometryIn->pose.covariance[8];
  mappingCov.at<float>(0, 5) = odometryIn->pose.covariance[9];
  mappingCov.at<float>(0, 3) = odometryIn->pose.covariance[10];
  mappingCov.at<float>(0, 4) = odometryIn->pose.covariance[11];
  mappingCov.at<float>(1, 2) = odometryIn->pose.covariance[12];
  mappingCov.at<float>(1, 0) = odometryIn->pose.covariance[13];
  mappingCov.at<float>(1, 1) = odometryIn->pose.covariance[14];
  mappingCov.at<float>(1, 5) = odometryIn->pose.covariance[15];
  mappingCov.at<float>(1, 3) = odometryIn->pose.covariance[16];
  mappingCov.at<float>(1, 4) = odometryIn->pose.covariance[17];
  mappingCov.at<float>(5, 2) = odometryIn->pose.covariance[18];
  mappingCov.at<float>(5, 0) = odometryIn->pose.covariance[19];
  mappingCov.at<float>(5, 1) = odometryIn->pose.covariance[20];
  mappingCov.at<float>(5, 5) = odometryIn->pose.covariance[21];
  mappingCov.at<float>(5, 3) = odometryIn->pose.covariance[22];
  mappingCov.at<float>(5, 4) = odometryIn->pose.covariance[23];
  mappingCov.at<float>(3, 2) = odometryIn->pose.covariance[24];
  mappingCov.at<float>(3, 0) = odometryIn->pose.covariance[25];
  mappingCov.at<float>(3, 1) = odometryIn->pose.covariance[26];
  mappingCov.at<float>(3, 5) = odometryIn->pose.covariance[27];
  mappingCov.at<float>(3, 3) = odometryIn->pose.covariance[28];
  mappingCov.at<float>(3, 4) = odometryIn->pose.covariance[29];
  mappingCov.at<float>(4, 2) = odometryIn->pose.covariance[30];
  mappingCov.at<float>(4, 0) = odometryIn->pose.covariance[31];
  mappingCov.at<float>(4, 1) = odometryIn->pose.covariance[32];
  mappingCov.at<float>(4, 5) = odometryIn->pose.covariance[33];
  mappingCov.at<float>(4, 3) = odometryIn->pose.covariance[34];
  mappingCov.at<float>(4, 4) = odometryIn->pose.covariance[35];

  transformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            (transformMapped[2], transformMapped[0], transformMapped[1]);

  odometry.header.stamp = odometryIn->header.stamp;
  odometry.header.frame_id = "map";
  odometry.child_frame_id = "sensor";
  odometry.pose.pose.orientation.x = geoQuat.x;
  odometry.pose.pose.orientation.y = geoQuat.y;
  odometry.pose.pose.orientation.z = geoQuat.z;
  odometry.pose.pose.orientation.w = geoQuat.w;
  odometry.pose.pose.position.x = transformMapped[5];
  odometry.pose.pose.position.y = transformMapped[3];
  odometry.pose.pose.position.z = transformMapped[4];
  odometry.twist.twist.angular.x = odometryIn->twist.twist.angular.x;
  odometry.twist.twist.angular.y = odometryIn->twist.twist.angular.y;
  odometry.twist.twist.angular.z = odometryIn->twist.twist.angular.z;
  odometry.twist.twist.linear.x = odometryIn->twist.twist.linear.x;
  odometry.twist.twist.linear.y = odometryIn->twist.twist.linear.y;
  odometry.twist.twist.linear.z = odometryIn->twist.twist.linear.z;

  for (int i = 0; i < 36; i++) {
    odometry.twist.covariance[i] = odometryIn->twist.covariance[i];
    if (odometryIn->pose.covariance[i] == 0) {
      odometry.pose.covariance[i] = 0;
    } else {
      odometry.pose.covariance[i] = localizationCovArray[i];
    }
  }

  pubOdomIntegratedPointer->publish(odometry);

  odometryTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odometryTrans.setOrigin(tf::Vector3(transformMapped[5], transformMapped[3], transformMapped[4]));
  tfBroadcasterPointer->sendTransform(tf::StampedTransform(odometryTrans.inverse(),
                                      odometryIn->header.stamp, "sensor", "map"));
}

void mappingHandler(const nav_msgs::Odometry::ConstPtr& mappingIn)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = mappingIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = mappingIn->pose.pose.position.x;
  transformSum[4] = mappingIn->pose.pose.position.y;
  transformSum[5] = mappingIn->pose.pose.position.z;

  mappingCov.at<float>(0, 0) = mappingIn->pose.covariance[0];
  mappingCov.at<float>(0, 1) = mappingIn->pose.covariance[1];
  mappingCov.at<float>(0, 2) = mappingIn->pose.covariance[2];
  mappingCov.at<float>(0, 3) = mappingIn->pose.covariance[3];
  mappingCov.at<float>(0, 4) = mappingIn->pose.covariance[4];
  mappingCov.at<float>(0, 5) = mappingIn->pose.covariance[5];
  mappingCov.at<float>(1, 0) = mappingIn->pose.covariance[6];
  mappingCov.at<float>(1, 1) = mappingIn->pose.covariance[7];
  mappingCov.at<float>(1, 2) = mappingIn->pose.covariance[8];
  mappingCov.at<float>(1, 3) = mappingIn->pose.covariance[9];
  mappingCov.at<float>(1, 4) = mappingIn->pose.covariance[10];
  mappingCov.at<float>(1, 5) = mappingIn->pose.covariance[11];
  mappingCov.at<float>(2, 0) = mappingIn->pose.covariance[12];
  mappingCov.at<float>(2, 1) = mappingIn->pose.covariance[13];
  mappingCov.at<float>(2, 2) = mappingIn->pose.covariance[14];
  mappingCov.at<float>(2, 3) = mappingIn->pose.covariance[15];
  mappingCov.at<float>(2, 4) = mappingIn->pose.covariance[16];
  mappingCov.at<float>(2, 5) = mappingIn->pose.covariance[17];
  mappingCov.at<float>(3, 0) = mappingIn->pose.covariance[18];
  mappingCov.at<float>(3, 1) = mappingIn->pose.covariance[19];
  mappingCov.at<float>(3, 2) = mappingIn->pose.covariance[20];
  mappingCov.at<float>(3, 3) = mappingIn->pose.covariance[21];
  mappingCov.at<float>(3, 4) = mappingIn->pose.covariance[22];
  mappingCov.at<float>(3, 5) = mappingIn->pose.covariance[23];
  mappingCov.at<float>(4, 0) = mappingIn->pose.covariance[24];
  mappingCov.at<float>(4, 1) = mappingIn->pose.covariance[25];
  mappingCov.at<float>(4, 2) = mappingIn->pose.covariance[26];
  mappingCov.at<float>(4, 3) = mappingIn->pose.covariance[27];
  mappingCov.at<float>(4, 4) = mappingIn->pose.covariance[28];
  mappingCov.at<float>(4, 5) = mappingIn->pose.covariance[29];
  mappingCov.at<float>(5, 0) = mappingIn->pose.covariance[30];
  mappingCov.at<float>(5, 1) = mappingIn->pose.covariance[31];
  mappingCov.at<float>(5, 2) = mappingIn->pose.covariance[32];
  mappingCov.at<float>(5, 3) = mappingIn->pose.covariance[33];
  mappingCov.at<float>(5, 4) = mappingIn->pose.covariance[34];
  mappingCov.at<float>(5, 5) = mappingIn->pose.covariance[35];

  for (int i = 0; i < 6; i++) {
    transformMappedRec[i] = transformMapped[i];
  }

  transformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            (transformMapped[2], -transformMapped[0], -transformMapped[1]);

  odometry.header.stamp = mappingIn->header.stamp;
  odometry.header.frame_id = "map_rot";
  odometry.child_frame_id = "loop_closed";
  odometry.pose.pose.orientation.x = -geoQuat.y;
  odometry.pose.pose.orientation.y = -geoQuat.z;
  odometry.pose.pose.orientation.z = geoQuat.x;
  odometry.pose.pose.orientation.w = geoQuat.w;
  odometry.pose.pose.position.x = transformMapped[3];
  odometry.pose.pose.position.y = transformMapped[4];
  odometry.pose.pose.position.z = transformMapped[5];
  odometry.twist.twist.angular.x = 0;
  odometry.twist.twist.angular.y = 0;
  odometry.twist.twist.angular.z = 0;
  odometry.twist.twist.linear.x = 0;
  odometry.twist.twist.linear.y = 0;
  odometry.twist.twist.linear.z = 0;

  odometry.pose.covariance[0] = localizationCovArray[7];
  odometry.pose.covariance[1] = localizationCovArray[8];
  odometry.pose.covariance[2] = localizationCovArray[6];
  odometry.pose.covariance[3] = localizationCovArray[10];
  odometry.pose.covariance[4] = localizationCovArray[11];
  odometry.pose.covariance[5] = localizationCovArray[9];
  odometry.pose.covariance[6] = localizationCovArray[13];
  odometry.pose.covariance[7] = localizationCovArray[14];
  odometry.pose.covariance[8] = localizationCovArray[12];
  odometry.pose.covariance[9] = localizationCovArray[16];
  odometry.pose.covariance[10] = localizationCovArray[17];
  odometry.pose.covariance[11] = localizationCovArray[15];
  odometry.pose.covariance[12] = localizationCovArray[1];
  odometry.pose.covariance[13] = localizationCovArray[2];
  odometry.pose.covariance[14] = localizationCovArray[0];
  odometry.pose.covariance[15] = localizationCovArray[4];
  odometry.pose.covariance[16] = localizationCovArray[5];
  odometry.pose.covariance[17] = localizationCovArray[3];
  odometry.pose.covariance[18] = localizationCovArray[25];
  odometry.pose.covariance[19] = localizationCovArray[26];
  odometry.pose.covariance[20] = localizationCovArray[24];
  odometry.pose.covariance[21] = localizationCovArray[28];
  odometry.pose.covariance[22] = localizationCovArray[29];
  odometry.pose.covariance[23] = localizationCovArray[27];
  odometry.pose.covariance[24] = localizationCovArray[31];
  odometry.pose.covariance[25] = localizationCovArray[32];
  odometry.pose.covariance[26] = localizationCovArray[30];
  odometry.pose.covariance[27] = localizationCovArray[34];
  odometry.pose.covariance[28] = localizationCovArray[35];
  odometry.pose.covariance[29] = localizationCovArray[33];
  odometry.pose.covariance[30] = localizationCovArray[19];
  odometry.pose.covariance[31] = localizationCovArray[20];
  odometry.pose.covariance[32] = localizationCovArray[18];
  odometry.pose.covariance[33] = localizationCovArray[22];
  odometry.pose.covariance[34] = localizationCovArray[23];
  odometry.pose.covariance[35] = localizationCovArray[21];

  for (int i = 0; i < 36; i++) {
    odometry.twist.covariance[i] = 0;
    if (mappingIn->pose.covariance[i] == 0) {
      odometry.pose.covariance[i] = 0;
    }
  }

  pubOdomAftMappedPointer->publish(odometry);

  odometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  odometryTrans.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
  tfBroadcasterPointer->sendTransform(tf::StampedTransform(odometryTrans,
                                      mappingIn->header.stamp, "map_rot", "loop_closed"));

  for (int i = 0; i < 6; i++) {
    transformMapped[i] = transformMappedRec[i];
  }
}

void localizationHandler(const nav_msgs::Odometry::ConstPtr& localIn)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = localIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = localIn->pose.pose.position.x;
  transformAftMapped[4] = localIn->pose.pose.position.y;
  transformAftMapped[5] = localIn->pose.pose.position.z;

  transformBefMapped[0] = localIn->twist.twist.angular.x;
  transformBefMapped[1] = localIn->twist.twist.angular.y;
  transformBefMapped[2] = localIn->twist.twist.angular.z;

  transformBefMapped[3] = localIn->twist.twist.linear.x;
  transformBefMapped[4] = localIn->twist.twist.linear.y;
  transformBefMapped[5] = localIn->twist.twist.linear.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformMaintainer2");
  ros::NodeHandle nh;

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/integrated_to_init", 50, odometryHandler);

  ros::Subscriber subMapping = nh.subscribe<nav_msgs::Odometry> ("/aft_mapped_to_init", 50, mappingHandler);

  ros::Subscriber subLocalization = nh.subscribe<nav_msgs::Odometry> ("/key_pose_to_map", 5, localizationHandler);

  ros::Publisher pubOdomIntegrated = nh.advertise<nav_msgs::Odometry> ("/integrated_to_map", 50);
  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/loop_closed_to_map", 50);

  pubOdomIntegratedPointer = &pubOdomIntegrated;
  pubOdomAftMappedPointer = &pubOdomAftMapped;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::spin();

  return 0;
}
