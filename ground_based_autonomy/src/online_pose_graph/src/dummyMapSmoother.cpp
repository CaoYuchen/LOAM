#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

double poseSkipDis = 0.05;
int poseStackNum = 20;

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

pcl::PointCloud<PosePoint>::Ptr keyPose(new pcl::PointCloud<PosePoint>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMap(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr relativePose(new pcl::PointCloud<pcl::PointXYZHSV>());

PosePoint keyPoseCur;
bool newKeyPose = false;
bool acceptLaserCloud = true;
bool localInited = false;

float poseXLast = -1000000;
float poseYLast = -1000000;
float poseZLast = -1000000;
int poseCount = 0;

float transformSum[6] = {0};
float transformMapped[6] = {0};
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

  transformMapped[0] = pitch;
  transformMapped[1] = yaw;
  transformMapped[2] = roll;
  transformMapped[3] = tx;
  transformMapped[4] = ty;
  transformMapped[5] = tz;
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

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (acceptLaserCloud) {
    laserCloudIn->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

    *stackMap += *laserCloudIn;

    int laserCloudInNum = laserCloudIn->points.size();
    int stackMapNum = stackMap->points.size();

    pcl::PointXYZHSV pose;
    pose.x = poseXLast;
    pose.y = poseYLast;
    pose.z = poseZLast;
    pose.h = stackMapNum - laserCloudInNum;
    pose.s = stackMapNum - 1;

    relativePose->push_back(pose);

    poseCount--;
  }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometry)
{
  double time = odometry->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  float x = odometry->pose.pose.position.x;
  float y = odometry->pose.pose.position.y;
  float z = odometry->pose.pose.position.z;

  float disX = x - poseXLast;
  float disY = y - poseYLast;
  float disZ = z - poseZLast;

  if (disX * disX + disY * disY + disZ * disZ > poseSkipDis * poseSkipDis) {
    poseXLast = x;
    poseYLast = y;
    poseZLast = z;
    acceptLaserCloud = true;
  } else {
    acceptLaserCloud = false;
  }

  if (poseCount <= 0) {
    keyPoseCur.x = x;
    keyPoseCur.y = y;
    keyPoseCur.z = z;
    keyPoseCur.roll = roll;
    keyPoseCur.pitch = -pitch;
    keyPoseCur.yaw = -yaw;
    keyPoseCur.time = time;

    keyPose->push_back(keyPoseCur);

    poseCount = poseStackNum;

    newKeyPose = true;
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

  localInited = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummyMapSmoother");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("poseSkipDis", poseSkipDis);
  nhPrivate.getParam("poseStackNum", poseStackNum);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/velodyne_cloud_registered", 5, laserCloudHandler);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/aft_mapped_to_init", 5, odometryHandler);

  ros::Subscriber subLocalization = nh.subscribe<nav_msgs::Odometry>
                                    ("/aft_local_to_map", 5, localizationHandler);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_key_pose", 2);

  ros::Publisher pubRelativePose = nh.advertise<sensor_msgs::PointCloud2> ("/relative_pose_to_key_pose", 2);

  ros::Publisher pubKeyPose = nh.advertise<nav_msgs::Odometry> ("/key_pose_to_map", 5);
  nav_msgs::Odometry keyPoseOdom;
  keyPoseOdom.header.frame_id = "map_rot";
  keyPoseOdom.child_frame_id = "key_pose";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform keyPoseTrans;
  keyPoseTrans.frame_id_ = "map_rot";
  keyPoseTrans.child_frame_id_ = "key_pose";

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newKeyPose) {
      newKeyPose = false;

      int keyPoseID = keyPose->points.size() - 1;
      float srx1 = sin(keyPose->points[keyPoseID].pitch);
      float crx1 = cos(keyPose->points[keyPoseID].pitch);
      float sry1 = sin(keyPose->points[keyPoseID].yaw);
      float cry1 = cos(keyPose->points[keyPoseID].yaw);
      float srz1 = sin(keyPose->points[keyPoseID].roll);
      float crz1 = cos(keyPose->points[keyPoseID].roll);
      float tx1 = keyPose->points[keyPoseID].x;
      float ty1 = keyPose->points[keyPoseID].y;
      float tz1 = keyPose->points[keyPoseID].z;

      int stackMapNum = stackMap->points.size();
      for (int i = 0; i < stackMapNum; i++) {
        pointAssociateTobeMapped(&stackMap->points[i], &stackMap->points[i],
                                 srx1, crx1, sry1, cry1, srz1, crz1, tx1, ty1, tz1);
      }

      pcl::PointXYZ point;
      int relativePoseNum = relativePose->points.size();
      for (int i = 0; i < relativePoseNum; i++) {
        point.x = relativePose->points[i].x;
        point.y = relativePose->points[i].y;
        point.z = relativePose->points[i].z;

        pointAssociateTobeMapped(&point, &point, srx1, crx1, sry1, cry1, srz1, crz1, tx1, ty1, tz1);

        relativePose->points[i].x = point.x;
        relativePose->points[i].y = point.y;
        relativePose->points[i].z = point.z;
        relativePose->points[i].v = keyPoseID;
      }

      if (localInited) {
        transformSum[0] = keyPose->points[keyPoseID].pitch;
        transformSum[1] = keyPose->points[keyPoseID].yaw;
        transformSum[2] = keyPose->points[keyPoseID].roll;
        transformSum[3] = keyPose->points[keyPoseID].x;
        transformSum[4] = keyPose->points[keyPoseID].y;
        transformSum[5] = keyPose->points[keyPoseID].z;

        transformAssociateToMap();
      } else {
        transformMapped[0] = keyPose->points[keyPoseID].pitch;
        transformMapped[1] = keyPose->points[keyPoseID].yaw;
        transformMapped[2] = keyPose->points[keyPoseID].roll;
        transformMapped[3] = keyPose->points[keyPoseID].x;
        transformMapped[4] = keyPose->points[keyPoseID].y;
        transformMapped[5] = keyPose->points[keyPoseID].z;
      }

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                          (transformMapped[2], -transformMapped[0], -transformMapped[1]);

      keyPoseOdom.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      keyPoseOdom.pose.pose.orientation.x = -geoQuat.y;
      keyPoseOdom.pose.pose.orientation.y = -geoQuat.z;
      keyPoseOdom.pose.pose.orientation.z = geoQuat.x;
      keyPoseOdom.pose.pose.orientation.w = geoQuat.w;
      keyPoseOdom.pose.pose.position.x = transformMapped[3];
      keyPoseOdom.pose.pose.position.y = transformMapped[4];
      keyPoseOdom.pose.pose.position.z = transformMapped[5];
      keyPoseOdom.twist.twist.angular.x = keyPose->points[keyPoseID].pitch;
      keyPoseOdom.twist.twist.angular.y = keyPose->points[keyPoseID].yaw;
      keyPoseOdom.twist.twist.angular.z = keyPose->points[keyPoseID].roll;
      keyPoseOdom.twist.twist.linear.x = keyPose->points[keyPoseID].x;
      keyPoseOdom.twist.twist.linear.y = keyPose->points[keyPoseID].y;
      keyPoseOdom.twist.twist.linear.z = keyPose->points[keyPoseID].z;
      keyPoseOdom.pose.covariance[0] = keyPoseID;
      pubKeyPose.publish(keyPoseOdom);

      keyPoseTrans.stamp_ = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      keyPoseTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      keyPoseTrans.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
      tfBroadcaster.sendTransform(keyPoseTrans);

      sensor_msgs::PointCloud2 stackMap3;
      pcl::toROSMsg(*stackMap, stackMap3);
      stackMap3.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      stackMap3.header.frame_id = "key_pose";
      pubLaserCloud.publish(stackMap3);

      sensor_msgs::PointCloud2 relativePose2;
      pcl::toROSMsg(*relativePose, relativePose2);
      relativePose2.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      relativePose2.header.frame_id = "key_pose";
      pubRelativePose.publish(relativePose2);

      stackMap->clear();
      relativePose->clear();
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
