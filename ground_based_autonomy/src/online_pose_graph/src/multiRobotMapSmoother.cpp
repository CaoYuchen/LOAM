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

#include "isam/isam.h"
#include "isam/robust.h"

using namespace std;
using namespace isam;
using namespace Eigen;

double poseSkipDis = 0.05;
int poseStackNum = 20;
int optIterNum = 30;
int transOptIterNum = 0;
int optSkipKeyPoseNum = 0;
int minPointNum = 2000;
double oriUpdateRate = 1.0;
double searchDis = 4.5;
double matchDis = 4.0;
double planeVertDis = 0.2;
double voxelSize = 0.4;
double matchRegionHori = 15.0;
double matchRegionVert = 10.0;
double adjMapRegionHori = 25.0;
double adjMapRegionVert = 10.0;
double matchNeglectRegHori = 25.0;
double matchNeglectRegVert = 10.0;
bool matchWEqualVert = false;
bool matchNearestKeyPose = false;
bool matchFromOwnKeyPose = false;
int matchSkipKeyPoseNum = 0;

bool sensorFwd = true;
double rollFwdCorr = 0;
double rollBwdCorr = 0;
double pitchFwdCorr = 0;
double pitchBwdCorr = 0;
double yawFwdCorr = 0;
double yawBwdCorr = 0;
double vertFwdCorr = 0;
double vertBwdCorr = 0;
double scaleCorr = 0;

bool sensorFwdS1 = true;
double rollFwdCorrS1 = 0;
double rollBwdCorrS1 = 0;
double pitchFwdCorrS1 = 0;
double pitchBwdCorrS1 = 0;
double yawFwdCorrS1 = 0;
double yawBwdCorrS1 = 0;
double vertFwdCorrS1 = 0;
double vertBwdCorrS1 = 0;
double scaleCorrS1 = 0;

bool sensorFwdS2 = true;
double rollFwdCorrS2 = 0;
double rollBwdCorrS2 = 0;
double pitchFwdCorrS2 = 0;
double pitchBwdCorrS2 = 0;
double yawFwdCorrS2 = 0;
double yawBwdCorrS2 = 0;
double vertFwdCorrS2 = 0;
double vertBwdCorrS2 = 0;
double scaleCorrS2 = 0;

bool sensorFwdS3 = true;
double rollFwdCorrS3 = 0;
double rollBwdCorrS3 = 0;
double pitchFwdCorrS3 = 0;
double pitchBwdCorrS3 = 0;
double yawFwdCorrS3 = 0;
double yawBwdCorrS3 = 0;
double vertFwdCorrS3 = 0;
double vertBwdCorrS3 = 0;
double scaleCorrS3 = 0;

bool usePoseInitS1 = false;
double poseInitXS1 = 0;
double poseInitYS1 = 0;
double poseInitZS1 = 0;
double poseInitYawS1 = 0;
double poseInitPitchS1 = 0;
double poseInitRollS1 = 0;

bool usePoseInitS2 = false;
double poseInitXS2 = 0;
double poseInitYS2 = 0;
double poseInitZS2 = 0;
double poseInitYawS2 = 0;
double poseInitPitchS2 = 0;
double poseInitRollS2 = 0;

bool usePoseInitS3 = false;
double poseInitXS3 = 0;
double poseInitYS3 = 0;
double poseInitZS3 = 0;
double poseInitYawS3 = 0;
double poseInitPitchS3 = 0;
double poseInitRollS3 = 0;

struct PosePoint {
     float x, y, z;
     float roll, pitch, yaw;
     double time;
     int mapPtStartID, mapPtNum;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PosePoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, roll, roll)
                                   (float, pitch, pitch)
                                   (float, yaw, yaw)
                                   (double, time, time)
                                   (int, mapPtStartID, mapPtStartID)
                                   (int, mapPtNum, mapPtNum))

pcl::PointCloud<PosePoint>::Ptr keyPose(new pcl::PointCloud<PosePoint>());
pcl::PointCloud<PosePoint>::Ptr keyPoseAdj(new pcl::PointCloud<PosePoint>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMap(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMap2(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMapS1(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMapS2(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMapS3(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr stackMapArray(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr relativePose(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZ>::Ptr adjMap(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr adjMapDS(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr adjMapExtract(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseMap(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseMapOri(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseMapReg(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeMap(new pcl::KdTreeFLANN<pcl::PointXYZ>());

std::vector<int> keyPoseIDArray;
std::vector<int> keyPoseIDArrayS1;
std::vector<int> keyPoseIDArrayS2;
std::vector<int> keyPoseIDArrayS3;
std::vector<int> keyPoseIDArrayAll;

int laserVoxelCenWidth = 30;
int laserVoxelCenHeight = 30;
int laserVoxelCenDepth = 30;
const int laserVoxelWidth = 61;
const int laserVoxelHeight = 61;
const int laserVoxelDepth = 61;
const int laserVoxelNum = laserVoxelWidth * laserVoxelHeight * laserVoxelDepth;

int laserVoxelInd[laserVoxelNum];

PosePoint keyPoseCur;
PosePoint keyPoseCurS1;
PosePoint keyPoseCurS2;
PosePoint keyPoseCurS3;

bool newKeyPose = false;
bool newKeyPoseS1 = false;
bool newKeyPoseS2 = false;
bool newKeyPoseS3 = false;

bool newlaserCloudS1 = false;
bool newlaserCloudS2 = false;
bool newlaserCloudS3 = false;

double laserCloudTimeS1 = 0;
double laserCloudTimeS2 = 0;
double laserCloudTimeS3 = 0;

bool acceptLaserCloud = true;

float poseXLast = -1000000;
float poseYLast = -1000000;
float poseZLast = -1000000;
int poseCount = 0;

float transformSum[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

int optKeyPoseRecIDArrayNum[4] = {0};
int matchKeyPoseRecIDArrayNum[4] = {0};
float keyPoseRecX[4] = {0};
float keyPoseRecY[4] = {0};
float keyPoseRecZ[4] = {0};
int adjacentMatch[4] = {1};

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

void diffTrans(float cx, float cy, float cz, float lx, float ly, float lz, 
               float &ox, float &oy, float &oz, float pitch, float yaw, float roll)
{
  float x0 = cx - lx;
  float y0 = cy - ly;
  float z0 = cz - lz;

  float x1 = cos(yaw) * x0 - sin(yaw) * z0;
  float y1 = y0;
  float z1 = sin(yaw) * x0 + cos(yaw) * z0;

  float x2 = x1;
  float y2 = cos(pitch) * y1 + sin(pitch) * z1;
  float z2 = -sin(pitch) * y1 + cos(pitch) * z1;

  ox = cos(roll) * x2 + sin(roll) * y2;
  oy = -sin(roll) * x2 + cos(roll) * y2;
  oz = z2;
}

void diffRot(float cx, float cy, float cz, float lx, float ly, float lz, 
             float &ox, float &oy, float &oz)
{
  float srx = cos(cx)*cos(cy)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx)) 
            - cos(cx)*sin(cy)*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly)) - cos(lx)*cos(lz)*sin(cx);
  ox = -asin(srx);

  float srycrx = cos(cx)*sin(cy)*(cos(ly)*cos(lz) + sin(lx)*sin(ly)*sin(lz)) 
               - cos(cx)*cos(cy)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) - cos(lx)*sin(cx)*sin(lz);
  float crycrx = sin(cx)*sin(lx) + cos(cx)*cos(cy)*cos(lx)*cos(ly) + cos(cx)*cos(lx)*sin(cy)*sin(ly);
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = cos(cx)*cos(lx)*cos(lz)*sin(cz) - (cos(cz)*sin(cy) 
               - cos(cy)*sin(cx)*sin(cz))*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx)) 
               - (cos(cy)*cos(cz) + sin(cx)*sin(cy)*sin(cz))*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly));
  float crzcrx = (sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx))*(sin(ly)*sin(lz) 
               + cos(ly)*cos(lz)*sin(lx)) + (cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy))*(cos(ly)*sin(lz) 
               - cos(lz)*sin(lx)*sin(ly)) + cos(cx)*cos(cz)*cos(lx)*cos(lz);
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
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

void laserCloudHandlerS1(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudTimeS1 = laserCloud2->header.stamp.toSec();
  pcl::fromROSMsg(*laserCloud2, *stackMapS1);

  newlaserCloudS1 = true;
}

void laserCloudHandlerS2(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudTimeS2 = laserCloud2->header.stamp.toSec();
  pcl::fromROSMsg(*laserCloud2, *stackMapS2);

  newlaserCloudS2 = true;
}

void laserCloudHandlerS3(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  laserCloudTimeS3 = laserCloud2->header.stamp.toSec();
  pcl::fromROSMsg(*laserCloud2, *stackMapS3);

  newlaserCloudS3 = true;
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

    poseCount = poseStackNum;
    newKeyPose = true;
  }
}

void keyPoseHandlerS1(const nav_msgs::Odometry::ConstPtr& keyPose)
{
  keyPoseCurS1.x = keyPose->twist.twist.linear.x;
  keyPoseCurS1.y = keyPose->twist.twist.linear.y;
  keyPoseCurS1.z = keyPose->twist.twist.linear.z;
  keyPoseCurS1.roll = keyPose->twist.twist.angular.z;
  keyPoseCurS1.pitch = keyPose->twist.twist.angular.x;
  keyPoseCurS1.yaw = keyPose->twist.twist.angular.y;
  keyPoseCurS1.time = keyPose->header.stamp.toSec();

  newKeyPoseS1 = true;
}

void keyPoseHandlerS2(const nav_msgs::Odometry::ConstPtr& keyPose)
{
  keyPoseCurS2.x = keyPose->twist.twist.linear.x;
  keyPoseCurS2.y = keyPose->twist.twist.linear.y;
  keyPoseCurS2.z = keyPose->twist.twist.linear.z;
  keyPoseCurS2.roll = keyPose->twist.twist.angular.z;
  keyPoseCurS2.pitch = keyPose->twist.twist.angular.x;
  keyPoseCurS2.yaw = keyPose->twist.twist.angular.y;
  keyPoseCurS2.time = keyPose->header.stamp.toSec();

  newKeyPoseS2 = true;
}

void keyPoseHandlerS3(const nav_msgs::Odometry::ConstPtr& keyPose)
{
  keyPoseCurS3.x = keyPose->twist.twist.linear.x;
  keyPoseCurS3.y = keyPose->twist.twist.linear.y;
  keyPoseCurS3.z = keyPose->twist.twist.linear.z;
  keyPoseCurS3.roll = keyPose->twist.twist.angular.z;
  keyPoseCurS3.pitch = keyPose->twist.twist.angular.x;
  keyPoseCurS3.yaw = keyPose->twist.twist.angular.y;
  keyPoseCurS3.time = keyPose->header.stamp.toSec();

  newKeyPoseS3 = true;
}

int scanMatching()
{
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::PointXYZ pointOri, pointSel, pointProj;
  pcl::PointXYZI coeff;

  cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  for (int i = 0; i < laserVoxelNum; i++) {
    laserVoxelInd[i] = 0;
  }

  float srx2 = sin(transformTobeMapped[0]);
  float crx2 = cos(transformTobeMapped[0]);
  float sry2 = sin(transformTobeMapped[1]);
  float cry2 = cos(transformTobeMapped[1]);
  float srz2 = sin(transformTobeMapped[2]);
  float crz2 = cos(transformTobeMapped[2]);
  float tx2 = transformTobeMapped[3];
  float ty2 = transformTobeMapped[4];
  float tz2 = transformTobeMapped[5];

  int keyPoseMapNum = keyPoseMap->points.size();
  for (int i = 0; i < keyPoseMapNum; i++) {
    pointAssociateToMap(&keyPoseMap->points[i], &pointSel, 
                        srx2, crx2, sry2, cry2, srz2, crz2, tx2, ty2, tz2);
    int voxelI = int((pointSel.x - transformTobeMapped[3] + 5.0) / 10.0) + laserVoxelCenWidth;
    int voxelJ = int((pointSel.y - transformTobeMapped[4] + 5.0) / 10.0) + laserVoxelCenHeight;
    int voxelK = int((pointSel.z - transformTobeMapped[5] + 5.0) / 10.0) + laserVoxelCenDepth;

    if (pointSel.x - transformTobeMapped[3] + 5.0 < 0) voxelI--;
    if (pointSel.y - transformTobeMapped[4] + 5.0 < 0) voxelJ--;
    if (pointSel.z - transformTobeMapped[5] + 5.0 < 0) voxelK--;

    for (int ii = voxelI - 1; ii <= voxelI + 1; ii++) {
      for (int jj = voxelJ - 1; jj <= voxelJ + 1; jj++) {
        for (int kk = voxelK - 1; kk <= voxelK + 1; kk++) {
          if (ii >= 0 && ii < laserVoxelWidth && jj >= 0 && 
              jj < laserVoxelHeight && kk >= 0 && kk < laserVoxelDepth) {
            laserVoxelInd[ii + laserVoxelWidth * jj + laserVoxelWidth * laserVoxelHeight * kk] = 1;
          }
        }
      }
    }
  }

  adjMapExtract->clear();
  int adjMapDSNum = adjMapDS->points.size();
  for (int i = 0; i < adjMapDSNum; i++) {
    int voxelI = int((adjMapDS->points[i].x - transformTobeMapped[3] + 5.0) / 10.0) + laserVoxelCenWidth;
    int voxelJ = int((adjMapDS->points[i].y - transformTobeMapped[4] + 5.0) / 10.0) + laserVoxelCenHeight;
    int voxelK = int((adjMapDS->points[i].z - transformTobeMapped[5] + 5.0) / 10.0) + laserVoxelCenDepth;

    if (adjMapDS->points[i].x - transformTobeMapped[3] + 5.0 < 0) voxelI--;
    if (adjMapDS->points[i].y - transformTobeMapped[4] + 5.0 < 0) voxelJ--;
    if (adjMapDS->points[i].z - transformTobeMapped[5] + 5.0 < 0) voxelK--;

    if (voxelI >= 0 && voxelI < laserVoxelWidth && voxelJ >= 0 && 
        voxelJ < laserVoxelHeight && voxelK >= 0 && voxelK < laserVoxelDepth) {
      int voxelID = voxelI + laserVoxelWidth * voxelJ + laserVoxelWidth * laserVoxelHeight * voxelK;
      if (laserVoxelInd[voxelID] == 1) {
        adjMapExtract->push_back(adjMapDS->points[i]);
      }
    }
  }

  int adjMapExtractNum = adjMapExtract->points.size();
  if (adjMapExtractNum > minPointNum) {
    kdtreeMap->setInputCloud(adjMapExtract);

    int* searchIndArray = new int[5 * keyPoseMapNum];

    int pointSelSkipNum = 1;
    for (int iterCount = 0; iterCount < optIterNum; iterCount++) {
      keyPoseMapOri->clear();
      coeffSel->clear();

      bool isPointSel = false;
      if (iterCount % (pointSelSkipNum + 1) == 0) {
        isPointSel = true;
      }

      float srx2 = sin(transformTobeMapped[0]);
      float crx2 = cos(transformTobeMapped[0]);
      float sry2 = sin(transformTobeMapped[1]);
      float cry2 = cos(transformTobeMapped[1]);
      float srz2 = sin(transformTobeMapped[2]);
      float crz2 = cos(transformTobeMapped[2]);
      float tx2 = transformTobeMapped[3];
      float ty2 = transformTobeMapped[4];
      float tz2 = transformTobeMapped[5];

      for (int i = 0; i < keyPoseMapNum; i++) {
        pointOri = keyPoseMap->points[i];
        pointAssociateToMap(&pointOri, &pointSel, srx2, crx2, sry2, cry2, srz2, crz2, tx2, ty2, tz2);

        if (isPointSel) {
          kdtreeMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
          for (int j = 0; j < 5; j++) {
            searchIndArray[5 * i + j] = pointSearchInd[j];
          }
        } else {
          pointSearchInd.resize(5);
          pointSearchSqDis.resize(5);
          for (int j = 0; j < 5; j++) {
            int ind = searchIndArray[5 * i + j];
            float sx = adjMapExtract->points[ind].x;
            float sy = adjMapExtract->points[ind].y;
            float sz = adjMapExtract->points[ind].z;
            pointSearchInd[j] = ind;
            pointSearchSqDis[j] = (pointSel.x - sx) * (pointSel.x - sx) + (pointSel.y - sy) * (pointSel.y - sy)
                                + (pointSel.z - sz) * (pointSel.z - sz);
          }
        }

        if (pointSearchSqDis[4] < searchDis) {
          for (int j = 0; j < 5; j++) {
            matA0.at<float>(j, 0) = adjMapExtract->points[pointSearchInd[j]].x;
            matA0.at<float>(j, 1) = adjMapExtract->points[pointSearchInd[j]].y;
            matA0.at<float>(j, 2) = adjMapExtract->points[pointSearchInd[j]].z;
          }
          cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

          float pa = matX0.at<float>(0, 0);
          float pb = matX0.at<float>(1, 0);
          float pc = matX0.at<float>(2, 0);
          float pd = 1;
 
          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          bool planeValid = true;
          for (int j = 0; j < 5; j++) {
            if (fabs(pa * adjMapExtract->points[pointSearchInd[j]].x +
                pb * adjMapExtract->points[pointSearchInd[j]].y +
                pc * adjMapExtract->points[pointSearchInd[j]].z + pd) > planeVertDis) {
              planeValid = false;
              break;
            }
          }

          if (planeValid) {
            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

            pointProj = pointSel;
            pointProj.x -= pa * pd2;
            pointProj.y -= pb * pd2;
            pointProj.z -= pc * pd2;

            coeff.x = pa;
            coeff.y = pb;
            coeff.z = pc;
            coeff.intensity = pd2;

            if (fabs(pd2) < matchDis) {
              keyPoseMapOri->push_back(pointOri);
              coeffSel->push_back(coeff);
            }
          }
        }
      }

      float srx = sin(transformTobeMapped[0]);
      float crx = cos(transformTobeMapped[0]);
      float sry = sin(transformTobeMapped[1]);
      float cry = cos(transformTobeMapped[1]);
      float srz = sin(transformTobeMapped[2]);
      float crz = cos(transformTobeMapped[2]);

      int keyPoseMapOriNum = keyPoseMapOri->points.size();
      if (keyPoseMapOriNum < minPointNum) {
        return 0;
      }

      cv::Mat matA(keyPoseMapOriNum, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matAt(6, keyPoseMapOriNum, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matB(keyPoseMapOriNum, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
      for (int i = 0; i < keyPoseMapOriNum; i++) {
        pointOri = keyPoseMapOri->points[i];
        coeff = coeffSel->points[i];

        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                  + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                  + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                  + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                  + ((-cry*crz - srx*sry*srz)*pointOri.x 
                  + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                  + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                  + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arz;
        matA.at<float>(i, 3) = coeff.x;
        matA.at<float>(i, 4) = coeff.y;
        matA.at<float>(i, 5) = coeff.z;
        matB.at<float>(i, 0) = -coeff.intensity;
      }
      cv::transpose(matA, matAt);
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

      if (iterCount == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
          if (matE.at<float>(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2.at<float>(i, j) = 0;
            }
            isDegenerate = true;
          } else {
            break;
          }
        }
        matP = matV.inv() * matV2;
      }

      if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
      }

      if (matX.at<float>(0, 0) < 10 && matX.at<float>(0, 0) > -10 && 
          matX.at<float>(1, 0) < 10 && matX.at<float>(1, 0) > -10 && 
          matX.at<float>(2, 0) < 10 && matX.at<float>(2, 0) > -10 && 
          matX.at<float>(3, 0) < 10 && matX.at<float>(3, 0) > -10 && 
          matX.at<float>(4, 0) < 10 && matX.at<float>(4, 0) > -10 && 
          matX.at<float>(5, 0) < 10 && matX.at<float>(5, 0) > -10) {

        if (iterCount >= transOptIterNum) {
          transformTobeMapped[0] += oriUpdateRate * matX.at<float>(0, 0);
          transformTobeMapped[1] += oriUpdateRate * matX.at<float>(1, 0);
          transformTobeMapped[2] += oriUpdateRate * matX.at<float>(2, 0);
        }

        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);
      }

      float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
                   + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
                   + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
      float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
                   + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
                   + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

      if (deltaR < 0.05 && deltaT < 0.05) {
        break;
      }
    }

    delete[] searchIndArray;
  } else {
    return 0;
  }

  return 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiRobotMapSmoother");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("poseSkipDis", poseSkipDis);
  nhPrivate.getParam("poseStackNum", poseStackNum);
  nhPrivate.getParam("optIterNum", optIterNum);
  nhPrivate.getParam("transOptIterNum", transOptIterNum);
  nhPrivate.getParam("optSkipKeyPoseNum", optSkipKeyPoseNum);
  nhPrivate.getParam("minPointNum", minPointNum);
  nhPrivate.getParam("oriUpdateRate", oriUpdateRate);
  nhPrivate.getParam("searchDis", searchDis);
  nhPrivate.getParam("matchDis", matchDis);
  nhPrivate.getParam("planeVertDis", planeVertDis);
  nhPrivate.getParam("voxelSize", voxelSize);
  nhPrivate.getParam("matchRegionHori", matchRegionHori);
  nhPrivate.getParam("matchRegionVert", matchRegionVert);
  nhPrivate.getParam("adjMapRegionHori", adjMapRegionHori);
  nhPrivate.getParam("adjMapRegionVert", adjMapRegionVert);
  nhPrivate.getParam("matchNeglectRegHori", matchNeglectRegHori);
  nhPrivate.getParam("matchNeglectRegVert", matchNeglectRegVert);
  nhPrivate.getParam("matchWEqualVert", matchWEqualVert);
  nhPrivate.getParam("matchNearestKeyPose", matchNearestKeyPose);
  nhPrivate.getParam("matchFromOwnKeyPose", matchFromOwnKeyPose);
  nhPrivate.getParam("matchSkipKeyPoseNum", matchSkipKeyPoseNum);
  nhPrivate.getParam("sensorFwd", sensorFwd);
  nhPrivate.getParam("rollFwdCorr", rollFwdCorr);
  nhPrivate.getParam("rollBwdCorr", rollBwdCorr);
  nhPrivate.getParam("pitchFwdCorr", pitchFwdCorr);
  nhPrivate.getParam("pitchBwdCorr", pitchBwdCorr);
  nhPrivate.getParam("yawFwdCorr", yawFwdCorr);
  nhPrivate.getParam("yawBwdCorr", yawBwdCorr);
  nhPrivate.getParam("vertFwdCorr", vertFwdCorr);
  nhPrivate.getParam("vertBwdCorr", vertBwdCorr);
  nhPrivate.getParam("scaleCorr", scaleCorr);
  nhPrivate.getParam("sensorFwdS1", sensorFwdS1);
  nhPrivate.getParam("rollFwdCorrS1", rollFwdCorrS1);
  nhPrivate.getParam("rollBwdCorrS1", rollBwdCorrS1);
  nhPrivate.getParam("pitchFwdCorrS1", pitchFwdCorrS1);
  nhPrivate.getParam("pitchBwdCorrS1", pitchBwdCorrS1);
  nhPrivate.getParam("yawFwdCorrS1", yawFwdCorrS1);
  nhPrivate.getParam("yawBwdCorrS1", yawBwdCorrS1);
  nhPrivate.getParam("vertFwdCorrS1", vertFwdCorrS1);
  nhPrivate.getParam("vertBwdCorrS1", vertBwdCorrS1);
  nhPrivate.getParam("scaleCorrS1", scaleCorrS1);
  nhPrivate.getParam("sensorFwdS2", sensorFwdS2);
  nhPrivate.getParam("rollFwdCorrS2", rollFwdCorrS2);
  nhPrivate.getParam("rollBwdCorrS2", rollBwdCorrS2);
  nhPrivate.getParam("pitchFwdCorrS2", pitchFwdCorrS2);
  nhPrivate.getParam("pitchBwdCorrS2", pitchBwdCorrS2);
  nhPrivate.getParam("yawFwdCorrS2", yawFwdCorrS2);
  nhPrivate.getParam("yawBwdCorrS2", yawBwdCorrS2);
  nhPrivate.getParam("vertFwdCorrS2", vertFwdCorrS2);
  nhPrivate.getParam("vertBwdCorrS2", vertBwdCorrS2);
  nhPrivate.getParam("scaleCorrS2", scaleCorrS2);
  nhPrivate.getParam("sensorFwdS3", sensorFwdS3);
  nhPrivate.getParam("rollFwdCorrS3", rollFwdCorrS3);
  nhPrivate.getParam("rollBwdCorrS3", rollBwdCorrS3);
  nhPrivate.getParam("pitchFwdCorrS3", pitchFwdCorrS3);
  nhPrivate.getParam("pitchBwdCorrS3", pitchBwdCorrS3);
  nhPrivate.getParam("yawFwdCorrS3", yawFwdCorrS3);
  nhPrivate.getParam("yawBwdCorrS3", yawBwdCorrS3);
  nhPrivate.getParam("vertFwdCorrS3", vertFwdCorrS3);
  nhPrivate.getParam("vertBwdCorrS3", vertBwdCorrS3);
  nhPrivate.getParam("scaleCorrS3", scaleCorrS3);
  nhPrivate.getParam("usePoseInitS1", usePoseInitS1);
  nhPrivate.getParam("poseInitXS1", poseInitXS1);
  nhPrivate.getParam("poseInitYS1", poseInitYS1);
  nhPrivate.getParam("poseInitZS1", poseInitZS1);
  nhPrivate.getParam("poseInitYawS1", poseInitYawS1);
  nhPrivate.getParam("poseInitPitchS1", poseInitPitchS1);
  nhPrivate.getParam("poseInitRollS1", poseInitRollS1);
  nhPrivate.getParam("usePoseInitS2", usePoseInitS2);
  nhPrivate.getParam("poseInitXS2", poseInitXS2);
  nhPrivate.getParam("poseInitYS2", poseInitYS2);
  nhPrivate.getParam("poseInitZS2", poseInitZS2);
  nhPrivate.getParam("poseInitYawS2", poseInitYawS2);
  nhPrivate.getParam("poseInitPitchS2", poseInitPitchS2);
  nhPrivate.getParam("poseInitRollS2", poseInitRollS2);
  nhPrivate.getParam("usePoseInitS3", usePoseInitS3);
  nhPrivate.getParam("poseInitXS3", poseInitXS3);
  nhPrivate.getParam("poseInitYS3", poseInitYS3);
  nhPrivate.getParam("poseInitZS3", poseInitZS3);
  nhPrivate.getParam("poseInitYawS3", poseInitYawS3);
  nhPrivate.getParam("poseInitPitchS3", poseInitPitchS3);
  nhPrivate.getParam("poseInitRollS3", poseInitRollS3);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/velodyne_cloud_registered", 5, laserCloudHandler);

  ros::Subscriber subLaserCloudS1 = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/velodyne_cloud_key_pose_s1", 2, laserCloudHandlerS1);

  ros::Subscriber subLaserCloudS2 = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/velodyne_cloud_key_pose_s2", 2, laserCloudHandlerS2);

  ros::Subscriber subLaserCloudS3 = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/velodyne_cloud_key_pose_s3", 2, laserCloudHandlerS3);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/aft_mapped_to_init", 5, odometryHandler);

  ros::Subscriber subKeyPoseS1 = nh.subscribe<nav_msgs::Odometry>
                                 ("/key_pose_to_map_org_s1", 5, keyPoseHandlerS1);

  ros::Subscriber subKeyPoseS2 = nh.subscribe<nav_msgs::Odometry>
                                 ("/key_pose_to_map_org_s2", 5, keyPoseHandlerS2);

  ros::Subscriber subKeyPoseS3 = nh.subscribe<nav_msgs::Odometry>
                                 ("/key_pose_to_map_org_s3", 5, keyPoseHandlerS3);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_key_pose", 2);

  //ros::Publisher pubLaserCloudS1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_key_pose_resend_s1", 2);

  //ros::Publisher pubLaserCloudS2 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_key_pose_resend_s2", 2);

  //ros::Publisher pubLaserCloudS3 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_key_pose_resend_s3", 2);

  ros::Publisher pubRelativePose = nh.advertise<sensor_msgs::PointCloud2> ("/relative_pose_to_key_pose", 2);

  ros::Publisher pubMapCloud = nh.advertise<sensor_msgs::PointCloud2> ("/map_cloud", 2);

  ros::Publisher pubKeyPose = nh.advertise<nav_msgs::Odometry> ("/key_pose_to_map", 5);

  ros::Publisher pubKeyPoseS1 = nh.advertise<nav_msgs::Odometry> ("/key_pose_to_map_s1", 5);

  ros::Publisher pubKeyPoseS2 = nh.advertise<nav_msgs::Odometry> ("/key_pose_to_map_s2", 5);

  ros::Publisher pubKeyPoseS3 = nh.advertise<nav_msgs::Odometry> ("/key_pose_to_map_s3", 5);

  nav_msgs::Odometry keyPoseOdom;
  keyPoseOdom.header.frame_id = "map_rot";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform keyPoseTrans;
  keyPoseTrans.frame_id_ = "map_rot";

  ros::Publisher pubKeyPosePath = nh.advertise<nav_msgs::Path> ("/key_pose_path", 5);
  nav_msgs::Path keyPosePath;
  keyPosePath.header.frame_id = "map_rot";

  ros::Publisher pubKeyPosePathS1 = nh.advertise<nav_msgs::Path> ("/key_pose_path_s1", 5);
  nav_msgs::Path keyPosePathS1;
  keyPosePathS1.header.frame_id = "map_rot";

  ros::Publisher pubKeyPosePathS2 = nh.advertise<nav_msgs::Path> ("/key_pose_path_s2", 5);
  nav_msgs::Path keyPosePathS2;
  keyPosePathS2.header.frame_id = "map_rot";

  ros::Publisher pubKeyPosePathS3 = nh.advertise<nav_msgs::Path> ("/key_pose_path_s3", 5);
  nav_msgs::Path keyPosePathS3;
  keyPosePathS3.header.frame_id = "map_rot";

  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
  downSizeFilter.setLeafSize(voxelSize, voxelSize, voxelSize);

  std::vector<int> *keyPoseIDArrayPtr;

  MatrixXd infoInit;

  Slam iSAM;
  vector<Pose3d_Node*> iSAMPoses;
  vector<Pose3d_Node*> iSAMGPSPoses;

  Properties prop = iSAM.properties();
  prop.method = DOG_LEG;
  iSAM.set_properties(prop);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    int procID = -1;

    if (newKeyPose) {
      newKeyPose = false;

      float srx1 = sin(keyPoseCur.pitch);
      float crx1 = cos(keyPoseCur.pitch);
      float sry1 = sin(keyPoseCur.yaw);
      float cry1 = cos(keyPoseCur.yaw);
      float srz1 = sin(keyPoseCur.roll);
      float crz1 = cos(keyPoseCur.roll);
      float tx1 = keyPoseCur.x;
      float ty1 = keyPoseCur.y;
      float tz1 = keyPoseCur.z;

      int stackMapNum = stackMap->points.size();
      for (int i = 0; i < stackMapNum; i++) {
        pointAssociateTobeMapped(&stackMap->points[i], &stackMap->points[i],
                                 srx1, crx1, sry1, cry1, srz1, crz1, tx1, ty1, tz1);
      }

      keyPoseMap->clear();
      downSizeFilter.setInputCloud(stackMap);
      downSizeFilter.filter(*keyPoseMap);

      keyPoseCur.mapPtStartID = stackMapArray->points.size();
      keyPoseCur.mapPtNum = keyPoseMap->points.size();

      *stackMapArray += *keyPoseMap;

      int keyPoseID = keyPose->points.size();
      keyPoseIDArray.push_back(keyPoseID);
      keyPoseIDArrayPtr = &keyPoseIDArray;

      procID = 0;
      keyPoseIDArrayAll.push_back(procID);

      keyPose->push_back(keyPoseCur);
      keyPoseAdj->push_back(keyPoseCur);

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
    } else if (newlaserCloudS1 && newKeyPoseS1 && fabs(laserCloudTimeS1 - keyPoseCurS1.time) < 0.005) {
      newlaserCloudS1 = false;
      newKeyPoseS1 = false;

      keyPoseMap->clear();
      downSizeFilter.setInputCloud(stackMapS1);
      downSizeFilter.filter(*keyPoseMap);

      keyPoseCurS1.mapPtStartID = stackMapArray->points.size();
      keyPoseCurS1.mapPtNum = keyPoseMap->points.size();

      *stackMapArray += *keyPoseMap;

      keyPoseIDArrayS1.push_back(keyPose->points.size());
      keyPoseIDArrayPtr = &keyPoseIDArrayS1;

      procID = 1;
      keyPoseIDArrayAll.push_back(procID);

      keyPose->push_back(keyPoseCurS1);
      keyPoseAdj->push_back(keyPoseCurS1);

      if (keyPoseIDArrayPtr->size() <= 1 && usePoseInitS1) {
        int keyPoseAdjSize = keyPose->points.size();
        keyPoseAdj->points[keyPoseAdjSize - 1].x = poseInitYS1;
        keyPoseAdj->points[keyPoseAdjSize - 1].y = poseInitZS1;
        keyPoseAdj->points[keyPoseAdjSize - 1].z = poseInitXS1;
        keyPoseAdj->points[keyPoseAdjSize - 1].roll = poseInitRollS1;
        keyPoseAdj->points[keyPoseAdjSize - 1].pitch = poseInitPitchS1;
        keyPoseAdj->points[keyPoseAdjSize - 1].yaw = poseInitYawS1;
      }
    } else if (newlaserCloudS2 && newKeyPoseS2 && fabs(laserCloudTimeS2 - keyPoseCurS2.time) < 0.005) {
      newlaserCloudS2 = false;
      newKeyPoseS2 = false;

      keyPoseMap->clear();
      downSizeFilter.setInputCloud(stackMapS2);
      downSizeFilter.filter(*keyPoseMap);

      keyPoseCurS2.mapPtStartID = stackMapArray->points.size();
      keyPoseCurS2.mapPtNum = keyPoseMap->points.size();

      *stackMapArray += *keyPoseMap;

      keyPoseIDArrayS2.push_back(keyPose->points.size());
      keyPoseIDArrayPtr = &keyPoseIDArrayS2;

      procID = 2;
      keyPoseIDArrayAll.push_back(procID);

      keyPose->push_back(keyPoseCurS2);
      keyPoseAdj->push_back(keyPoseCurS2);

      if (keyPoseIDArrayPtr->size() <= 1 && usePoseInitS2) {
        int keyPoseAdjSize = keyPose->points.size();
        keyPoseAdj->points[keyPoseAdjSize - 1].x = poseInitYS2;
        keyPoseAdj->points[keyPoseAdjSize - 1].y = poseInitZS2;
        keyPoseAdj->points[keyPoseAdjSize - 1].z = poseInitXS2;
        keyPoseAdj->points[keyPoseAdjSize - 1].roll = poseInitRollS2;
        keyPoseAdj->points[keyPoseAdjSize - 1].pitch = poseInitPitchS2;
        keyPoseAdj->points[keyPoseAdjSize - 1].yaw = poseInitYawS2;
      }
    } else if (newlaserCloudS3 && newKeyPoseS3 && fabs(laserCloudTimeS3 - keyPoseCurS3.time) < 0.005) {
      newlaserCloudS3 = false;
      newKeyPoseS3 = false;

      keyPoseMap->clear();
      downSizeFilter.setInputCloud(stackMapS3);
      downSizeFilter.filter(*keyPoseMap);

      keyPoseCurS3.mapPtStartID = stackMapArray->points.size();
      keyPoseCurS3.mapPtNum = keyPoseMap->points.size();

      *stackMapArray += *keyPoseMap;

      keyPoseIDArrayS3.push_back(keyPose->points.size());
      keyPoseIDArrayPtr = &keyPoseIDArrayS3;

      procID = 3;
      keyPoseIDArrayAll.push_back(procID);

      keyPose->push_back(keyPoseCurS3);
      keyPoseAdj->push_back(keyPoseCurS3);

      if (keyPoseIDArrayPtr->size() <= 1 && usePoseInitS3) {
        int keyPoseAdjSize = keyPose->points.size();
        keyPoseAdj->points[keyPoseAdjSize - 1].x = poseInitYS3;
        keyPoseAdj->points[keyPoseAdjSize - 1].y = poseInitZS3;
        keyPoseAdj->points[keyPoseAdjSize - 1].z = poseInitXS3;
        keyPoseAdj->points[keyPoseAdjSize - 1].roll = poseInitRollS3;
        keyPoseAdj->points[keyPoseAdjSize - 1].pitch = poseInitPitchS3;
        keyPoseAdj->points[keyPoseAdjSize - 1].yaw = poseInitYawS3;
      }
    }

    if (procID >= 0) {
      int keyPoseID = keyPose->points.size() - 1;
      int keyPoseIDPre = -1;
      int keyPoseIDArrayPtrNum = keyPoseIDArrayPtr->size();

      if (keyPoseIDArrayPtrNum <= 1) {
        Pose3d_Node* pose = new Pose3d_Node();
        iSAM.add_node(pose);
        iSAMPoses.push_back(pose);

        if (procID == 0) {
          infoInit = 10000.0 * eye(6);
        } else {
          infoInit = 0.0001 * eye(6);
        }

        Pose3d_Factor* poseFactor = new Pose3d_Factor(iSAMPoses[keyPoseID], Pose3d(keyPoseAdj->points[keyPoseID].z, 
                                                      keyPoseAdj->points[keyPoseID].x, keyPoseAdj->points[keyPoseID].y, 
                                                      keyPoseAdj->points[keyPoseID].yaw, keyPoseAdj->points[keyPoseID].pitch, 
                                                      keyPoseAdj->points[keyPoseID].roll), Information(infoInit));
        iSAM.add_factor(poseFactor);
      } else {
        keyPoseIDPre = (*keyPoseIDArrayPtr)[keyPoseIDArrayPtrNum - 2];

        transformSum[0] = keyPose->points[keyPoseID].pitch;
        transformSum[1] = keyPose->points[keyPoseID].yaw;
        transformSum[2] = keyPose->points[keyPoseID].roll;
        transformSum[3] = keyPose->points[keyPoseID].x;
        transformSum[4] = keyPose->points[keyPoseID].y;
        transformSum[5] = keyPose->points[keyPoseID].z;

        transformBefMapped[0] = keyPose->points[keyPoseIDPre].pitch;
        transformBefMapped[1] = keyPose->points[keyPoseIDPre].yaw;
        transformBefMapped[2] = keyPose->points[keyPoseIDPre].roll;
        transformBefMapped[3] = keyPose->points[keyPoseIDPre].x;
        transformBefMapped[4] = keyPose->points[keyPoseIDPre].y;
        transformBefMapped[5] = keyPose->points[keyPoseIDPre].z;

        transformAftMapped[0] = keyPoseAdj->points[keyPoseIDPre].pitch;
        transformAftMapped[1] = keyPoseAdj->points[keyPoseIDPre].yaw;
        transformAftMapped[2] = keyPoseAdj->points[keyPoseIDPre].roll;
        transformAftMapped[3] = keyPoseAdj->points[keyPoseIDPre].x;
        transformAftMapped[4] = keyPoseAdj->points[keyPoseIDPre].y;
        transformAftMapped[5] = keyPoseAdj->points[keyPoseIDPre].z;

        transformAssociateToMap();

        float diffX = transformTobeMapped[3] - transformAftMapped[3];
        float diffY = transformTobeMapped[4] - transformAftMapped[4];
        float diffZ = transformTobeMapped[5] - transformAftMapped[5];
        float dis = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

        float srx1 = sin(keyPose->points[keyPoseIDPre].pitch);
        float crx1 = cos(keyPose->points[keyPoseIDPre].pitch);
        float sry1 = sin(keyPose->points[keyPoseIDPre].yaw);
        float cry1 = cos(keyPose->points[keyPoseIDPre].yaw);
        float srz1 = sin(keyPose->points[keyPoseIDPre].roll);
        float crz1 = cos(keyPose->points[keyPoseIDPre].roll);
        float tx1 = keyPose->points[keyPoseIDPre].x;
        float ty1 = keyPose->points[keyPoseIDPre].y;
        float tz1 = keyPose->points[keyPoseIDPre].z;

        pcl::PointXYZ point;
        point.x = keyPose->points[keyPoseID].x;
        point.y = keyPose->points[keyPoseID].y;
        point.z = keyPose->points[keyPoseID].z;

        pointAssociateTobeMapped(&point, &point, srx1, crx1, sry1, cry1, srz1, crz1, tx1, ty1, tz1);

        if (procID == 0) {
          if ((sensorFwd && point.z > 0) || (!sensorFwd && point.x > 0)) {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchFwdCorr;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawFwdCorr;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollFwdCorr;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorr;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorr + dis * vertFwdCorr;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorr;
          } else {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchBwdCorr;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawBwdCorr;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollBwdCorr;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorr;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorr + dis * vertBwdCorr;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorr;
          }
        } else if (procID == 1) {
          if ((sensorFwdS1 && point.z > 0) || (!sensorFwdS1 && point.x > 0)) {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchFwdCorrS1;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawFwdCorrS1;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollFwdCorrS1;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorrS1;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorrS1 + dis * vertFwdCorrS1;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorrS1;
          } else {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchBwdCorrS1;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawBwdCorrS1;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollBwdCorrS1;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorrS1;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorrS1 + dis * vertBwdCorrS1;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorrS1;
          }
        } else if (procID == 2) {
          if ((sensorFwdS2 && point.z > 0) || (!sensorFwdS2 && point.x > 0)) {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchFwdCorrS2;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawFwdCorrS2;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollFwdCorrS2;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorrS2;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorrS2 + dis * vertFwdCorrS2;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorrS2;
          } else {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchBwdCorrS2;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawBwdCorrS2;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollBwdCorrS2;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorrS2;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorrS2 + dis * vertBwdCorrS2;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorrS2;
          }
        } else if (procID == 3) {
          if ((sensorFwdS3 && point.z > 0) || (!sensorFwdS3 && point.x > 0)) {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchFwdCorrS3;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawFwdCorrS3;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollFwdCorrS3;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorrS3;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorrS3 + dis * vertFwdCorrS3;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorrS3;
          } else {
            keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0] + dis * pitchBwdCorrS3;
            keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1] + dis * yawBwdCorrS3;
            keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2] + dis * rollBwdCorrS3;
            keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3] + diffX * scaleCorrS3;
            keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4] + diffY * scaleCorrS3 + dis * vertBwdCorrS3;
            keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5] + diffZ * scaleCorrS3;
          }
        }

        float roll, pitch, yaw, tx, ty, tz;
        diffTrans(keyPoseAdj->points[keyPoseID].x, keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z, 
                  keyPoseAdj->points[keyPoseIDPre].x, keyPoseAdj->points[keyPoseIDPre].y, keyPoseAdj->points[keyPoseIDPre].z, 
                  tx, ty, tz, keyPoseAdj->points[keyPoseIDPre].pitch, keyPoseAdj->points[keyPoseIDPre].yaw, 
                  keyPoseAdj->points[keyPoseIDPre].roll);
        diffRot(keyPoseAdj->points[keyPoseID].pitch, keyPoseAdj->points[keyPoseID].yaw, keyPoseAdj->points[keyPoseID].roll,
                keyPoseAdj->points[keyPoseIDPre].pitch, keyPoseAdj->points[keyPoseIDPre].yaw, 
                keyPoseAdj->points[keyPoseIDPre].roll, pitch, yaw, roll);

        Pose3d_Node* pose = new Pose3d_Node();
        iSAM.add_node(pose);
        iSAMPoses.push_back(pose);

        Pose3d_Pose3d_Factor* poseposeFactor = new Pose3d_Pose3d_Factor(iSAMPoses[keyPoseIDPre], iSAMPoses[keyPoseID], 
                                                   Pose3d(tz, tx, ty, yaw, pitch, roll), Information(100.0 * eye(6)));
        iSAM.add_factor(poseposeFactor);
      }

      bool adjacentPose = true;
      bool scanMatched = false;
      for (int i = keyPoseID - 1; i >= 0; i--) {
        float disX = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[i].x;
        float disY = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[i].y;
        float disZ = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[i].z;
        float disSq = disX * disX + disZ * disZ;

        if (disSq < matchRegionHori * matchRegionHori && fabs(disY) < matchRegionVert) {
          if (i != keyPoseIDPre && procID == keyPoseIDArrayAll[i] && adjacentPose) {
            float roll, pitch, yaw, tx, ty, tz;
            diffTrans(keyPoseAdj->points[keyPoseID].x, keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z, 
                      keyPoseAdj->points[i].x, keyPoseAdj->points[i].y, keyPoseAdj->points[i].z, tx, ty, tz, 
                      keyPoseAdj->points[i].pitch, keyPoseAdj->points[i].yaw, keyPoseAdj->points[i].roll);
            diffRot(keyPoseAdj->points[keyPoseID].pitch, keyPoseAdj->points[keyPoseID].yaw, keyPoseAdj->points[keyPoseID].roll,
                    keyPoseAdj->points[i].pitch, keyPoseAdj->points[i].yaw, keyPoseAdj->points[i].roll, pitch, yaw, roll);

            Pose3d_Pose3d_Factor* poseposeFactor = new Pose3d_Pose3d_Factor(iSAMPoses[i], iSAMPoses[keyPoseID], 
                                                       Pose3d(tz, tx, ty, yaw, pitch, roll), Information(100.0 * eye(6)));
            iSAM.add_factor(poseposeFactor);
          } else if (i != keyPoseIDPre && (!matchFromOwnKeyPose || procID == 0) && 
                     (keyPoseIDArrayPtrNum - matchKeyPoseRecIDArrayNum[procID] > matchSkipKeyPoseNum || adjacentMatch[procID] == 0)) {
            if (matchNearestKeyPose) {
              int iPre = -1;
              for (int j = i - 1; j >= 0; j--) {
                if (keyPoseIDArrayAll[i] == keyPoseIDArrayAll[j]) {
                  iPre = j;
                  break;
                }
              }

              if (iPre >= 0) {
                float disX1 = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[iPre].x;
                float disY1 = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[iPre].y;
                float disZ1 = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[iPre].z;
                float disSq1 = disX1 * disX1 + disZ1 * disZ1;

                if (disSq1 < disSq && fabs(disY1) < matchRegionVert) {
                  continue;
                }
              }

              int iNext = -1;
              for (int j = i + 1; j < keyPoseID; j++) {
                if (keyPoseIDArrayAll[i] == keyPoseIDArrayAll[j]) {
                  iNext = j;
                  break;
                }
              }

              if (iNext >= 0) {
                float disX1 = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[iNext].x;
                float disY1 = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[iNext].y;
                float disZ1 = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[iNext].z;
                float disSq1 = disX1 * disX1 + disZ1 * disZ1;

                if (disSq1 < disSq && fabs(disY1) < matchRegionVert) {
                  continue;
                }
              }
            }

            adjMap->clear();
            for (int j = i; j < keyPoseID; j++) {
              if (keyPoseIDArrayAll[i] != keyPoseIDArrayAll[j]) {
                continue;
              }

              float disX2 = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[j].x;
              float disY2 = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[j].y;
              float disZ2 = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[j].z;
              float disSq2 = disX2 * disX2 + disZ2 * disZ2;

              if (disSq2 > adjMapRegionHori * adjMapRegionHori || fabs(disY2) > adjMapRegionVert) {
                break;
              }

              float srx2 = sin(keyPoseAdj->points[j].pitch);
              float crx2 = cos(keyPoseAdj->points[j].pitch);
              float sry2 = sin(keyPoseAdj->points[j].yaw);
              float cry2 = cos(keyPoseAdj->points[j].yaw);
              float srz2 = sin(keyPoseAdj->points[j].roll);
              float crz2 = cos(keyPoseAdj->points[j].roll);
              float tx2 = keyPoseAdj->points[j].x;
              float ty2 = keyPoseAdj->points[j].y;
              float tz2 = keyPoseAdj->points[j].z;

              int mapPtStartID = keyPose->points[j].mapPtStartID;
              int mapPtNum = keyPose->points[j].mapPtNum;
              stackMap2->resize(mapPtNum);
              for (int k = 0; k < mapPtNum; k++) {
                pointAssociateToMap(&stackMapArray->points[mapPtStartID + k], &stackMap2->points[k], 
                                    srx2, crx2, sry2, cry2, srz2, crz2, tx2, ty2, tz2);
              }
              *adjMap += *stackMap2;
            }

            for (int j = i - 1; j >= 0; j--) {
              if (keyPoseIDArrayAll[i] != keyPoseIDArrayAll[j]) {
                continue;
              }

              float disX2 = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[j].x;
              float disY2 = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[j].y;
              float disZ2 = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[j].z;
              float disSq2 = disX2 * disX2 + disZ2 * disZ2;

              if (disSq2 > adjMapRegionHori * adjMapRegionHori || fabs(disY2) > adjMapRegionVert) {
                break;
              }

              float srx2 = sin(keyPoseAdj->points[j].pitch);
              float crx2 = cos(keyPoseAdj->points[j].pitch);
              float sry2 = sin(keyPoseAdj->points[j].yaw);
              float cry2 = cos(keyPoseAdj->points[j].yaw);
              float srz2 = sin(keyPoseAdj->points[j].roll);
              float crz2 = cos(keyPoseAdj->points[j].roll);  
              float tx2 = keyPoseAdj->points[j].x;
              float ty2 = keyPoseAdj->points[j].y;
              float tz2 = keyPoseAdj->points[j].z;

              int mapPtStartID = keyPose->points[j].mapPtStartID;
              int mapPtNum = keyPose->points[j].mapPtNum;
              stackMap2->resize(mapPtNum);
              for (int k = 0; k < mapPtNum; k++) {
                pointAssociateToMap(&stackMapArray->points[mapPtStartID + k], &stackMap2->points[k], 
                                    srx2, crx2, sry2, cry2, srz2, crz2, tx2, ty2, tz2);
              }
              *adjMap += *stackMap2;
            }

            adjMapDS->clear();
            downSizeFilter.setInputCloud(adjMap);
            downSizeFilter.filter(*adjMapDS);

            transformTobeMapped[0] = keyPoseAdj->points[keyPoseID].pitch;
            transformTobeMapped[1] = keyPoseAdj->points[keyPoseID].yaw;
            transformTobeMapped[2] = keyPoseAdj->points[keyPoseID].roll;
            transformTobeMapped[3] = keyPoseAdj->points[keyPoseID].x;
            transformTobeMapped[4] = keyPoseAdj->points[keyPoseID].y;
            transformTobeMapped[5] = keyPoseAdj->points[keyPoseID].z;

            if (matchWEqualVert && (adjacentMatch[procID] == 0 || adjacentMatch[keyPoseIDArrayAll[i]] == 0)) {
              transformTobeMapped[4] = keyPoseAdj->points[i].y;
            }

            if (scanMatching() == 1) {
              scanMatched = true;

              keyPoseAdj->points[keyPoseID].pitch = transformTobeMapped[0];
              keyPoseAdj->points[keyPoseID].yaw = transformTobeMapped[1];
              keyPoseAdj->points[keyPoseID].roll = transformTobeMapped[2];
              keyPoseAdj->points[keyPoseID].x = transformTobeMapped[3];
              keyPoseAdj->points[keyPoseID].y = transformTobeMapped[4];
              keyPoseAdj->points[keyPoseID].z = transformTobeMapped[5];

              float roll, pitch, yaw, tx, ty, tz;
              diffTrans(keyPoseAdj->points[keyPoseID].x, keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z, 
                        keyPoseAdj->points[i].x, keyPoseAdj->points[i].y, keyPoseAdj->points[i].z, tx, ty, tz, 
                        keyPoseAdj->points[i].pitch, keyPoseAdj->points[i].yaw, keyPoseAdj->points[i].roll);
              diffRot(keyPoseAdj->points[keyPoseID].pitch, keyPoseAdj->points[keyPoseID].yaw, 
                      keyPoseAdj->points[keyPoseID].roll, keyPoseAdj->points[i].pitch, keyPoseAdj->points[i].yaw, 
                      keyPoseAdj->points[i].roll, pitch, yaw, roll);

              Pose3d_Pose3d_Factor* poseposeFactor = new Pose3d_Pose3d_Factor(iSAMPoses[i], iSAMPoses[keyPoseID], 
                                                         Pose3d(tz, tx, ty, yaw, pitch, roll), Information(10000.0 * eye(6)));
              iSAM.add_factor(poseposeFactor);

              sensor_msgs::PointCloud2 adjMap2;
              pcl::toROSMsg(*adjMapDS, adjMap2);
              adjMap2.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
              adjMap2.header.frame_id = "map_rot";
              pubMapCloud.publish(adjMap2);
            }
          }
        } else if ((disSq > matchNeglectRegHori * matchNeglectRegHori || fabs(disY) > matchNeglectRegVert) && 
                   procID == keyPoseIDArrayAll[i]) {
          adjacentPose = false;
        }
      }

      if (scanMatched) {
        matchKeyPoseRecIDArrayNum[procID] = keyPoseIDArrayPtrNum;
        bool optExecuted = false;
        if (keyPoseIDArrayPtrNum - optKeyPoseRecIDArrayNum[procID] > optSkipKeyPoseNum || adjacentMatch[procID] == 0) {
          iSAM.batch_optimization();

          optKeyPoseRecIDArrayNum[procID] = keyPoseIDArrayPtrNum;
          optExecuted = true;
        }

        for (int i = 0; i <= keyPoseID; i++) {
          keyPoseAdj->points[i].pitch = iSAMPoses[i]->value().pitch();
          keyPoseAdj->points[i].yaw = iSAMPoses[i]->value().yaw();
          keyPoseAdj->points[i].roll = iSAMPoses[i]->value().roll();
          keyPoseAdj->points[i].x = iSAMPoses[i]->value().y();
          keyPoseAdj->points[i].y = iSAMPoses[i]->value().z();
          keyPoseAdj->points[i].z = iSAMPoses[i]->value().x();
        }

        keyPoseRecX[procID] = keyPoseAdj->points[keyPoseID].x;
        keyPoseRecY[procID] = keyPoseAdj->points[keyPoseID].y;
        keyPoseRecZ[procID] = keyPoseAdj->points[keyPoseID].z;
        adjacentMatch[procID] = 1;

        if (optExecuted) {
          int keyPoseIDArrayNum = keyPoseIDArray.size();
          if (keyPoseIDArrayNum > 0) {
            keyPosePath.poses.resize(keyPoseIDArrayNum);
            for (int i = 0; i < keyPoseIDArrayNum; i++) {
              int j = keyPoseIDArray[i];

              geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(keyPoseAdj->points[j].roll, 
                                                 -keyPoseAdj->points[j].pitch, -keyPoseAdj->points[j].yaw);

              keyPosePath.poses[i].pose.orientation.x = -geoQuat.y;
              keyPosePath.poses[i].pose.orientation.y = -geoQuat.z;
              keyPosePath.poses[i].pose.orientation.z = geoQuat.x;
              keyPosePath.poses[i].pose.orientation.w = geoQuat.w;
              keyPosePath.poses[i].pose.position.x = keyPoseAdj->points[j].x;
              keyPosePath.poses[i].pose.position.y = keyPoseAdj->points[j].y;
              keyPosePath.poses[i].pose.position.z = keyPoseAdj->points[j].z;
            }

            keyPosePath.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
            pubKeyPosePath.publish(keyPosePath);
          }

          int keyPoseIDArrayS1Num = keyPoseIDArrayS1.size();
          if (keyPoseIDArrayS1Num > 0) {
            keyPosePathS1.poses.resize(keyPoseIDArrayS1Num);
            for (int i = 0; i < keyPoseIDArrayS1Num; i++) {
              int j = keyPoseIDArrayS1[i];

              geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(keyPoseAdj->points[j].roll, 
                                                 -keyPoseAdj->points[j].pitch, -keyPoseAdj->points[j].yaw);

              keyPosePathS1.poses[i].pose.orientation.x = -geoQuat.y;
              keyPosePathS1.poses[i].pose.orientation.y = -geoQuat.z;
              keyPosePathS1.poses[i].pose.orientation.z = geoQuat.x;
              keyPosePathS1.poses[i].pose.orientation.w = geoQuat.w;
              keyPosePathS1.poses[i].pose.position.x = keyPoseAdj->points[j].x;
              keyPosePathS1.poses[i].pose.position.y = keyPoseAdj->points[j].y;
              keyPosePathS1.poses[i].pose.position.z = keyPoseAdj->points[j].z;
            }

            keyPosePathS1.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
            pubKeyPosePathS1.publish(keyPosePathS1);
          }

          int keyPoseIDArrayS2Num = keyPoseIDArrayS2.size();
          if (keyPoseIDArrayS2Num > 0) {
            keyPosePathS2.poses.resize(keyPoseIDArrayS2Num);
            for (int i = 0; i < keyPoseIDArrayS2Num; i++) {
              int j = keyPoseIDArrayS2[i];

              geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(keyPoseAdj->points[j].roll, 
                                                 -keyPoseAdj->points[j].pitch, -keyPoseAdj->points[j].yaw);

              keyPosePathS2.poses[i].pose.orientation.x = -geoQuat.y;
              keyPosePathS2.poses[i].pose.orientation.y = -geoQuat.z;
              keyPosePathS2.poses[i].pose.orientation.z = geoQuat.x;
              keyPosePathS2.poses[i].pose.orientation.w = geoQuat.w;
              keyPosePathS2.poses[i].pose.position.x = keyPoseAdj->points[j].x;
              keyPosePathS2.poses[i].pose.position.y = keyPoseAdj->points[j].y;
              keyPosePathS2.poses[i].pose.position.z = keyPoseAdj->points[j].z;
            }

            keyPosePathS2.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);;
            pubKeyPosePathS2.publish(keyPosePathS2);
          }

          int keyPoseIDArrayS3Num = keyPoseIDArrayS3.size();
          if (keyPoseIDArrayS3Num > 0) {
            keyPosePathS3.poses.resize(keyPoseIDArrayS3Num);
            for (int i = 0; i < keyPoseIDArrayS3Num; i++) {
              int j = keyPoseIDArrayS3[i];

              geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(keyPoseAdj->points[j].roll, 
                                                 -keyPoseAdj->points[j].pitch, -keyPoseAdj->points[j].yaw);

              keyPosePathS3.poses[i].pose.orientation.x = -geoQuat.y;
              keyPosePathS3.poses[i].pose.orientation.y = -geoQuat.z;
              keyPosePathS3.poses[i].pose.orientation.z = geoQuat.x;
              keyPosePathS3.poses[i].pose.orientation.w = geoQuat.w;
              keyPosePathS3.poses[i].pose.position.x = keyPoseAdj->points[j].x;
              keyPosePathS3.poses[i].pose.position.y = keyPoseAdj->points[j].y;
              keyPosePathS3.poses[i].pose.position.z = keyPoseAdj->points[j].z;
            }

            keyPosePathS3.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);;
            pubKeyPosePathS3.publish(keyPosePathS3);
          }
        }
      }

      float disX3 = keyPoseAdj->points[keyPoseID].x - keyPoseRecX[procID];
      float disY3 = keyPoseAdj->points[keyPoseID].y - keyPoseRecY[procID];
      float disZ3 = keyPoseAdj->points[keyPoseID].z - keyPoseRecZ[procID];
      float disSq3 = disX3 * disX3 + disZ3 * disZ3;

      if (disSq3 > matchNeglectRegHori * matchNeglectRegHori || fabs(disY3) > matchNeglectRegVert) {
        adjacentMatch[procID] = 0;
      }

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(keyPoseAdj->points[keyPoseID].roll, 
                                         -keyPoseAdj->points[keyPoseID].pitch, -keyPoseAdj->points[keyPoseID].yaw);

      keyPoseOdom.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      keyPoseOdom.pose.pose.orientation.x = -geoQuat.y;
      keyPoseOdom.pose.pose.orientation.y = -geoQuat.z;
      keyPoseOdom.pose.pose.orientation.z = geoQuat.x;
      keyPoseOdom.pose.pose.orientation.w = geoQuat.w;
      keyPoseOdom.pose.pose.position.x = keyPoseAdj->points[keyPoseID].x;
      keyPoseOdom.pose.pose.position.y = keyPoseAdj->points[keyPoseID].y;
      keyPoseOdom.pose.pose.position.z = keyPoseAdj->points[keyPoseID].z;
      keyPoseOdom.twist.twist.angular.x = keyPose->points[keyPoseID].pitch;
      keyPoseOdom.twist.twist.angular.y = keyPose->points[keyPoseID].yaw;
      keyPoseOdom.twist.twist.angular.z = keyPose->points[keyPoseID].roll;
      keyPoseOdom.twist.twist.linear.x = keyPose->points[keyPoseID].x;
      keyPoseOdom.twist.twist.linear.y = keyPose->points[keyPoseID].y;
      keyPoseOdom.twist.twist.linear.z = keyPose->points[keyPoseID].z;
      keyPoseOdom.pose.covariance[1] = int(keyPose->points[keyPoseID].time / 100.0);
      keyPoseOdom.pose.covariance[2] = keyPose->points[keyPoseID].time / 100.0 - int(keyPose->points[keyPoseID].time / 100.0);

      keyPoseTrans.stamp_ = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      keyPoseTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      keyPoseTrans.setOrigin(tf::Vector3(keyPoseAdj->points[keyPoseID].x, 
                                         keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z));

      if (procID == 0) {
        keyPoseOdom.pose.covariance[0] = keyPoseIDArray.size() - 1;
        keyPoseOdom.child_frame_id = "key_pose";
        pubKeyPose.publish(keyPoseOdom);

        keyPoseTrans.child_frame_id_ = "key_pose";
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
      } else if (procID == 1) {
        keyPoseOdom.pose.covariance[0] = keyPoseIDArrayS1.size() - 1;
        keyPoseOdom.child_frame_id = "key_pose_s1";
        pubKeyPoseS1.publish(keyPoseOdom);

        keyPoseTrans.child_frame_id_ = "key_pose_s1";
        tfBroadcaster.sendTransform(keyPoseTrans);

        /*sensor_msgs::PointCloud2 stackMap3;
        pcl::toROSMsg(*stackMapS1, stackMap3);
        stackMap3.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
        stackMap3.header.frame_id = "key_pose_s1";
        pubLaserCloudS1.publish(stackMap3);*/

        stackMapS1->clear();
      } else if (procID == 2) {
        keyPoseOdom.pose.covariance[0] = keyPoseIDArrayS2.size() - 1;
        keyPoseOdom.child_frame_id = "key_pose_s2";
        pubKeyPoseS2.publish(keyPoseOdom);

        keyPoseTrans.child_frame_id_ = "key_pose_s2";
        tfBroadcaster.sendTransform(keyPoseTrans);

        /*sensor_msgs::PointCloud2 stackMap3;
        pcl::toROSMsg(*stackMapS2, stackMap3);
        stackMap3.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
        stackMap3.header.frame_id = "key_pose_s2";
        pubLaserCloudS2.publish(stackMap3);*/

        stackMapS2->clear();
      } else if (procID == 3) {
        keyPoseOdom.pose.covariance[0] = keyPoseIDArrayS3.size() - 1;
        keyPoseOdom.child_frame_id = "key_pose_s3";
        pubKeyPoseS3.publish(keyPoseOdom);

        keyPoseTrans.child_frame_id_ = "key_pose_s3";
        tfBroadcaster.sendTransform(keyPoseTrans);

        /*sensor_msgs::PointCloud2 stackMap3;
        pcl::toROSMsg(*stackMapS3, stackMap3);
        stackMap3.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
        stackMap3.header.frame_id = "key_pose_s3";
        pubLaserCloudS3.publish(stackMap3);*/

        stackMapS3->clear();
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
