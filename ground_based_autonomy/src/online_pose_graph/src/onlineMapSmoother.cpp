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

int laserVoxelCenWidth = 30;
int laserVoxelCenHeight = 30;
int laserVoxelCenDepth = 30;
const int laserVoxelWidth = 61;
const int laserVoxelHeight = 61;
const int laserVoxelDepth = 61;
const int laserVoxelNum = laserVoxelWidth * laserVoxelHeight * laserVoxelDepth;

int laserVoxelInd[laserVoxelNum];

PosePoint keyPoseCur;
bool newKeyPose = false;
bool acceptLaserCloud = true;

float poseXLast = -1000000;
float poseYLast = -1000000;
float poseZLast = -1000000;
int poseCount = 0;

float transformSum[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

int optKeyPoseRecID = 0, matchKeyPoseRecID = 0;
float keyPoseRecX = 0, keyPoseRecY = 0, keyPoseRecZ = 0;
bool adjacentMatch = true;

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
    keyPoseAdj->push_back(keyPoseCur);

    poseCount = poseStackNum;

    newKeyPose = true;
  }
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
  ros::init(argc, argv, "onlineMapSmoother");
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

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/velodyne_cloud_registered", 5, laserCloudHandler);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/aft_mapped_to_init", 5, odometryHandler);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_key_pose", 2);

  ros::Publisher pubRelativePose = nh.advertise<sensor_msgs::PointCloud2> ("/relative_pose_to_key_pose", 2);

  ros::Publisher pubMapCloud = nh.advertise<sensor_msgs::PointCloud2> ("/map_cloud", 2);

  ros::Publisher pubKeyPose = nh.advertise<nav_msgs::Odometry> ("/key_pose_to_map", 5);
  nav_msgs::Odometry keyPoseOdom;
  keyPoseOdom.header.frame_id = "map_rot";
  keyPoseOdom.child_frame_id = "key_pose";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform keyPoseTrans;
  keyPoseTrans.frame_id_ = "map_rot";
  keyPoseTrans.child_frame_id_ = "key_pose";

  ros::Publisher pubKeyPosePath = nh.advertise<nav_msgs::Path> ("/key_pose_path", 5);
  nav_msgs::Path keyPosePath;
  keyPosePath.header.frame_id = "map_rot";

  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
  downSizeFilter.setLeafSize(voxelSize, voxelSize, voxelSize);

  MatrixXd infoInit = 10000.0 * eye(6);

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

      keyPoseMap->clear();
      downSizeFilter.setInputCloud(stackMap);
      downSizeFilter.filter(*keyPoseMap);

      int keyPoseMapNum = keyPoseMap->points.size();
      keyPose->points[keyPoseID].mapPtStartID = stackMapArray->points.size();
      keyPose->points[keyPoseID].mapPtNum = keyPoseMapNum;

      *stackMapArray += *keyPoseMap;

      if (keyPoseID == 0) {
        Pose3d_Node* pose = new Pose3d_Node();
        iSAM.add_node(pose);
        iSAMPoses.push_back(pose);

        Pose3d_Factor* poseFactor = new Pose3d_Factor(iSAMPoses[0], Pose3d(keyPose->points[0].z, 
                                                      keyPose->points[0].x, keyPose->points[0].y, 
                                                      keyPose->points[0].yaw, keyPose->points[0].pitch, 
                                                      keyPose->points[0].roll), Information(infoInit));
        iSAM.add_factor(poseFactor);
      } else {
        transformSum[0] = keyPose->points[keyPoseID].pitch;
        transformSum[1] = keyPose->points[keyPoseID].yaw;
        transformSum[2] = keyPose->points[keyPoseID].roll;
        transformSum[3] = keyPose->points[keyPoseID].x;
        transformSum[4] = keyPose->points[keyPoseID].y;
        transformSum[5] = keyPose->points[keyPoseID].z;

        transformBefMapped[0] = keyPose->points[keyPoseID - 1].pitch;
        transformBefMapped[1] = keyPose->points[keyPoseID - 1].yaw;
        transformBefMapped[2] = keyPose->points[keyPoseID - 1].roll;
        transformBefMapped[3] = keyPose->points[keyPoseID - 1].x;
        transformBefMapped[4] = keyPose->points[keyPoseID - 1].y;
        transformBefMapped[5] = keyPose->points[keyPoseID - 1].z;

        transformAftMapped[0] = keyPoseAdj->points[keyPoseID - 1].pitch;
        transformAftMapped[1] = keyPoseAdj->points[keyPoseID - 1].yaw;
        transformAftMapped[2] = keyPoseAdj->points[keyPoseID - 1].roll;
        transformAftMapped[3] = keyPoseAdj->points[keyPoseID - 1].x;
        transformAftMapped[4] = keyPoseAdj->points[keyPoseID - 1].y;
        transformAftMapped[5] = keyPoseAdj->points[keyPoseID - 1].z;

        transformAssociateToMap();

        float diffX = transformTobeMapped[3] - transformAftMapped[3];
        float diffY = transformTobeMapped[4] - transformAftMapped[4];
        float diffZ = transformTobeMapped[5] - transformAftMapped[5];
        float dis = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

        float srx1 = sin(keyPose->points[keyPoseID - 1].pitch);
        float crx1 = cos(keyPose->points[keyPoseID - 1].pitch);
        float sry1 = sin(keyPose->points[keyPoseID - 1].yaw);
        float cry1 = cos(keyPose->points[keyPoseID - 1].yaw);
        float srz1 = sin(keyPose->points[keyPoseID - 1].roll);
        float crz1 = cos(keyPose->points[keyPoseID - 1].roll);
        float tx1 = keyPose->points[keyPoseID - 1].x;
        float ty1 = keyPose->points[keyPoseID - 1].y;
        float tz1 = keyPose->points[keyPoseID - 1].z;

        pcl::PointXYZ point;
        point.x = keyPose->points[keyPoseID].x;
        point.y = keyPose->points[keyPoseID].y;
        point.z = keyPose->points[keyPoseID].z;

        pointAssociateTobeMapped(&point, &point, srx1, crx1, sry1, cry1, srz1, crz1, tx1, ty1, tz1);

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

        float roll, pitch, yaw, tx, ty, tz;
        diffTrans(keyPoseAdj->points[keyPoseID].x, keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z, 
                  keyPoseAdj->points[keyPoseID - 1].x, keyPoseAdj->points[keyPoseID - 1].y, keyPoseAdj->points[keyPoseID - 1].z, 
                  tx, ty, tz, keyPoseAdj->points[keyPoseID - 1].pitch, keyPoseAdj->points[keyPoseID - 1].yaw, 
                  keyPoseAdj->points[keyPoseID - 1].roll);
        diffRot(keyPoseAdj->points[keyPoseID].pitch, keyPoseAdj->points[keyPoseID].yaw, keyPoseAdj->points[keyPoseID].roll,
                keyPoseAdj->points[keyPoseID - 1].pitch, keyPoseAdj->points[keyPoseID - 1].yaw, 
                keyPoseAdj->points[keyPoseID - 1].roll, pitch, yaw, roll);

        Pose3d_Node* pose = new Pose3d_Node();
        iSAM.add_node(pose);
        iSAMPoses.push_back(pose);

        Pose3d_Pose3d_Factor* poseposeFactor = new Pose3d_Pose3d_Factor(iSAMPoses[keyPoseID - 1], iSAMPoses[keyPoseID], 
                                                   Pose3d(tz, tx, ty, yaw, pitch, roll), Information(100.0 * eye(6)));
        iSAM.add_factor(poseposeFactor);

        bool adjacentPose = true;
        bool scanMatched = false;
        for (int i = keyPoseID - 2; i >= 0; i--) {
          float disX = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[i].x;
          float disY = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[i].y;
          float disZ = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[i].z;
          float disSq = disX * disX + disZ * disZ;

          if (disSq < matchRegionHori * matchRegionHori && fabs(disY) < matchRegionVert) {
            if (adjacentPose) {
              diffTrans(keyPoseAdj->points[keyPoseID].x, keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z, 
                        keyPoseAdj->points[i].x, keyPoseAdj->points[i].y, keyPoseAdj->points[i].z, tx, ty, tz, 
                        keyPoseAdj->points[i].pitch, keyPoseAdj->points[i].yaw, keyPoseAdj->points[i].roll);
              diffRot(keyPoseAdj->points[keyPoseID].pitch, keyPoseAdj->points[keyPoseID].yaw, keyPoseAdj->points[keyPoseID].roll,
                      keyPoseAdj->points[i].pitch, keyPoseAdj->points[i].yaw, keyPoseAdj->points[i].roll, pitch, yaw, roll);

              Pose3d_Pose3d_Factor* poseposeFactor = new Pose3d_Pose3d_Factor(iSAMPoses[i], iSAMPoses[keyPoseID], 
                                                         Pose3d(tz, tx, ty, yaw, pitch, roll), Information(100.0 * eye(6)));
              iSAM.add_factor(poseposeFactor);
            } else if (keyPoseID - matchKeyPoseRecID > matchSkipKeyPoseNum || !adjacentMatch) {
              if (matchNearestKeyPose && i > 0) {
                float disX11 = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[i - 1].x;
                float disY11 = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[i - 1].y;
                float disZ11 = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[i - 1].z;
                float disSq11 = disX11 * disX11 + disZ11 * disZ11;

                float disX12 = keyPoseAdj->points[keyPoseID].x - keyPoseAdj->points[i + 1].x;
                float disY12 = keyPoseAdj->points[keyPoseID].y - keyPoseAdj->points[i + 1].y;
                float disZ12 = keyPoseAdj->points[keyPoseID].z - keyPoseAdj->points[i + 1].z;
                float disSq12 = disX12 * disX12 + disZ12 * disZ12;

                if ((disSq11 < disSq && fabs(disY11) < matchRegionVert) || (disSq12 < disSq && fabs(disY12) < matchRegionVert)) {
                  continue;
                }
              }

              adjMap->clear();
              for (int j = i; j < keyPoseID; j++) {
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

              if (matchWEqualVert && !adjacentMatch) {
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
          } else if (disSq > matchNeglectRegHori * matchNeglectRegHori || fabs(disY) > matchNeglectRegVert) {
            adjacentPose = false;
          }
        }

        if (scanMatched) {
          matchKeyPoseRecID = keyPoseID;
          bool optExecuted = false;
          if (keyPoseID - optKeyPoseRecID > optSkipKeyPoseNum || !adjacentMatch) {
            iSAM.batch_optimization();

            optKeyPoseRecID = keyPoseID;
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

          keyPoseRecX = keyPoseAdj->points[keyPoseID].x;
          keyPoseRecY = keyPoseAdj->points[keyPoseID].y;
          keyPoseRecZ = keyPoseAdj->points[keyPoseID].z;
          adjacentMatch = true;

          if (optExecuted) {
            keyPosePath.poses.resize(keyPoseID + 1);
            for (int i = 0; i <= keyPoseID; i++) {
              geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(keyPoseAdj->points[i].roll, 
                                                 -keyPoseAdj->points[i].pitch, -keyPoseAdj->points[i].yaw);

              keyPosePath.poses[i].pose.orientation.x = -geoQuat.y;
              keyPosePath.poses[i].pose.orientation.y = -geoQuat.z;
              keyPosePath.poses[i].pose.orientation.z = geoQuat.x;
              keyPosePath.poses[i].pose.orientation.w = geoQuat.w;
              keyPosePath.poses[i].pose.position.x = keyPoseAdj->points[i].x;
              keyPosePath.poses[i].pose.position.y = keyPoseAdj->points[i].y;
              keyPosePath.poses[i].pose.position.z = keyPoseAdj->points[i].z;
            }

            keyPosePath.header.stamp = ros::Time().fromSec(keyPose->points[keyPoseID].time);
            pubKeyPosePath.publish(keyPosePath);
          }
        }

        float disX3 = keyPoseAdj->points[keyPoseID].x - keyPoseRecX;
        float disY3 = keyPoseAdj->points[keyPoseID].y - keyPoseRecY;
        float disZ3 = keyPoseAdj->points[keyPoseID].z - keyPoseRecZ;
        float disSq3 = disX3 * disX3 + disZ3 * disZ3;

        if (disSq3 > matchNeglectRegHori * matchNeglectRegHori || fabs(disY3) > matchNeglectRegVert) {
          adjacentMatch = false;
        }
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
      keyPoseOdom.pose.covariance[0] = keyPoseID;
      keyPoseOdom.pose.covariance[1] = int(keyPose->points[keyPoseID].time / 100.0);
      keyPoseOdom.pose.covariance[2] = keyPose->points[keyPoseID].time / 100.0 - int(keyPose->points[keyPoseID].time / 100.0);
      pubKeyPose.publish(keyPoseOdom);

      keyPoseTrans.stamp_ = ros::Time().fromSec(keyPose->points[keyPoseID].time);
      keyPoseTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      keyPoseTrans.setOrigin(tf::Vector3(keyPoseAdj->points[keyPoseID].x, 
                                         keyPoseAdj->points[keyPoseID].y, keyPoseAdj->points[keyPoseID].z));
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
