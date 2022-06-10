/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() {}
  
  /** Uppdate parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;
    
    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);
    
    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion 
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {

    if (!private_nh.getParam("rotCorrX", rotCorrX) || !private_nh.getParam("rotCorrY", rotCorrY))
      {
        ROS_ERROR_STREAM("No rot correction specified. Not using rot correction");

        // not use rot correction
        rotCorrX = 0;
        rotCorrY = 0;
      }

    if (!private_nh.getParam("filterDis", filterDis)) 
      {
        ROS_ERROR_STREAM("No filter distance specified. Not using filter");

        // not use filter
        filterDis = -1.0;
      }

    /*if (!private_nh.getParam("useHDL32", useHDL32)) 
      {
        ROS_ERROR_STREAM("No device specified. Using VLP-16");

        // use default VLP-16
        useHDL32 = false;
      }*/
    useHDL32 = false;

    if (!private_nh.getParam("upward", upward))
      {
        ROS_ERROR_STREAM("No mounting direction specified! Using upward mounting!");

        // use default upward mounting direction
        upward = true;
      }
    if (upward) dirInd = 1;
    else dirInd = -1;

    if (useHDL32) {
      if (!private_nh.getParam("calibration32", config_.calibrationFile))
        {
          ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

          // have to use something: grab unit test version as a default
          std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
          config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
        }
    } else {
      if (!private_nh.getParam("calibration16", config_.calibrationFile))
        {
          ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

          // have to use something: grab unit test version as a default
          std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
          config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
        }
    }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << 
          config_.calibrationFile);
      return -1;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (std::uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
   return 0;
  }

  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt,
                       VPointCloud &pc)
  {
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        float x, y, z;
        float intensity;
	float rot_comp;
        uint8_t laser_number;       ///< hardware laser number

        laser_number = (j + bank_origin);
        if (!useHDL32) laser_number = laser_number % 16;

        velodyne_pointcloud::LaserCorrection &corrections = 
          calibration_.laser_corrections[laser_number];

        /** Position Calculation */

        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[i].data[k];
        tmp.bytes[1] = raw->blocks[i].data[k+1];
        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((raw->blocks[i].rotation >= config_.min_angle 
             && raw->blocks[i].rotation <= config_.max_angle 
             && config_.min_angle < config_.max_angle)
             ||(config_.min_angle > config_.max_angle 
             && (raw->blocks[i].rotation <= config_.max_angle 
             || raw->blocks[i].rotation >= config_.min_angle))){
          float distance = tmp.uint * DISTANCE_RESOLUTION;
          distance += corrections.dist_correction;

          if (filterDis > 0 && i > 0 && i < BLOCKS_PER_PACKET - 1) {
            union two_bytes tmpBef;
            tmpBef.bytes[0] = raw->blocks[i - 1].data[k];
            tmpBef.bytes[1] = raw->blocks[i - 1].data[k+1];
            float distanceBef = tmpBef.uint * DISTANCE_RESOLUTION;
            distanceBef += corrections.dist_correction;

            union two_bytes tmpAft;
            tmpAft.bytes[0] = raw->blocks[i + 1].data[k];
            tmpAft.bytes[1] = raw->blocks[i + 1].data[k+1];
            float distanceAft = tmpAft.uint * DISTANCE_RESOLUTION;
            distanceAft += corrections.dist_correction;

            if (fabs(distance - distanceBef) > filterDis && fabs(distance - distanceAft) > filterDis) {
              distance = 0.01;
            }
          }

          if (distance < 0.01) distance = 0.01;

          if (useHDL32) {
            rot_comp = 0.41472 * j;
          } else {
            rot_comp = 0.82944 * j;
            if (j >= 16) rot_comp += 6.63552;
          }
          
          int rotation = raw->blocks[i].rotation + int(rot_comp);
          if (rot_comp - int(rot_comp) > 0.5) rotation++;
          if (rotation >= ROTATION_MAX_UNITS) rotation -= ROTATION_MAX_UNITS;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          // float cos_rot_correction = corrections.cos_rot_correction;
          // float sin_rot_correction = corrections.sin_rot_correction;

          // Compute the rot correction
          float rot_corr_angle = rotCorrX * cos_rot_table_[rotation] - rotCorrY * sin_rot_table_[rotation];
          int rot_correction = int(rot_corr_angle / ROTATION_RESOLUTION + 0.5);
          if (rot_corr_angle / ROTATION_RESOLUTION + 0.5 < 0) rot_correction--;

          if (rot_correction < 0) rot_correction += ROTATION_MAX_UNITS;
          else if (rot_correction >= ROTATION_MAX_UNITS) rot_correction -= ROTATION_MAX_UNITS;

          float cos_rot_correction = cos_rot_table_[rot_correction];
          float sin_rot_correction = sin_rot_table_[rot_correction];

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[rotation] * cos_rot_correction + 
            sin_rot_table_[rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[rotation] * cos_rot_correction - 
            cos_rot_table_[rotation] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          float xy_distance = distance * cos_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
          }
  
          float distance_x = distance + distance_corr_x;
          xy_distance = distance_x * cos_vert_angle;
          x = xy_distance * sin_rot_angle + horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          z = distance * sin_vert_angle + vert_offset;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
          if (useHDL32) {
            x_coord = -x;
            y_coord = -y;
          }
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
  
          intensity = raw->blocks[i].data[k+2];
  
          float focal_offset = 256 
                             * (1 - corrections.focal_distance / 13100) 
                             * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (abs(focal_offset - 256 * 
            (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;
  
          //if (raw->blocks[i].rotation != 0 /*&& pointInRange(distance)*/) {
  
            // convert polar coordinates to Euclidean XYZ
            VPoint point;
            point.ring = corrections.laser_ring;
            point.x = x_coord;
            point.y = dirInd * y_coord;
            point.z = dirInd * z_coord;
            point.intensity = (uint8_t) intensity;
  
            // append this point to the cloud
            pc.points.push_back(point);
            ++pc.width;
          //}
        }
      }
    }
  }  

} // namespace velodyne_rawdata
