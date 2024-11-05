/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_SERVER_OccupancyMap_H
#define OCTOMAP_SERVER_OccupancyMap_H

#include <ros/ros.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <std_srvs/Empty.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/concave_hull.h>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"



#include <vector>
#include "pcl/impl/point_types.hpp"
#include <occupancy_map_generator/MapMerger.h>

#include "ros/console.h"
//Add communication distance limits, distance topic, uav count
//Change to params for resolution, height, width , topic names,start and end point
//if time permits, frame transfomration and params frame_id
// time permits, merge with obstacles as well, we assume that we are getting a space limited occupancy map
//Add pose topic for distance calculation
// Change it for a single agent for real world experiments or add a param for real world experiments that take only the current uav
//Change to pointers for Occupancy maps



class OccupancyMap {

public:
  typedef pcl::PointXYZ PointF;

  OccupancyMap(const ros::NodeHandle &nh_ = ros::NodeHandle());

  void Initialize();
  void reset(const ros::NodeHandle &n,std::string frame);
  void publishProjected2DMap(pcl::PointCloud<PointF>::Ptr Cloud);
  ros::Subscriber m_mapSub;
  std::vector<ros::Subscriber> team_occ_map_subscribers;
  ros::Subscriber pointCloudSub;
  pcl::PointCloud<OccupancyMap::PointF>::Ptr Cloud;
  std::vector<nav_msgs::OccupancyGrid> team_occmaps;
  bool intialize_merged_occupancy_map=false;



protected:
  std::string m_worldFrameId;
  void update2DMap(PointF current_3dpoint, bool occupied);
  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Inserting points in a clockwise rotaion for computation of concave points
  double calculateAngle(const Eigen::Vector4f centroid, const PointF p) {
      return atan2(p.y - centroid[1], p.x - centroid[0]);
  }
  bool comparePoints(const Eigen::Vector4f  centroid, const PointF a, const PointF b) {
      double angleA = calculateAngle(centroid, a);
      double angleB = calculateAngle(centroid, b);
      return angleA > angleB; // For clockwise order
  }
  void sortPointsClockwise(std::vector<PointF>& points, Eigen::Vector4f centroid) {
      std::sort(points.begin(), points.end(), [&centroid, this](const PointF a, const PointF b) {
          return comparePoints(centroid, a, b);
      });
  }
  void insertPointClockwise(std::vector<PointF>& points, Eigen::Vector4f  centroid, const PointF newPoint) {
      auto it = std::lower_bound(points.begin(), points.end(), newPoint, [&centroid, this](const PointF a, const PointF b) {
          return comparePoints(centroid, a, b);
      });
      points.insert(it, newPoint);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void getOccupiedLimits(pcl::PointCloud<PointF>::Ptr Cloud);
  void generateOccupancyMap();
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, int i);
  void invokeMerger();
  



  ros::NodeHandle nodehandle;
  ros::NodeHandle nodehandle_private;
  ros::Publisher  m_mapPub,mergedPub,m_frontierPub;

  PointF minPt,maxPt;  
  PointF min_occupied,max_occupied;

  pcl::PointCloud<PointF>::Ptr filteredVoxelCloud,convexCloud,downsampledCloud;
  pcl::PointXYZ voxelpoints;
  pcl::ConcaveHull<PointF> chull;
  pcl::VoxelGrid<PointF> sor;
  pcl::PassThrough<PointF> pass;
  cv::Mat merged_occupancy_matrix;


  double m_res,height=60,width=60;
  double concave_alpha;
  int uav_count=3;
  std::string uav_name;
  std::vector<std::string> team_members;

  double m_occupancyMinX,m_occupancyMinY,m_occupancyMinZ;
  double m_occupancyMaxX,m_occupancyMaxY,m_occupancyMaxZ;
  bool m_explorer_mode,m_merger_mode,frontier_detector;

  // downprojected 2D map:
  bool initial_check=true;  

  nav_msgs::OccupancyGrid occupancy_map,merged_occupancy_map;
  nav_msgs::MapMetaData oldMapInfo;
  
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  MapMerger mapmerger;
  
};

#endif

// When moving window is enabled, you should change the way adjustmap is handled