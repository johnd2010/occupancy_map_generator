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


#include "opencv2/core/types.hpp"
#include "ros/console.h"
#include "ros/time.h"
#include <occupancy_map_generator/OccupancyMap.h>

OccupancyMap::OccupancyMap( const ros::NodeHandle &nh_)
: nodehandle(nh_),
  nodehandle_private(nh_),
  m_res(0.5),
  concave_alpha(0.1),
  m_worldFrameId("map"),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_explorer_mode(true),
  m_occupancyMaxZ(std::numeric_limits<double>::max())
{
  
}

void OccupancyMap::Initialize()
{
  nodehandle_private.param("map/explorer_mode", m_explorer_mode,m_explorer_mode);
  nodehandle_private.param("map/occupancy_map_min_z", m_occupancyMinZ,m_occupancyMinZ);
  nodehandle_private.param("map/occupancy_map_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  nodehandle_private.param("map/occupancy_map_resolution", m_res, m_res);
  nodehandle_private.param("map/concave_tightness", concave_alpha, concave_alpha);
  nodehandle_private.param("map/frame_id", m_worldFrameId, m_worldFrameId);
  m_mapPub = nodehandle.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, true);
  ROS_INFO("Occupancy Map with Resolution %f and Z (MinZ,MaxZ) : (%f,%f)",m_res,m_occupancyMinZ,m_occupancyMaxZ);
  ROS_INFO("Concave Hull is computed with alpha %f",concave_alpha);
  pointCloudSub = nodehandle.subscribe<sensor_msgs::PointCloud2>("pointcloud_input", 5, &OccupancyMap::pointCloudCallback, this);
  Cloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
}


void OccupancyMap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, *Cloud);
  publishProjected2DMap(Cloud);
}


void OccupancyMap::publishProjected2DMap(pcl::PointCloud<PointF>::Ptr Cloud){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = Cloud->size();
  occupancy_map.info.resolution = m_res;
  // oldMapInfo = occupancy_map.info;

  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }
  ROS_DEBUG("Cloud Size %lu",octomapSize);
  getOccupiedLimits(Cloud);
  ROS_DEBUG("Timing");
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Frame");
  occupancy_map.header.frame_id = m_worldFrameId;
  ROS_DEBUG("Time");
  occupancy_map.header.stamp = ros::Time::now();
  ROS_DEBUG("Map Ready to publish");
  m_mapPub.publish(occupancy_map);
  // occupancy_map.data.clear();
  ROS_INFO("Map publishing in OccupancyMap took %f sec", total_elapsed);
}

// void OccupancyMap::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{

//   int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
//   int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

//   if (i_off < 0 || j_off < 0
//       || oldMapInfo.width  + i_off > map.info.width
//       || oldMapInfo.height + j_off > map.info.height)
//   {
//     ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
//     return;
//   }

//   nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

//   map.data.clear();
//   // init to unknown:
//   map.data.resize(map.info.width * map.info.height, -1);

//   nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

//   for (int j =0; j < int(oldMapInfo.height); ++j ){
//     fromStart = oldMapData.begin() + j*oldMapInfo.width;
//     fromEnd = fromStart + oldMapInfo.width;
//     toStart = map.data.begin() + ((j+j_off)*occupancy_map.info.width + i_off);
//     copy(fromStart, fromEnd, toStart);
//   }

// }

void OccupancyMap::getOccupiedLimits(pcl::PointCloud<PointF>::Ptr Cloud)
{
  filteredVoxelCloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
  convexCloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
  downsampledCloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
  //downsample pointcloud
  ROS_DEBUG("Downsampling");
  sor.setInputCloud(this->Cloud);
  sor.setLeafSize(m_res,m_res,m_res);
  sor.filter(*downsampledCloud);
  //crop pointcloud
  ROS_DEBUG("Cropping");
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_occupancyMinZ,m_occupancyMaxZ);
  pass.setInputCloud(downsampledCloud);
  pass.filter(*filteredVoxelCloud);
  //get concave points
  ROS_DEBUG("Concave Check");
  chull.setInputCloud(filteredVoxelCloud);
  chull.setAlpha(concave_alpha);
  chull.setDimension(2);
  chull.reconstruct(*convexCloud);
  ROS_DEBUG("Concave Check success");
  generateOccupancyMap();
}




void OccupancyMap::generateOccupancyMap(){     
  
    std::vector<PointF> polygonVertices;
    cv::Point occ_point,occ_centroid;
    Eigen::Vector4f centroid;
    initial_check=true;
    pcl::compute3DCentroid(*convexCloud,centroid);
    ROS_DEBUG("Got centroid");
    for (const auto& concave_points : *convexCloud) {
      insertPointClockwise(polygonVertices, centroid, concave_points);
      if(!initial_check){
        min_occupied.x = std::min(concave_points.x, min_occupied.x);
        min_occupied.y = std::min(concave_points.y, min_occupied.y);
        min_occupied.z = std::min(concave_points.z, min_occupied.z);
        max_occupied.x = std::max(concave_points.x, max_occupied.x);
        max_occupied.y = std::max(concave_points.y, max_occupied.y);
        max_occupied.z = std::max(concave_points.z, max_occupied.z);
      }
      else {
        min_occupied = concave_points;
        max_occupied = concave_points;
        initial_check=false;
        ROS_DEBUG("False check");
      }
    }
    temp_map = occupancy_map;
    occupancy_map.info.width = static_cast<unsigned int>(std::abs((max_occupied.x - min_occupied.x))/m_res);
    occupancy_map.info.height = static_cast<unsigned int>(std::abs((max_occupied.y - min_occupied.y))/m_res);
    occupancy_map.info.resolution = m_res;
    occupancy_map.info.origin.position.x = min_occupied.x;
    occupancy_map.info.origin.position.y = min_occupied.y;
    ROS_DEBUG("occupancy map ready");
    cv::Mat binary_occupancy_map(occupancy_map.info.height,occupancy_map.info.width,CV_8SC1, cv::Scalar(-1));
    std::vector<cv::Point> concave_occupancy;
    std::vector<std::vector<cv::Point>> polygons;
    ROS_DEBUG("CV processing");
    for (const auto& point : polygonVertices) 
    {
      concave_occupancy.push_back(cv::Point((std::abs((point.x - min_occupied.x))/m_res),(std::abs((point.y - min_occupied.y))/m_res)));
    }
    polygons.push_back(concave_occupancy);
    ROS_DEBUG("CV processing complete");
    cv::fillPoly(binary_occupancy_map, polygons, cv::Scalar(0));
    ROS_DEBUG("Filling polygon");
    if(!m_explorer_mode)
    {
      for (const auto& occupied_center : *filteredVoxelCloud){      
        uchar* ptr = binary_occupancy_map.ptr<uchar>(static_cast<unsigned int>(std::abs((occupied_center.y - min_occupied.y))/m_res));
        ptr[static_cast<unsigned int>(std::abs((occupied_center.x - min_occupied.x))/m_res)] = 100;
      }
    }
    
    ROS_DEBUG("Making CV occgrid");
    std::vector<int8_t> occupancyGridData(binary_occupancy_map.begin<uchar>(), binary_occupancy_map.end<uchar>());
    occupancy_map.data = occupancyGridData;
    
    ROS_DEBUG("occgrid ready");
}
