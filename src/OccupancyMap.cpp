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


#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "ros/console.h"
#include "ros/this_node.h"
#include "ros/time.h"
#include <occupancy_map_generator/OccupancyMap.h>
#include <string>

OccupancyMap::OccupancyMap( const ros::NodeHandle &nh_)
: nodehandle(nh_),
  nodehandle_private(nh_),
  map_resolution(0.5),
  concave_alpha(0.1),
  m_worldFrameId("map"),
  m_occupancyMinX(-std::numeric_limits<double>::max()),
  m_occupancyMinY(-std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_explorer_mode(true),
  m_merger_mode(true),
  frontier_detector(true),
  m_occupancyMaxX(std::numeric_limits<double>::max()),
  m_occupancyMaxY(std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max())
{
  
}

void OccupancyMap::Initialize()
{
  uav_name = ros::this_node::getNamespace();
  nodehandle_private.param("/explorer/area/h", height,height);
  nodehandle_private.param("/explorer/area/w", width,width);
  
  nodehandle_private.param("frontier_detector/frontier_detector", frontier_detector,frontier_detector);
  nodehandle_private.param("map/explorer_mode", m_explorer_mode,m_explorer_mode);
  nodehandle_private.param("map/occupancy_map_min_x", m_occupancyMinX,m_occupancyMinX);
  nodehandle_private.param("map/occupancy_map_min_y", m_occupancyMinY,m_occupancyMinY);
  nodehandle_private.param("map/occupancy_map_min_z", m_occupancyMinZ,m_occupancyMinZ);
  nodehandle_private.param("map/occupancy_map_max_x", m_occupancyMaxX,m_occupancyMaxX);
  nodehandle_private.param("map/occupancy_map_max_y", m_occupancyMaxY,m_occupancyMaxY);
  nodehandle_private.param("map/occupancy_map_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  nodehandle_private.param("map/occupancy_map_resolution", map_resolution, map_resolution);
  nodehandle_private.param("map/concave_tightness", concave_alpha, concave_alpha);
  nodehandle_private.param("map/frame_id", m_worldFrameId, m_worldFrameId);
  m_mapPub = nodehandle.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, true);
  ROS_INFO("Occupancy Map with Resolution %f and Z (MinZ,MaxZ) : (%f,%f)",map_resolution,m_occupancyMinZ,m_occupancyMaxZ);
  ROS_INFO("Concave Hull is computed with alpha %f",concave_alpha);
  pointCloudSub = nodehandle.subscribe<sensor_msgs::PointCloud2>("pointcloud_input", 5, &OccupancyMap::pointCloudCallback, this);
  Cloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
  nodehandle_private.param("map_merger/merger_mode", m_merger_mode,m_merger_mode);
  
  
  
  if(m_merger_mode)
  {
    std::string OCC_MAP_TOPIC = "octomap_server/merged_map";
    
    nodehandle_private.param("/explorer/uav_count", uav_count,uav_count);
    ROS_INFO("height : %f width : %f UAV count : %d",height,width,uav_count);
    team_members.resize(uav_count-1);
    team_occmaps.resize(uav_count-1);
    team_occ_map_subscribers.resize(uav_count-1);
    merged_occupancy_matrix = cv::Mat::zeros(height/map_resolution,width/map_resolution, CV_8S) - 1; //need to get hxw from the params
    int count = 0;
    for (size_t id = 1; id <= uav_count; id++)
    {
      std::string current_uav_name = "uav"+std::to_string(id);
      if(uav_name.substr(1) != current_uav_name)
      {
        team_members.at(count)=current_uav_name;
        team_occ_map_subscribers.at(count)=nodehandle.subscribe<nav_msgs::OccupancyGrid>("/"+current_uav_name+"/"+OCC_MAP_TOPIC, 10, [count, this](const nav_msgs::OccupancyGrid::ConstPtr& msg) {OccupancyMap::occupancyMapCallback(msg, count);});
        count++;
      }
    }
    mergedPub = nodehandle.advertise<nav_msgs::OccupancyGrid>("merged_map", 5, true);
  }
  if(frontier_detector)
  {
    m_frontierPub = nodehandle.advertise<visualization_msgs::MarkerArray>("merged_frontiers", 5, true);
  }
}


void OccupancyMap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, *Cloud);
  publishProjected2DMap(Cloud);
}

void OccupancyMap::occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg,int id)
{
  team_occmaps.at(id)=*msg;
}


void OccupancyMap::publishProjected2DMap(pcl::PointCloud<PointF>::Ptr Cloud){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = Cloud->size();
  occupancy_map.info.resolution = map_resolution;
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
  ROS_INFO("Map publishing in OccupancyMap took %f sec", total_elapsed);
  
  if(m_merger_mode)
    {
      mapmerger.map_resolution = map_resolution;
      mapmerger.height = height;
      mapmerger.width = width;
      invokeMerger();
      if(frontier_detector)
      {
        ros::WallTime startTime = ros::WallTime::now();
        nodehandle_private.param("frontier_detector/enable_frontier_weights",mapmerger.enable_frontier_weights,mapmerger.enable_frontier_weights);
        nodehandle_private.param("frontier_detector/extreme_square",mapmerger.extreme_square,mapmerger.extreme_square);
        nodehandle_private.param("frontier_detector/frontier_resolution",mapmerger.frontier_resolution,mapmerger.frontier_resolution);
        nodehandle_private.param("frontier_detector/mass_square",mapmerger.mass_square,mapmerger.mass_square);
        m_frontierPub.publish(mapmerger.generateFrontiers(merged_occupancy_matrix,occupancy_map.header));
        ROS_INFO("Frontier publishing took %f sec", (ros::WallTime::now() - startTime).toSec());
      }
    }
  }

void OccupancyMap::invokeMerger()
{
  ros::WallTime startTime = ros::WallTime::now();
      if(!intialize_merged_occupancy_map)
        {
          mapmerger.Initialize(nodehandle,team_members);          
          intialize_merged_occupancy_map = true;
        }
        merged_occupancy_matrix = mapmerger.AddToFinalMap( occupancy_map, merged_occupancy_matrix);
        for (size_t other_id = 0;other_id < team_members.size(); ++other_id) 
        {
            if(mapmerger.checkDistance(other_id))
            {
              merged_occupancy_matrix = mapmerger.AddToFinalMap( team_occmaps.at(other_id), merged_occupancy_matrix);
            }
        }
    mergedPub.publish(mapmerger.return_merged_map(merged_occupancy_matrix));
  ROS_INFO("Map publishing in OccupancyMap took %f sec", (ros::WallTime::now() - startTime).toSec());
}

void OccupancyMap::getOccupiedLimits(pcl::PointCloud<PointF>::Ptr Cloud)
{
  // Takes in a pointcloud and limits the cloud and generates an occupancy grid. Explorer word works with MRS 

  filteredVoxelCloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
  convexCloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);
  downsampledCloud = pcl::PointCloud<PointF>::Ptr(new pcl::PointCloud<PointF>);

  //crop pointcloud
  ROS_DEBUG("Cropping");
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_occupancyMinZ,m_occupancyMaxZ);
  pass.setInputCloud(this->Cloud);
  pass.filter(*filteredVoxelCloud);
  pass.setFilterFieldName("x");
  //Needed to do m_occupancyMaxX+1 and m_occupancyMaxY+1 for a proper cropping. Else the limits were mismatched. Alternate solution welcomed
  pass.setFilterLimits(m_occupancyMinX,m_occupancyMaxX+1);
  pass.setInputCloud(filteredVoxelCloud);
  pass.filter(*filteredVoxelCloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(m_occupancyMinY,m_occupancyMaxY+1);
  pass.setInputCloud(filteredVoxelCloud);
  pass.filter(*filteredVoxelCloud);
  //downsample pointcloud
  ROS_DEBUG("Downsampling");
  sor.setInputCloud(filteredVoxelCloud);
  sor.setLeafSize(map_resolution,map_resolution,map_resolution);
  sor.filter(*downsampledCloud);
  //get concave points
  ROS_DEBUG("Concave Check");
  chull.setInputCloud(downsampledCloud);
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
    occupancy_map.info.width = static_cast<unsigned int>(std::abs((max_occupied.x - min_occupied.x))/map_resolution);
    occupancy_map.info.height = static_cast<unsigned int>(std::abs((max_occupied.y - min_occupied.y))/map_resolution);
    occupancy_map.info.resolution = map_resolution;
    occupancy_map.info.origin.position.x = min_occupied.x;
    occupancy_map.info.origin.position.y = min_occupied.y;
    ROS_DEBUG("occupancy map ready");
    cv::Mat binary_occupancy_map(occupancy_map.info.height,occupancy_map.info.width,CV_8SC1, cv::Scalar(-1));
    std::vector<cv::Point> concave_occupancy;
    std::vector<std::vector<cv::Point>> polygons;
    ROS_DEBUG("CV processing");
    for (const auto& point : polygonVertices) 
    {
      concave_occupancy.push_back(cv::Point((std::abs((point.x - min_occupied.x))/map_resolution),(std::abs((point.y - min_occupied.y))/map_resolution)));
    }
    polygons.push_back(concave_occupancy);
    ROS_DEBUG("CV processing complete");
    cv::fillPoly(binary_occupancy_map, polygons, cv::Scalar(0));
    ROS_DEBUG("Filling polygon");
    if(!m_explorer_mode)
    {
      for (const auto& occupied_center : *filteredVoxelCloud){      
        uchar* ptr = binary_occupancy_map.ptr<uchar>(static_cast<unsigned int>(std::abs((occupied_center.y - min_occupied.y))/map_resolution));
        ptr[static_cast<unsigned int>(std::abs((occupied_center.x - min_occupied.x))/map_resolution)] = 100;
      }
    }
    
    ROS_DEBUG("Making CV occgrid");
    std::vector<int8_t> occupancyGridData(binary_occupancy_map.begin<uchar>(), binary_occupancy_map.end<uchar>());
    occupancy_map.data = occupancyGridData;
    
    ROS_DEBUG("occgrid ready");
}
