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



#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/console.h"
#include <occupancy_map_generator/MapMerger.h>
#include <string>
#include <vector>
#include "ros/forwards.h"
#include "std_msgs/String.h"

// MapMerger::MapMerger( const ros::NodeHandle &nh_,std::vector<std::string> uav_names)
// : nodehandle(nh_),
//   uav_names(uav_names),
//   nodehandle_private(nh_),
//   worldFrameId("map")
//   // m_occupancyMinX(-std::numeric_limits<double>::min()),
//   // m_occupancyMinY(-std::numeric_limits<double>::min()),
//   // m_occupancyMinZ(-std::numeric_limits<double>::min()),
//   // m_occupancyMaxX(std::numeric_limits<double>::max()),
//   // m_occupancyMaxY(std::numeric_limits<double>::max()),
//   // m_occupancyMaxZ(std::numeric_limits<double>::max())
// {
  
// }

nav_msgs::OccupancyGrid MapMerger::Initialize(const ros::NodeHandle &nh_,std::vector<std::string> uav_names)
// void MapMerger::Initialize(const ros::NodeHandle &nh_,std::vector<std::string> uav_names,int current_uav,nav_msgs::OccupancyGrid::ConstPtr current_occupancy_map)
{
  nodehandle = nh_;
  nodehandle_private = nh_;
  current_name = ros::this_node::getNamespace().substr(1);
  // nodehandle_private.param("map/occupancy_map_min_x", m_occupancyMinX,m_occupancyMinX);
  // nodehandle_private.param("map/occupancy_map_min_y", m_occupancyMinY,m_occupancyMinY);
  // nodehandle_private.param("map/occupancy_map_min_z", m_occupancyMinZ,m_occupancyMinZ);
  // nodehandle_private.param("map/occupancy_map_max_x", m_occupancyMaxX,m_occupancyMaxX);
  // nodehandle_private.param("map/occupancy_map_max_y", m_occupancyMaxY,m_occupancyMaxY);
  // nodehandle_private.param("map/occupancy_map_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  // nodehandle_private.param("map/occupancy_map_resolution", m_res, m_res);
  // nodehandle_private.param("map/frame_id", worldFrameId, worldFrameId);

  occupancy_maps.resize(uav_names.size());
  occupancy_matrices.resize(uav_names.size());
  merged_occupancy_map.header.frame_id=current_name+"/world_origin";
  merged_occupancy_map.info.height = height/m_res;
  merged_occupancy_map.info.width = width/m_res;
  merged_occupancy_map.info.resolution = m_res;
  merged_occupancy_map.info.origin.position.x = origin_x;
  merged_occupancy_map.info.origin.position.y = origin_y;
  mergedContours = cv::Mat::zeros(height/m_res,width/m_res, CV_8UC1);  //blank image for contour processing


  // merged_occupancy_matrix.resize(uav_names.size());
  // merged_occupancy_map.resize(uav_names.size());
  // frontier_pose_arrays.resize(uav_names.size());
  
  // marker_array_publishers.resize(uav_names.size());
  // random_publisher.resize(uav_names.size());


  // for (size_t id = 0; id < uav_names.size(); ++id)
  // {
  //   occupancy_maps[id] = nullptr;
  //   merged_occupancy_matrix[id] = cv::Mat::zeros(height/m_res,width/m_res, CV_8S)-1; //need to get hxw from the params
  //   merged_occupancy_map[id].header.frame_id=uav_names[id]+"/world_origin";
  //   merged_occupancy_map[id].info.height = height/m_res;
  //   merged_occupancy_map[id].info.width = width/m_res;
  //   merged_occupancy_map[id].info.resolution = m_res;
  //   merged_occupancy_map[id].info.origin.position.x = origin_x;
  //   merged_occupancy_map[id].info.origin.position.y = origin_y;
  //   frontier_pose_arrays[id].header.frame_id = uav_names[id]+"/world_origin";
  //   topic = "/"+ uav_names[id] + "/octomap_server/projected_map";
  //   ROS_DEBUG("%s",topic.c_str());
  //   // map_publishers.push_back(nodehandle.advertise<nav_msgs::OccupancyGrid>("/"+uav_names[id]+"/merged_map", 5, true));
  //   marker_array_publishers.push_back(nodehandle.advertise<geometry_msgs::PoseStamped>("/"+uav_names[id]+"/frontiers",1, true));
  //   random_publisher.push_back(nodehandle.advertise<std_msgs::String>("/"+uav_names[id]+"/string",0));
  // }
  return merged_occupancy_map;
}


bool MapMerger::checkDistance(int  other_id) //Add pose topic
{
return true;
}

void assign_map(nav_msgs::OccupancyGrid::ConstPtr current_occupancy_map, int id)
{
}


// void MapMerger::Merge(nav_msgs::OccupancyGrid::ConstPtr current_occupancy_map)
// {
//   int id = 0;
//   //add communication constraint here
//   ros::WallTime startTime = ros::WallTime::now();
//   merged_occupancy_map[id].header.stamp = ros::Time::now();
//   AddToFinalMap( current_occupancy_map, id);
//   for (size_t other_id = 0;other_id < uav_names.size(); ++other_id) 
//   {
//     if(id!=other_id)
//     {
//       if(checkDistance(id,other_id))
//       {
//         AddToFinalMap( current_occupancy_map, other_id);
//       }
//     }
//   }
//   std::vector<std::vector<cv::Point>> contours = generateFrontiers(merged_occupancy_matrix[id].clone(),id);
//   std::vector<int8_t> occupancyGridData(merged_occupancy_matrix[id].begin<uchar>(), merged_occupancy_matrix[id].end<uchar>());
//   merged_occupancy_map[id].data = occupancyGridData;
//   // map_publishers[id].publish(merged_occupancy_map[id]);
//   std_msgs::String msg;
//   msg.data = uav_names[id].c_str();
  
//   random_publisher[id].publish(msg);
  
//   frontier_pose_arrays[id] = frontier_generator(contours,merged_occupancy_map[id].header);
//   geometry_msgs::PoseStamped pose;
//   pose.header = merged_occupancy_map[id].header;
//   marker_array_publishers[id].publish(pose);
//   // ROS_INFO_STREAM(frontier_pose_arrays[id]);
//   ROS_INFO_STREAM(pose);
//   ROS_INFO("%f",frontier_pose_arrays[id].poses.at(0).position.x);
//   ros::WallDuration totalElapsed = ros::WallTime::now() - startTime;
//   ROS_INFO("Merging the maps took %f seconds", totalElapsed.toSec());

// }

cv::Mat MapMerger::AddToFinalMap(nav_msgs::OccupancyGrid current_occupancy_map,cv::Mat merged_occupancy_matrix)
{
  for (int row = 0; row < current_occupancy_map.info.height; ++row) {
    for (int col = 0; col < current_occupancy_map.info.width; ++col) {
        // Calculate the index in the 1D data array
        int index = row * current_occupancy_map.info.width + col;
        if(merged_occupancy_matrix.at<int8_t>((current_occupancy_map.info.origin.position.y/m_res)+row, (current_occupancy_map.info.origin.position.x/m_res)+col)==-1)
          merged_occupancy_matrix.at<int8_t>((current_occupancy_map.info.origin.position.y/m_res)+row, (current_occupancy_map.info.origin.position.x/m_res)+col) =  current_occupancy_map.data[index];
    }
  }
return merged_occupancy_matrix;
}
nav_msgs::OccupancyGrid MapMerger::return_merged_map(cv::Mat merged_occupancy_matrix)
{
    std::vector<int8_t> occupancyGridData(merged_occupancy_matrix.begin<uchar>(), merged_occupancy_matrix.end<uchar>());
    merged_occupancy_map.data = occupancyGridData;
    merged_occupancy_map.header.stamp = ros::Time::now();
    return merged_occupancy_map;
}

geometry_msgs::PoseArray MapMerger::generateFrontiers(cv::Mat matrix,std_msgs::Header header)
{
    matrix.setTo(255, matrix == 0);
    matrix.setTo(0, matrix == -1);
    matrix.convertTo(matrix, CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> interpolatedContours;
    cv::findContours(matrix, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);   
    interpolatedContours.resize(contours.size());
    for (const auto& contour : contours) {
      for (int i=0;i<contour.size();i++)
      if(i==contour.size()-1)
        interpolatedContours.push_back(interpolateLine(contour[i],contour[0],0.5));
      else
        interpolatedContours.push_back(interpolateLine(contour[i],contour[i+1],0.5));
    }
    return frontier_generator(interpolatedContours,header);
}

std::vector<cv::Point> MapMerger::interpolateLine(const cv::Point& p1, const cv::Point& p2,float resolution) {
    std::vector<cv::Point> linePoints;
    float distance = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    int numPoints = static_cast<int>(distance / resolution);
    for (int i = 0; i <= numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints;
        int x = static_cast<int>((1 - t) * p1.x + t * p2.x);
        int y = static_cast<int>((1 - t) * p1.y + t * p2.y);
        linePoints.emplace_back(x, y);
    }
    return linePoints;
}
geometry_msgs::PoseArray MapMerger::frontier_generator(std::vector<std::vector<cv::Point>> contours,std_msgs::Header header)
{
  geometry_msgs::PoseArray current_pose_array;
  geometry_msgs::Pose current_pose;
  for (const auto& contour : contours) {
    for (const cv::Point& point : contour) {
      current_pose.position.x = point.x * m_res;
      current_pose.position.y = point.y * m_res;
      current_pose_array.poses.push_back(current_pose);
      }
    }
  current_pose_array.header = header;
  
  return current_pose_array;
}