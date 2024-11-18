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
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/console.h"
#include <algorithm>
#include <occupancy_map_generator/MapMerger.h>
#include <string>
#include <vector>
#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

nav_msgs::OccupancyGrid MapMerger::Initialize(const ros::NodeHandle &nh_,std::vector<std::string> uav_names)
{
  nodehandle = nh_;
  nodehandle_private = nh_;
  current_name = ros::this_node::getNamespace().substr(1);
  
  //Unchanging Marker parameters 
  current_marker.pose.orientation.x = 0;
  current_marker.pose.orientation.y = 0;
  current_marker.pose.orientation.z = 0;
  current_marker.pose.orientation.w = 1;
  current_marker.scale.x=1;
  current_marker.scale.y=1;
  current_marker.scale.z=1;
  current_marker.color.a= 1;
  current_marker.color.r= 1;
  current_marker.color.g= 1;
  current_marker.color.b= 1;
  current_marker.lifetime = ros::Duration();
  current_marker.type = current_marker.SPHERE;

  occupancy_maps.resize(uav_names.size());
  occupancy_matrices.resize(uav_names.size());
  merged_occupancy_map.header.frame_id=current_name+"/world_origin";
  merged_occupancy_map.info.height = height/map_resolution;
  merged_occupancy_map.info.width = width/map_resolution;
  merged_occupancy_map.info.resolution = map_resolution;
  merged_occupancy_map.info.origin.position.x = origin_x;
  merged_occupancy_map.info.origin.position.y = origin_y;
  mergedContours = cv::Mat::zeros(height/map_resolution,width/map_resolution, CV_8UC1);  //blank image for contour processing
  return merged_occupancy_map;
}


bool MapMerger::checkDistance(int  other_id) //Add pose topic
{
return true;
}



cv::Mat MapMerger::AddToFinalMap(nav_msgs::OccupancyGrid current_occupancy_map,cv::Mat merged_occupancy_matrix)
{
  for (int row = 0; row < current_occupancy_map.info.height; ++row) {
    for (int col = 0; col < current_occupancy_map.info.width; ++col) {
        // Calculate the index in the 1D data array
        int index = row * current_occupancy_map.info.width + col;
        if(merged_occupancy_matrix.at<int8_t>((current_occupancy_map.info.origin.position.y/map_resolution)+row, (current_occupancy_map.info.origin.position.x/map_resolution)+col)==-1)
          merged_occupancy_matrix.at<int8_t>((current_occupancy_map.info.origin.position.y/map_resolution)+row, (current_occupancy_map.info.origin.position.x/map_resolution)+col) =  current_occupancy_map.data[index];
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

visualization_msgs::MarkerArray MapMerger::generateFrontiers(cv::Mat matrix,std_msgs::Header header)
{
    temp_matrix_frontier = matrix.clone();
    temp_matrix_frontier.setTo(255, temp_matrix_frontier == 0);
    temp_matrix_frontier.setTo(0, temp_matrix_frontier == -1);
    temp_matrix_frontier.convertTo(temp_matrix_frontier, CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> interpolatedContours;
    cv::findContours(temp_matrix_frontier, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);   
    interpolatedContours.resize(contours.size());
    for (const auto& contour : contours) {
      for (int i=0;i<contour.size();i++)
      {
        if(i==contour.size()-1)
          interpolatedContours.push_back(interpolateLine(contour[i],contour[0],frontier_resolution));
        else
          interpolatedContours.push_back(interpolateLine(contour[i],contour[i+1],frontier_resolution));
      }
    }
    return frontier_generator(interpolatedContours,header,matrix);
}

std::vector<cv::Point> MapMerger::interpolateLine(const cv::Point& p1, const cv::Point& p2,float resolution) {
    std::vector<cv::Point> linePoints;
    std::vector<float> mass;
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
float MapMerger::mass_calculator(cv::Mat matrix,cv::Point point)
{
int x1 = std::max(point.x - mass_square,0);
int y1 = std::max(point.y - mass_square, 0);
int x2 = std::min(point.x + mass_square, matrix.cols - 1);
int y2 = std::min(point.y + mass_square, matrix.rows - 1);
cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);
cv::Mat roiMat = matrix(roi);
return cv::sum(roiMat)[0]+(roi.size().height*roi.size().width);
}
visualization_msgs::MarkerArray MapMerger::frontier_generator(std::vector<std::vector<cv::Point>> contours,std_msgs::Header header,cv::Mat matrix)
{
  current_marker_array.markers.clear();
  int count = 0;
  mass_array.clear();
  max_mass = 0;
  for (const auto& contour : contours) {
    for (const cv::Point& point : contour) 
    {
      if(point.x>1 && point.y>1 && point.x<matrix.cols-2 && point.y<matrix.rows-2)
        {
          ROS_INFO("%d",matrix.cols);
          ROS_INFO("%d",matrix.rows);
          ROS_INFO("%d",point.x);
          ROS_INFO("%d",point.y);
          if(enable_frontier_weights)
            {
              mass_array.push_back(mass_calculator( matrix, point));
              max_mass = std::max(max_mass,mass_array.back());
            }
          current_marker.pose.position.x = point.x * map_resolution;
          current_marker.pose.position.y = point.y * map_resolution;
          current_marker.header = header;
          current_marker.id = count++;
          current_marker_array.markers.push_back(current_marker);
        }
      }
    }
    if(enable_frontier_weights)
    {
      for(int i=0; i < mass_array.size();i++)
      {
        current_marker_array.markers[i].scale.x = mass_array[i]/max_mass;
        current_marker_array.markers[i].scale.y = mass_array[i]/max_mass;
        current_marker_array.markers[i].scale.z = mass_array[i]/max_mass;
      }
    }
  return current_marker_array;
}