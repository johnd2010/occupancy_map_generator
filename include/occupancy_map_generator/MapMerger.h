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


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>

#include <std_srvs/Empty.h>

#include <opencv2/opencv.hpp>


#include <vector>
#include "ros/console.h"
#include "ros/publisher.h"

#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"


//Add communication distance limits
//Change to params for resolution, height, width , topic names,start and end point
//if time permits, frame transfomration and params frame_id
// time permits, merge with obstacles as well, we assume that we are getting a space limited occupancy map
//Add pose topic for distance calculation
// Change it for a single agent for real world experiments or add a param for real world experiments that take only the current uav


class MapMerger {

public:

  std::vector<nav_msgs::OccupancyGrid::ConstPtr> occupancy_maps;
  std::vector<std::string> uav_names;
  std:: string current_name;
  cv::Rect extreme_bounding_box;

  nav_msgs::OccupancyGrid Initialize(const ros::NodeHandle &nh_,std::vector<std::string> uav_names);
  std::vector<cv::Point> interpolateLine(const cv::Point& p1, const cv::Point& p2,float resolution);
  std::vector<float> mass_array;
  void Merge(nav_msgs::OccupancyGrid::ConstPtr map);
  bool checkDistance(int  other_id);
  visualization_msgs::MarkerArray generateFrontiers(cv::Mat matrix,std_msgs::Header header);
  void reset(const ros::NodeHandle &n,std::string frame);
  visualization_msgs::MarkerArray frontier_generator(std::vector<std::vector<cv::Point>> contours,std_msgs::Header header,cv::Mat matrix);
  cv::Mat AddToFinalMap(nav_msgs::OccupancyGrid current_occupancy_map,cv::Mat merged_occupancy_matrix);
  nav_msgs::OccupancyGrid return_merged_map(cv::Mat merged_occupancy_matrix);
  float mass_calculator(cv::Mat matrix,cv::Point point);
  ros::Subscriber m_mapSub;



protected:
  std::string worldFrameId;
  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;
  void generateOccupancyMap();


  ros::NodeHandle nodehandle;
  ros::NodeHandle nodehandle_private;
  ros::Publisher  m_mapPub;



  double concave_alpha;
  double m_res=0.5,height=60,width=60,origin_x=0,origin_y=0,distance=50,resolution=0.5,extreme_square = 5;
  int mass_square = 5;
  bool enable_frontier_weights = true;


  double m_occupancyMinX,m_occupancyMinY,m_occupancyMinZ;
  double m_occupancyMaxX,m_occupancyMaxY,m_occupancyMaxZ;
  bool m_explorer_mode;

  // downprojected 2D map:
  bool initial_check=true;  

  nav_msgs::OccupancyGrid merged_occupancy_map;
  cv::Mat merged_occupancy_matrix;

  std::vector<cv::Mat> occupancy_matrices;
  
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  std::vector<ros::Publisher> map_publishers,marker_array_publishers,random_publisher;
  std::string topic;
  std::vector<geometry_msgs::PoseArray> frontier_pose_arrays;
  cv::Mat mergedContours;
  visualization_msgs::MarkerArray current_marker_array;
  visualization_msgs::Marker current_marker;
  cv::Mat temp_matrix_frontier;
  float mass;
  float max_mass;
};

