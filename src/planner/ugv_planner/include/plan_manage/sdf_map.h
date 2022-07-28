/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "plan_manage/voxel_map.hpp"

#define logit(x) (log((x) / (1 - (x))))

using namespace std;



class SDFMap {
public:
  // SDFMap() {}
  // ~SDFMap() {}
  int esdf_vis_slice_z;
  SDFMap() = default;
        SDFMap(const double &esdf_vis_slice_height_)//resolution
            : esdf_vis_slice_z(esdf_vis_slice_height_) {}
  voxel_map::VoxelMap::Ptr voxelMapptr_for_esdf;
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  void updateESDF3d();
  double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  void vis_esdf_Callback(const ros::TimerEvent& /*event*/);

  double resolution_;
  Eigen::Vector3i buffer_size;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
  vector<double> distance_buffer_positive,distance_buffer_negtive,distance_buffer_;
  void setEnvironment(voxel_map::VoxelMap::Ptr & global_map_ptr){
      voxelMapptr_for_esdf = global_map_ptr;
      resolution_=voxelMapptr_for_esdf->getScale();
      buffer_size=voxelMapptr_for_esdf->getSize();
      tmp_buffer1_= vector<double>(buffer_size[0]*buffer_size[1]*buffer_size[2], 0);
      tmp_buffer2_= vector<double>(buffer_size[0]*buffer_size[1]*buffer_size[2], 0);
      distance_buffer_positive= vector<double>(buffer_size[0]*buffer_size[1]*buffer_size[2], 0);
      distance_buffer_negtive= vector<double>(buffer_size[0]*buffer_size[1]*buffer_size[2], 0);
      distance_buffer_= vector<double>(buffer_size[0]*buffer_size[1]*buffer_size[2], 0);
      updateESDF3d();
  }
  void initMap(ros::NodeHandle& nh);
  int toAddress(int x,int y,int z){
    return x*buffer_size[1]*buffer_size[2]+ y*buffer_size[2]+z;
  }

  double getDistance( Eigen::Vector3i& id) 
  {
    int number=toAddress(id(0),id(1),id(2));
    return distance_buffer_[number];
  }
  double getDistance( Eigen::Vector3d& pos) 
  {
    Eigen::Vector3i id=voxelMapptr_for_esdf->posD2I(pos);
    int number=toAddress(id(0),id(1),id(2));
    return distance_buffer_[number];
  }
  typedef std::shared_ptr<SDFMap> Ptr;

  ros::Publisher grad_pub;
private:
  ros::Publisher esdf_pub_;
  ros::Timer esdfpub_timer_;
};





#endif