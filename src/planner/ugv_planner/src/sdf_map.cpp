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



#include "plan_manage/sdf_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void SDFMap::initMap(ros::NodeHandle& nh) {
  esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  esdfpub_timer_ = nh.createTimer(ros::Duration(1), &SDFMap::vis_esdf_Callback, this);
  grad_pub=nh.advertise<visualization_msgs::Marker>("esdf/grad", 1);
}



template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[buffer_size[dim]];
  double z[buffer_size[dim] + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  //从最小的index开始填到最大的，是否占用有voxelMapptr_for_esdf->query( Eigen::Vector3i);
  Eigen::Vector3i min_esdf(0,0,0);//md_是map_Data,min_esdf和max_esdf是局部范围
  Eigen::Vector3i max_esdf(buffer_size[0]-1,buffer_size[1]-1,buffer_size[2]-1);
  Eigen::Vector3i temp_index;
  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            temp_index[0]=x;temp_index[1]=y;temp_index[2]=z;
            return voxelMapptr_for_esdf->query(temp_index) == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[toAddress(x,y,z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { 
        return tmp_buffer1_[toAddress(x, y, z)]; 
        },
               [&](int y, double val) { tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 distance_buffer_positive[toAddress(x, y, z)] = resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        temp_index[0]=x;temp_index[1]=y;temp_index[2]=z;
        if (voxelMapptr_for_esdf->query(temp_index) == 0) {
          distance_buffer_negtive[idx] = 1;

        } else if (voxelMapptr_for_esdf->query(temp_index)== 1) {
          distance_buffer_negtive[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {//free则 f为0  occ则f为max  找的是距离最近的free栅格
            return distance_buffer_negtive[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 distance_buffer_negtive[toAddress(x, y, z)] =resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        distance_buffer_[idx] = distance_buffer_positive[idx];

        if (distance_buffer_negtive[idx] > 0.0)
          distance_buffer_[idx] += (-distance_buffer_negtive[idx] + resolution_);
      }
}


void SDFMap::vis_esdf_Callback(const ros::TimerEvent& /*event*/)
{
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 1.0;

  for (int x = 0; x <= buffer_size(0)-1; ++x)
    for (int y = 0; y <= buffer_size(1)-1; ++y)
      {  
      int z=esdf_vis_slice_z;
      Eigen::Vector3i pos_index(x,y,z);
      dist = getDistance(pos_index);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      Eigen::Vector3d pos=voxelMapptr_for_esdf->posI2D(pos_index);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);
}


 double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
  // if (!isInMap(pos)) {
  //   grad.setZero();
  //   return 0;
  // }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx=voxelMapptr_for_esdf->posD2I(pos_m);

  Eigen::Vector3d idx_pos, diff;
  idx_pos=voxelMapptr_for_esdf->posI2D(idx);
  double resolution_inv_=1.0/resolution_;
  diff = (pos - idx_pos) * resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }
//diff[0]相当于t   (1-t)*A+t*B
//v00 v01 v10 v11是算a,c,b,d这个 面  
//v0 v1是算 面上ac边和bd边的插值
//dist是算v0 v1连线插值
  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= resolution_inv_;

  return dist;
}



// SDFMap
