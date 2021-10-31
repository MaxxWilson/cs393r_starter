//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/math/statistics.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;
using pose_2d::Pose2D;

namespace slam {

CONFIG_FLOAT(dist_update_thresh, "dist_update_thresh");
CONFIG_FLOAT(angle_update_thresh, "angle_update_thresh");
CONFIG_FLOAT(dist_res, "dist_res");
CONFIG_FLOAT(theta_res, "theta_res");
CONFIG_INT(map_size, "map_size");
CONFIG_FLOAT(laser_offset, "laser_offset");
CONFIG_FLOAT(map_length_dist, "map_length_dist");
CONFIG_DOUBLE(gamma, "gamma");


CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    cost_map_initialized(false){
  map_.reserve(1000000);
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = curr_odom_loc_;
  *angle = curr_odom_angle_;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        float angle_increment) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  // 
  if (!odom_initialized_) {
    return;
  }
  if(odom_initialized_ && !cost_map_initialized){
    cost_map = costmap::CostMap();
    cost_map.UpdateMap(ranges, range_min, range_max, angle_min, angle_max, angle_increment);
    BuildMapFromScan(ranges, range_min, range_max, angle_min, angle_max, angle_increment, poses.back());
      
    // Reset odom reference to new pose
    prev_odom_loc_ = curr_odom_loc_;
    prev_odom_angle_ = curr_odom_angle_;
    cost_map_initialized = true;
    return;
  }
  // Get odometry change since last pose update
  Vector2f odom_loc_diff = curr_odom_loc_ - prev_odom_loc_;
  double odom_dist = odom_loc_diff.norm();
  float odom_angle_diff = AngleDiff(curr_odom_angle_, prev_odom_angle_);

  // See if we have moved far enough to update
  if(odom_dist > CONFIG_dist_update_thresh || abs(odom_angle_diff) > CONFIG_angle_update_thresh) {
    double start = GetMonotonicTime();
    
    double P = -1e10;   // Log Probability of pose
    Pose2D<float> T;  // Transform to new Pose

    // Calculate standard deviations for motion model based on odom from last pose
    float trans_sigma = CONFIG_k1*odom_dist + CONFIG_k2*abs(odom_angle_diff);
    float angle_sigma = CONFIG_k3*odom_dist + CONFIG_k4*abs(odom_angle_diff);

    // Set search region to 3 standard deviations from mean to cover 99.73% of values
    float theta_low = (2*angle_sigma < 3.14159) ? -2*angle_sigma : 0;
    double theta_high = (2*angle_sigma < 3.14159) ? 2*angle_sigma : 2*3.14159;
    float translation_low = (2*trans_sigma < CONFIG_map_length_dist) ? -2*trans_sigma : 0;
    float translation_high = (2*trans_sigma < CONFIG_map_length_dist) ? 2*trans_sigma : CONFIG_map_length_dist;

    // Iterate over possible change in theta
    for(int angle_index = 0; angle_index < (theta_high - theta_low) / CONFIG_theta_res; angle_index++){
      float dtheta = theta_low + angle_index * CONFIG_theta_res;
      // Convert Point Cloud to Cartesian
      static vector<Vector2f> cloud(ranges.size());
      for(std::size_t i = 0; i < ranges.size(); i++){
        // Polar to Cartesian conversion, transforms to base link frame
        float angle = angle_min + angle_increment*i + odom_angle_diff + dtheta;
        float x = ranges[i]*cos(angle) + 0.2;
        float y = ranges[i]*sin(angle);
        cloud[i] = Vector2f(x, y);
      }
      
      // Iterate over possible changes in X and Y
      for(int x_index = 0; x_index < (translation_high - translation_low) / CONFIG_dist_res; x_index++){
        float dx = translation_low + x_index * CONFIG_dist_res;
        for(int y_index = 0; y_index < (translation_high - translation_low) / CONFIG_dist_res; y_index++){
          float dy = translation_low + y_index * CONFIG_dist_res;
          double pose_log_prob = 0.0;
          
          // For each XY point in scan, get log likelihood from CSM
          // Pose2D<float> curPos(dtheta + curr_odom_angle_, dx + curr_odom_loc_.x(), dy + curr_odom_loc_.y());
          for(std::size_t i = 0; i < cloud.size(); i++){
              Vector2f point = cloud[i];
              // Transform the new scan to the previos form?
              // Vector2f trans_diff = curPos.translation - poses.back().translation;
              // float angle_diff = AngleDiff(curPos.angle, poses.back().angle);
              // Eigen::Rotation2Df R_oldPos2Map(-poses.back().angle);
              // Eigen::Rotation2Df R_NewPos2OldPos(angle_diff);
              // Vector2f scanPos = R_NewPos2OldPos * point + R_oldPos2Map * trans_diff;
              //std::cout << cost_map.GetLikelihoodAtPosition(dx + point.x(), dy + point.y()) << std::endl;
              // std::cout << cost_map.max_likelihood << std::endl;
              double gaussian_prob = cost_map.GetLikelihoodAtPosition(dx + point.x(), dy + point.y());
              pose_log_prob -= CONFIG_gamma*std::log(gaussian_prob);
              // newP += cost_map.GetLikelihoodAtPosition(scanPos.x(), scanPos.y());
          }
          
          // compute motion model likelihood
          double log_px = -Sq(dx - odom_loc_diff.x())/Sq(trans_sigma);
          double log_py = -Sq(dy - odom_loc_diff.y())/Sq(trans_sigma);
          double log_ptheta = -Sq(dtheta - odom_angle_diff)/Sq(angle_sigma);

          std::cout << "P, M, dx, dy, dtheta: " << pose_log_prob << ", " << log_px << ", " << log_py << ", " << log_ptheta << ", " << dx << ", " << dy << ", " << dtheta << std::endl;

          // Update most likely transform
          if(P < pose_log_prob + log_px + log_py + log_ptheta){
            P = pose_log_prob + log_px + log_py + log_ptheta;
            T = Pose2D<float>(dtheta, Eigen::Vector2f(dx, dy));
            // T = Pose2D<float>(AngleDiff(dtheta + curr_odom_angle_, poses.front().angle), Eigen::Vector2f(dx, dy));
          }
        }
      }
    }

    // Update cost map based on new laser scan
    cost_map.UpdateMap(ranges, range_min, range_max, angle_min, angle_max, angle_increment);

    // Append new pose to list of poses
    Pose2D<float> latest_pose(poses.back());
    latest_pose.ApplyPose(T);
    poses.push_back(latest_pose);

    //std::cout << "New Pose: " << latest_pose.translation.x() << ", " << latest_pose.translation.y() << ", " << latest_pose.angle << std::endl;

    // Update the enivronment map with new scan and pose
    BuildMapFromScan(ranges, range_min, range_max, angle_min, angle_max, angle_increment, poses.back());

    // Reset odom reference to new pose
    prev_odom_loc_ = curr_odom_loc_;
    prev_odom_angle_ = curr_odom_angle_;
  
  double end = GetMonotonicTime();
  std::cout << "Update Laser: " << end - start << std::endl;
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    curr_odom_loc_ = odom_loc;
    curr_odom_angle_ = odom_angle;

    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    
    odom_initialized_ = true;

    Pose2D<float> init_pose(odom_angle, odom_loc);
    poses.push_back(init_pose);

    std::cout << "Pose Count: " << poses.size() << std::endl;

    return;
  }

  curr_odom_loc_ = odom_loc;
  curr_odom_angle_ = odom_angle;
}
void SLAM::BuildMapFromScan(const vector<float>& ranges,
                     float range_min,
                     float range_max,
                     float angle_min,
                     float angle_max,
                     float angle_increment,
                     const Pose2D<float> MLE_pose) {
  for(std::size_t i = 0; i < ranges.size(); i++){
    // Polar to Cartesian conversion, transforms to base link frame
    float angle = angle_min + angle_increment*i;
    float x = ranges[i]*cos(angle) + CONFIG_laser_offset;
    float y = ranges[i]*sin(angle);
    if(ranges[i] >= range_max) {
      continue;
    }
    // Rotate into map frame, then translate according to pose
    Eigen::Rotation2Df pose_to_map(MLE_pose.angle);
    Eigen::Vector2f scan_in_map = pose_to_map.toRotationMatrix()*Vector2f(x, y) + MLE_pose.translation;
    
    map_.push_back(scan_in_map);
  }
  
}
vector<Vector2f> SLAM::GetMap() {
  int size = std::min(CONFIG_map_size, (int) map_.size());
  vector<Vector2f> sparse_map(size);


  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  // int incr = ceil(map_.size() / map_.size());
  // for(int i = 0; i < size; i ++) {
  //   sparse_map[i] = map_[i * incr];
  // }
  // return sparse_map;
  return map_;
}

}  // namespace slam
