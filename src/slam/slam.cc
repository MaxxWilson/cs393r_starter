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

#include "visualization/CImg.h"
using cimg_library::CImg;
using cimg_library::CImgDisplay;

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
CONFIG_DOUBLE(resize_factor, "resize_factor");
CONFIG_FLOAT(range_max, "range_max");
CONFIG_INT(row_num, "row_num");

CONFIG_FLOAT(theta_search_const, "theta_search_const");
CONFIG_FLOAT(dist_search_const, "dist_search_const");

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
  *loc = current_pose_.translation;
  *angle = current_pose_.angle;
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
  
  static CImg<float> image(CONFIG_row_num, CONFIG_row_num, 1,3,1);

  if (!odom_initialized_) {
    return;
  }
  if(odom_initialized_ && !cost_map_initialized){
    cost_map = costmap::CostMap();

    cloud_ = vector<Vector2f>(ranges.size());

    std::cout << "First Cloud" << std::endl;
    for(std::size_t i = 0; i < ranges.size(); i++){
      // Polar to Cartesian conversion, transforms to base link frame of new pose
      float angle = angle_min + angle_increment*i;
      float x = ranges[i]*cos(angle) + CONFIG_laser_offset;
      float y = ranges[i]*sin(angle);
      cloud_[i] = Vector2f(x, y);
    }

    cost_map.UpdateCostMap(cloud_);
    BuildMapFromScan(cloud_, poses.back());
    cost_map_initialized = true;

    // New cloud + static transform
    // for(int i = 0; i < cloud_.size(); i++){
    //   cloud_[i] = R_NewPos2OldPos.inverse()*(cloud_[i] - trans);
    // }

    //cost_map.DrawCostMap(image);
    
    // DrawPoints(image, cloud_);
    // cost_map.DisplayImage(image);
    return;
  }

  // Get odometry change since last pose update
  auto rot_to_last_pose = Eigen::Rotation2D<float>(-poses.back().angle).toRotationMatrix();
  
  Vector2f odom_loc_diff = rot_to_last_pose*(current_pose_.translation - poses.back().translation);
  double odom_dist = odom_loc_diff.norm();
  float odom_angle_diff = AngleDiff(current_pose_.angle, poses.back().angle);

  // See if we have moved far enough to update
  if(curr_dist_err > CONFIG_dist_update_thresh || abs(odom_angle_diff) > CONFIG_angle_update_thresh) {
    double start = GetMonotonicTime();

    double P = -1e10;   // Log Probability of pose
    Pose2D<float> T;  // Transform to new Pose

    // Calculate standard deviations for motion model based on odom from last pose
    float trans_sigma = CONFIG_k1*odom_dist + CONFIG_k2*abs(odom_angle_diff);
    float angle_sigma = CONFIG_k3*odom_dist + CONFIG_k4*abs(odom_angle_diff);
    // std::cout << "trans sigma: " << trans_sigma << std::endl;
    // std::cout << "angle sigma: " << angle_sigma << std::endl;

    // Set search region to 3 standard deviations from mean to cover 99.73% of values
    double search_factor = 1.0;
    float theta_low = (search_factor*angle_sigma < 3.14159) ? -search_factor*angle_sigma : 0;
    float theta_high = (search_factor*angle_sigma < 3.14159) ? search_factor*angle_sigma : 2*3.14159;
    float translation_low = (search_factor*trans_sigma < CONFIG_map_length_dist) ? -search_factor*trans_sigma : 0;
    float translation_high = (search_factor*trans_sigma < CONFIG_map_length_dist) ? search_factor*trans_sigma : CONFIG_map_length_dist;

    // theta_low = cost_map.RoundToResolution(theta_low - CONFIG_theta_res, CONFIG_theta_res);
    // theta_high = cost_map.RoundToResolution(theta_high + CONFIG_theta_res, CONFIG_theta_res);
    // translation_low = cost_map.RoundToResolution(translation_low - CONFIG_dist_res, CONFIG_dist_res);
    // translation_high = cost_map.RoundToResolution(translation_high + CONFIG_dist_res, CONFIG_dist_res);

    theta_low = -CONFIG_theta_search_const*CONFIG_theta_res;
    theta_high = CONFIG_theta_search_const*CONFIG_theta_res;
    translation_low = -CONFIG_dist_search_const*CONFIG_dist_res;
    translation_high = CONFIG_dist_search_const*CONFIG_dist_res;

    // theta_low = -5*CONFIG_theta_res;
    // theta_high = 5*CONFIG_theta_res;
    // translation_low = -7*CONFIG_dist_res;
    // translation_high = 7*CONFIG_dist_res;

    cloud_.resize(ranges.size());
    for(std::size_t i = 0; i < ranges.size(); i++){
      // Polar to Cartesian conversion, transforms to base link frame of new pose
      float angle = angle_min + angle_increment*i;
      float x = ranges[i]*cos(angle) + CONFIG_laser_offset;
      float y = ranges[i]*sin(angle);
      cloud_[i] = Vector2f(x, y);
    }

    // std::cout << "Num Theta: " << (theta_high - theta_low) / CONFIG_theta_res << std::endl;
    // std::cout << "Num Dist: " << (translation_high - translation_low) / CONFIG_dist_res << std::endl;
    // std::cout << "Odom Diff: " << odom_loc_diff.x() << ", " << odom_loc_diff.y() << ", " << odom_angle_diff << std::endl;

    // Iterate over possible change in theta
    for(int angle_index = 0; angle_index <= (theta_high - theta_low) / CONFIG_theta_res; angle_index++){
      float dtheta = theta_low + angle_index * CONFIG_theta_res;
        double log_ptheta = -Sq(dtheta)/Sq(angle_sigma);
        log_ptheta = (log_ptheta > std::log(1e-50)) ? log_ptheta : std::log(1e-50);

      for(int x_index = 0; x_index <= (translation_high - translation_low) / CONFIG_dist_res; x_index++){
        float dx = translation_low + x_index * CONFIG_dist_res;
        double log_px = -Sq(dx)/Sq(trans_sigma);
        log_px = (log_px > std::log(1e-50)) ? log_px : std::log(1e-50);

        for(int y_index = 0; y_index <= (translation_high - translation_low) / CONFIG_dist_res; y_index++){
          float dy = translation_low + y_index * CONFIG_dist_res;

          double pose_log_prob = 0.0;
          
          // For each XY point in scan, get log likelihood from CSM
          // Pose2D<float> curPos(dtheta + curr_odom_angle_, dx + curr_odom_loc_.x(), dy + curr_odom_loc_.y());
          
          Vector2f trans_diff;
          trans_diff[0] = cost_map.RoundToResolution(odom_loc_diff.x() + dx, CONFIG_dist_res);
          trans_diff[1] = cost_map.RoundToResolution(odom_loc_diff.y() + dy, CONFIG_dist_res);
          //odom_loc_diff + Eigen::Vector2f(dx, dy);
          float angle_diff = cost_map.RoundToResolution(odom_angle_diff + dtheta, CONFIG_theta_res);
          Eigen::Rotation2Df R_NewPos2OldPos1(angle_diff);
          
          //double start = GetMonotonicTime();
          for(std::size_t i = 0; i < cloud_.size(); i+=10){
            Vector2f scanPos = R_NewPos2OldPos1 * cloud_[i] + trans_diff;
              
            //float blue[3]= {0, 0, 1};
    
            if(scanPos.norm() >= CONFIG_range_max) {
              continue;
            }
            
            //image.draw_point(cost_map.GetIndexFromDist(scanPos.x()), CONFIG_row_num - cost_map.GetIndexFromDist(scanPos.y()) - 1, blue, 1);

              try{
                double gaussian_prob = cost_map.GetLikelihoodAtPosition(scanPos.x(), scanPos.y());
                gaussian_prob = (gaussian_prob > 1e-50) ? gaussian_prob : 1e-50;
                pose_log_prob += CONFIG_gamma*std::log(gaussian_prob);
              } catch(std::out_of_range) {
                continue;
              }
          }
          //double end = GetMonotonicTime();
          //std::cout << "Inner Loop " << end - start << std::endl;

          
          // compute motion model likelihood


          double log_py = -Sq(dy)/Sq(trans_sigma);
          log_py = (log_py > std::log(1e-50)) ? log_py : std::log(1e-50);

          //std::cout << "dtheta, dx, dy, obs, x, y, theta: " << ((float) ((int) (1000*dtheta)))/1000 << ", " << ((float) ((int) (1000*dx)))/1000 << ", " << ((float) ((int) (1000*dy)))/1000 << ", ";
          //std::cout << ((float) ((int) (1000*pose_log_prob)))/1000 << ", " << ((float) ((int) (1000*log_px)))/1000 << ", " << ((float) ((int) (1000*log_py)))/1000 << ", " << ((float) ((int) (1000*log_ptheta)))/1000 << std::endl;

          // double log_px = -Sq(trans_diff.x())/Sq(trans_sigma);
          // double log_py = -Sq(trans_diff.y())/Sq(trans_sigma);
          // double log_ptheta = -Sq(angle_diff)/Sq(angle_sigma);

          // cost_map.DisplayImage(image);
          // image.fill(1);
          // cost_map.DrawCostMap(image);

          // Update most likely transform
          if( P < pose_log_prob + log_px + log_py + log_ptheta){
            P = pose_log_prob + log_px + log_py + log_ptheta;
            T = Pose2D<float>(angle_diff, trans_diff);
            // T = Pose2D<float>(AngleDiff(dtheta + curr_odom_angle_, poses.front().angle), Eigen::Vector2f(dx, dy));
          }

        }
      }


      
    }

    // Update cost map based on new laser scan
    cost_map.UpdateCostMap(cloud_);

    // Append new pose to list of poses
    Pose2D<float> latest_pose(poses.back());
    latest_pose.ApplyPose(T);
    poses.push_back(latest_pose);
    current_pose_ = Pose2D<float>(latest_pose);
    curr_dist_err = 0;

    // TESTING //

    // std::cout << "Theta bounds: " << theta_low << ", " << theta_high << std::endl;
    // std::cout << "Location bounds: " << translation_low << ", " << translation_high << std::endl;
    // std::cout << "Actual Transform: (" << odom_loc_diff.x() << ", " << odom_loc_diff.y() << "), " << odom_angle_diff << "\n";
    // std::cout << "Estimated Transform: (" << T.translation.x() << ", " << T.translation.y() << "), " << T.angle << "\n" << "\n";

    // if(abs(AngleDiff(T.angle, odom_angle_diff)) > 5*CONFIG_theta_res){

    //   cost_map.DrawCostMap(image);
    //   float blue[3]= {0, 0, 1};
    //   for(std::size_t i = 0; i < cloud_.size(); i += CONFIG_resize_factor){
    
    //     if(cloud_[i].norm() >= CONFIG_range_max) {
    //       continue;
    //     }
        
    //     // Rotate into map frame, then translate according to pose
    //     Eigen::Rotation2Df pose_to_map(T.angle);
    //     Eigen::Vector2f scan_in_map = pose_to_map*cloud_[i] + T.translation;
    //   image.draw_point(cost_map.GetIndexFromDist(scan_in_map.x()), CONFIG_row_num - cost_map.GetIndexFromDist(scan_in_map.y()) - 1, blue, 1);
    //   }
    //   cost_map.DisplayImage(image);
    // }
    // TESTING //


    // Update the environment map with new scan and pose
    BuildMapFromScan(cloud_, poses.back());

    double end = GetMonotonicTime();
    std::cout << end - start << std::endl;
  }
}

void SLAM::DrawPoints(CImg<float> &image, std::vector<Eigen::Vector2f> &point_cloud){
    float blue[3]= {0, 0, 1};
    for(std::size_t i = 0; i < point_cloud.size(); i++){
    
      if(point_cloud[i].norm() >= CONFIG_range_max) {
        continue;
      }
      
      // // Rotate into map frame, then translate according to pose
      // Eigen::Rotation2Df pose_to_map(poses.back().angle);
      // Eigen::Vector2f scan_in_map = pose_to_map*cloud_[i] + poses.back().translation;
      
      image.draw_point(cost_map.GetIndexFromDist(point_cloud[i].x()), CONFIG_row_num - cost_map.GetIndexFromDist(point_cloud[i].y()) - 1, blue, 1);
    }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;

    current_pose_ = Pose2D<float>(0.0, 0.0, 0.0);
    poses.push_back(current_pose_);

    std::cout << "Pose Count: " << poses.size() << std::endl;
    odom_initialized_ = true;
    return;
  }

  // rotation matrix from last odom to last baselink
  auto rot_odom1_to_bl1 = Eigen::Rotation2D<float>(-prev_odom_angle_).toRotationMatrix();
  
  // Change in translation and angle from odometry
  Eigen::Vector2f delta_translation = rot_odom1_to_bl1 * (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);

  //std::cout << "X, Y, Theta: " << delta_translation.x() << ", " << delta_translation.y() << ", " << delta_angle << std::endl;

  Pose2D<float>delta_pose_(delta_angle, delta_translation);
  current_pose_.ApplyPose(delta_pose_);
  curr_dist_err += delta_translation.norm();

  //std::cout << current_pose_.translation.x() << ", " << current_pose_.translation.y() << ", " << current_pose_.angle << std::endl;

  // Update last odometry values
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void SLAM::BuildMapFromScan(const vector<Vector2f>& cloud, const Pose2D<float> MLE_pose) {
  for(std::size_t i = 0; i < cloud.size(); i += CONFIG_resize_factor){
    
    if(cloud[i].norm() >= CONFIG_range_max) {
      continue;
    }
    
    // Rotate into map frame, then translate according to pose
    Eigen::Rotation2Df pose_to_map(MLE_pose.angle);
    Eigen::Vector2f scan_in_map = pose_to_map*cloud[i] + MLE_pose.translation;
    
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
