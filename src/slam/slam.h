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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "cost_map/cost_map.h"
#include "shared/math/poses_2d.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:

  costmap::CostMap cost_map;
  // Map 
  std::vector<Eigen::Vector2f> map_;
  // Current odometry
  Eigen::Vector2f curr_odom_loc_;
  float curr_odom_angle_;

  // Odometry at last Pose update
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;

  pose_2d::Pose2D<float> previous_pose_;
  pose_2d::Pose2D<float> current_pose_;
  
  bool odom_initialized_;
  bool cost_map_initialized;

  std::vector<pose_2d::Pose2D<float>> poses;

  double GetMotionModelLikelihood(double x, double y, double theta);
  void BuildMapFromScan(const vector<Eigen::Vector2f>& cloud, const pose_2d::Pose2D<float> MLE_pose);
  
};
}  // namespace slam

#endif   // SRC_SLAM_H_
