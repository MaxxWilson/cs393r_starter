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


CONFIG_FLOAT(dist_update_thresh, "dist_update_thresh");
CONFIG_FLOAT(angle_update_thresh, "angle_update_thresh");

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = curr_odom_loc_;
  *angle = curr_odom_angle_;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  // Pseudocode:
  /*
  if(curr_odom_loc_.norm() > CONFIG_dist_update_thresh || abs(curr_odom_angle_) > CONFIG_angle_update_thresh){

    double P = 0.0;
    Pose T = Pose(0, 0, 0);

    for(all dtheta){
      Convert scan to XY points relative to base link (including tf from lidar->BL)
      for(each dx){
        for(each dy){
          double newP = 0.0;
          // For each |XY point in scan, get log likelihood from CSM
          for(each point){
            double newP += getLikelihoodCSM(dtheta, dx + point.x, dy + point.y);
          }
          convert log likelihood newP to normalized likelihood
          compute motion model likelihood

          // Update most likely transform
          if(P < newP*motionP){
            P = newP*motionP;
            T = Pose(dx, dy, dtheta)
          }
        }
      }
    }

    updateCSM(scan)
    Poses.add(T + Poses(Poses.end));
    odom_sum = Pose(0, 0, 0);
  } */
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    curr_odom_loc_ = odom_loc;
    curr_odom_angle_ = odom_angle;

    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;

    odom_initialized_ = true;
    return;
  }

  curr_odom_loc_ += odom_loc;
  curr_odom_angle_ += odom_angle;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
