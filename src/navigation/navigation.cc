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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "obstacle_avoidance/car_params.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

//Solve the TOC Problem
void Navigation::timeOptimalControl(const PathOption& path) {
    double current_speed = robot_vel_.norm();
    double min_stop_distance = car_params::safe_distance-0.5*current_speed*current_speed/car_params::min_acceleration; //calculate the minimum stopping distance at current velocity
    double set_speed = (path.free_path_length>min_stop_distance)?car_params::max_velocity:0; //decelerate if free path is is smaller than minimum stopping distance otherwise accelerate
    //publish command to topic 
    drive_msg_.header.seq++;
    drive_msg_.header.stamp = ros::Time::now();
    drive_msg_.curvature = path.curvature;
    drive_msg_.velocity = set_speed;
    drive_pub_.publish(drive_msg_);
    //TODO: record the commands used for latency compensation
}
//Show all the obstacles

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  /// Control Loop ///

  // 1) Forward predict state according to system latency (MELISSA)
  // 2) Transform point cloud and goal point to predicted base_link frame at time=t+latency? (MELISSA)
  // 3) Calculate curvature to goal point (For assignment 1, its always zero, but will need in future) (MAXX)
  // 4) Generate range of possible paths centered on goal_curvature, using std::vector<struct PathOption>(MAXX)
  // 5) Eliminate Obstacle Points - (MAXX)
  // 6) For possible paths and point_cloud:
  //    - Get Path Length (MAXX)
  //    - Get Clearance (MELISSA)
  //    - Get Distance to Goal (YUHONG, look at math functions)
  // 7) Select best path from scoring function (Easy, YUHONG)
  // 8) Publish commands with 1-D TOC (YUHONG)

  /*
  GetMinStoppingDistance();

  if (dist to obstacle - next predicted position) < min stopping distance
  then, deccelerate

  else set max speed

  drive_msg_.velocity = setVelocity1DTOC(path_length, current_velocity);

  */
  // CarOutliner(local_viz_msg_);
  // Issue vehicle commands
  drive_msg_.curvature = 0.0;
  drive_msg_.velocity = 0.0;

  //debug LimitFreePath function
  // navigation::PathOption path{0.5,0,4*M_PI,{0,0},{0,0}};
  // Eigen::Vector2f goal(1,3);
  // SelectedPathOutliner(path,local_viz_msg_);
  // LimitFreePath(path,goal);
  // SelectedPathOutliner(path,local_viz_msg_);
  // GoalOutliner(goal,local_viz_msg_);
  // visualization::DrawPathOption(1.0, 4.0, 0.0, local_viz_msg_);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
