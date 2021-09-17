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

  vel_commands_ = std::vector<CommandStamped>(10);
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
                                float ang_vel,
                                ros::Time time) {
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
  odom_stamp_ = time.toNSec();
  has_new_odom_ = true;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   uint64_t time) {
  point_cloud_ = cloud;
  point_cloud_stamp_ = time;
  has_new_points_= true;
  //std::cout << "points!" << "\n";
}

void Navigation::Run() {

  //This function gets called 20 times a second to form the control loop.
  //Time at begnning of control loop, defined for latency function
  //uint64_t start_loop_time = ros::Time::now().toNSec();

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

    // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  obstacle_avoidance::CleanVelocityBuffer(vel_commands_, std::min(point_cloud_stamp_, odom_stamp_));

  if(has_new_points_){
      float points_del_x = obstacle_avoidance::Integrate(point_cloud_stamp_, vel_commands_);
      std::cout<< "points del_x (m): " << points_del_x << "\n";
      has_new_points_ = false;
  }

  if(has_new_odom_){
      float odom_del_x = obstacle_avoidance::Integrate(odom_stamp_, vel_commands_);
      std::cout<< "odom del_x (m): " << odom_del_x << "\n";
      has_new_odom_ = false;
  }
  // The control iteration goes here. 
 
  // if(last_odom == odom_loc_){
  //   Eigen::Vector2f predicted_state = obstacle_avoidance::LatencyCompensate(odom_vel_, last_state, t_start_control_function,t_end_control_function_avg, odom_time_stamp);
  // }
  // else{
  //   last_odom = odom_loc_;
  //   Eigen::Vector2f predicted_state = obstacle_avoidance::LatencyCompensate(odom_vel_, last_odom + last_state, 
  //                                                                            t_start_control_function,
  //                                                                            t_end_control_function_avg, 
  //                                                                            odom_time_stamp);
  // }
  // last_state = predicted_state;
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
    
  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 1;
  drive_msg_.velocity = 2;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();

  CommandStamped drive_cmd(drive_msg_.curvature, drive_msg_.velocity, drive_msg_.header.stamp.toNSec());
  vel_commands_.push_back(drive_cmd);

  //uint64_t last_loop_time = ros::Time::now().toNSec() - start_loop_time;
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);


  //Time at end of control loop, defined for latency function
  // ros::Time t_end_control_function = ros::Time::now();
}

}  // namespace navigation
