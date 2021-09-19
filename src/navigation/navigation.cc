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
void Navigation::TimeOptimalControl(const PathOption& path) {
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

double Navigation::sineVel(double index){

  return 0.25 * sin(index) + 0.75;
}
//Show all the obstacles

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  static float index = 0; 
  index += 3.14/48;
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // std::cout << ros::Time::now().toNSec() << ", " << robot_vel_[0] << ", " << robot_vel_[1] << std::endl;

  /// Control Loop ///

  // 1) Forward predict state according to system latency (MELISSA)
  // 2) Transform point cloud and goal point to predicted base_link frame at time=t+latency? (MELISSA)
  // 3) Calculate curvature to goal point (For assignment 1, its always zero, but will need in future) (MAXX)

    Eigen::Vector2f goal_point(4, 0.0);

    float goal_curvature = obstacle_avoidance::GetCurvatureFromGoalPoint(goal_point);
    goal_curvature = Clamp(goal_curvature, car_params::min_curvature, car_params::max_curvature);

  // 4) Generate range of possible paths centered on goal_curvature, using std::vector<struct PathOption>(MAXX)

    static std::vector<struct PathOption> path_options(car_params::num_curves);

  // 6) For possible paths and point_cloud:
  //auto start_time = ros::Time::now().toSec();
    for(std::size_t curve_index = 0; curve_index < path_options.size(); curve_index++){
      float curvature = obstacle_avoidance::GetCurvatureOptionFromRange(curve_index, goal_curvature, car_params::min_curvature, car_params::curvature_increment);
      
      // Initialize path_option and collision bounds for curvature
      obstacle_avoidance::PathBoundaries collision_bounds(abs(curvature));
      path_options[curve_index] = navigation::PathOption{
        curvature,                    // curvature
        10,                           // default clearance
        car_params::max_path_length,  // free Path Length
        Eigen::Vector2f(0, 0),        // obstacle point
        Eigen::Vector2f(0, 0)};       // closest point

      obstacle_avoidance::EvaluatePathWithPointCloud(path_options[curve_index], collision_bounds, point_cloud_);
      //    - Get Distance to Goal (YUHONG, look at math functions)
      obstacle_avoidance::LimitFreePath(path_options[curve_index], goal_point);
      obstacle_avoidance::EvaluateClearanceWithPointCloud(path_options[curve_index], collision_bounds, point_cloud_);
      // Visualization test code
      
      // visualization::DrawPathOption(path_options[curve_index].curvature, path_options[curve_index].free_path_length, path_options[curve_index].clearance, local_viz_msg_);
      // visualization::DrawCross(path_options[curve_index].obstruction, 0.1,  0x0046FF, local_viz_msg_);
    }
  
  // 7) Select best path from scoring function (Easy, YUHONG)
  struct PathOption best_path = obstacle_avoidance::ChooseBestPath(path_options,goal_point);

  std::cout << "Length, Clearance, Dist: " << best_path.free_path_length << ", " << 10.0 * best_path.clearance << ", " << -0.1 * obstacle_avoidance::GetDistanceToGoal(best_path,goal_point) << std::endl;
  
  obstacle_avoidance::VisualizeObstacleAvoidanceInfo(goal_point,path_options,best_path,local_viz_msg_);
  // 8) Publish commands with 1-D TOC (YUHONG)s
  TimeOptimalControl(best_path);
  // Issue vehicle commands
  drive_msg_.curvature = 0.0;
  drive_msg_.velocity = sineVel(index);
  // std::cout << "Velocity: " << drive_msg_.velocity << "  Reported Velocity: " << robot_vel_[0] << "  Index: " << index << "\n" ;
  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  // drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
