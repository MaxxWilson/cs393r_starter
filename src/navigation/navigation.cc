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
    robot_angle_(0),//angle w.r.t global x- axis (rad)
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
  //std::cout << "    robot location: " << robot_loc_ << "\n";
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel,
                                uint64_t time) {
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
  odom_stamp_ = time - car_params::sensing_latency;
  has_new_odom_ = true;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   uint64_t time) {
  point_cloud_ = cloud;
  point_cloud_stamp_ = time - car_params::sensing_latency;
  has_new_points_= true;
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

void Navigation::TransformPointCloud(float del_x, float del_y){
  // std::cout<<"point del: " << del_x << " " << del_y << "\n";
  for(std::size_t i = 0; i < point_cloud_.size(); i++){

    // Polar to Cartesian conversion, transforms to base link frame
    //float angle = msg.angle_min + msg.angle_increment*i;
    float xT = point_cloud_[i][0] - del_x;
    float yT = point_cloud_[i][1] - del_y;
    
    point_cloud_[i] = Vector2f(xT, yT);
  }

  //ObservePointCloud(point_cloud_, ros::Time::now().toNSec());
}

void Navigation::TransformOdom(float del_x, float del_y){
  //std::cout << "Odom del x and y: " << del_x << " " << del_y << "\n";
  UpdateOdometry(
      Vector2f(robot_loc_[0] + del_x, robot_loc_[1] + del_y),
      odom_angle_,
      robot_vel_,
      robot_omega_,
      ros::Time::now().toNSec());
}

void Navigation::Run() {
  //This function gets called 20 times a second to form the control loop.
  //Time at begnning of control loop, defined for latency function
  uint64_t start_loop_time = ros::Time::now().toNSec();
  uint64_t actuation_time = start_loop_time + car_params::sys_latency; // TODO + EXEC TIME
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

    // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  //Latency Compensation
  obstacle_avoidance::CleanVelocityBuffer(vel_commands_, odom_stamp_);

  float future_odom = robot_vel_.norm()*(vel_commands_[1].stamp - odom_stamp_)*pow(10.0, -9.0); // TF from last odom to future
  std::cout << "1 "<< future_odom << std::endl;
  for(std::size_t i = 1; i < vel_commands_.size()-1; i++){
    future_odom += vel_commands_[i].velocity*(vel_commands_[i+1].stamp - vel_commands_[i].stamp)*pow(10.0, -9.0);
  }
  future_odom += vel_commands_.back().velocity*(actuation_time - vel_commands_.back().stamp)*pow(10.0, -9.0);

  std::cout << "2 "<< future_odom << std::endl;

  // Future odom is good!

  //auto odom_to_point_cloud = robot_vel_.norm()*(odom_stamp_ - point_cloud_stamp_)*pow(10.0, -9.0);

  //auto future_point_cloud = future_odom + odom_to_point_cloud;

  //std::cout << future_point_cloud << std::endl;

  //TransformPointCloud(future_point_cloud, 0);

  std::cout << "hasNew Points " << has_new_points_ << std::endl;));
  If no new sensor data
  if(!has_new_points_){
      Eigen::Vector2f points_del = obstacle_avoidance::Integrate(drive_msg_.header.stamp.toNSec(), vel_commands_, robot_angle_);
      std::cout<< "no points! " << "\n";
      TransformPointCloud(points_del[0], points_del[1]);
      obstacle_avoidance::VisualizeLatencyInfo(local_viz_msg_, point_cloud_, points_del, odom_angle_);
  }

  if(!has_new_odom_){
      Eigen::Vector2f points_del = obstacle_avoidance::Integrate(drive_msg_.header.stamp.toNSec(), vel_commands_, robot_angle_);
      std::cout<< "no odom! " << "\n";
      TransformPointCloud(points_del[0], points_del[1]);
      obstacle_avoidance::VisualizeLatencyInfo(local_viz_msg_, point_cloud_, points_del, odom_angle_);
  }
  If new sensor data
  if(has_new_points_){
      std::cout << "New points" << std::endl;
      Eigen::Vector2f points_del = obstacle_avoidance::Integrate(point_cloud_stamp_, vel_commands_, robot_angle_);
      TransformPointCloud(points_del[0], points_del[1]);
      obstacle_avoidance::VisualizeLatencyInfo(local_viz_msg_, point_cloud_, points_del, odom_angle_);
      has_new_points_ = false;
  }

  if(has_new_odom_){//change to update both x and y pos based on odom.x and odom.y

      Eigen::Vector2f odom_del = obstacle_avoidance::Integrate(odom_stamp_, vel_commands_, robot_angle_);;
      TransformOdom(odom_del[0], odom_del[1]);
      std::cout << "Odom Loc: " << odom_loc_ << "\n"; 
      obstacle_avoidance::VisualizeLatencyInfo(local_viz_msg_, point_cloud_, odom_del, odom_angle_);
      has_new_odom_ = false;
  }



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

      //std::cout << path_options[curve_index].clearance << std::endl;
      // Visualization test code
      
      // visualization::DrawPathOption(path_options[curve_index].curvature, path_options[curve_index].free_path_length, path_options[curve_index].clearance, local_viz_msg_);
      // visualization::DrawCross(path_options[curve_index].obstruction, 0.1,  0x0046FF, local_viz_msg_);
    }
  //auto end_time = ros::Time::now().toSec();
  //std::cout << end_time - start_time << std::endl;
  // 7) Select best path from scoring function (Easy, YUHONG)
  struct PathOption best_path = obstacle_avoidance::ChooseBestPath(path_options,goal_point);


  //std::cout << "Length, Clearance, Dist: " << best_path.free_path_length << ", " << 2.5 * best_path.clearance << ", " << -0.1 * obstacle_avoidance::GetDistanceToGoal(best_path,goal_point) << std::endl;
  
  obstacle_avoidance::VisualizeObstacleAvoidanceInfo(goal_point,path_options,best_path,local_viz_msg_);
  // 8) Publish commands with 1-D TOC (YUHONG)s
  TimeOptimalControl(best_path);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();

  CommandStamped drive_cmd(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toNSec() + car_params::sys_latency);
  vel_commands_.push_back(drive_cmd);

  //uint64_t last_loop_time = ros::Time::now().toNSec() - start_loop_time;
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);

  //Time at end of control loop, defined for latency function
  // ros::Time t_end_control_function = ros::Time::now();

}

}  // namespace navigation
