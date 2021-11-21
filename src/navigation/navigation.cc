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
#include "visualization/visualization.h"
#include "config_reader/config_reader.h"
#include "navigation.h"
#include "Astar.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "visualization/CImg.h"

#include <utility>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using cimg_library::CImg;
using cimg_library::CImgDisplay;

using namespace math_util;
using namespace ros_helpers;

CONFIG_FLOAT(pursuit_radius, "pursuit_radius");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
const float dist_res = 0.1;
const float map_length_dist = 50;
const float row_num = 2*(map_length_dist)/dist_res + 1;
} //namespace

namespace navigation {
CONFIG_INT(row_num, "row_num");
CONFIG_FLOAT(pursuit_radius, "pursuit_radius");
CONFIG_FLOAT(goal_threshold, "goal_threshold");

CONFIG_FLOAT(num_curves, "num_curves");
CONFIG_UINT(sensing_latency, "sensing_latency");
CONFIG_UINT(actuation_latency, "actuation_latency");

CONFIG_FLOAT(min_acceleration, "min_acceleration");
CONFIG_FLOAT(safe_distance, "safe_distance");
CONFIG_FLOAT(max_velocity, "max_velocity");

CONFIG_FLOAT(min_curvature, "min_curvature");
CONFIG_FLOAT(max_curvature, "max_curvature");
CONFIG_FLOAT(curvature_increment, "curvature_increment");

CONFIG_FLOAT(max_path_length, "max_path_length");

CONFIG_FLOAT(clearance_gain, "clearance_gain");
CONFIG_FLOAT(dist_goal_gain, "dist_goal_gain");

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
  map_.Load(map_file);
  collision_map_ = costmap::CostMap();
  collision_map_.ClearMap();
  collision_map_.UpdateCollisionMap(map_.lines);
  
  CImg<float> image(CONFIG_row_num, CONFIG_row_num, 1,1,1);
  float color = 0.2;

  for(int x = 0; x < CONFIG_row_num; x++){
    for(int y = 0; y < CONFIG_row_num; y++){
      // Image coordinate frame is LHR
      image.draw_point(x, CONFIG_row_num - y - 1, &color, collision_map_.GetValueAtIdx(x, y));
    }
  }

  // collision_map_.DisplayImage(image);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = Eigen::Vector2f(loc);
  Replan();
  nav_complete_ = false;
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
                                uint64_t time) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    odom_stamp_ = time - CONFIG_sensing_latency;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;

  //std::cout << "omega, vel, loc, angle: " << ang_vel << ", " << robot_vel_[0] << ", " << loc << ", " << angle << std::endl;

  odom_stamp_ = time - CONFIG_sensing_latency;
  last_odom_stamp_ = odom_stamp_;
  //std::cout << "odom stamp: " << odom_stamp_ << std::endl;
  has_new_odom_ = true;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   uint64_t time) {
  point_cloud_ = cloud;
  point_cloud_stamp_ = time - CONFIG_sensing_latency;
  has_new_points_= true;
}

//Solve the TOC Problem
void Navigation::TimeOptimalControl(const PathOption& path) {
    double current_speed = odom_state_tf.speed;
    double min_stop_distance = CONFIG_safe_distance-0.5*current_speed*current_speed/CONFIG_min_acceleration; //calculate the minimum stopping distance at current velocity
    double set_speed = (path.free_path_length>min_stop_distance)?CONFIG_max_velocity:0; //decelerate if free path is is smaller than minimum stopping distance otherwise accelerate
    
    // Publish command to topic 
    drive_msg_.header.seq++;
    drive_msg_.header.stamp = ros::Time::now();
    drive_msg_.curvature = path.curvature;
    drive_msg_.velocity = set_speed;
}

void Navigation::TransformPointCloud(TimeShiftedTF transform){

  Eigen::Matrix2f R;
  R << cos(transform.theta), sin(transform.theta), -sin(transform.theta), cos(transform.theta);

  transformed_point_cloud_.resize(point_cloud_.size());

  for(std::size_t i = 0; i < point_cloud_.size(); i++){
    transformed_point_cloud_[i] =  R*(point_cloud_[i] - transform.position);
  }
}

void Navigation::Replan(){
  double start_time = GetMonotonicTime();
  visualization::ClearVisualizationMsg(global_viz_msg_);
  astar::Astar global_planner(collision_map_);

  visualization::DrawCross(robot_loc_, 0.25,0xfc4103,global_viz_msg_);
  visualization::DrawCross(nav_goal_loc_, 0.25,0xfc4103,global_viz_msg_);
  if(global_planner.AstarSearch(collision_map_, collision_map_.GetIndexPairFromDist(robot_loc_), collision_map_.GetIndexPairFromDist(nav_goal_loc_))) {
    global_planner.tracePath(global_viz_msg_ , collision_map_, collision_map_.GetIndexPairFromDist(nav_goal_loc_));
    global_planner.GeneratePathVector(collision_map_, collision_map_.GetIndexPairFromDist(nav_goal_loc_));
    global_plan_ = global_planner.GetPathVector();
  }

  double end_time = GetMonotonicTime();
  std::cout << "Global Planning Time: " << end_time - start_time << std::endl << std::endl;

  // for(std::size_t i = 0; i < global_plan_.size(); i++){
  //   visualization::DrawCross(global_plan_[i], 0.02, 0xff0000, global_viz_msg_);
  // }
}

bool Navigation::PurePursuit(Eigen::Vector2f &goal_point){
  goal_point = Eigen::Vector2f(0, 0);
  Eigen::Vector2f intersection(0, 0);
  int curr_wpt_index_ = 0;

  // Search global plan for intersections, and select intersection in front of robot as goal point
  while(curr_wpt_index_ < global_plan_.size()){
    if(IsIntersectingCircle(global_plan_[curr_wpt_index_], global_plan_[curr_wpt_index_+1], intersection)){
      intersection = Eigen::Rotation2D<float>(-robot_angle_)*(intersection - robot_loc_);
      if(intersection.x() > goal_point.x() || goal_point.norm() == 0){
        goal_point = intersection;
      }
    }
    curr_wpt_index_++;
  }

  // If robot is within pursuit radius of goal, go directly to goal
  if((nav_goal_loc_ - robot_loc_).norm() < CONFIG_pursuit_radius){
    goal_point = Eigen::Rotation2D<float>(-robot_angle_)*(nav_goal_loc_ - robot_loc_);
  }

  // If goal point is set, return true
  if(goal_point.norm() != 0){
    return true;
  }

  return false;
}

bool Navigation::pointIsInCircle(Eigen::Vector2f point, Eigen::Vector2f circle_center, double radius){
  return (point - circle_center).norm() < radius;
}

/**
 * @brief https://math.stackexchange.com/questions/228841/how-do-i-calculate-the-intersections-of-a-straight-line-and-a-circle
 * 
 * @param point_1 
 * @param point_2 
 * @param intersection_point 
 * @return true 
 * @return false 
 */
bool Navigation::IsIntersectingCircle(Eigen::Vector2f point_1, Eigen::Vector2f point_2, Eigen::Vector2f& intersection_point){
  bool p1InCircle = pointIsInCircle(point_1, robot_loc_, CONFIG_pursuit_radius);
  bool p2InCircle = pointIsInCircle(point_2, robot_loc_, CONFIG_pursuit_radius);

  if(p1InCircle && p2InCircle){
    return false;
  }
  
  Eigen::Vector2f projected_point = geometry::ProjectPointOntoLine(robot_loc_, point_1, point_2);
  bool point_on_segment = geometry::IsBetween(point_1, point_2, projected_point, (float) 0.001);
  bool passes_through_circle = pointIsInCircle(projected_point, robot_loc_, CONFIG_pursuit_radius) && point_on_segment;
  
  if(!p1InCircle && !p2InCircle && !passes_through_circle){
    return false;
  }
  
  // Center of pursuit circle coordinates
  float cx = robot_loc_.x();
  float cy = robot_loc_.y();
  float cx2 = math_util::Pow(cx, 2);
  float cy2 = math_util::Pow(cy, 2);
  float r2 = math_util::Pow(CONFIG_pursuit_radius, 2);

  if(point_1.x() == point_2.x()){ // Vertical Line Case (inf slope)
      //Intersection solution points
    float y1_sol;
    float y2_sol;

    float aqy = 1.0;
    float bqy = -2*cy;
    float cqy = cx2 + cy2 - r2 - 2*cx*point_1.x() + math_util::Pow(point_1.x(), 2);

    int root_num = math_util::SolveQuadratic(aqy, bqy, cqy, &y1_sol, &y2_sol);

    if(root_num == 0){
      return false;
    }
    else if(root_num == 1){
      intersection_point = Eigen::Vector2f(point_1.x(), y1_sol);
      return true;
    }
    else if(root_num == 2){
      if(geometry::IsBetween(point_1, point_2, Eigen::Vector2f(point_1.x(), y1_sol), (float) 0.001)){
        intersection_point = Eigen::Vector2f(point_1.x(), y1_sol);
      }
      else if(geometry::IsBetween(point_1, point_2, Eigen::Vector2f(point_1.x(), y2_sol), (float) 0.001)){
        intersection_point = Eigen::Vector2f(point_1.x(), y2_sol);
      }
      return true;
    }
  }
  else{
    // Equation of a line parameters
    float m = (point_2.y() - point_1.y()) / (point_2.x() - point_1.x());
    float b = point_2.y() - m * point_2.x(); 

    // Calculates squares of variables
    float m2 = math_util::Pow(m, 2);
    float b2 = math_util::Pow(b, 2);

    //Intersection solution points
    float x1_sol;
    float x2_sol;

    //Quadratic Formula parameters solving for x intersection values
    float aqx = 1 + m2;
    float bqx = 2*(m*b - m*cy - cx);
    float cqx = cx2 + cy2 + b2 - 2 * cy * b - r2;

    int root_num = math_util::SolveQuadratic(aqx, bqx, cqx, &x1_sol, &x2_sol);

    if(root_num == 0){
      return false;
    }
    else if(root_num == 1){
      float y = m*x1_sol+b;
      intersection_point = Eigen::Vector2f(x1_sol, y);
      return true;
    }
    else if(root_num == 2){
      float y1 = m*x1_sol+b;
      float y2 = m*x2_sol+b;

      if(geometry::IsBetween(point_1, point_2, Eigen::Vector2f(x1_sol, y1), (float) 0.001)){
        intersection_point = Eigen::Vector2f(x1_sol, y1);
      }
      else if(geometry::IsBetween(point_1, point_2, Eigen::Vector2f(x2_sol, y2), (float) 0.001)){
        intersection_point = Eigen::Vector2f(x2_sol, y2);
      }
      return true;
    }
  }
}

void Navigation::Run(){
  
  uint64_t start_loop_time = ros::Time::now().toNSec();
  uint64_t actuation_time = start_loop_time + CONFIG_actuation_latency;
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  //visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  if (nav_complete_) return;
  
  //This function gets called 20 times a second to form the control loop.
  //Latency Compensation
  obstacle_avoidance::CleanVelocityBuffer(vel_commands_, std::min(odom_stamp_, point_cloud_stamp_));

  if(has_new_odom_){
    odom_state_tf.position = Eigen::Vector2f(0, 0);
    odom_state_tf.speed = robot_vel_[0];
    odom_state_tf.theta = 0;
    odom_state_tf.stamp = odom_stamp_;
    has_new_odom_ = false;
  }

  // Find drive cmd directly before odom msg
  int cmd_start_index = std::lower_bound(vel_commands_.begin(), vel_commands_.end(), odom_state_tf.stamp) - vel_commands_.begin() - 1;

  // Integrate odometry into the future across the vector of vehicle commands
  for(std::size_t i = cmd_start_index; i < vel_commands_.size() - 1; i++){
    odom_state_tf = obstacle_avoidance::IntegrateState(odom_state_tf, vel_commands_[i], vel_commands_[i+1].stamp - odom_state_tf.stamp);
  }
  odom_state_tf = obstacle_avoidance::IntegrateState(odom_state_tf, vel_commands_.back(), actuation_time - odom_state_tf.stamp);

  // odom_state_tf is now transform from last odom msg to future actuation time
  
  // get transform from point cloud msg to odom msg
  uint64_t latest_sensor_msg_stamp; // whichever msg is newer
  TimeShiftedTF pc_to_odom_transform; // Empty transform to populate as we integrate between sensor msgs
  pc_to_odom_transform.speed = robot_vel_[0];

  if(point_cloud_stamp_ > odom_stamp_){
    latest_sensor_msg_stamp = point_cloud_stamp_;
    pc_to_odom_transform.stamp = odom_stamp_;
  }
  else{
    latest_sensor_msg_stamp = odom_stamp_;
    pc_to_odom_transform.stamp = point_cloud_stamp_;
  }

  // Integrates from older sensor message to newer message
  int pc_to_odom_index = 0;
  while(vel_commands_[pc_to_odom_index+1].stamp < latest_sensor_msg_stamp){
    pc_to_odom_transform = obstacle_avoidance::IntegrateState(pc_to_odom_transform, vel_commands_[pc_to_odom_index], vel_commands_[pc_to_odom_index+1].stamp - pc_to_odom_transform.stamp);
    pc_to_odom_index++;
  }
  pc_to_odom_transform = obstacle_avoidance::IntegrateState(pc_to_odom_transform, vel_commands_[pc_to_odom_index], latest_sensor_msg_stamp - pc_to_odom_transform.stamp);

  // With odom->future_odom and point_cloud->odom, we can find transform for point cloud at future actuation time
  if(point_cloud_stamp_ > odom_stamp_){
    pc_to_odom_transform.theta = odom_state_tf.theta - pc_to_odom_transform.theta;
    pc_to_odom_transform.position = odom_state_tf.position - pc_to_odom_transform.position;
  }
  else{
    pc_to_odom_transform.theta += odom_state_tf.theta;
    pc_to_odom_transform.position += odom_state_tf.position;
  }

  // Transform point cloud into future actuation time, storing as transformed_point_cloud_
  TransformPointCloud(pc_to_odom_transform);

  // Visualize Latency Compensation
  //obstacle_avoidance::LatencyPointCloud(local_viz_msg_, transformed_point_cloud_);
  //obstacle_avoidance::DrawCarLocal(local_viz_msg_, odom_state_tf.position, odom_state_tf.theta);


  if((robot_loc_ - global_plan_.back()).norm() < CONFIG_goal_threshold){
    nav_complete_ = true;
    drive_msg_.curvature = 0;
    drive_msg_.velocity = 0;
    drive_pub_.publish(drive_msg_);
    return;
  }

  // "Carrot on a stick" goal point, and resulting goal curvature
  Eigen::Vector2f goal_point(0, 0);  
  if(!PurePursuit(goal_point)){
    Replan();
    std::cout << "Replanning" << std::endl << std::endl;
    return;
  }
  
  visualization::DrawArc(Eigen::Vector2f(0, 0), CONFIG_pursuit_radius, 0, 2*M_PI, 0xfcba03, local_viz_msg_); // draws pure pursuit circle
  visualization::DrawCross(goal_point, 0.25, 0xFF0000, local_viz_msg_);

  float goal_curvature = obstacle_avoidance::GetCurvatureFromGoalPoint(goal_point);
  goal_curvature = Clamp(goal_curvature, CONFIG_min_curvature, CONFIG_max_curvature);

  // 4) Generate range of possible paths centered on goal_curvature, using std::vector<struct PathOption>
  static std::vector<struct PathOption> path_options(floor(CONFIG_num_curves) + 1);

  // 5) For possible paths and point_cloud:
  for(std::size_t curve_index = 0; curve_index < path_options.size(); curve_index++){
    float curvature = obstacle_avoidance::GetCurvatureOptionFromRange(curve_index, goal_curvature, CONFIG_min_curvature, CONFIG_curvature_increment);
    
    // Initialize path_option and collision bounds for curvature
    obstacle_avoidance::PathBoundaries collision_bounds(abs(curvature));
    path_options[curve_index] = navigation::PathOption{
      curvature,                    // curvature
      10,                           // default clearance
      CONFIG_max_path_length,       // free Path Length
      0.0,                          // dist to goal
      Eigen::Vector2f(0, 0),        // obstacle point
      Eigen::Vector2f(0, 0)};       // closest point

    obstacle_avoidance::EvaluatePathWithPointCloud(path_options[curve_index], collision_bounds, transformed_point_cloud_);
    obstacle_avoidance::LimitFreePath(path_options[curve_index], goal_point);
    obstacle_avoidance::EvaluateClearanceWithPointCloud(path_options[curve_index], collision_bounds, transformed_point_cloud_);

    // Visualization test code
    // visualization::DrawPathOption(path_options[curve_index].curvature, path_options[curve_index].free_path_length, path_options[curve_index].clearance, local_viz_msg_);
    // visualization::DrawCross(path_options[curve_index].obstruction, 0.1,  0x0046FF, local_viz_msg_);
  }

  // 6) Select best path from scoring function
  struct PathOption best_path = obstacle_avoidance::ChooseBestPath(path_options, goal_point);

  std::cout << "Path Length Score: " << best_path.free_path_length << std::endl;
  std::cout << "Clearance Score: " << CONFIG_clearance_gain * best_path.clearance << std::endl;
  std::cout << "Dist To Goal Score: " << CONFIG_dist_goal_gain * best_path.dist_to_goal << std::endl << std::endl;

  obstacle_avoidance::VisualizeObstacleAvoidanceInfo(goal_point,path_options,best_path,local_viz_msg_);
  
  // 7) Publish commands with 1-D TOC, update vector of previous vehicle commands
  TimeOptimalControl(best_path);
    
  CommandStamped drive_cmd(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toNSec() + CONFIG_actuation_latency);
  vel_commands_.push_back(drive_cmd);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}
    
}  // namespace navigation
