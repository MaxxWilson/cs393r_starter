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
\file    obstacle_avoidance.h
\brief   Function implementations for obstacle avoidance 
\authors  Melissa Cruz, Yuhong Kan, Maxx Wilson (C) 2021
*/
//========================================================================

#include <iostream>

#include "shared/math/math_util.h"

#include "obstacle_avoidance/obstacle_avoidance.h"

using namespace math_util;
namespace obstacle_avoidance{

bool IsPointCollisionPossible(float curvature, const Eigen::Vector2f &point){
    if(point[0] < 0){
        // Ignore points behind car
        return false;
    }
    else if(Sign(curvature)*point[1] < -(car_params::width + car_params::safety_margin)){
        // Ignore points in the direction opposite of curvature
        // (Can't hit any point to the left when turning right)
        return false;
    }
    else if(point.norm() > car_params::max_path_length){
        // Ignore points outside of max search area
        // This will change latency depending on environment,
        // longer but consistent might be preferable to sometimes long sometimes short
        return false;
    }
    else{
        return true;
    }
}
   
void EvaluatePathWithPointCloud(navigation::PathOption &path_option, const PathBoundaries &collision_bounds, std::vector<Eigen::Vector2f> &point_cloud_){
    for(std::size_t point_index = 0; point_index < point_cloud_.size(); point_index++){
        Eigen::Vector2f point = point_cloud_[point_index];

        if(!obstacle_avoidance::IsPointCollisionPossible(path_option.curvature, point)){
          continue;
        }

        // Easier to mirror a point for negative curvature than to make everything conditional on sign
        if(path_option.curvature < 0){
          point[1] *= -1;
        }

        navigation::PathOption path_result = EvaluatePathWithPoint(collision_bounds, point);

        // Update path_option with shorter paths or smaller clearances
        if(path_result.free_path_length < path_option.free_path_length){
          path_option.free_path_length = path_result.free_path_length;
          path_option.obstruction = point_cloud_[point_index];
        }
        if(abs(path_result.clearance) < abs(path_option.clearance)){
          path_option.clearance = path_result.clearance;
        }
      }
}

navigation::PathOption EvaluatePathWithPoint(const PathBoundaries &collision_bounds, Eigen::Vector2f point){
        navigation::PathOption path_result{0, 1000, car_params::max_path_length};

        // Zero Curvature Collision
        if(abs(collision_bounds.curvature) < 1e-5 && abs(point[1]) <= (car_params::safety_margin + car_params::width / 2)){
            path_result.free_path_length = point[0] - car_params::dist_to_front_bumper;
        }

        float radius_to_obstacle = (point - Eigen::Vector2f(0, 1/collision_bounds.curvature)).norm();
        float angle_to_obstacle = atan2(point[0], 1/collision_bounds.curvature - point[1]);

        // Evaluate Collision Region
        if(radius_to_obstacle < collision_bounds.min_radius){
            // No Collision, Inner Miss
            path_result.clearance = collision_bounds.min_radius - radius_to_obstacle;
        }
        else if(collision_bounds.min_radius <= radius_to_obstacle && radius_to_obstacle <= collision_bounds.boundary_radius){
            // Side Collision
            path_result.free_path_length = obstacle_avoidance::GetPathLengthToSideCollision(radius_to_obstacle, angle_to_obstacle, collision_bounds.min_radius);
        }
        else if(collision_bounds.boundary_radius <= radius_to_obstacle && radius_to_obstacle <= collision_bounds.max_radius){
            // Front Collision
            path_result.free_path_length = obstacle_avoidance::GetPathLengthToFrontCollision(radius_to_obstacle, angle_to_obstacle, car_params::dist_to_front_bumper);
        }
        else{
            // No Collision, Outer Miss
            path_result.clearance = collision_bounds.max_radius - collision_bounds.min_radius;
        }
        return path_result;
}

float GetPathLengthToSideCollision(float radius_to_collision, float angle_to_obstacle, float min_collision_radius){
    float angle_to_collision = atan2(
        sqrt(radius_to_collision*radius_to_collision - min_collision_radius*min_collision_radius),
        abs(min_collision_radius));
    return abs(radius_to_collision * (abs(angle_to_obstacle) - angle_to_collision));
}

float GetPathLengthToFrontCollision(float radius_to_collision, float angle_to_obstacle, float dist_to_front){
        float angle_to_collision = atan2(
            dist_to_front,
            sqrt(radius_to_collision*radius_to_collision - dist_to_front*dist_to_front));
    return abs(radius_to_collision * (abs(angle_to_obstacle) - angle_to_collision));
}

float GetCurvatureFromGoalPoint(Eigen::Vector2f point){
    float x = point[0];
    float y = point[1];

    if(abs(y)>1e-5){ // Non-Zero Curvature
        return 2/(y + Sq(x)/y);
    }
    else{
        return 0.0;
    }
}

float GetCurvatureOptionFromRange(float desired_val_index, float req_val, float min_val, float increment){
    float offset = - floor((req_val - min_val)/increment) * increment;
    return req_val + offset + desired_val_index*increment;
}

} // namespace obstacle_avoidance