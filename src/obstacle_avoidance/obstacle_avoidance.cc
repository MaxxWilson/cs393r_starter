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

#include "obstacle_avoidance/obstacle_avoidance.h"
#include "obstacle_avoidance/car_params.h"

namespace obstacle_avoidance{

// Uses curvature and point cloud to calculate path option info
float GetPathLengthToObstacle(float curvature, Eigen::Vector2f point_obstacle){
    
    float dist_to_front_bumper = (car_params::length + car_params::wheel_base)/2 + car_params::safety_margin;

    // Straight Ahead / Zero Curvature
    if(curvature < 1e-5){
        if(abs(point_obstacle[1]) > (car_params::safety_margin + car_params::width / 2)){
            // No Collision
            return car_params::max_path_length;
        }
        else{
            // Front Collision
            return point_obstacle[0] - dist_to_front_bumper;
        }
    }

    float max_radius = hypot(
        dist_to_front_bumper,
        1/curvature + car_params::width/2 + car_params::safety_margin);

    float boundary_radius = hypot(
        dist_to_front_bumper,
        1/curvature - car_params::width/2 - car_params::safety_margin);
    
    float min_radius = 1/curvature - car_params::width/2 - car_params::safety_margin;

    float obstacle_radius = (point_obstacle - Eigen::Vector2f(0, 1/curvature)).norm();
    
    float angle_to_obstacle = atan2(point_obstacle[0], 1/curvature - point_obstacle[1]);

    if(obstacle_radius < min_radius){
        // No Collision, Inner Miss
        return car_params::max_path_length;
    }
    else if(min_radius <= obstacle_radius && obstacle_radius <= boundary_radius){
        // Side Collision
        float angle_to_collision = atan2(sqrt(obstacle_radius*obstacle_radius - min_radius*min_radius), min_radius);
        return obstacle_radius * (angle_to_obstacle - angle_to_collision);
    }
    else if(boundary_radius <= obstacle_radius && obstacle_radius <= max_radius){
        // Front Collision
        float angle_to_collision = atan2(dist_to_front_bumper, sqrt(obstacle_radius*obstacle_radius - dist_to_front_bumper*dist_to_front_bumper));
        return obstacle_radius * (angle_to_obstacle - angle_to_collision);
    }
    else{
        // No Collision, Outer Miss
        return car_params::max_path_length;
    }
}

float GetCurvatureFromGoalPoint(Eigen::Vector2f point){
    float x = point[0];
    float y = point[1];

    if(abs(y)>1e-5){ // Non-Zero Curvature
        return 2/(y + pow(x, 2)/y);
    }
    else{
        return 0.0;
    }
}

} // namespace obstacle_avoidance