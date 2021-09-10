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

// Uses curvature and point cloud to calculate path option info
struct navigation::PathOption EvaluatePath(float curvature, std::vector<Eigen::Vector2f> point_cloud){
    struct navigation::PathOption path;

    // TODO: Needs to handle small/zero curvatures

    // Calculate radii that bound collision regions
    float max_radius = hypot(
        1/curvature + car_params::width/2 + car_params::safety_margin,
        (car_params::length + car_params::wheel_base)/2 + car_params::safety_margin);

    float boundary_radius = hypot(
        1/curvature - car_params::width/2 - car_params::safety_margin,
        (car_params::length + car_params::wheel_base)/2 + car_params::safety_margin);
    
    float min_radius = 1/curvature - car_params::width/2 - car_params::safety_margin;

    // FOR EACH POINT IN POINT CLOUD
    Eigen::Vector2f point_obstacle;
    Eigen::Vector2f arc_center(0, -1/curvature);

    // Calculate radius to obstacle
    float obstacle_radius = (point_obstacle - arc_center).norm();

    if(obstacle_radius < min_radius){
        // No Collision, Inner Miss
    }
    else if(min_radius <= obstacle_radius && obstacle_radius <= boundary_radius){
        // Side Collision
    }
    else if(boundary_radius <= obstacle_radius && obstacle_radius <= max_radius){
        // Front Collision
    }
    else{
        // No Collision, Outer Miss
    }

    return path;
}

float GetCurvatureFromGoalPoint(Eigen::Vector2f point){
    return 0.0;
}