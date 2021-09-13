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

#include <float.h>
#include "shared/math/line2d.h"

#include "gflags/gflags.h"


// line arguments used in obstacle avoidance function
DEFINE_double(clearance_param,0.1,"clearance parameter used in scoring function");
DEFINE_double(distance_goal_param,-0.1,"distance to goal parameter used in scoring function");


// Uses curvature and point cloud to calculate path option info
struct navigation::PathOption EvaluatePath(float curvature, const std::vector<Eigen::Vector2f>& point_cloud){
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

//helper function: get the angle between vector[p_middle,p_left] and vector[p_middle,p_right]
double getAngle(const Eigen::Vector2f& p_middle, const Eigen::Vector2f& p_left, const Eigen::Vector2f& p_right){
    Eigen::Vector2f v1 = p_middle - p_left;
    Eigen::Vector2f v2 = p_middle - p_right;
    double angle = acos((v1/v1.norm()).dot(v2/v2.norm()));
    return angle;
}

// limit free path length and calculate closest point to goal 
void LimitFreePath(navigation::PathOption& path,const Eigen::Vector2f& goal){
    Eigen::Vector2f rotate_center(0, 1/path.curvature);
    Eigen::Vector2f car_odem(0,0);
    double angle = getAngle(rotate_center,goal,car_odem);
    if (goal[0] < 0) angle = 2*M_PI - angle; // if goal is at the back of the car
    // check whether the free path need to be trimmed
    if(abs(angle/path.curvature)<path.free_path_length){
        // trim free path and change obstruction point to the closest point
        path.free_path_length = abs(angle/path.curvature);
        // set the obstruction to the closet point
        path.obstruction = rotate_center + 1/path.curvature*((goal-rotate_center)/(goal-rotate_center).norm());
    }
    double theta = path.free_path_length * path.curvature;
    path.closest_point.x() = 1/path.curvature*sin(theta);
    path.closest_point.y() = 1/path.curvature*(1-cos(theta));
}

//calculate distance to goal of the specific path
double GetDistanceToGoal(const navigation::PathOption& path,const Eigen::Vector2f& goal){
    return (path.closest_point-goal).norm();
}

//Scoring function
struct navigation::PathOption ChooseBestPath(std::vector<navigation::PathOption>& paths, const Eigen::Vector2f& goal){
    navigation::PathOption* bestPath = NULL;
    double best_score = -DBL_MAX;
    for(auto& path:paths){
        double distance_to_goal = GetDistanceToGoal(path,goal);
        double score = path.free_path_length + FLAGS_clearance_param * path.clearance + FLAGS_distance_goal_param * distance_to_goal;
        if(score>best_score){
            best_score = score;
            bestPath = &path;
        }
    }
    return *bestPath;
}

// Visualization for needed for obstacle avoidance
void VisualizeObstacleAvoidanceInfo(Eigen::Vector2f& goal,
                           std::vector<navigation::PathOption>& paths,
                           const navigation::PathOption& selected_path,
                           amrl_msgs::VisualizationMsg &msg){
    CarOutliner(msg);
    PossiblePathsOutliner(paths,msg);
    SelectedPathOutliner(selected_path,msg);
    GoalOutliner(goal,msg);
}

// Outline the car[Yellow Line] and safety margin
void CarOutliner(amrl_msgs::VisualizationMsg &msg){
    Eigen::Vector2f front_left((car_params::wheel_base+car_params::length)/2,car_params::width/2);
    Eigen::Vector2f front_right((car_params::wheel_base+car_params::length)/2,-car_params::width/2);
    Eigen::Vector2f back_left(-(car_params::length-car_params::wheel_base)/2,car_params::width/2);
    Eigen::Vector2f back_right(-(car_params::length-car_params::wheel_base)/2,-car_params::width/2);
    //car front
    visualization::DrawLine(front_left,front_right,0xfcd703,msg);
    //car back
    visualization::DrawLine(back_left,back_right,0xfcd703,msg);
    //car left
    visualization::DrawLine(front_left,back_left,0xfcd703,msg);
    //car right
    visualization::DrawLine(front_right,back_right,0xfcd703,msg);
    //TODO: safety margin
}
// Draw all the possible path options
void PossiblePathsOutliner(const std::vector<navigation::PathOption>& paths,amrl_msgs::VisualizationMsg& msg){
    for(auto& path:paths){
        visualization::DrawPathOption(path.curvature,path.free_path_length,0.0,msg);
    }
}
// Draw the selected path
void SelectedPathOutliner(const navigation::PathOption& selected_path,amrl_msgs::VisualizationMsg& msg){
    visualization::DrawPathOption(selected_path.curvature,selected_path.free_path_length,selected_path.clearance,msg);
    //draw the closest point [Blue Cross]
    visualization::DrawCross(selected_path.closest_point,0.25,0x1e9aa8,msg);
}
// Draw goal [Green Cross]
void GoalOutliner(Eigen::Vector2f& goal, amrl_msgs::VisualizationMsg& msg){
    visualization::DrawCross(goal,0.5,0x1ea845,msg);
}


