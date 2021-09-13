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
\file     obstacle_avoidance.h
\brief    Function interfaces for obstacle avoidance 
\authors  Melissa Cruz, Yuhong Kan, Maxx Wilson (C) 2021
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "navigation/navigation.h"
#include "visualization/visualization.h"

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

void EvaluatePathLength(struct navigation::PathOption path, std::vector<Eigen::Vector2f> point_cloud);
// limit free path length and calculate closest point to goal 
void LimitFreePath(navigation::PathOption& path,const Eigen::Vector2f& goal);

// Given a goal point in base_link frame, return a curvature path that intersects the point
float GetCurvatureFromGoalPoint(Eigen::Vector2f point);

// visualization functions
// call the other visualization functions to visualize all the information needed for obstacle avoidance
void VisualizeObstacleAvoidanceInfo(Eigen::Vector2f& goal,
                           std::vector<navigation::PathOption>& paths,
                           const navigation::PathOption& selected_path,
                           amrl_msgs::VisualizationMsg &msg);
void CarOutliner(amrl_msgs::VisualizationMsg& msg);
void PossiblePathsOutliner(const std::vector<navigation::PathOption>& paths,amrl_msgs::VisualizationMsg& msg);
void SelectedPathOutliner(const navigation::PathOption& selected_path,amrl_msgs::VisualizationMsg& msg);
void GoalOutliner(Eigen::Vector2f& goal, amrl_msgs::VisualizationMsg& msg);
#endif // OBSTACLE_AVOIDANCE_H