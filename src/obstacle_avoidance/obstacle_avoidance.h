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

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

namespace obstacle_avoidance{

void EvaluatePathLength(struct navigation::PathOption path, std::vector<Eigen::Vector2f> point_cloud);

void CleanVelocityBuffer(std::vector<navigation::CommandStamped> &v, uint64_t time);

// Given a goal point in base_link frame, return a curvature path that intersects the point
float GetCurvatureFromGoalPoint(Eigen::Vector2f point);

//Checks if there is new point cloud data
float Integrate(uint64_t time, std::vector<navigation::CommandStamped> &v);
}                    

#endif // OBSTACLE_AVOIDANCE_H