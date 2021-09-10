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

#include "navigation/navigation.h"

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

struct navigation::PathOption PathOptionFromPointCloud(float curvature);

#endif // OBSTACLE_AVOIDANCE_H