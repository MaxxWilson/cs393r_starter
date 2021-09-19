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
\file     car_params.h
\brief    Constants defining car geometry and dynamics
\authors  Melissa Cruz, Yuhong Kan, Maxx Wilson (C) 2021
*/
//========================================================================

#ifndef CAR_PARAMS_H
#define CAR_PARAMS_H

// TODO: Values stolen from ut_automata/config/, idk if they are accurate - Maxx
namespace car_params{

// Geometry
const float length = 0.508; // 20"
const float width = 0.2667; // 10.5"
const float wheel_base = 0.32385; // 12.75"
const float track_width = 0.235; // 9.25
const float safety_margin = 0.05; // 2"

const float dist_to_front_bumper = (length + wheel_base)/2 + safety_margin;

const float max_curvature = 0.6;
const float min_curvature = -0.6;

// Dynamics
const float max_acceleration = 4.0; //1.865;
const float min_acceleration = -4.0; //-1.171;
const float max_velocity = 1.0;

// Algorithmic Parameters
const float safe_distance = 0.125; // safe distance used in TOC control, stops with 5" left to obstacle
const float max_path_length = 5.0;
const float curvature_increment = 0.05;
const float num_curves = floor((max_curvature - min_curvature)/curvature_increment) + 1;
const float clearance_factor = 0.15; // ~6"
}

#endif // CAR_PARAMS_H