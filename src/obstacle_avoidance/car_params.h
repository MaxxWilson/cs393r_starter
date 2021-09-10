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
const float length = 0.535;
const float width = 0.281;
const float wheel_base = 0.5;
const float track_width = 0.25;
const float safety_margin = 0.05;

const float max_curvature = 2.0;
const float min_curvature = -2.0;

// Dynamics
const float max_acceleration = 5.0;
const float min_acceleration = -5.0;
const float max_velocity = 1.0;
}

#endif // CAR_PARAMS_H