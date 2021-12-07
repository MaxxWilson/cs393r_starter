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
\file    collision_map.cc
\brief   General 2D Probability Map With ROS Image interfacing
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#ifndef COLLISION_MAP_H_
#define COLLISION_MAP_H_

#include "raster_map/xy_raster_map.h"

#include "shared/math/line2d.h"

namespace collision_map
{

/** YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTTT
 * @brief Construct a new XYProbabilityMap object
 * 
 * Implements a 2D Rasterized Likelihood Image. Uses log weights internally,
 * with getters for log weight, normalized weight, and weight from standard Gaussian form.
 * Image corresponds directly to XY distance in the world, so the image is initialized with length and width in meters.
 * Values are discretized into a grid, where the value of a bin is the the maximum probability within that area.
 * Object can easily be normalized into an 8-Bit BGR image for display in ROS.
 */
    class CollisionMap : xy_raster_map::XYRasterMap {
    public:
        CollisionMap();

        CollisionMap(double map_half_dimension, double dist_res, double init_grid_val, int dilation_factor);
        
        void DrawMap(const std::vector<geometry::line2f> lines);

    private:
        int dilation_factor;
    };
} // namespace collision_map

#endif // COLLISION_MAP_H_