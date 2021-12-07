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

#include "raster_map/collision_map.h"

namespace collision_map{

    CollisionMap::CollisionMap() : xy_raster_map::XYRasterMap(){
    
    }

    CollisionMap::CollisionMap(
        double map_half_dimension,
        double dist_res,
        double init_grid_val,
        int dilation_factor
        ) :
        xy_raster_map::XYRasterMap(map_half_dimension, dist_res, init_grid_val),
        dilation_factor(dilation_factor)
        {
    }

    void CollisionMap::DrawMap(const std::vector<geometry::line2f> lines){
        for (size_t i = 0; i < lines.size(); ++i) {
            const geometry::line2f line = lines[i];
            for(int i = 0; i <= line.Length() / dist_res; i++){
                Eigen::Vector2f point = line.p0 + line.Dir() * dist_res * i;
                for(int x = -dilation_factor; x <= dilation_factor; x++){
                    for(int y = -dilation_factor; y <= dilation_factor; y++){
                        int xIdx = GetIndexFromDist(point.x() + x*dist_res);
                        int yIdx = GetIndexFromDist(point.y() + y*dist_res);
                        prob_map[xIdx][yIdx] = 1.0;
                    }
                }
            }
        }
    }

} // namespace collision_map