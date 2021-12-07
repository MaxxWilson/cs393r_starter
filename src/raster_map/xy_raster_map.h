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
\file    xy_raster_map.cc
\brief   General 2D Raster Map With ROS Image interfacing
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#ifndef XY_PROBABILITY_MAP_H_
#define XY_PROBABILITY_MAP_H_

#include "shared/math/math_util.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

#include <vector>
#include <utility>

namespace xy_raster_map
{

/**
 * @brief Construct a new XYRasterMap object
 * 
 * Implements a 2D Rasterized Image.
 * Image corresponds directly to XY distance in the world, so the image is initialized with length and width in meters.
 * Values are discretized into a grid, where the value of a bin is the value within that area.
 * Object can easily be normalized into an 8-Bit BGR image for display in ROS.
 */
    class XYRasterMap {
    public:
        XYRasterMap();
        
        /**
         * @brief Construct a new XYRasterMap object with initial size and grid values
         * 
         * @param map_dimension length/width of map in meters
         * @param dist_res dimension of map grid squares in meters
         * @param init_grid_val initial grid values as double
         */
        XYRasterMap(double map_dimension, double dist_res, double init_grid_val);

        /**
         * @brief Clears all values in map. For log like
         */
        void ClearMap();

        /**
         * @brief Rounds a float value to a given decimal resolution. Used to
         * convert distances to map indexes dependent on the desired map resolution.
         * 
         * @param value number to round
         * @param res decimal resolution in meters
         * @return float rounded value
         */
        float RoundToResolution(float value, float res) const;

        /**
         * @brief Get map index from a given position distance
         * 
         * This class assumes a square image, so X and Y both use this function
         * 
         * @param dist position in meters
         * @return int index of grid area corresponding to that distance value
         */
        int GetIndexFromDist(double dist) const;

        /**
         * @brief Get indexes as pair given a position
         * 
         * @param loc position
         * @return std::pair<int, int> index pair
         */
        std::pair<int, int> GetIndexPairFromDist(Eigen::Vector2f loc) const;

        /**
         * @brief Get the XY position at an index value
         * 
         * @param xIdx x-index as integer
         * @param yIdx y-index as integer
         * @return Eigen::Vector2f XY position
         */
        Eigen::Vector2f GetLocFromIndex(int xIdx, int yIdx) const;

        /**
         * @brief Getter for number of rows
         * @return int number of rows
         */
        inline int GetRowNum() const {
            return prob_map.size();
        };

        /**
         * @brief Getter for number of columns
         * @return int number of columns
         */
        inline int GetColNum() const {
            return prob_map[0].size();
        };

        /**
         * @brief Get the Value at raw index
         * 
         * @param xIdx x-index as integer
         * @param yIdx y-index as integer
         * @return double value in raster map
         */
        inline double GetValueAtIdx(int xIdx, int yIdx) const{
            return prob_map[xIdx][yIdx];
        }

    protected:
        double map_length_dist;
        double dist_res;
        int row_num;
        double init_grid_val;
        double max_likelihood;

        std::vector<std::vector<double>> prob_map;
        cv::Mat image;
    };

} // namespace xy_probability_map

#endif // XY_PROBABILITY_MAP_H_