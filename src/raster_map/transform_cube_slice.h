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
\file    transform_cube_slice.h
\brief   Implements Likelihood Cube Display from Correlative Scan Matching
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include <vector>
#include <utility>

#include "raster_map/xy_raster_map.h"

#ifndef TRANSFORM_CUBE_SLICE_H_
#define TRANSFORM_CUBE_SLICE_H_

namespace transform_cube_slice {

class TransformCubeSlice : public xy_raster_map::XYRasterMap {
    public:
        TransformCubeSlice();

        /**
         * @brief Construct a new TransformCubeSlice object with initial size and grid values
         * 
         * @param map_dimension length/width of map in meters
         * @param dist_res dimension of map grid squares in meters
         * @param init_grid_val initial grid values as double
         */
        TransformCubeSlice(double map_dimension, double dist_res, double init_grid_val);

        /**
         * @brief Clears scan map and regenerates given a new laser scan
         * 
         * @param cloud laser scan as cartesian points
         */
        void GenerateMapFromNewScan(const std::vector<Eigen::Vector2f> &cloud);

        /**
         * @brief Set the Log Likelihood at given position
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @param likelihood log likelihood as a double
         */
        void SetLogLikelihoodAtPosition(double x, double y, double likelihood);

        /**
         * @brief Get the Log Likelihood at given position
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @return double log likelihood
         */
        double GetLogLikelihoodAtPosition(double x, double y) const;

        /**
         * @brief Get the Likelihood at given position normalzied by the max value in image
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @param likelihood normalized likelihood as a double
         */
        double GetNormalizedLikelihoodAtPosition(double x, double y) const;

        /**
         * @brief Get the Likelihood at given position as standard gaussian probability (area under curve of 1)
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @param likelihood standard likelihood as a double
         */
        double GetStandardLikelihoodAtPosition(double x, double y) const;

        /**
         * @brief Converts log likelihood to standard gaussian probability sample
         * 
         * @param log_likelihood 
         * @return double standard likelihood
         */
        double ConvertLogToStandard(double log_likelihood) const;

        /**
         * @brief updates image object, drawing the current values in the probability map
         */
        void DrawCSMImage();

        /**
         * @brief Draws points on the image as black 3x3 squares
         * 
         * @param cloud vector of cartesian points
         */
        void DrawScanCloudOnImage(const std::vector<Eigen::Vector2f> &cloud);

    private:
        double range_max;
        double sigma_observation;

};

}

#endif   // TRANSFORM_CUBE_SLICE_H_