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
\file    cost_map.h
\brief   CostMap Header
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include "shared/math/line2d.h"
#include "vector_map/vector_map.h"
#include "visualization/CImg.h"

#include <vector>
#include <utility>

#include "raster_map/xy_raster_map.h"

using std::vector;
using cimg_library::CImg;
using cimg_library::CImgDisplay;

#ifndef COST_MAP_H_
#define COST_MAP_H_

namespace costmap{

class CostMap : public xy_raster_map::XYRasterMap {
    public:
        CostMap();

        /**
         * @brief Construct a new CostMap object with initial size and grid values
         * 
         * @param map_dimension length/width of map in meters
         * @param dist_res dimension of map grid squares in meters
         * @param init_grid_val initial grid values as double
         */
        CostMap(double map_dimension, double dist_res, double init_grid_val,  double range_max, double sigma_observation);

        //Updates rasterized cost table given a new laser scan  
        void GenerateMapFromNewScan(const std::vector<Eigen::Vector2f> &cloud);

        void SetLogLikelihoodAtPosition(double x, double y, double likelihood);

        double GetLogLikelihoodAtPosition(double x, double y) const;

        double GetNormalizedLikelihoodAtPosition(double x, double y) const;

        double GetStandardLikelihoodAtPosition(double x, double y) const;

        double ConvertLogToStandard(double log_likelihood) const;

        void UpdateCSMImage();
        void AddPointsToCSMImage(const std::vector<Eigen::Vector2f> &cloud);
        cv::Mat GetCSMImage();
    private:
        double range_max;
        double sigma_observation;

};

}

#endif   // COST_MAP_H