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

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <vector>

using std::vector;

#ifndef COST_MAP_H_
#define COST_MAP_H_


namespace costmap{

class CostMap{
    public:
        CostMap();

        //Used to compute normalized log likelihood lookup values from rasterized cost table
        double max_likelihood;

        //Updates rasterized cost table given a new laser scan  
        void UpdateMap(const std::vector<Eigen::Vector2f> &cloud);

        void SetLikelihoodAtPosition(double x, double y, double likelihood);
        double GetLikelihoodAtPosition(double x, double y);
        int GetIndexFromDist(double dist);
        float RoundToResolution(float value);
        void ClearMap();
    private:
        //int row_num = 2*(CONFIG_dist_update_thresh + CONFIG_range_max + CONFIG_laser_offset)/CONFIG_dist_res + 1;
        vector<vector<double>> cost_map_vector;

};

}

#endif   // COST_MAP_H