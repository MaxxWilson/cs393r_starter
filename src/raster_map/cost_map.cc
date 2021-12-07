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
\file    cost_map.cc
\brief   CostMap Implementation
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include "shared/math/statistics.h"
#include "cost_map.h"

namespace costmap{

    CostMap::CostMap() : xy_raster_map::XYRasterMap(){
    
    }

    CostMap::CostMap(
        double map_half_dimension,
        double dist_res,
        double init_grid_val, 
        double range_max,
        double sigma_observation) :
        xy_raster_map::XYRasterMap(map_half_dimension, dist_res, init_grid_val),
        range_max(range_max),
        sigma_observation(sigma_observation)
        {
    }

    /**
     * @brief Given a new laser scan, clear the previous map and build a new image from the scan.
     * 
     * @param cloud vector of cartesian laser scan points
     */
    void CostMap::GenerateMapFromNewScan(const std::vector<Eigen::Vector2f> &cloud){
        ClearMap();

        // For square kernel of odd size, length / 2 - 1, EX. 5x5 -> 2, 17x17 -> 8
        // Size Kernel based on Std of sensor measurement, where past 3 sigma probabilty falls off to zero
        int kernel_half_width = 3 * sigma_observation / dist_res;

        // Iterate through rays in scan
        for(std::size_t i = 0; i < cloud.size(); i++){
            if(cloud[i].norm() > range_max){
                continue;
            }
            auto x = cloud[i].x();
            auto y = cloud[i].y();

            // Get the position of bin corresponding to scan point
            Eigen::Vector2f scan_point_bin(x, y);
            Eigen::Vector2f last_position(0, 0);

            // Apply Gaussian Kernel to Scan point
            for(int row = -kernel_half_width; row <= kernel_half_width; row++){
                for(int col = -kernel_half_width; col <= kernel_half_width; col++){
                    // Get current bin location based on scan point as reference
                    Eigen::Vector2f curr_position = scan_point_bin + Eigen::Vector2f(col, row) * dist_res;
                    
                    // Add bin likelihood using distance from mean
                    double dist_from_scan_point = (scan_point_bin - curr_position).norm();

                    //Calculate normal gaussian distribution to be put into cost map
                    double new_log_likelihood = -(dist_from_scan_point*dist_from_scan_point)/(sigma_observation*sigma_observation);
                    double current_log_likelihood = GetLogLikelihoodAtPosition(curr_position.x(), curr_position.y());
                    
                    max_likelihood = std::max(max_likelihood, new_log_likelihood);

                    if(new_log_likelihood > current_log_likelihood){
                        try{
                            SetLogLikelihoodAtPosition(curr_position.x(), curr_position.y(), new_log_likelihood);
                        } catch(std::out_of_range) {
                            std::cout << "Out of Range" << std::endl;
                            continue;
                        }
                    }
                }   
            }
        }
    }

    void CostMap::UpdateCSMImage(){
        image = cv::Mat(row_num, row_num, CV_8UC3, cv::Scalar(255, 255, 255));
        for(int x = 0; x < row_num; x++){
            for(int y = 0; y < row_num; y++){
                // Image coordinate frame is LHR, BGR
                int image_y = row_num - y - 1;
                image.at<cv::Vec3b>(image_y, x)[0] = 255 - (int) (255*(ConvertLogToStandard(prob_map[x][y])/ConvertLogToStandard(max_likelihood)));
                image.at<cv::Vec3b>(image_y, x)[1] = 255 - (int) (255*(ConvertLogToStandard(prob_map[x][y])/ConvertLogToStandard(max_likelihood)));
                image.at<cv::Vec3b>(image_y, x)[2] = 255;
            }
        }

        double origin = GetIndexFromDist(0.0);

        for(int x = origin - 1; x <= origin + 1; x++){
            for(int y = origin - 1; y <= origin + 1; y++){
                image.at<cv::Vec3b>(y, x)[0] = 0;
                image.at<cv::Vec3b>(y, x)[1] = 0;
                image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    void CostMap::AddPointsToCSMImage(const std::vector<Eigen::Vector2f> &cloud){
        for(Eigen::Vector2f point: cloud){
            try{
                if(point.norm() >= range_max) {
                    continue;
                }
                double xIdx = GetIndexFromDist(point.x());
                double yIdx = GetIndexFromDist(point.y());
                if(xIdx < 0 || xIdx >= prob_map.size() || yIdx < 0 || yIdx >= prob_map[0].size()) throw std::out_of_range("Out of boundaries");
                int image_y = row_num - yIdx - 1;

                for(int x = xIdx - 1; x <= xIdx + 1; x++){
                    for(int y = image_y - 1; y <= image_y + 1; y++){
                        image.at<cv::Vec3b>(y, x)[0] = 0;
                        image.at<cv::Vec3b>(y, x)[1] = 0;
                        image.at<cv::Vec3b>(y, x)[2] = 0;
                    }
                }
            }
            catch(std::out_of_range) {
                continue;
            }
        }
    }

    cv::Mat CostMap::GetCSMImage(){
        return image;
    }

    void CostMap::SetLogLikelihoodAtPosition(double x, double y, double likelihood){
        uint64_t xIdx = GetIndexFromDist(x);
        uint64_t yIdx = GetIndexFromDist(y);
        if(xIdx >= prob_map.size() || yIdx >= prob_map[0].size()) throw std::out_of_range("Out of boundaries");
        prob_map[xIdx][yIdx] = likelihood;
    }

    double CostMap::GetLogLikelihoodAtPosition(double x, double y) const{
        uint64_t xIdx = GetIndexFromDist(x);
        uint64_t yIdx = GetIndexFromDist(y);
        if(xIdx >= prob_map.size() || yIdx >= prob_map[0].size()) throw std::out_of_range("Out of boundaries");
        return prob_map[xIdx][yIdx];
    }

    double CostMap::GetNormalizedLikelihoodAtPosition(double x, double y) const{
        return GetStandardLikelihoodAtPosition(x, y)/ConvertLogToStandard(max_likelihood);
    }

    double CostMap::GetStandardLikelihoodAtPosition(double x, double y) const{
        return ConvertLogToStandard(GetLogLikelihoodAtPosition(x, y));
    }

    double CostMap::ConvertLogToStandard(double log_likelihood) const{
        return 1/(sigma_observation*sqrt(2*M_PI))*exp(log_likelihood);
    }
} // namespace CostMap