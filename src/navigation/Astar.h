#ifndef ASTAR_H_
#define ASTAR_H_
#include "cost_map/cost_map.h"
#include <iostream>
#include <utility>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "visualization/visualization.h"
namespace astar {
    struct cell {
        int parent_i, parent_j;
        double f, g, h; // f = g + h
    };
    typedef std::pair<int, int> Grid;
    bool isValid(int row, int col, costmap::CostMap const& collision_map);
    bool isUnBlocked(costmap::CostMap const& collision_map, int row, int col);
    bool isDestination(int row, int col, Grid dest);
    double getHValue(int row, int col, Grid dest); 
    class Astar {
    private:
        struct cell** cellDetails;
        bool** closedList;
        int ROW;
        int COL;
        std::vector<Eigen::Vector2f> path_vector_;
    public:
        Astar(costmap::CostMap const& collision_map);
        ~Astar();
        bool AstarSearch(costmap::CostMap const& collision_map, Grid src, Grid dest);
        void InitializeCellDetails(costmap::CostMap const& collision_map);
        void InitializeClosedList(costmap::CostMap const& collision_map);
        void IntializeStart(Grid src);
        void tracePath(amrl_msgs::VisualizationMsg& msg, costmap::CostMap const& collision_map, Grid dest);
        void GeneratePathVector(costmap::CostMap const& collision_map, Grid dest);
        std::vector<Eigen::Vector2f> GetPathVector();
    };
}


#endif