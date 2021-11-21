#include "Astar.h"
#include <set>
#include <stack>
namespace astar {
    int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        // {0, 0}, {0, 0}, {0, 0}, {0, 0}    
        {1, -1}, {1, 1}, {-1, 1}, {-1, -1}    
    };
    Astar::Astar(costmap::CostMap const& collision_map) {
        ROW = collision_map.GetRowNum();
        COL = collision_map.GetColNum();
        cellDetails = new cell*[ROW];
        for(int i = 0; i < ROW; ++i) {
            cellDetails[i] = new cell[COL];
        }
        closedList = new bool*[ROW];
        for(int i = 0; i < ROW; ++i) {
            closedList[i] = new bool[COL];
        }
    }
    Astar::~Astar() {
        for(int i = 0; i < ROW; ++i) {
            delete [] cellDetails[i];
        }
        delete [] cellDetails;
        for(int i = 0; i < ROW; ++i) {
            delete [] closedList[i];
        }
        delete [] closedList;
    }
    bool isValid(int row, int col, costmap::CostMap const& collision_map)
    {
        // Returns true if row number and column number
        // is in range
        return (row >= 0) && (row < collision_map.GetRowNum()) && (col >= 0)
            && (col < collision_map.GetColNum());
    }
    bool isNearObstacle(int r, int c, costmap::CostMap const& collision_map) {
        for(int i = 0; i < 8; i++) {
            int newR = r + dirs[i][0];
            int newC = c + dirs[i][1];
            if(!isValid(newR, newC, collision_map)) return true;
            if(!isUnBlocked(collision_map, newR, newC)) return true;
        }
        return false;
    }
    bool isUnBlocked(costmap::CostMap const& collision_map, int row, int col)
    {
        return abs((collision_map.GetValueAtIdx(row, col) - 1.0)) > 1e-4;
    }
    bool isDestination(int row, int col, Grid dest)
    {
        if (row == dest.first && col == dest.second)
            return true;
        else
            return false;
    }

    double getHValue(int row, int col, Grid dest)
    {
        // Return using the distance formula
        return ((double)sqrt(
            (row - dest.first) * (row - dest.first)
            + (col - dest.second) * (col - dest.second)));
    }

    void Astar::InitializeCellDetails(costmap::CostMap const& collision_map) {
        int ROW = collision_map.GetRowNum();
        int COL = collision_map.GetColNum();
        for (int i = 0; i < ROW; i++) {
            for (int j = 0; j < COL; j++) {
                cellDetails[i][j].f = FLT_MAX;
                cellDetails[i][j].g = FLT_MAX;
                cellDetails[i][j].h = FLT_MAX;
                cellDetails[i][j].parent_i = -1;
                cellDetails[i][j].parent_j = -1;
            }
        }
    }
    void Astar::InitializeClosedList(costmap::CostMap const& collision_map) {
        int ROW = collision_map.GetRowNum();
        int COL = collision_map.GetColNum();
        for (int i = 0; i < ROW; i++) {
            for (int j = 0; j < COL; j++) {
                closedList[i][j] = false;
            }
        }
    }
    void Astar::IntializeStart(Grid src) {
        int i = src.first, j = src.second;
        cellDetails[i][j].f = 0.0;
        cellDetails[i][j].g = 0.0;
        cellDetails[i][j].h = 0.0;
        cellDetails[i][j].parent_i = i;
        cellDetails[i][j].parent_j = j;
    }

    bool Astar::AstarSearch(costmap::CostMap const& collision_map, Grid src, Grid dest) {
        printf("AstarSearch\n");
        printf("src:(%d,%d) dest:(%d,%d)\n", src.first, src.second, dest.first, dest.second);

        // If the source is out of range
        if (isValid(src.first, src.second, collision_map) == false) {
            printf("Invalid Src\n");
            return false;
        }
        // If the destination is out of range
        if (isValid(dest.first, dest.second, collision_map) == false) {
            printf("Invalid Dest\n");
            return false;
        }
    
        // Either the source or the destination is blocked
        if (isUnBlocked(collision_map, src.first, src.second) == false
            || isUnBlocked(collision_map, dest.first, dest.second)
                == false) {
            printf("Blocked Src or Dest\n");
            return false;
        }
    
        // If the destination cell is the same as source cell
        if (isDestination(src.first, src.second, dest) == true) {
            printf("Src == Dest\n");
            return true;
        }
        InitializeClosedList(collision_map);
        InitializeCellDetails(collision_map);
        IntializeStart(src);
        std::set<std::pair<double, Grid>> openList; // treeset
        openList.insert(std::make_pair(0.0, std::make_pair(src.first, src.second)));
        bool foundDest = false;
        while(!openList.empty()) {
            std::pair<double, Grid> p = *openList.begin();
            openList.erase(openList.begin());
            int r = p.second.first;
            int c = p.second.second;
            closedList[r][c] = true;
            for(int i = 0; i < 8; i++) {
                int newR = r + dirs[i][0];
                int newC = c + dirs[i][1];
                if(!isValid(newR, newC, collision_map)) continue;
                // printf("new Node:(%d,%d)\n", newR, newC);
                if (isDestination(newR, newC, dest) == true) {
                    // Set the Parent of the destination cell
                    cellDetails[newR][newC].parent_i = r;
                    cellDetails[newR][newC].parent_j = c;
                    printf("The destination cell is found\n");
                    foundDest = true;
                    return true;
                }
                else if (closedList[newR][newC] == false
                     && isUnBlocked(collision_map, newR, newC)
                            == true) {
                    double gNew = cellDetails[r][c].g + 1.0;
                    if(i >= 4) {
                        gNew = cellDetails[r][c].g + 1.414;
                    }
                    double hNew = getHValue(newR, newC, dest);
                    double fNew = gNew + hNew;
                    if(isNearObstacle(newR, newC, collision_map)) {
                        fNew += 2;
                    }
                    if (cellDetails[newR][newC].f == FLT_MAX
                        || cellDetails[newR][newC].f > fNew) {
                        openList.insert(std::make_pair(
                            fNew, std::make_pair(newR, newC)));
    
                        // Update the details of this cell
                        cellDetails[newR][newC].f = fNew;
                        cellDetails[newR][newC].g = gNew;
                        cellDetails[newR][newC].h = hNew;
                        cellDetails[newR][newC].parent_i = r;
                        cellDetails[newR][newC].parent_j = c;
                    }
                }
            }
        }
        return foundDest;
    }

    void Astar::tracePath(amrl_msgs::VisualizationMsg& msg, costmap::CostMap const& collision_map, Grid dest) {
        int row = dest.first;
        int col = dest.second;
    
        std::stack<Grid> Path;
    
        while (!(cellDetails[row][col].parent_i == row
                && cellDetails[row][col].parent_j == col)) {
            Path.push(std::make_pair(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
        }
    
        Path.push(std::make_pair(row, col));
        Eigen::Vector2f prev = collision_map.GetLocFromIndex(row, col);
        while (!Path.empty()) {
            Grid p = Path.top();
            Path.pop();
            Eigen::Vector2f cur = collision_map.GetLocFromIndex(p.first, p.second);
            visualization::DrawLine(prev, cur, 0x1e9aa8, msg);
            prev = cur;
        }
    
        return;
    }

    void Astar::GeneratePathVector(costmap::CostMap const& collision_map, Grid dest){
        int row = dest.first;
        int col = dest.second;
    
        while (!(cellDetails[row][col].parent_i == row
                && cellDetails[row][col].parent_j == col)) {
            Eigen::Vector2f cur = collision_map.GetLocFromIndex(row, col);
            path_vector_.insert(path_vector_.begin(), Eigen::Vector2f(cur));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
        }
    }

    std::vector<Eigen::Vector2f> Astar::GetPathVector(){
        return this->path_vector_;
    }
}