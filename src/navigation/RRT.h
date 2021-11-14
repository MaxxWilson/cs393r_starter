#ifndef RRT_H
#define RRT_H
#include <vector>
#include "shared/math/geometry.h"
#include "cost_map/cost_map.h"
using geometry::line2f;
using std::vector;
using std::list;
using costmap::CostMap;
namespace rrt{
class Node {
public:
    // Rasterized map Idx
    int xIdx;
    int yIdx;
    // map frame coordinate
    float x;
    float y;
    float angle; // car angle
    Node* parent;
    Node(int _xIdx, int _yIdx): xIdx(_xIdx), yIdx(_yIdx) {}
    friend bool operator == (Node const &, Node const &);
};
// 
int RRT(cosnt CostMap& collision_map, const Node& start, const Node& end, list<Node*>& plan);
bool IsInObstacle(cosnt CostMap& collision_map, const Node& node);
Node CreateNewNode(int xSize, int ySize);
Node* FindNearestNode(const Node& newNode, vector<Node>& graph);

}


#endif // RRT_H