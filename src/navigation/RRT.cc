#include "RRT.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <float.h>
using std::vector;
namespace RRT{
    bool operator == (Node const& n1,Node const& n2)
    {
        return (n1.xIdx == n2.xIdx) && (n1.yIdx == n2.yIdx);
    }
    bool operator != (Node const& n1,Node const& n2)
    {
        return !(n1 == n2);
    }
    bool IsNodeCollide(cosnt CostMap& collision_map, const Node& node) {
        if(collision_map.GetValueAtIdx(node.xIdx, node.yIdx) == 1.0) return true
        return false;
    }

    int RRT(cosnt CostMap& collision_map, const Node& start, const Node& end, list<Node>& plan) {
        int xSize = collision_map.GetRowNum();
        int ySize = collision_map.GetColNum();
        vector<Node> graph;
        graph.push_back(start);
        int cnt = 0;
        int lim = 1000000;
        float cur_angle = start_angle;
        while(cnt < lim) {
            cnt++;
            Node newNode = CreateNewNode(xSize, ySize);
            if(IsNodeCollide(collision_map, newNode)) continue;
            Node* nearestNode = FindNearestNode(newNode, graph); 
            if((*nearestNode) == newNode) continue;
            newNode.parent = nearestNode;
            graph.push_back(newNode);
            if(newNode == end) {
                ConstructPath(start, end, path);
                return 1;
            }
        }
        return 0;
    }
    Node CreateNewNode(int xSize, int ySize) {
        int xIdx = rand() % xSize;
        int yIdx = rand() % ySize;
        Node newNode(xIdx, yIdx);
        return newNode;
    }
    Node* FindNearestNode(const Node& newNode, vector<Node>& graph) {
        Node* nearestNode = NULL;
        double minDist = DBL_MAX;
        for(Node& n: graph) {
            double curDist = sqrt(pow(newNode.xIdx - n.xIdx, 2) + pow(newNode.yIdx - n.yIdx, 2));
            if(curDist < minDist) {
                nearstNode = &n;
                minDist = curDist;
            }
        }
        // TODO: control, change newNode
        
        return nearestNode;
    }
    void ConstructPath(const Node& start, const Node& end, list<Node>& plan) {
        const Node* cur = &end;
        do {
            plan.push_front(*cur);
            cur = cur.parent;
        } while((*cur) != start);
    }
}
