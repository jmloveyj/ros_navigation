#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H
#include "move_control/map_global_planner.h"

using namespace std;
using namespace grid_map;
namespace move_control {
class RrtPlanner: public GlobalPlanner {
public:
    typedef struct {
        grid_map::Position pos;
        int parentIndex;
    } RrtNode;



    RrtPlanner(GridMap& map,
               Position& start,
               Position& target,
               double closeTolerance=0.2):
        GlobalPlanner(map, start,target,closeTolerance){
        //TODO the plan stride can be configured from ros::Parameter
        strideStep_ = 0.4; //m
        targetTendency_ = 0.5;
        targetRatio_ = 5.0;
    }
    ~RrtPlanner(){}
    bool makePlan(vector<Position>& path);
private:
    vector<RrtNode> rrtTree_;
    double strideStep_;
    int targetTendency_;
    double targetRatio_;

    void extendTree(RrtNode& node);
    void sample(Position& randomPos);
    int findNearNode(Position& pos);
    void backtraceTree(vector<Position>& path);

};

}
#endif // RRT_PLANNER_H
