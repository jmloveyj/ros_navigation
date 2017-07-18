#include "move_control/rrt_planner.h"

namespace move_control {
bool RrtPlanner::makePlan(vector<Position> &path)
{
    int iteratorNum = 2000;
    RrtNode node;
    node.parentIndex = -1;
    node.pos = start_;

    for(int i=0; i< iteratorNum; i++){
        rrtTree_.push_back(node);
        if(ifFinishPlan(node.pos)) {
            backtraceTree(path);
            return true;
        }

        extendTree(node);

    }
    backtraceTree(path);

    return false;
}

void RrtPlanner::extendTree(RrtPlanner::RrtNode &node)
{
    while(true)
    {

        Position randomPos;
        if(rand()%10 > 3)
            sample(randomPos);
        else
            randomPos = target_;
        //        sample(randomPos);


        int nearNodeIndex = findNearNode(randomPos);
        Position& nearPos = rrtTree_[nearNodeIndex].pos;

        Position newPos;
        if(hypot(nearPos[0]-randomPos[0], nearPos[1]-randomPos[1])<strideStep_){
            newPos = randomPos;
        } else {

            double angleBetweenNodeAndRandom =
                    atan2(randomPos[1]-nearPos[1],randomPos[0] -nearPos[0]);
            newPos[0] = nearPos[0] + strideStep_*cos(angleBetweenNodeAndRandom);
            newPos[1] = nearPos[1] + strideStep_*sin(angleBetweenNodeAndRandom);
        }

        if(!ifBlocked(newPos)) {
            node.pos = newPos;
            node.parentIndex = nearNodeIndex;
            return;
        }
    }
}

void RrtPlanner::sample(Position &randomPos)
{
    Index randomIndex;
    randomIndex[0] = rand()% mapWidthIndex_;
    randomIndex[1] = rand()% mapHeightIndex_;
    map_.getPosition(randomIndex, randomPos);

}

int RrtPlanner::findNearNode(Position &pos)
{
    size_t size = rrtTree_.size();
    int shortestIndex;
    double shortest = 9999.0;
    for(int i=0; i<size; i++) {
        Position& posOnTree = rrtTree_[i].pos;

        double distance =
                hypot(pos[0] - posOnTree[0], pos[1] -posOnTree[1]);
        distance = distance +
                targetTendency_* hypot(posOnTree[0] - target_[0],posOnTree[1] - target_[1]);
        if(distance < shortest){
            shortest = distance;
            shortestIndex = i;
        }
    }

    return shortestIndex;
}

void RrtPlanner::backtraceTree(vector<Position>& path)
{
    path.clear();

    RrtNode& node = rrtTree_.back();
    while(true){
        path.push_back(node.pos);
        if(node.parentIndex == -1)
            break;
        node = rrtTree_[node.parentIndex];
    }
    return;

}


}
