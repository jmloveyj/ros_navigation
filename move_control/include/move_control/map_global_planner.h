#ifndef MAP_GLOBAL_PLANNER_H
#define MAP_GLOBAL_PLANNER_H

#include "grid_map_core/grid_map_core.hpp"
using namespace grid_map;
using namespace std;
namespace move_control {
class GlobalPlanner {
public:
    GlobalPlanner(GridMap& map,
                  Position& start,
                  Position& target,
                  double closeTolerance):
        map_(map),start_(start),target_(target),
        closeTolerance_(closeTolerance){
        if(&map == nullptr)
            return;
        ifTargetInsideMap_=map_.isInside(target);
        Size size= map_.getSize();
        mapWidthIndex_ = size[0];
        mapHeightIndex_ = size[1];
    }
    ~GlobalPlanner(){}
    virtual bool makePlan(vector<Position>& path)=0;

protected:
    GridMap& map_;
    Position start_,target_;
    int mapWidthIndex_,mapHeightIndex_;


    bool ifFinishPlan(Position& pointA) {
        if(ifTargetInsideMap_)
            return ifCloseToTarget(pointA);
        else
            return ifCloseToMapBoundary(pointA);
    }

    bool ifBlocked(Position& pointA) {
        //TODO the radius should be configured from ROS::Parameter footPrint
        double radius = 0.3;
        Matrix& masterMap = map_["master"];
        for (grid_map::CircleIterator iterator(map_, pointA, radius);
             !iterator.isPastEnd(); ++iterator) {
             float& cellValue = masterMap((*iterator)(0),(*iterator)(1));

             if(isnan(cellValue))
                 continue;
             if(cellValue>0.0)
                 return true;
        }

        return false;
    }

private:
    double closeTolerance_;
    bool ifTargetInsideMap_;


    bool ifCloseToTarget(Position& pointA){
        double distance = hypot(pointA[0]-target_[0], pointA[1]-target_[1]);
        return (distance<closeTolerance_);

    }

    bool ifCloseToMapBoundary(Position& pointA) {
        Length lennth= map_.getLength();
        Position origin = map_.getPosition();
        double plus_boundary_x = origin[0]+lennth[0]/2;
        double plus_boundary_y = origin[1]+lennth[1]/2;
        double minus_boundary_x = origin[0]-lennth[0]/2;
        double minus_boundary_y = origin[1]-lennth[1]/2;

        if((plus_boundary_x -pointA[0]) < closeTolerance_)
            return true;
        if((plus_boundary_y -pointA[1]) < closeTolerance_)
            return true;
        if((pointA[0] - minus_boundary_x) < closeTolerance_)
            return true;
        if((pointA[1] - minus_boundary_y) < closeTolerance_)
            return true;

        return false;

    }

};

}

#endif // MAP_GLOBAL_PLANNER_H
