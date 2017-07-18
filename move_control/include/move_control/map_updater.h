#ifndef MAP_UPDATER_H
#define MAP_UPDATER_H
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "grid_map_core/grid_map_core.hpp"
namespace move_control {

class MapUpdater {
public:
    MapUpdater(ros::NodeHandle& nh, tf::TransformListener& tf, grid_map::GridMap &map,const std::string& typeName):
        nh_(nh),tf_(tf),map_(map),typeName_(typeName){
        if(!map_.exists(typeName))
            map_.add(typeName);
    }
    ~MapUpdater(){}
    // update map and point out the map range updated
    virtual void updateMap(double &minX,double &minY,double &maxX,double &maxY)=0;

    std::string getTypeName(){
        return typeName_;
    }

    virtual void addMonitorTopic(const std::string &topicName)=0;


protected:
    std::string typeName_;
    typedef struct {
      grid_map::Position start;
      grid_map::Position end;
      bool ifClearEnd;   // true: clear the end in the map. false: mark as obstacle
    } RangeSample;

    ros::NodeHandle& nh_;
    tf::TransformListener& tf_;
    grid_map::GridMap& map_;

    void lineOnMap(const RangeSample &rangeSample){
        for(grid_map::LineIterator it(map_,rangeSample.start,rangeSample.end);
            !it.isPastEnd(); ++it){
            clearCell(*it);
        }

        if(!rangeSample.ifClearEnd) {
            grid_map::Index endIndex;
            if(map_.getIndex(rangeSample.end,endIndex)){
                markCell(endIndex);
            }
        }
    }

    void markCell(const grid_map::Index& id) {
        grid_map::Matrix& map = map_[typeName_];
        float& cellValue = map(id(0),id(1));
        if(cellValue <= 0 || std::isnan(cellValue))
            cellValue = 30.0;
        else if(cellValue <= 150.0)
            cellValue = cellValue + 30.0;
    }

    void clearCell(const grid_map::Index& id){
        grid_map::Matrix& map = map_[typeName_];
        float& cellValue = map(id(0),id(1));
        if(cellValue<=0 || std::isnan(cellValue))
            cellValue = 0.0;
        else
            cellValue = cellValue -10.0;

        if(cellValue <0.0)
            cellValue = 0.0;
    }

    void touch(double &minX, double &minY, double &maxX, double &maxY, double &x, double &y) {
         minX = std::min(minX,x);
         minY = std::min(minY,y);
         maxX = std::max(maxX,x);
         maxY = std::max(maxY,y);
    }

};

}
#endif // MAP_UPDATER_H
