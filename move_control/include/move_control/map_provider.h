#ifndef MAP_PROVIDER_H
#define MAP_PROVIDER_H

#include "grid_map_core/grid_map_core.hpp"
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include <tf/transform_listener.h>
using namespace std;
using namespace grid_map;
namespace move_control{
class MapUpdater;
/*!
 * This class holds the interface to get the instant map which
 * includes not only global map but also any local map.  Internally, this
 * class would handle anything about updating map. It should provide
 * easy way to extend any sensor source to update map. It should also provide
 * extension to publish map in any desired message type.
 */
class MapProvider {
public:
    MapProvider(ros::NodeHandle& nh,Length mapLength= Length(30.0,30.0),bool ifMoving=false);
    ~MapProvider(){}

    bool getRobotPos(Position& pos);
    bool getRobotPos(Position& pos, double& orientAngle);
    bool getSubMap(GridMap& map, Position& center, Length length);
    bool ifCloseToPostion(Position& pos, double tolerance);
    void publishMap(GridMap& map);
    bool getMap(GridMap& map);

private:
    ros::NodeHandle& nh_;
    tf::TransformListener tf_;
    GridMap map_;
    boost::shared_mutex mapMutex_;
    vector<boost::shared_ptr<MapUpdater> > mapUpdaters_;
    uint updateRate_,publishRate_,moveMapRate_;
    Length mapLength_;
    bool ifMovingWithRobot_;
    double mapCenterX_, mapCenterY_; //m
    double resolution_; // meter/cell
    std::string mapFrameId_,robotFrameId_;

    ros::Publisher globalGridPub_;
    ros::Publisher localGridPub_;


    void initParameter();
    void initMap();
    void loopUpdateAndPublishMap();
    void loopMoveMap();
    void updateMap();
    void publishMap();
    void composeMasterMapFromLayerdMap(double &minX,double &minY,double &maxX,double &maxY);
    void startRunLoop();
    void startMoveMapThread();
    void registerMapUpdater(const std::string &dataSrcTopicName, const std::string &sensorType="range");
    bool getMapUpdater(const std::string &sensorType, boost::shared_ptr<MapUpdater> &pMapUpdater);
    MapUpdater* createMapUpdater(const std::string &sensorType);

};

}

#endif // MAP_PROVIDER_H
