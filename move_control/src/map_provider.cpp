#include "move_control/map_provider.h"
#include <boost/thread/thread.hpp>
#include <move_control/range_map_updater.h>
#include <move_control/laser_map_updater.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>

using namespace grid_map;
namespace move_control{


#define BOILERPLATE_CODE(name, className)\
    if(sensorType == name){\
    return (new className(nh_,tf_,map_,sensorType));\
}

MapProvider::MapProvider(ros::NodeHandle& nh,Length mapLength,bool ifMoving):
    nh_(nh),ifMovingWithRobot_(ifMoving),map_({"master"}),tf_(ros::Duration(10))
{
    mapLength_ = mapLength;

    initParameter();
    initMap();

    //TODO: Now register what we want. Can replace
    // them with parameter from ROS::Parameter externl XML
    registerMapUpdater("/left_range","range");
    registerMapUpdater("/right_range","range");
    registerMapUpdater("/front_left_range","range");
    registerMapUpdater("/front_right_range","range");
    registerMapUpdater("/front_range","range");
    registerMapUpdater("/laser_scan","laser");

    globalGridPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("global_map",1);
    localGridPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map",1);

    startRunLoop();

    if(ifMovingWithRobot_)
        startMoveMapThread();
}

bool MapProvider::getRobotPos(Position &pos)
{
    geometry_msgs::PointStamped posOnMap,posOnRobot;
    posOnRobot.header.stamp = ros::Time();  //latest
    posOnRobot.header.frame_id = robotFrameId_;

    if(!tf_.waitForTransform(mapFrameId_, posOnRobot.header.frame_id,
                             posOnRobot.header.stamp, ros::Duration(1)) ) {
        ROS_ERROR_THROTTLE(1.0, "map provider can't transform from %s to %s at %f",
                           mapFrameId_.c_str(), posOnRobot.header.frame_id.c_str(),
                           posOnRobot.header.stamp.toSec());
        return false;
    }

    tf_.transformPoint(mapFrameId_,posOnRobot,posOnMap);
    pos[0] = posOnMap.point.x;
    pos[1] = posOnMap.point.y;


    return true;

}

bool MapProvider::getRobotPos(Position &pos, double &orientAngle)
{
    geometry_msgs::PoseStamped posOnMap,posOnRobot;
    posOnRobot.header.stamp = ros::Time();  //latest
    posOnRobot.header.frame_id = robotFrameId_;
    posOnRobot.pose.orientation.w = 1;


    if(!tf_.waitForTransform(mapFrameId_, posOnRobot.header.frame_id,
                             posOnRobot.header.stamp, ros::Duration(1)) ) {
        ROS_ERROR_THROTTLE(1.0, "map provider can't transform from %s to %s at %f",
                           mapFrameId_.c_str(), posOnRobot.header.frame_id.c_str(),
                           posOnRobot.header.stamp.toSec());
        return false;
    }

    tf_.transformPose(mapFrameId_,posOnRobot,posOnMap);

    pos[0] = posOnMap.pose.position.x;
    pos[1] = posOnMap.pose.position.y;

    geometry_msgs::Quaternion rotation =posOnMap.pose.orientation;
    orientAngle = tf::getYaw(rotation);

    return true;
}

bool MapProvider::getSubMap(GridMap& map,Position& center,Length length)
{
    boost::shared_lock<boost::shared_mutex> lock(mapMutex_);
    bool ifSuccess;
    map = map_.getSubmap(center,length,ifSuccess);
    return ifSuccess;

}

bool MapProvider::ifCloseToPostion(Position &pos, double tolerance)
{
    Position currentPos;
    getRobotPos(currentPos);
    double distance = hypot(pos[0]-currentPos[0], pos[1]-currentPos[1]);

    return (distance<tolerance);


}

void MapProvider::publishMap(GridMap &map)
{
    nav_msgs::OccupancyGrid occupancyGrid;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "master", 0.0, 255.0, occupancyGrid);
    localGridPub_.publish(occupancyGrid);

}

bool MapProvider::getMap(GridMap &map)
{
    boost::unique_lock<boost::shared_mutex> lock(mapMutex_);
    map = map_;
    return true;
}
//TODO: can replace the way of configure the parameter from const to
// ros::parameter external XML

void MapProvider::initParameter()
{
    mapCenterX_ = 0.0;
    mapCenterY_ = 0.0;

    resolution_ = 0.05;

    mapFrameId_ = "odom";
    robotFrameId_="base_link";

    updateRate_ = 5;
    publishRate_ = 1;
    moveMapRate_ = 2;
}

void MapProvider::initMap()
{
    map_.setFrameId(mapFrameId_);
    map_.setGeometry(mapLength_, resolution_, Position(mapCenterX_, mapCenterY_));
}

void MapProvider::loopUpdateAndPublishMap()
{
    ros::Rate r(updateRate_);
    ros::Duration updateCycle(1.0/updateRate_);
    ros::Duration publishCycle(1.0/publishRate_);
    ros::Time lastPublishTime(0);

    while(nh_.ok())
    {
        updateMap();

        ros::Time now = ros::Time::now();
        if(lastPublishTime+publishCycle <now){
            publishMap();
            lastPublishTime = now;
        }

        r.sleep();

        if(r.cycleTime()> updateCycle) {
            ROS_WARN("Map update loop missed its desired rate of %.4dHz... the loop actually took %.4f seconds", updateRate_,
                     r.cycleTime().toSec());
        }
    }
}

void MapProvider::loopMoveMap()
{
    ros::Rate r(moveMapRate_);

    while(nh_.ok())
    {
        Position robotPos;
        getRobotPos(robotPos);
        map_.move(robotPos);
        r.sleep();
    }
}

void MapProvider::updateMap()
{
    double minX,minY,maxX,maxY;
    minX = 0;
    minY = 0;
    maxX = 0;
    maxY =0 ;
    boost::unique_lock<boost::shared_mutex> lock(mapMutex_);
    vector<boost::shared_ptr<MapUpdater> >::iterator it;
    for(it=mapUpdaters_.begin();it!=mapUpdaters_.end();it++)
    {
        (*it)->updateMap(minX,minY,maxX,maxY);
    }

    composeMasterMapFromLayerdMap(minX,minY,maxX,maxY);
}

void MapProvider::publishMap()
{
    nav_msgs::OccupancyGrid occupancyGrid;
    boost::unique_lock<boost::shared_mutex> lock(mapMutex_);
    grid_map::GridMapRosConverter::toOccupancyGrid(map_, "master", 0.0, 255.0, occupancyGrid);
    lock.unlock();
    globalGridPub_.publish(occupancyGrid);
}

void MapProvider::composeMasterMapFromLayerdMap(double &minX,double &minY,double &maxX,double &maxY)
{
//   map_["master"] =
//           (map_["range"].array().isNaN()&&!map_["laser"].array().isNaN()).select(0,map_["range"])+
//           (map_["laser"].array().isNaN()&&!map_["range"].array().isNaN()).select(0,map_["laser"]);
     map_["master"] = map_["laser"];

}

void MapProvider::startRunLoop()
{
    boost::thread loopThread(boost::bind(&MapProvider::loopUpdateAndPublishMap,this));
}

void MapProvider::startMoveMapThread()
{
   boost::thread loopThread(boost::bind(&MapProvider::loopMoveMap,this));
}

void MapProvider::registerMapUpdater(const string &dataSrcTopicName, const string &sensorType)
{
    boost::shared_ptr<MapUpdater> pMapUpdater ;
    if(!getMapUpdater(sensorType,pMapUpdater)) {
        pMapUpdater = boost::shared_ptr<MapUpdater>(createMapUpdater(sensorType));
        mapUpdaters_.push_back(pMapUpdater);
    }

    pMapUpdater->addMonitorTopic(dataSrcTopicName);
}

bool MapProvider::getMapUpdater(const string &sensorType, boost::shared_ptr<MapUpdater> &pMapUpdater)
{
    bool ifFound = false;
    vector<boost::shared_ptr<MapUpdater> >::iterator it;
    for(it=mapUpdaters_.begin();it!=mapUpdaters_.end();it++)
    {
        if((*it)->getTypeName() == sensorType ) {
            ifFound = true;
            pMapUpdater =  (*it);
            break;
        }
    }
    return ifFound;

}

MapUpdater *MapProvider::createMapUpdater(const string &sensorType)
{
    BOILERPLATE_CODE("range", RangeMapUpdater);
    BOILERPLATE_CODE("laser", LaserMapUpdater);
}

}



