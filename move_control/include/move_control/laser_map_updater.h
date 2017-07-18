#include "move_control/map_updater.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include <boost/thread/mutex.hpp>
#include "sensor_msgs/PointCloud2.h"

#ifndef LASER_MAP_UPDATER_H
#define LASER_MAP_UPDATER_H
using namespace grid_map;

namespace move_control {
class LaserMapUpdater:public MapUpdater {
public:
    LaserMapUpdater(ros::NodeHandle& nh, tf::TransformListener& tf, grid_map::GridMap& map, const std::string& sensorType="laser"):
        MapUpdater(nh,tf,map,sensorType),
        lastIncomingMsgTime_(0),
        msgProcessCycle_(0.2){}

    ~LaserMapUpdater(){}
    // update map and point out the map range updated
    void updateMap(double &minX,double &minY,double &maxX,double &maxY);


    void addMonitorTopic(const std::string &topicName);
private:
    std::vector<boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>
    > > laser_notifiers_;
    std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>
    > > laser_subs_;

    std::vector<RangeSample> laserSampleBuffer_;
    boost::mutex laserSampleBufferMutex_;

    ros::Time lastIncomingMsgTime_;
    ros::Duration msgProcessCycle_;


    void bufferIncomingMsg(const sensor_msgs::LaserScanConstPtr& msg);
    void transformLaserToGlobalCloud(const sensor_msgs::LaserScanConstPtr &msg, sensor_msgs::PointCloud2& cloud);
    void getLaserOriginOnGlobal(const sensor_msgs::LaserScanConstPtr& msg, Position& origin);
    void simplifyLaserScan(const sensor_msgs::LaserScanConstPtr& msg, sensor_msgs::LaserScan& convertedLaserScan);
};
}

#endif // LASER_MAP_UPDATER_H
