#include "move_control/map_updater.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/Range.h"
#include <boost/thread/mutex.hpp>

#ifndef RANGE_MAP_UPDATER_H
#define RANGE_MAP_UPDATER_H

namespace move_control {
class RangeMapUpdater:public MapUpdater {
public:
    RangeMapUpdater(ros::NodeHandle& nh, tf::TransformListener& tf, grid_map::GridMap& map,const std::string& sensorType="range"):
        MapUpdater(nh,tf,map,sensorType){}

    ~RangeMapUpdater(){}
    // update map and point out the map range updated
    void updateMap(double &minX,double &minY,double &maxX,double &maxY);

    std::string getTypeName() {
        return "range";
    }

    void addMonitorTopic(const std::string &topicName);
private:
    std::vector<boost::shared_ptr<tf::MessageFilter<sensor_msgs::Range>
    > > range_notifiers_;
    std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Range>
    > > range_subs_;

    std::vector<RangeSample> rangeSampleBuffer_;
    boost::mutex rangeSampleBufferMutex_;

    void bufferIncomingMsg(const sensor_msgs::RangeConstPtr& msg);

};
}

#endif // RANGE_MAP_UPDATER_H
