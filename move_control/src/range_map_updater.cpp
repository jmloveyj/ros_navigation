#include "move_control/range_map_updater.h"

namespace move_control{



void RangeMapUpdater::updateMap(double &minX, double &minY, double &maxX, double &maxY)
{
    std::vector<RangeSample> bufferCopy;
    boost::unique_lock<boost::mutex> lock(rangeSampleBufferMutex_);
    bufferCopy = rangeSampleBuffer_;
    rangeSampleBuffer_.clear();
    lock.unlock();

    std::vector<RangeSample>::iterator it;
    for (it=bufferCopy.begin();it != bufferCopy.end();++it) {
        lineOnMap((*it));
        touch(minX,minY,maxX,maxY,(*it).start(0),(*it).start(1));
        touch(minX,minY,maxX,maxY,(*it).end(0),(*it).end(1));
    }
}

void RangeMapUpdater::addMonitorTopic(const std::string &topicName)
{
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Range> >
            sub(new message_filters::Subscriber<sensor_msgs::Range>(nh_, topicName, 50));

    boost::shared_ptr<tf::MessageFilter<sensor_msgs::Range> >
            filter(new tf::MessageFilter<sensor_msgs::Range>(*sub, tf_, "/odom", 50));

    filter->registerCallback(boost::bind(&RangeMapUpdater::bufferIncomingMsg, this, _1));

    range_subs_.push_back(sub);
    range_notifiers_.push_back(filter);

}

void RangeMapUpdater::bufferIncomingMsg(const sensor_msgs::RangeConstPtr &msg)
{
    geometry_msgs::PointStamped in, out;
    in.header.stamp = msg->header.stamp;
    in.header.frame_id = msg->header.frame_id;
    std::string mapFrame = map_.getFrameId();

    try
    {
        tf_.transformPoint(mapFrame, in, out);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }

    RangeSample rangeSample;
    if(msg->range < msg->max_range) {
        rangeSample.ifClearEnd = false;
    } else {
        rangeSample.ifClearEnd = true;
    }
    rangeSample.start = grid_map::Position(out.point.x,out.point.y);

    in.point.x = msg->range;
    try
    {
        tf_.transformPoint(mapFrame, in, out);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    rangeSample.end = grid_map::Position(out.point.x,out.point.y);

    boost::unique_lock<boost::mutex> lock(rangeSampleBufferMutex_);
    rangeSampleBuffer_.push_back(rangeSample);

}

}
