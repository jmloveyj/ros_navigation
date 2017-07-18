#include "move_control/laser_map_updater.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/point_cloud2_iterator.h"

namespace move_control{

void LaserMapUpdater::updateMap(double &minX, double &minY, double &maxX, double &maxY)
{
    std::vector<RangeSample> bufferCopy;
    boost::unique_lock<boost::mutex> lock(laserSampleBufferMutex_);
    bufferCopy = laserSampleBuffer_;
    laserSampleBuffer_.clear();
    lock.unlock();

    std::vector<RangeSample>::iterator it;
    for (it=bufferCopy.begin();it != bufferCopy.end();++it) {
        lineOnMap((*it));
        touch(minX,minY,maxX,maxY,(*it).start(0),(*it).start(1));
        touch(minX,minY,maxX,maxY,(*it).end(0),(*it).end(1));
    }
}

void LaserMapUpdater::addMonitorTopic(const std::string &topicName)
{
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> >
            sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, topicName, 50));

    boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan> >
            filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, tf_, "/odom", 50));

    filter->registerCallback(boost::bind(&LaserMapUpdater::bufferIncomingMsg, this, _1));

    laser_subs_.push_back(sub);
    laser_notifiers_.push_back(filter);

}

void LaserMapUpdater::bufferIncomingMsg(const sensor_msgs::LaserScanConstPtr &msg)
{
    //filter out too frequent msg
    ros::Time now = ros::Time::now();
    if(lastIncomingMsgTime_+msgProcessCycle_> now)
        return;
    lastIncomingMsgTime_ = now;

    RangeSample rangeSample;
    getLaserOriginOnGlobal(msg, rangeSample.start);


    sensor_msgs::PointCloud2 cloud;
    transformLaserToGlobalCloud(msg, cloud);

    sensor_msgs::PointCloud2Iterator<float> itX(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> itY(cloud, "y");
    sensor_msgs::PointCloud2Iterator<int> itIndex(cloud, "index");

    boost::unique_lock<boost::mutex> lock(laserSampleBufferMutex_);
    for(;itX != itX.end();++itX,++itY,++itIndex) {
        // x,y
        rangeSample.end = Position(*itX,*itY);

        // index
        int index = *itIndex;
        float range = msg->ranges[index];
        if(std::isinf(range)||(range==msg->range_max)) {
            rangeSample.ifClearEnd = true;
        } else
            rangeSample.ifClearEnd = false;
        laserSampleBuffer_.push_back(rangeSample);
    }


}


void LaserMapUpdater::transformLaserToGlobalCloud(const sensor_msgs::LaserScanConstPtr &msg, sensor_msgs::PointCloud2 &cloud)
{
    std::string globalFrameId = map_.getFrameId();
    boost::shared_ptr<sensor_msgs::LaserScan> scanPtr =boost::const_pointer_cast<sensor_msgs::LaserScan>(msg);
    if(msg->angle_increment<0.017) {
        scanPtr = boost::shared_ptr<sensor_msgs::LaserScan>(new sensor_msgs::LaserScan);
        simplifyLaserScan(msg,*scanPtr);

    }


    laser_geometry::LaserProjection projector;
    try
    {
        projector.transformLaserScanToPointCloud(globalFrameId, *scanPtr, cloud, tf_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", globalFrameId.c_str(),
                 ex.what());
        return;
    }
}

void LaserMapUpdater::getLaserOriginOnGlobal(const sensor_msgs::LaserScanConstPtr &msg, Position &origin)
{
    geometry_msgs::PointStamped in,out;
    in.header.stamp = msg->header.stamp;
    in.header.frame_id = msg->header.frame_id;
    std::string globalFrameId = map_.getFrameId();
    try
    {
        tf_.transformPoint(globalFrameId, in, out);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        return;
    }

    origin = Position(out.point.x,out.point.y);
}

void LaserMapUpdater::simplifyLaserScan(const sensor_msgs::LaserScanConstPtr &msg, sensor_msgs::LaserScan &convertedLaserScan)
{
    convertedLaserScan.header = msg->header;
    convertedLaserScan.range_max = msg->range_max;
    convertedLaserScan.range_min=msg->range_min;
    convertedLaserScan.angle_max=msg->angle_max;
    convertedLaserScan.angle_min=msg->angle_min;
    convertedLaserScan.time_increment = msg->time_increment;
    convertedLaserScan.scan_time = msg->scan_time;



    float increment = 0.0;
    convertedLaserScan.ranges.push_back(msg->ranges[0]);
    for(int i=0; i< msg->ranges.size();i++){
        increment += msg->angle_increment;
        if(increment>=0.017){
            convertedLaserScan.angle_increment = increment;
            increment = 0.0;
            convertedLaserScan.ranges.push_back(msg->ranges[i]);
        }
    }



}


}
