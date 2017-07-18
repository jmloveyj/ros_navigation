#ifndef STEERER_H
#define STEERER_H

#include "ros/ros.h"
#include "move_control/vfh.h"
#include "boost/thread.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "move_control/continuous_sensor_helper_ros.h"
#include "nav_msgs/Odometry.h"
#include "move_control/map_provider.h"

using namespace grid_map;


namespace move_control {


class Steerer {
public:
    Steerer(ros::NodeHandle& nh, MapProvider& mapProvider);
    ~Steerer();
    void acceptPlan(std::vector<Position>& plan);

private:
    VFH *vfhP_;
    std::vector<Position> plan_;
    int planIndex_;
    bool ifPlanReady_;
    boost::mutex planMutex_;
    boost::shared_ptr<boost::thread> vfhThreadP_;

    move_control::ContinuousSensorHelperRos<nav_msgs::Odometry> odomMon_;

    // ros
    ros::NodeHandle& nh_;
    MapProvider& mapProvider_;
    ros::Publisher  velPublisher_;
    ros::Publisher histPublisher_;
    double ranges_[361][2];

    void initVfh();
    void vfhLoop();
    void getRangesFromSubmap();
    void pubVel(int vel, int turnrate);
    void pubHist();
    void update();



};

}



#endif // STEERER_H
