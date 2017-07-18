#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "boost/thread.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "move_control/map_provider.h"

#include "move_control/steerer.h"

namespace move_control {
#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)

class NavVfh {
public:
    NavVfh(ros::NodeHandle& nh);
    ~NavVfh(){
    }

private:
    void initParameters();
    void registerGoalMonitor();

    void steer();

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &goal);

    ros::NodeHandle& nh_;
    MapProvider mapProvider_;
    Steerer steerer_;

    ros::Subscriber goalSub_;
    ros::Publisher velPublisher_;

    grid_map::Position target_;
    std::vector<grid_map::Position> pathPlan_;
    std::string goalTopic_;

};

NavVfh::NavVfh(ros::NodeHandle& nh):nh_(nh),mapProvider_(nh,Length(4,4),true),
    steerer_(nh, mapProvider_)
{
    initParameters();

    registerGoalMonitor();

    velPublisher_= nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",5);

}

//TODO the parameters can't be obtained from ros parameter externally
void NavVfh::initParameters()
{
    goalTopic_ = "move_base_simple/goal";

}

void NavVfh::registerGoalMonitor()
{
    goalSub_ =  nh_.subscribe(goalTopic_, 1, &NavVfh::goalCb ,this);
}

void NavVfh::goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_[0] =  msg->pose.position.x;
    target_[1] =  msg->pose.position.y;
    pathPlan_.clear();
    Position currentPos;
    mapProvider_.getRobotPos(currentPos);
    pathPlan_.push_back(currentPos);
    pathPlan_.push_back(target_);
    steer();
}

void NavVfh::steer()
{
    // find next plan point as steer target
    steerer_.acceptPlan(pathPlan_);

}

}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "NavVfh_node");
    ros::NodeHandle nh;

    move_control::NavVfh NavVfhTester(nh);

    ros::spin();
    return 0;
}
