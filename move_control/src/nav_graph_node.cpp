#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "boost/thread.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "move_control/map_provider.h"
#include "move_control/astar_planner.h"
#include "move_control/steerer.h"
#include "nav_msgs/Path.h"

namespace move_control {
#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)

class NavGraph {
public:
    NavGraph(ros::NodeHandle& nh);
    ~NavGraph(){
    }

private:
    void initParameters();
    void registerGoalMonitor();

    void steer();

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &goal);
    void publishPlan();

    ros::NodeHandle& nh_;
    MapProvider mapProvider_;
    Steerer steerer_;
    AStarPlanner planner_;

    ros::Subscriber goalSub_;
    ros::Publisher velPublisher_;
    ros::Publisher planPublisher_;

    grid_map::Position target_;
    std::vector<grid_map::Position> pathPlan_;
    std::string goalTopic_;

};

NavGraph::NavGraph(ros::NodeHandle& nh):nh_(nh),
    planner_(nh),
    mapProvider_(nh,Length(4,4),true),
    steerer_(nh, mapProvider_)
{
    initParameters();

    registerGoalMonitor();

    velPublisher_= nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",5);
    planPublisher_ = nh_.advertise<nav_msgs::Path>("plan",1);
}

//TODO the parameters can't be obtained from ros parameter externally
void NavGraph::initParameters()
{
    goalTopic_ = "move_base_simple/goal";

}

void NavGraph::registerGoalMonitor()
{
    goalSub_ =  nh_.subscribe(goalTopic_, 1, &NavGraph::goalCb ,this);
}

void NavGraph::goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_[0] =  msg->pose.position.x;
    target_[1] =  msg->pose.position.y;
    pathPlan_.clear();
    Position currentPos;
    mapProvider_.getRobotPos(currentPos);
    planner_.makePlan(currentPos,target_,pathPlan_);
    publishPlan();

    steer();
}

void NavGraph::publishPlan()
{
    nav_msgs::Path gui_path;
    size_t size = pathPlan_.size();
    gui_path.poses.resize(size);
    //TODO should get from a sigle place
    gui_path.header.frame_id = "odom";
    gui_path.header.stamp = ros::Time::now();

    for(unsigned int i=0; i < size; i++){
        gui_path.poses[i].pose.position.x = pathPlan_[i][0];
        gui_path.poses[i].pose.position.y = pathPlan_[i][1];
    }
    planPublisher_.publish(gui_path);
}

void NavGraph::steer()
{
    // find next plan point as steer target
    steerer_.acceptPlan(pathPlan_);

}

}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "NavGraph_node");
    ros::NodeHandle nh;

    move_control::NavGraph NavGraphTester(nh);

    ros::spin();
    return 0;
}
