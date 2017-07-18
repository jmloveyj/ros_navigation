#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "boost/thread.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "move_control/map_provider.h"
#include "move_control/rrt_planner.h"
#include "nav_msgs/Path.h"
#include "move_control/Histogram.h"
#include "move_control/steerer.h"

namespace move_control {
#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)

class Nav {
public:
    Nav(ros::NodeHandle& nh);
    ~Nav(){
    }

private:

    void initParameters();
    void registerGoalMonitor();

    void startPlanThread();

    void loopPlan();

    bool makePlan();
    void publishPlan();
    bool ifGoalAchieved();
    void steer();
    void taileredPlan(vector<grid_map::Position>& detailedPlan);
    void wakePlanner(const ros::TimerEvent& event);
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &goal);

    ros::NodeHandle& nh_;
    MapProvider mapProvider_;
    Steerer steerer_;
    bool ifNaving_;

    ros::Subscriber goalSub_;
    ros::Publisher velPublisher_;
    ros::Publisher planPublisher_;
    ros::Publisher histPublisher_;
    grid_map::Position target_;
    std::vector<grid_map::Position> pathPlan_;

    boost::mutex plannerMutex_;
    boost::condition_variable plannerCond_;

    std::string goalTopic_;
    double planInterval_;  //s
    double mapPlanLength_;
    uint tailerPlanStride_;
    double closeTolerance_;
    double robotVel_;




};

Nav::Nav(ros::NodeHandle& nh):nh_(nh),mapProvider_(nh),
    ifNaving_(false),robotVel_(0.0),steerer_(nh, mapProvider_)
{
    initParameters();

    registerGoalMonitor();

    velPublisher_= nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",5);
    planPublisher_ = nh_.advertise<nav_msgs::Path>("plan",1);
    histPublisher_ =  nh_.advertise<move_control::Histogram>("hist", 5);
    startPlanThread();
}

//TODO the parameters can't be obtained from ros parameter externally
void Nav::initParameters()
{
    goalTopic_ = "move_base_simple/goal";
    planInterval_ = 20; //s
    mapPlanLength_ = 10.0;
    tailerPlanStride_ = 5;
    closeTolerance_ = 0.2;

}

void Nav::registerGoalMonitor()
{
    goalSub_ =  nh_.subscribe(goalTopic_, 1, &Nav::goalCb ,this);
}

void Nav::goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    boost::unique_lock<boost::mutex> lock(plannerMutex_);
    target_[0] =  msg->pose.position.x;
    target_[1] =  msg->pose.position.y;
    ifNaving_ = true;
    plannerCond_.notify_one();
}

void Nav::startPlanThread()
{
    boost::thread loopThread(boost::bind(&Nav::loopPlan,this));
}

void Nav::loopPlan()
{
    ros::Timer timer;
    boost::unique_lock<boost::mutex> lock(plannerMutex_);
    while (ros::ok()) {
        plannerCond_.wait(lock);
        timer.stop();

        if(ifGoalAchieved()) {
            continue;
        }

        if(makePlan()) {
            steer();
        } else{
            ROS_ERROR("can't get plan here");
        }
        timer =nh_.createTimer(ros::Duration(planInterval_), &Nav::wakePlanner, this);
    }
}

void Nav::wakePlanner(const ros::TimerEvent& event)
{
    // we have slept long enough for rate
    plannerCond_.notify_one();
}


bool Nav::makePlan()
{
    grid_map::GridMap mapForPlan;
    Position start;
    mapProvider_.getRobotPos(start);
    mapProvider_.getSubMap(mapForPlan,start,Length(mapPlanLength_,mapPlanLength_));
    mapProvider_.publishMap(mapForPlan);

    std::vector<grid_map::Position> detailedPlan;


    RrtPlanner planner(mapForPlan,start,target_,closeTolerance_);
    if(!planner.makePlan(detailedPlan))
        return false;

    taileredPlan(detailedPlan);

    return true;
}

void Nav::publishPlan()
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



bool Nav::ifGoalAchieved()
{
    Position currentPos;
    mapProvider_.getRobotPos(currentPos);
    double deltaX= (target_[0] - currentPos[0]);
    double deltaY= (target_[1] - currentPos[1]);
    double distance = hypot(deltaX,deltaY);
    return (distance<closeTolerance_);

}

void Nav::steer()
{
    // find next plan point as steer target
    steerer_.acceptPlan(pathPlan_);

}

void Nav::taileredPlan(vector<grid_map::Position> &detailedPlan)
{
    pathPlan_.clear();
    size_t size = detailedPlan.size();
    for(int i=size-1; i >= 0; i--){
        if((i%tailerPlanStride_ == 0)||i==size-1)
            pathPlan_.push_back(detailedPlan[i]);

    }

    publishPlan();

}



}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "mapTester");
    ros::NodeHandle nh;

    move_control::Nav navTester(nh);

    ros::spin();
    return 0;
}
