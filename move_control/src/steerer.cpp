#include "move_control/steerer.h"
#include "geometry_msgs/Twist.h"
#include "move_control/Histogram.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>

#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)
#define LOW_OBSTACLE_THRESHOLD 2000000.0
#define HIGH_OBSTACLE_THRESHOLD 4000000.0
namespace move_control {


Steerer::~Steerer()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x=0.0;
    cmd_vel.angular.z=0.0;
    velPublisher_.publish(cmd_vel);
    delete vfhP_;
}

void Steerer::acceptPlan(std::vector<Position>& plan)
{
    boost::unique_lock<boost::mutex> lock(planMutex_);
    plan_ = plan;
    ifPlanReady_ = true;
    planIndex_ = 1; //first element to steer
}

Steerer::Steerer(ros::NodeHandle& nh, MapProvider& mapProvider):nh_(nh),mapProvider_(mapProvider),
    ifPlanReady_(false),
    odomMon_("/odom")
{
    ROS_INFO("Starting VFH");
    initVfh();
    velPublisher_= nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",5);
    histPublisher_ =  nh_.advertise<move_control::Histogram>("hist", 5);
    vfhThreadP_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Steerer::vfhLoop, this)));
}

void Steerer::initVfh()
{
    double cell_size;			// 100 mm
    int window_diameter;		// cells
    int sector_angle;			// in deg
    double safety_dist_0ms;
    double safety_dist_1ms;
    int max_speed;
    int max_speed_narrow_opening;
    int max_speed_wide_opening;
    int max_acceleration;
    int min_turnrate;
    int max_turnrate_0ms;
    int max_turnrate_1ms;
    double min_turn_radius_safety_factor;
    double free_space_cutoff_0ms;
    double obs_cutoff_0ms;
    double free_space_cutoff_1ms;
    double obs_cutoff_1ms;
    double weight_desired_dir;
    double weight_current_dir;
    double robot_radius;

    cell_size = 100;							// mm, cell dimension
    window_diameter = 30;						// number of cells
    sector_angle = 5;							// deg, sector angle

    if (!nh_.getParam ("safety_dist_0ms", safety_dist_0ms))
        safety_dist_0ms = 10; 				// mm, double, safe distance at 0 m/s

    if (!nh_.getParam ("safety_dist_1ms", safety_dist_1ms))
        safety_dist_1ms = 50; 				// mm, double, safe distance at 1 m/s

    if (!nh_.getParam ("max_speed", max_speed))
        max_speed= 200;						// mm/sec, int, max speed

    if (!nh_.getParam ("max_speed_narrow_opening", max_speed_narrow_opening))
        max_speed_narrow_opening= 200; 		// mm/sec, int, max speed in the narrow opening

    if (!nh_.getParam ("max_speed_wide_opening", max_speed_wide_opening))
        max_speed_wide_opening= 300; 			// mm/sec, int, max speed in the wide opening

    if (!nh_.getParam ("max_acceleration", max_acceleration))
        max_acceleration = 200;    			// mm/sec^2, int, max acceleration

    if (!nh_.getParam ("min_turnrate", min_turnrate))
        min_turnrate = 40;	 				// deg/sec, int, min turn rate <--- not used

    if (!nh_.getParam ("max_turnrate_0ms", max_turnrate_0ms))
        max_turnrate_0ms = 40;				// deg/sec, int, max turn rate at 0 m/s

    if (!nh_.getParam ("max_turnrate_1ms", max_turnrate_1ms))
        max_turnrate_1ms = 40;				// deg/sec, int, max turn rate at 1 m/s

    min_turn_radius_safety_factor = 1.0; 		// double ????

    if (!nh_.getParam ("free_space_cutoff_0ms", free_space_cutoff_0ms))
        free_space_cutoff_0ms = LOW_OBSTACLE_THRESHOLD; 	//double, low threshold free space at 0 m/s

    if (!nh_.getParam ("obs_cutoff_0ms", obs_cutoff_0ms))
        obs_cutoff_0ms = HIGH_OBSTACLE_THRESHOLD;			//double, high threshold obstacle at 0 m/s

    if (!nh_.getParam ("free_space_cutoff_1ms", free_space_cutoff_1ms))
        free_space_cutoff_1ms = LOW_OBSTACLE_THRESHOLD; 	//double, low threshold free space at 1 m/s

    if (!nh_.getParam ("obs_cutoff_1ms", obs_cutoff_1ms))
        obs_cutoff_1ms = HIGH_OBSTACLE_THRESHOLD;			//double, high threshold obstacle at 1 m/s

    if (!nh_.getParam ("weight_desired_dir", weight_desired_dir))
        weight_desired_dir = 10.0;				//double, weight desired direction

    if (!nh_.getParam ("weight_current_dir", weight_current_dir))
        weight_current_dir = 1.0;				//double, weight current direction

    if (!nh_.getParam ("robot_radius", robot_radius))
        robot_radius = 178.0;					// robot radius in mm

    vfhP_ = new VFH(cell_size, window_diameter, sector_angle,
                    safety_dist_0ms, safety_dist_1ms, max_speed,
                    max_speed_narrow_opening, max_speed_wide_opening,
                    max_acceleration, min_turnrate, max_turnrate_0ms,
                    max_turnrate_1ms, min_turn_radius_safety_factor,
                    free_space_cutoff_0ms, obs_cutoff_0ms, free_space_cutoff_1ms,
                    obs_cutoff_1ms, weight_desired_dir, weight_current_dir);

    vfhP_->SetRobotRadius(robot_radius);
    vfhP_->Init();
}

void Steerer::vfhLoop()
{
    ros::Rate r(5);
    while (ros::ok())
    {
        if(ifPlanReady_)
            update();
        r.sleep();
    }
}


void Steerer::getRangesFromSubmap()
{
    for (unsigned i = 0; i < 361; i++)
        ranges_[i][0] = 5000.0;

    GridMap map;
    Position robotPos;
    double robotOrient;


    mapProvider_.getRobotPos(robotPos, robotOrient);
    mapProvider_.getSubMap(map, robotPos, Length(1.5,1.5));

    const auto& masterMap = map["master"];
    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      const float value = masterMap(index[0],index[1]);
      if(std::isnan(value))
          continue;

      if(value <= 3)
          continue;

      Position pos;
      map.getPosition(index, pos);

      double angle = atan2(pos[1]-robotPos[1],pos[0]-robotPos[0]);

      double angleDegree = angles::to_degrees(angles::normalize_angle_positive(angle-robotOrient + 3.14/2));
      if(angleDegree>180)
          continue;

      int floor = std::floor(angleDegree);
      int ceil = std::ceil(angleDegree);

      double distance = (robotPos-pos).matrix().norm()*1000.0;

      if(ranges_[floor*2][0]>distance)
          ranges_[floor*2][0] = distance;

      if(ranges_[ceil*2][0]>distance)
          ranges_[ceil*2][0] = distance;

    }
}

void Steerer::pubVel(int vel, int turnrate)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x=(float)(vel)/1000.0;
    cmd_vel.angular.z= DEG2RAD(turnrate);
    velPublisher_.publish(cmd_vel);
}

void Steerer::pubHist()
{
    move_control::Histogram histMsg;
    int binNum = vfhP_->getHistSize()/2;
    int sectorAngle = vfhP_->getSectorAngle();
    histMsg.num_bin = binNum;
    histMsg.yLowThreshold = (uint)(LOW_OBSTACLE_THRESHOLD/1000.0);
    histMsg.yHighThreshold = (uint)(HIGH_OBSTACLE_THRESHOLD/1000.0);

    float * histP = vfhP_->Hist;
    float * originHistP = vfhP_->OriginHist;

    for (int i=0; i<binNum;i++){
        histMsg.xData.push_back(i*sectorAngle);
        histMsg.yBinData.push_back((int)(histP[i]));
        histMsg.yData.push_back((int)(originHistP[i]));
    }

    histPublisher_.publish(histMsg);
}
void Steerer::update()
{
    Position currentPos;
    double currentDir;
    mapProvider_.getRobotPos(currentPos, currentDir);
    float desiredAngle,desiredDist,deltaX,deltaY;
    float currGoalDistanceTolerance=250;
    Position targetPos;

    boost::unique_lock<boost::mutex> lock(planMutex_);
    size_t size = plan_.size();
    while(true) {
        targetPos = plan_[planIndex_];
        deltaX= (targetPos[0] - currentPos[0])*1000.0;
        deltaY= (targetPos[1] - currentPos[1])*1000.0;
        desiredDist = hypot(deltaX,deltaY);

        if(desiredDist<currGoalDistanceTolerance){
            planIndex_++;
            if(planIndex_>=size)
            {
                ifPlanReady_ = false;
                return;
            }

        } else
            break;

    }
    lock.unlock();

    nav_msgs::Odometry currentOdom;
    odomMon_.getMsg(currentOdom);
    double robotVel = currentOdom.twist.twist.linear.x * 1000.0;

    desiredAngle = RAD2DEG(angles::normalize_angle_positive(atan2(deltaY,deltaX) - currentDir+M_PI/2));

    int chosenSpeed,chosenTurnrate;

    getRangesFromSubmap();
    vfhP_->Update_VFH(ranges_, (int) (robotVel), desiredAngle ,
                      desiredDist, currGoalDistanceTolerance, chosenSpeed,
                      chosenTurnrate);

    pubVel(chosenSpeed,chosenTurnrate);

    pubHist();


}

}

