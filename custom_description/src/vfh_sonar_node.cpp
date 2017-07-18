#include <custom_description/vfh_algorithm.h>
#include <custom_description/Histogram.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <move_control/continuous_sensor_helper_ros.h>
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
class VfhSonarNode
{
public:
    VfhSonarNode(ros::NodeHandle nh);
    ~VfhSonarNode() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x=0.0;
        cmd_vel.angular.z=0.0;
        vel_publisher_.publish(cmd_vel);
        delete vfhP_;
    }

private:
    VFH_Algorithm *vfhP_;
    geometry_msgs::Point targetPos_;
    bool ifTargetReady_;
    boost::shared_ptr<boost::thread> vfhThreadP_;

    move_control::ContinuousSensorHelperRos<sensor_msgs::Range> leftRangeMon_,rightRangeMon_,frontLeftRangeMon_,frontRightRangeMon_,frontRangeMon_;
    move_control::ContinuousSensorHelperRos<nav_msgs::Odometry> odomMon_;

    // ros
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Publisher  vel_publisher_;
    ros::Publisher hist_publisher_;
    double ranges_[361][2];

    void initVfh();
    void vfhLoop();
    void fillRanges();
    void pubVel(int vel, int turnrate);
    void pubHist();
    void update();
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
};

VfhSonarNode::VfhSonarNode(ros::NodeHandle nh):nh_(nh),
    ifTargetReady_(false),
    leftRangeMon_("/left_range"),
    rightRangeMon_("/right_range"),
    frontLeftRangeMon_("/front_left_range"),
    frontRightRangeMon_("/front_right_range"),
    frontRangeMon_("/front_range"),
    odomMon_("/odom")
{
    ROS_INFO("Starting VFH");
    initVfh();
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&VfhSonarNode::goalCB, this, _1));
    vel_publisher_= nh_.advertise<geometry_msgs::Twist>("cmd_vel",5);
    hist_publisher_ =  nh_.advertise<custom_description::Histogram>("hist", 5);
}

void VfhSonarNode::initVfh()
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
        safety_dist_0ms = 50; 				// mm, double, safe distance at 0 m/s

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

    vfhP_ = new VFH_Algorithm(cell_size, window_diameter, sector_angle,
            safety_dist_0ms, safety_dist_1ms, max_speed,
            max_speed_narrow_opening, max_speed_wide_opening,
            max_acceleration, min_turnrate, max_turnrate_0ms,
            max_turnrate_1ms, min_turn_radius_safety_factor,
            free_space_cutoff_0ms, obs_cutoff_0ms, free_space_cutoff_1ms,
            obs_cutoff_1ms, weight_desired_dir, weight_current_dir);

    vfhP_->SetRobotRadius(robot_radius);
    vfhP_->Init();
}

void VfhSonarNode::vfhLoop()
{
    ros::Rate r(5);
    while (ros::ok())
    {
      update();
      r.sleep();
    }
}

void VfhSonarNode::fillRanges()
{
    for (unsigned i = 0; i < 361; i++)
        ranges_[i][0] = 5000.0;

    sensor_msgs::Range range;
    rightRangeMon_.getMsg(range);
    if (range.max_range > range.range){
       for(int i=30;i<90;i++)
            ranges_[i][0] = range.range*1000.0+178;
    }

    frontRightRangeMon_.getMsg(range);
    if (range.max_range > range.range){
         for(int i=90;i<150;i++)
            ranges_[i][0] = range.range*1000.0+178;
    }

    frontRangeMon_.getMsg(range);
    if (range.max_range > range.range) {
        for(int i=150;i<210;i++)
            ranges_[i][0] = range.range*1000.0+178;
    }

    frontLeftRangeMon_.getMsg(range);
    if (range.max_range > range.range) {
         for(int i=210;i<270;i++)
            ranges_[i][0] = range.range*1000.0+178;
    }

    leftRangeMon_.getMsg(range);
    if (range.max_range > range.range) {
         for(int i=270;i<330;i++)
            ranges_[i][0] = range.range*1000.0+178;
    }
}

void VfhSonarNode::pubVel(int vel, int turnrate)
{
     geometry_msgs::Twist cmd_vel;
     cmd_vel.linear.x=(float)(vel)/1000.0;
     cmd_vel.angular.z= DEG2RAD(turnrate);
     vel_publisher_.publish(cmd_vel);
}

void VfhSonarNode::pubHist()
{
    custom_description::Histogram histMsg;
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

    hist_publisher_.publish(histMsg);
}
void VfhSonarNode::update()
{
   fillRanges();

   nav_msgs::Odometry current;
   odomMon_.getMsg(current);

   double robotVel,currentDir;
   float desiredAngle,desiredDist,deltaX,deltaY;
   float currGoalDistanceTolerance=250;
   deltaX= targetPos_.x - current.pose.pose.position.x*1000.0;
   deltaY= targetPos_.y - current.pose.pose.position.y*1000.0;
   desiredDist = hypot(deltaX,deltaY);
   if(desiredDist<currGoalDistanceTolerance)
       return;

   robotVel = current.twist.twist.linear.x * 1000.0;
   currentDir = tf::getYaw(current.pose.pose.orientation);
   desiredAngle = RAD2DEG(angles::normalize_angle_positive(atan2(deltaY,deltaX) - currentDir+M_PI/2));

   int chosenSpeed,chosenTurnrate;
   vfhP_->Update_VFH(ranges_, (int) (robotVel), desiredAngle ,
           desiredDist, currGoalDistanceTolerance, chosenSpeed,
           chosenTurnrate);

   pubVel(chosenSpeed,chosenTurnrate);

   pubHist();


}

void VfhSonarNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    if(!ifTargetReady_) {
        ifTargetReady_ = true;
        vfhThreadP_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&VfhSonarNode::vfhLoop, this)));
    }

    targetPos_.x = goal->pose.position.x* 1000.0;
    targetPos_.y = goal->pose.position.y* 1000.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "VFH");
    ros::NodeHandle nh;
    VfhSonarNode vfh_node(nh);
    ros::spin();

    return 0;
}
