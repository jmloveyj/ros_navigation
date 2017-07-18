#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <custom_description/range_sensor_helper_ros.h>
#include <tf/transform_listener.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>


using namespace moveaction;

void spin()
{
  ros::spin();
}

class FollowEdgeController
{
public:
  FollowEdgeController(tf::TransformListener &tf);
  ~FollowEdgeController(){}
  void init();
  void decideVel(geometry_msgs::Twist &vel_cmd);
private:
  boost::function<void (geometry_msgs::Twist &)> processControl;
  RangeSensorHelperRos rangeSensor;
  RangeSensorHelperRos::AngleGroup angleGroup;
  void rotateParallelFront(geometry_msgs::Twist &vel_cmd);
  void findFrontEdge(geometry_msgs::Twist &vel_cmd);
  void followEdge(geometry_msgs::Twist &vel_cmd);
  void cleanupVel(geometry_msgs::Twist &vel_cmd);
};

FollowEdgeController::FollowEdgeController(tf::TransformListener &tf):rangeSensor(tf)
{
}

void FollowEdgeController::init() {
  // find edge at the first
  processControl = boost::bind(&FollowEdgeController::findFrontEdge,this,_1);
  ROS_INFO("findFrontEdge phase started");
}

void FollowEdgeController::decideVel(geometry_msgs::Twist &vel_cmd) {

  processControl(vel_cmd);
}

void FollowEdgeController::cleanupVel(geometry_msgs::Twist &vel_cmd) {
  vel_cmd.linear.x = 0.0;
  vel_cmd.linear.y =0.0;
  vel_cmd.linear.z = 0.0;
  vel_cmd.angular.x =0.0;
  vel_cmd.angular.y = 0.0;
  vel_cmd.angular.z = 0.0;
}

void FollowEdgeController::findFrontEdge(geometry_msgs::Twist &vel_cmd) {
  cleanupVel(vel_cmd);
  if(rangeSensor.ifClose(RangeSensorHelperRos::FRONT)) {
      processControl = boost::bind(&FollowEdgeController::rotateParallelFront,this,_1);
      ROS_INFO("rotateParallelFront phase started");
    } else {
      vel_cmd.linear.x = 0.3;
    }

}

void FollowEdgeController::rotateParallelFront(geometry_msgs::Twist &vel_cmd)
{
  double angle = RangeSensorHelperRos::INVALID_ANGLE;
  double angleTolerance = 5*3.14/180;


  cleanupVel(vel_cmd);

  rangeSensor.getAngleGroup(angleGroup);
  if(angleGroup.front_frontLeft != RangeSensorHelperRos::INVALID_ANGLE)
    angle = angleGroup.front_frontLeft;
  else if (angleGroup.frontRight_front != RangeSensorHelperRos::INVALID_ANGLE)
    angle = angleGroup.frontRight_front;
  else
    angle = angleGroup.right_frontRight;

  if(std::fabs(angle) >angleTolerance) {
      vel_cmd.angular.z = 0.15;
    } else
    {
      processControl = boost::bind(&FollowEdgeController::followEdge,this,_1);
      ROS_INFO("followEdge phase started");
    }

}

void FollowEdgeController::followEdge(geometry_msgs::Twist &vel_cmd)
{
  cleanupVel(vel_cmd);
  double distanceKeep = 0.06;

  double distanceRight =  rangeSensor.getRange(RangeSensorHelperRos::RIGHT);
  double distanceFrontRight =  rangeSensor.getRange(RangeSensorHelperRos::FRONT_RIGHT);
  double distanceFront =  rangeSensor.getRange(RangeSensorHelperRos::FRONT);

  vel_cmd.linear.x = 0.05;

  if(distanceFront != RangeSensorHelperRos::INVALID_RANGE) {
      ROS_INFO("stand still and rotate clock-wise because front is close to obstacle.");
      vel_cmd.linear.x = 0.0;
      vel_cmd.angular.z = 1.5;
      return;
    }

  if((distanceFrontRight == RangeSensorHelperRos::INVALID_RANGE)&&(distanceRight == RangeSensorHelperRos::INVALID_RANGE)) {
      ROS_INFO("anti-clock-wise because votex");
      vel_cmd.linear.x = 0.05;
      vel_cmd.angular.z = -0.3;
      return;
    }

  if(distanceRight == RangeSensorHelperRos::INVALID_RANGE) {
      ROS_INFO("anti-clock-wise because votex");
      vel_cmd.linear.x = 0.0;
      vel_cmd.angular.z = 1.5;
    }


  if(distanceFrontRight < distanceRight) {
      ROS_INFO("head to obstackle and turn left to parallel");
      vel_cmd.linear.x = 0.0;
      vel_cmd.angular.z = 1.5;
      return;
    }


//  if(distanceRight > distanceKeep) {
//      ROS_INFO("turn right because too far from edge");
//      vel_cmd.linear.x = 0.0;
//      vel_cmd.angular.z = -0.1;
//      return;
//    }

//  if(distanceRight < distanceKeep) {
//      ROS_INFO("turn left because too near from edge");
//      vel_cmd.linear.x = 0.0;
//      vel_cmd.angular.z = 0.3;
//      return;
//    }




}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_edge_node");

  ros::NodeHandle nh("~");

  tf::TransformListener tf;

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("vel",10);

  FollowEdgeController controller(tf);
  controller.init();

  boost::thread spinThread(&spin);

  geometry_msgs::Twist vel_cmd;




  // find edge
  while(ros::ok())
    {
      ros::Duration(0.25).sleep();
      controller.decideVel(vel_cmd);
      vel_pub.publish(vel_cmd);
    }

  ros::shutdown();
  spinThread.join();
  return(0);
}