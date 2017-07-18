#include <custom_description/range_sensor_helper_ros.h>
#include <algorithm>


namespace moveaction
{
  const std::string RangeSensorHelperRos::base_frame_const_ = "base_link";
  const std::string RangeSensorHelperRos::src_frame_const_[4]  =
  {"front_sonar_link", "front_left_sonar_link","front_right_sonar_link","right_sonar_link"};

  RangeSensorHelperRos::RangeSensorHelperRos(tf::TransformListener &tf, double close_threshold):
    close_threshold_(close_threshold),tf_(tf)
  {
    ros::NodeHandle nh;
    range_subs_.push_back(nh.subscribe("/front_range",10,&RangeSensorHelperRos::rangeCallback,this));
    range_subs_.push_back(nh.subscribe("/front_right_range",10,&RangeSensorHelperRos::rangeCallback,this));
    range_subs_.push_back(nh.subscribe("/front_left_range",10,&RangeSensorHelperRos::rangeCallback,this));
    range_subs_.push_back(nh.subscribe("/right_range",10,&RangeSensorHelperRos::rangeCallback,this));

    for (int i=0; i<4; i++)
      {
        range_values_[i] = INVALID_RANGE;
      }

  }

  double RangeSensorHelperRos::getRange(RangeSensorPos pos)
  {
    if (pos >= ALL)
      {
        ROS_FATAL_STREAM("Unexpected range position fetch request:"<<pos);
        return INVALID_RANGE;
      }
    return range_values_[pos];
  }

  bool RangeSensorHelperRos::ifClose(RangeSensorHelperRos::RangeSensorPos pos)
  {
    if (pos> ALL)
      {
        ROS_FATAL_STREAM("Unexpected range position fetch request:"<<pos);
        return false;
      }
    if (pos == ALL){
        return (range_values_[FRONT]<close_threshold_) ||
            (range_values_[FRONT_LEFT]<close_threshold_) ||
            (range_values_[FRONT_RIGHT]<close_threshold_) ||
            (range_values_[RIGHT]<close_threshold_);
      }
    return (range_values_[pos]< close_threshold_);
  }


  void RangeSensorHelperRos::getAngleGroup(AngleGroup &angleGroup)
  {
    geometry_msgs::Point32 front, frontLeft,frontRight,right;
    ErrorCode front_status, frontLeft_status,frontRight_status,right_status;
    front_status= getPointOnBaseFrame(FRONT, front);
    frontLeft_status=getPointOnBaseFrame(FRONT_LEFT, frontLeft);
    frontRight_status=getPointOnBaseFrame(FRONT_RIGHT, frontRight);
    right_status=getPointOnBaseFrame(RIGHT, right);

    if((front_status == OK)&&(frontLeft_status==OK))
      angleGroup.front_frontLeft = getAngleOnBaseFrame(front, frontLeft);
    else
      angleGroup.front_frontLeft = INVALID_ANGLE;

    if((front_status == OK)&&(frontRight_status==OK))
      angleGroup.frontRight_front = getAngleOnBaseFrame(frontRight, front);
    else
      angleGroup.frontRight_front = INVALID_ANGLE;

    if((frontRight_status == OK)&&(right_status==OK))
      angleGroup.right_frontRight = getAngleOnBaseFrame(right, frontRight);
    else
      angleGroup.right_frontRight = INVALID_ANGLE;

  }



  void RangeSensorHelperRos::rangeCallback(const sensor_msgs::Range::ConstPtr &msg_p)
  {
    boost::mutex::scoped_lock lock(range_sensor_mutex_);
    std::string frame = msg_p->header.frame_id;
    float range =  msg_p->range;
    if (range == msg_p->max_range)
      range = INVALID_RANGE;
    if(frame == src_frame_const_[FRONT])
      range_values_[FRONT] = range;
    if(frame == src_frame_const_[FRONT_LEFT])
      range_values_[FRONT_LEFT] = range;
    if(frame == src_frame_const_[FRONT_RIGHT])
      range_values_[FRONT_RIGHT] = range;
    if(frame == src_frame_const_[RIGHT])
      range_values_[RIGHT] = range;
  }

  RangeSensorHelperRos::ErrorCode RangeSensorHelperRos::getPointOnBaseFrame(RangeSensorPos pos, geometry_msgs::Point32 &point)
  {
    if (range_values_[pos] == INVALID_RANGE)
      return INVALID;
    geometry_msgs::PointStamped in, out;

    in.header.stamp = ros::Time::now();
    in.header.frame_id = src_frame_const_[pos];
    in.point.x = range_values_[pos];

    if(!tf_.waitForTransform(base_frame_const_, in.header.frame_id,
                             in.header.stamp, ros::Duration(0.1)) ) {
        ROS_ERROR_THROTTLE(1.0, "Range sensor helper can't transform from %s to %s at %f",
                           base_frame_const_.c_str(), in.header.frame_id.c_str(),
                           in.header.stamp.toSec());
        return INVALID;
      }

    tf_.transformPoint (base_frame_const_, in, out);

    point.x = out.point.x;
    point.y = out.point.y;
    return OK;

  }

  float RangeSensorHelperRos::getAngleOnBaseFrame(const geometry_msgs::Point32 &point1, const geometry_msgs::Point32 &point2)
  {
    float deltaX, deltaY,angle;
    deltaX =  point2.x - point1.x;
    deltaY = point2.y - point1.y;

    angle = std::atan2(deltaY, deltaX);
    return angle;
  }

} //name space moveaction
