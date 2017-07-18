#ifndef RANGESENSORHELPER_H
#define RANGESENSORHELPER_H


#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>


namespace moveaction {

  class RangeSensorHelperRos {
  public:
    typedef enum {
      FRONT = 0,
      FRONT_LEFT,
      FRONT_RIGHT,
      RIGHT,
      ALL
    } RangeSensorPos;

    typedef struct {
      float front_frontLeft;
      float frontRight_front;
      float right_frontRight;

    } AngleGroup;

    enum {
      INVALID_RANGE = 100,
      INVALID_ANGLE =1000
    };

    typedef enum {
      OK = 0,
      INVALID = 1
    } ErrorCode;



    /** @brief Constructor.
   * @param odom_topic The topic on which to subscribe to Odometry
   *        messages.  If the empty string is given (the default), no
   *        subscription is done. */
    RangeSensorHelperRos(tf::TransformListener &tf, double close_threshold=0.1);
    ~RangeSensorHelperRos() {}

    double getRange(RangeSensorPos pos);
    bool ifClose(RangeSensorPos pos);

    void getAngleGroup(AngleGroup &angleGroup);



  private:
    //range subscriber
    std::vector<ros::Subscriber> range_subs_;

    ros::Publisher range_close_pub_;

    //mutex to solve range container read/write competion
    boost::mutex range_sensor_mutex_;
    float range_values_[4];

    float close_threshold_;

    tf::TransformListener& tf_;

    static const std::string base_frame_const_;

    static const std::string src_frame_const_[4];

    void rangeCallback(const sensor_msgs::Range::ConstPtr & msg_p);

    ErrorCode getPointOnBaseFrame(RangeSensorPos pos, geometry_msgs::Point32 &point);

    float getAngleOnBaseFrame(const geometry_msgs::Point32 &point1, const geometry_msgs::Point32 &point2);


  };


} /* namespace moveaction */

#endif // RANGESENSORHELPER_H
