#ifndef CONTINUOUS_SENSOR_HELPER_ROS_H
#define SENSOR_HELPER_ROS_H
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

namespace move_control {

template<typename SensorMsg>
class ContinuousSensorHelperRos {
public:

    /** @brief Constructor.
   * @param  The topic on which to subscribe to
   *        messages.  If the empty string is given (the default), no
   *        subscription is done. */
    ContinuousSensorHelperRos(const std::string  &sensor_topic = ""){
        setTopic(sensor_topic);
    }

    ~ContinuousSensorHelperRos() {}

    void getMsg(SensorMsg &msg) {
        boost::mutex::scoped_lock lock(sensor_mutex_);
        msg = sensor_msg_;
    }

    void setTopic(const std::string &newTopic) {
        if(sensor_topic_==newTopic)
            return;
        sensor_topic_ = newTopic;

        if(sensor_topic_=="")
        {
            sensor_sub_.shutdown();
            return;
        }

        ros::NodeHandle globalNh;
        sensor_sub_=globalNh.subscribe( sensor_topic_, 10, &ContinuousSensorHelperRos<SensorMsg>::msgCallback,this );
    }


private:
    //range topic
    std::string sensor_topic_;

    // we listen on rangeetry on the range topic
    ros::Subscriber sensor_sub_;
    SensorMsg sensor_msg_;
    boost::mutex sensor_mutex_;


    /**
   * @brief  Callback for receiving rangeetry data
   * @param msg An rangeetry message
   */

    void msgCallback(const typename SensorMsg::ConstPtr &msg) {
        ROS_INFO_STREAM_ONCE("sensor "<<sensor_topic_<<" received");
        boost::mutex::scoped_lock lock(sensor_mutex_);
        sensor_msg_ = *msg;
    }
};

} /* namespace move_control */
#endif // CONTINUOUS_SENSOR_HELPER_ROS_H
