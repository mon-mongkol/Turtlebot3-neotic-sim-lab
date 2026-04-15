#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include "sensor_msgs/BatteryState.h"

class NumberCounter {

    private:
    ros::Subscriber number_subscriber;


    public:
    NumberCounter(ros::NodeHandle *nh) {
        counter = 0;

        number_subscriber = nh->subscribe("/battery_state", 10, 
            &NumberCounter::callback_batt, this);
    }

    void callback_batt(const sensor_msgs::BatteryState& msg) {
        // counter += msg.data;
        percentage = 100 *msg->percentage;
        //   ss = msg->serial_numberstr()s;
        ROS_INFO("percentage: [%d]",percentage);
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    NumberCounter nc = NumberCounter(&nh);
    ros::spin();
}