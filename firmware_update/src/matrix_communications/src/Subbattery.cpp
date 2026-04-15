#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/BatteryState.h"
#include <stdio.h>
#include <sstream>
// #include <compare>

int percentage; 
// std_msgs::String msg;
// std::stringstream ss;
// string serial_no;
// string ss = "ddfsdfdsd";
 
// namespace filesystem = std::filesystem;
// std::string ss ;
// std::string greeting="jjde" ;
void percentageCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  percentage = 100 *msg->percentage;
//   ss = msg->serial_numberstr()s;
    ROS_INFO("percentage: [%d]",percentage);
    // ROS_INFO("serial_number: [%s]",msg->serial_number.c_str());
    // printf("%s",ss);
}
int main(int argc, char **argv)

{
    ros::init(argc, argv, "percentage");
    ros::NodeHandle n;
    ros::Subscriber percentage_sub = n.subscribe("/battery_state", 10, percentageCallback);
    ros::spin();
    return 0;
}