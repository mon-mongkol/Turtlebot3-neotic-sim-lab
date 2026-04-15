#include <ros/ros.h>
// #include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include "sensor_msgs/BatteryState.h"
#include <sstream>
#include <bits/stdc++.h>
#include <matrix_msgs/SerialCommu.h>

using namespace std;

char Byte[] = {"#R001BL099B01L01P001M$"};
// serial::Serial ser;

std::string info_, str_;

bool isSendRepeats = false;
std::string SendRepeats_str ="";

double last_time_send = 0;

ros::Subscriber serial_data_receive_sub,
                serial_data_send_bridg_sub;


ros::Publisher serial_data_send_pub,
                serial_data_receive_bridge_pub;




bool cbService(matrix_msgs::SerialCommu::Request &req, matrix_msgs::SerialCommu::Response &res)
{
    ROS_INFO("[Serial_Xbee]: service call cmd: %s, arg0: %s, arg1: %s", req.cmd.c_str(), req.arg0.c_str(), req.arg1.c_str());
    if(req.cmd == matrix_msgs::SerialCommuRequest::send_repeats)
    {
        if(req.arg0 == matrix_msgs::SerialCommuRequest::enable)
        {
            isSendRepeats = true;
            SendRepeats_str = req.arg1;
            res.resutl = "Send Repeats:Enable, str: " + SendRepeats_str;
            last_time_send = ros::Time::now().toSec();
        }
        else
        {
            isSendRepeats = false;
            SendRepeats_str="";
        }
    }
    else
    {
        //disable send repeats 
        isSendRepeats = false;
        SendRepeats_str="";

        res.resutl = "command not found";
        ROS_ERROR("[Serial_Xbee]: command not found");
    }
    return true;
}

void cbSerialReceive(const std_msgs::String msg){
    std_msgs::String data_msg;
    data_msg = msg;
    serial_data_receive_bridge_pub.publish(data_msg);
}

void cbSerialSendBridge(const std_msgs::String msg){
    std_msgs::String  data_msg;
    data_msg = msg;
    serial_data_send_pub.publish(data_msg);
}

void SendRepeatsSerial(void)
{
    // send repeats
    if(isSendRepeats)
    {
        if((ros::Time::now().toSec() - last_time_send) > 1) 
        {
            std_msgs::String data_msg;
            data_msg.data = SendRepeats_str;
            serial_data_send_pub.publish(data_msg);

            ROS_INFO("send repeats data: %s", SendRepeats_str.c_str());
            last_time_send = ros::Time::now().toSec();
        }else;
        
    }else;
}


int main (int argc, char** argv){
    ros::init(argc, argv, "matrix_serial_bridge_rs485");
    ros::NodeHandle nh;

    std::string serial_data_receive_topic;
    std::string serial_data_send_topic;
    std::string serial_data_receive_bridge_topic;
    std::string serial_data_send_bridge_topic;
    std::string serial_communication_srv;

    ros::param::param<std::string>("~serial_data_receive_topic", serial_data_receive_topic, "/matrix_io/rs485_receive");
    ros::param::param<std::string>("~serial_data_send_topic", serial_data_send_topic, "/matrix_io/rs485_send");
    ros::param::param<std::string>("~serial_data_receive_bridge_topic", serial_data_receive_bridge_topic, "/matrix_serial_bridge/rs485_receive");
    ros::param::param<std::string>("~serial_data_send_bridge_topic", serial_data_send_bridge_topic, "/matrix_serial_bridge/rs485_send");
    ros::param::param<std::string>("~serial_communication_srv", serial_communication_srv, "/matrix_serial_bridge/serial_communication_srv");

    

    serial_data_receive_sub = nh.subscribe(serial_data_receive_topic, 10, cbSerialReceive);
    serial_data_send_pub = nh.advertise<std_msgs::String>(serial_data_send_topic, 1);
    serial_data_receive_bridge_pub = nh.advertise<std_msgs::String>(serial_data_receive_bridge_topic, 1);
    serial_data_send_bridg_sub = nh.subscribe(serial_data_send_bridge_topic, 10, cbSerialSendBridge);


    ros::ServiceServer service = nh.advertiseService(serial_communication_srv, cbService);
    

    ros::Rate loop_rate(10);
    while(ros::ok())
    {

        
        SendRepeatsSerial();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}









