#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Char.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include "sensor_msgs/BatteryState.h"
#include <bits/stdc++.h>
#include <iomanip> 
#include <stdio.h>
#include <iostream>  
#include <sstream>  
#include <string>  
#include <cstring> 
#include <stdlib.h>

#include <matrix_msgs/RobotMode.h>
#include <matrix_msgs/MovmentStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
using namespace std;
using namespace matrix_msgs;

class Xbee_server {

    private:
    int counter =0 ;
    float xp,yp, thetap ;
    int data;
    int host = 1408;
    int matrix_percentage_ = 0, matrix_building_, matrix_level_,matrix_poi_, matrix_mode_;
    ros::Subscriber sub_odom, movement_subscriber, battery_subscriber,matrix_building_subscriber ,matrix_level_subscriber, matrix_poi_subscriber, matrix_mode_subscriber ;
    ros::Publisher command_pub;

    std::stringstream matrix_percentage, matrix_building, matrix_level,matrix_poi, matrix_status;
    std::string matrix_number_,_data,_matrix_number, _matrix_percentage, _matrix_building, _matrix_level,_matrix_poi, _matrix_status;
    std::string fornt_split_serial = "ROHM020020220";
    std::string rear_split_serial = "A";
    std::string open_tag = "#";
    std::string open_data = "{INFO,";
    std::string close_data = "}";
    std::string number_tag = "0x";
    std::string battery_tag = "BL";
    std::string building_tag = "B";
    std::string level_tag = "L";
    std::string poi_tag = "P";
    std::string robot_mode_tag = "M";
    std::string close_tag = "$";
    std::string zero = "0";

    MovmentStatus movement_;

    float x_ = 0.0;
    float y_ = 0.0;
    float theta_ = 0.0;
    geometry_msgs::Quaternion orientation_;
    

    public:
    Xbee_server(ros::NodeHandle *nh) {

        battery_subscriber = nh->subscribe("/battery_state", 10, 
            &Xbee_server::callback_battery, this);
        matrix_building_subscriber = nh->subscribe("/building", 10, 
            &Xbee_server::callback_building_state, this);
        matrix_level_subscriber = nh->subscribe("/level", 10, 
            &Xbee_server::callback_level_state, this);
        matrix_poi_subscriber = nh->subscribe("/poi", 10, 
            &Xbee_server::callback_POI_state, this);
        matrix_mode_subscriber = nh->subscribe("/matrix_mode_controller/mode", 10, 
            &Xbee_server::callback_MatrixMode, this);
        movement_subscriber = nh->subscribe("/matrix_system/movement_status", 10,
            &Xbee_server::callback_movement_status, this);
        sub_odom = nh->subscribe("/odom", 10, &Xbee_server::cbOdom, this);

        command_pub = nh->advertise<std_msgs::String>("xbee_robot_info", 10);

        // convert_hex();
        ros::Rate loop_rate(0.2);

        while(ros::ok())
        {
            // getPositionState();
            convert_hex();
            ros::spinOnce();
            loop_rate.sleep();
        }

    }

    void cbOdom(const nav_msgs::Odometry& msg)
    {
        x_ = msg.pose.pose.position.x;
        y_ = msg.pose.pose.position.y;
        orientation_ = msg.pose.pose.orientation;

    }

    void callback_movement_status(const matrix_msgs::MovmentStatus& msg)
    {
        movement_ = msg;
    }

    void callback_battery(const sensor_msgs::BatteryState& msg) {
        matrix_number_ = msg.serial_number;
        
        matrix_percentage_ = 100*msg.percentage;
        
    }
   
    void callback_building_state(const std_msgs::Int64& msg) {
        matrix_building_=msg.data;
    }

    void callback_level_state(const std_msgs::Int64& msg) {
        matrix_level_=msg.data;
    }

    void callback_POI_state(const std_msgs::Int64& msg){
        matrix_poi_ = msg.data;
        if (matrix_poi_<10){
            _matrix_poi = zero + zero + std::to_string(matrix_poi_);
        }
        else{
            if(matrix_poi_<100){
                _matrix_poi = zero + std::to_string(matrix_poi_);
            }
            else{
                _matrix_poi = std::to_string(matrix_poi_);
            }
            
        }

    }
    void callback_MatrixMode(const matrix_msgs::RobotMode& msg) {
    // void callback_MatrixMode(const std_msgs::Int64& msg) {
        matrix_mode_=msg.robot_mode;
       
    }

    float getEuler()
    {
        float current_theta;
        tf::Quaternion q(
                orientation_.x,
                orientation_.y,
                orientation_.z,
                orientation_.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_theta = yaw * (180.0/3.141592653589793238463);
        if(current_theta < 0) current_theta += 360.0;

        return current_theta;
    }

    // geometry_msgs::Pose getRobotPose()
    // {
    //     geometry_msgs::Pose pose_;
    //     tf::StampedTransform transform;
    //     tf::TransformListener listener;

    //     try
    //     {
    //         listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    //     }
    //     catch (tf::TransformException &ex)
    //     {
    //         ROS_ERROR("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //         // continue;
    //     }

    //     pose_.position.x = transform.getOrigin().x();
    //     pose_.position.y = transform.getOrigin().y();
    //     pose_.orientation.z = transform.getRotation().getZ();
    //     pose_.orientation.w = transform.getRotation().getW();
    //     return pose_;
    // }

    // float getEuler(geometry_msgs::Quaternion quat)
    // {
    //     float current_theta;
    //     tf::Quaternion q(
    //             quat.x,
    //             quat.y,
    //             quat.z,
    //             quat.w);
    //     tf::Matrix3x3 m(q);
    //     double roll, pitch, yaw;
    //     m.getRPY(roll, pitch, yaw);
    //     current_theta = yaw * (180.0/3.141592653589793238463);
    //     if(current_theta < 0) current_theta += 360.0;

    //     return current_theta;
    // }

    void getPositionState(char* outStr) {

        // geometry_msgs::Pose robot_pose_ = getRobotPose();
        // x_ = robot_pose_.position.x;
        // y_ = robot_pose_.position.y;
        // float theta_ = getEuler(robot_pose_.orientation);
        float theta_ = getEuler();

        
        char position_data_[31];
        char data_x[9]={};
        char data_y[9]={};
        char data_theta[9]={};
        Hex4B_String(data_x, x_);
        Hex4B_String(data_y, y_);
        Hex4B_String(data_theta, theta_);


        strcat(position_data_, "XP");
        strcat(position_data_, data_x);
        strcat(position_data_, "YP");
        strcat(position_data_, data_y);
        strcat(position_data_, "OR");
        strcat(position_data_, data_theta);
        // printf("%f\n", x_);
        // printf("output %s\n", position_data_);

        for(int i=0; i < 31; ++i){
        outStr[i] = position_data_[i];
        }
    }
    void Hex4B_String(char* outStr, float value){
        unsigned char bytes[4] = {};
        int n = int(value * 100);
        // printf("%d", n);
        bytes[0] = (n >> 24) & 0xFF;
        bytes[1] = (n >> 16) & 0xFF;
        bytes[2] = (n >> 8) & 0xFF;
        bytes[3] = n & 0xFF;
        
        char hex_string[20] = {};
        
        for(int i =0; i<sizeof(bytes)/sizeof(*bytes); i++)
        {
            char hex_con_now[]={};
            if(bytes[i] == 0){
                if(n < 0)
                {
                    strcat(hex_string, "FF");
                }
                else{
                    strcat(hex_string, "00");
                }
                
            }else{
                sprintf(hex_con_now, "%X", bytes[i]);
                if(bytes[i] < 15)
                {
                    strcat(hex_string, "0");
                }
                strcat(hex_string, hex_con_now);
            // strcat(hex_string, "/");
            }
        // printf(" 0x%02x, ", bytes[i]);
        }
        for(int i=0; i < 20; ++i){
        outStr[i] = hex_string[i];
        }
    }
    
    void Hex1B_String(char* outStr, int value){
        unsigned char bytes[2] = {};
        int n = value ;

        bytes[0] = n;
        char hex_string[4] = {};
        
        for(int i =0; i<sizeof(bytes)/sizeof(*bytes); i++)
        {   
            char hex_con_now[]={};
            if(bytes[i] == 0){
                strcat(hex_string, "00");
            }else{
                sprintf(hex_con_now, "%X", bytes[i]);
                if(bytes[i] < 15)
                {
                    strcat(hex_string, "0");
                }
                strcat(hex_string, hex_con_now);
            }

        }
        for(int i=0; i < 2; ++i){
        outStr[i] = hex_string[i];
        }
    }
    void convert_hex(){
        if(matrix_percentage_){
            char position_[31];
            getPositionState(position_);
            
            _matrix_number = matrix_number_.substr(13,3);
            std::stringstream matrix_numberH;
            int matrix_number_i = stoi(_matrix_number) + host;
            matrix_numberH << std::uppercase << std::hex << matrix_number_i ;
            _matrix_number = matrix_numberH.str();

            char _matrix_MODE [3]={};
            char _matrix_BUILDING [3]={};
            char _matrix_LEVEL [3]={};
            char _matrix_BATTERY_LEVEL [3]={};
            Hex1B_String(_matrix_MODE, matrix_mode_);
            Hex1B_String(_matrix_BUILDING, matrix_building_);
            Hex1B_String(_matrix_LEVEL, matrix_level_);
            Hex1B_String(_matrix_BATTERY_LEVEL , matrix_percentage_);

            _data = open_tag +_matrix_number + open_data +_matrix_MODE +_matrix_BUILDING  
                    + _matrix_LEVEL  + _matrix_BATTERY_LEVEL+ close_data + close_tag;
            int n = _data.length();
            // cout<<n<<endl;
            char char_array[n-1];
            strcpy(char_array, _data.c_str()); 

            std_msgs::String robot_status_msg;
            robot_status_msg.data = char_array;
            command_pub.publish(robot_status_msg);
            std::cout<<robot_status_msg.data<<std::endl;
            counter=0;

       }
       
        else{
            counter +=1;
            if (counter == 5){
                ROS_WARN("data has stop sending");
            }
            
        }
        // return robot_status_msg;
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Xbee_server");
    ros::NodeHandle nh;
    Xbee_server nc = Xbee_server(&nh);
    ros::spin();
}
