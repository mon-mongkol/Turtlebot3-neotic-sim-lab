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

#include <matrix_msgs/RobotMode.h>
#include <matrix_msgs/MovmentStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

using namespace matrix_msgs;

class Xbee_server {

    private:
    int counter = 22;
    int data;
    int matrix_percentage_ = 0, matrix_building_, matrix_level_,matrix_poi_, matrix_mode_;
    ros::Subscriber sub_odom, movement_subscriber, battery_subscriber,matrix_building_subscriber ,matrix_level_subscriber, matrix_poi_subscriber, matrix_mode_subscriber ;
    ros::Publisher command_pub;
    std::string matrix_number_,splittedStrings_data;
    std::string fornt_split_serial = "ROHM020020220";
    std::string rear_split_serial = "A";
    std::string open_tag = "#";
    std::string number_tag = "R";
    std::string battery_tag = "BL";
    std::string building_tag = "B";
    std::string level_tag = "L";
    std::string poi_tag = "P";
    std::string close_tag = "$";
    std::string zero = "0";
    // data_hex[];

    

    // for byte convert
    std::string _data,_matrix_number, _matrix_percentage, _matrix_building, _matrix_level,_matrix_poi, _matrix_status;
    std::stringstream matrix_percentage, matrix_building, matrix_level,matrix_poi, matrix_status;
    //  static const char* data[];

    std::vector<std::string> splittedString;

    MovmentStatus movement_;

    float x_;
    float y_;
    float theta_;
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

        command_pub = nh->advertise<std_msgs::String>("remote_to_xbee", 10);

        // convert_hex();
        ros::Rate loop_rate(5);

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
        
        if (matrix_percentage_<100){
            _matrix_percentage = zero + std::to_string(matrix_percentage_);
        }
        else{
            _matrix_percentage = std::to_string(matrix_percentage_);
        }
    }
   
    void callback_building_state(const std_msgs::Int64& msg) {
        matrix_building_=msg.data;
        
        if (matrix_building_<10){
            _matrix_building =  zero + std::to_string(matrix_building_);
        }
        else{
            _matrix_building =  std::to_string(matrix_building_);
        }
    }

    void callback_level_state(const std_msgs::Int64& msg) {
        matrix_level_=msg.data;
        if (matrix_level_<10){
            _matrix_level = zero + std::to_string(matrix_level_);
        }
        else{
        _matrix_level = std::to_string(matrix_level_);
        }
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
        matrix_mode_= msg.robot_mode;
        if(matrix_mode_ == matrix_msgs::RobotMode::EMERGENCY)
        {
            _matrix_status = "S";
        }
        else if(matrix_mode_ == matrix_msgs::RobotMode::DOCKING_MODE_ON)
        {
            _matrix_status = "C";
        }
        else if(matrix_mode_ == matrix_msgs::RobotMode::ERROR_DEVICE)
        {
            _matrix_status = "E";
        }
        else
        {
            if(movement_.robot_moving)
            {
                _matrix_status = "M";
            }
            else
            {
                _matrix_status = "I";
            }
        }
        // std::cout<<matrix_mode_<<std::endl;
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

    void getPositionState(char* outStr)
    {
        // float x_ = 1000.5678910;
        // float y_ = 1500.5678910;
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

    std::vector<std::string> split(std::string stringToBeSplitted, std::string delimeter){
        std::vector<std::string> splittedString;
        int startIndex = 0;
        int  endIndex = 0;
        while( (endIndex = stringToBeSplitted.find(delimeter, startIndex)) < stringToBeSplitted.size() )
        {
            std::string val = stringToBeSplitted.substr(startIndex, endIndex - startIndex);
            splittedString.push_back(val);
            startIndex = endIndex + delimeter.size();
        }
        if(startIndex < stringToBeSplitted.size())
        {
            std::string val = stringToBeSplitted.substr(startIndex);
            splittedString.push_back(val);
        }
        return splittedString;
    }
    void hex(unsigned char a, char* buf) {
        // hex lookup table
        char data[] = "0123456789ABCDEF";
        buf[0] = '0';
        buf[1] = 'x';
        int i = 2;
        while (a) {
            buf[i ++] = data[a % 16];
            a /= 16;
        }    
        int j = 2;
        -- i;
        // reverse i..j
        while (j < i) {
            char t = buf[j];
            buf[j] = buf[i];
            buf[i] = t;
            i --;
            j ++;
        }
    }
    void string2hexString(char* input, char* output){
        int loop;
        int i; 
        
        i=0;
        loop=0;
        
        while(input[loop] != '\0')
        {
            sprintf((char*)(output+i),"%02X", input[loop]);
            loop+=1;
            i+=2;
        }
        //insert NULL at the end of the output string
        output[i++] = '\0';
    }

    void convert_hex(){
        
        if(matrix_percentage_){
           
        // BYTE  3-5 serial number robot
            std::vector<std::string> splittedStrings = split(matrix_number_, fornt_split_serial);
                for(int i = 0; i < splittedStrings.size() ; i++)
                    splittedStrings_data = splittedStrings[1];
                std::vector<std::string> splittedStrings_2 = split(splittedStrings_data, rear_split_serial);
                for(int i = 0; i < splittedStrings_2.size() ; i++)
                    _matrix_number = splittedStrings_2[0];
                    // std::cout<<_matrix_number<<std::endl; 

            char position_[31];
            getPositionState(position_);
            _data = open_tag + number_tag + _matrix_number + battery_tag + _matrix_percentage+ building_tag + _matrix_building + level_tag + _matrix_level + poi_tag + _matrix_poi + _matrix_status + position_ + close_tag ;
            int n = _data.length();
            char char_array[100];
            strcpy(char_array, _data.c_str()); 
            // strcpy(char_array, position_data_);       
            // for (int i = 0; i < n; i++)

            //     char_array[i];

            // std::cout<<char_array<<std::endl;

            std_msgs::String robot_status_msg;
            robot_status_msg.data = char_array;
    
            //char ss[] = char_array;
            // command_pub.publish(robot_status_msg);


            int len = strlen(char_array);
            char hex_str[(len*2)+1];
            
            //converting ascii string to hex string
            string2hexString(char_array, hex_str);
            
            //std::cout<<hex_str[1]<<std::endl;

       }
        else{
            // std::cout<<counter<<std::endl;
            ;
        }
    }

    void Hex4B_String(char* outStr, float value)
    {
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
                strcat(hex_string, "00");
            }else{
                sprintf(hex_con_now, "%X", bytes[i]);
                if(bytes[i] < 10)
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

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Xbee_server");
    ros::NodeHandle nh;
    Xbee_server nc = Xbee_server(&nh);
    ros::spin();
}
