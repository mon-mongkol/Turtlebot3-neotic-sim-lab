#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include "sensor_msgs/BatteryState.h"
#include <sstream>
#include <bits/stdc++.h>
#include <matrix_msgs/SerialCommu.h>
#include <matrix_msgs/Serial.h>

#include <std_msgs/Int8MultiArray.h>

using namespace std;

char Byte[] = {"#R001BL099B01L01P001M$"};
serial::Serial ser;

std::string info_, str_;

bool isSendRepeats = false;
std::string SendRepeats_str = "";

double last_time_send = 0;


void write_cxbee(const std_msgs::String::ConstPtr &msg)
{
    ser.write(msg->data);
    // str_ = msg->data;
}


bool cbService(matrix_msgs::SerialCommu::Request &req,
               matrix_msgs::SerialCommu::Response &res)
{
    ROS_INFO("[Serial_Xbee]: service call cmd: %s, arg0: %s, arg1: %s", req.cmd.c_str(), req.arg0.c_str(), req.arg1.c_str());
    if (req.cmd == matrix_msgs::SerialCommuRequest::send_repeats)
    {
        if (req.arg0 == matrix_msgs::SerialCommuRequest::enable)
        {
            isSendRepeats = true;
            SendRepeats_str = req.arg1;
            res.resutl = "Send Repeats:Enable, str: " + SendRepeats_str;
            last_time_send = ros::Time::now().toSec();
        }
        else
        {
            isSendRepeats = false;
            SendRepeats_str = "";
        }
    }
    else
    {
        // disable send repeats
        isSendRepeats = false;
        SendRepeats_str = "";

        res.resutl = "command not found";
        ROS_ERROR("[Serial_Xbee]: command not found");
    }
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_shl100_driver");
    ros::NodeHandle nh;

    // ros::Subscriber write_sub = nh.subscribe("xbee_robot_info", 10, write_callback);
    ros::Subscriber write_sub_xbee = nh.subscribe("/matrix_shl100/write", 10, write_cxbee);
    // ros::Subscriber send_xbee_sub = nh.subscribe("matrix_serial_xbee/xbee_send", 10, send_xbee_callback);
    // ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 10);
    ros::Publisher raw_read_pub = nh.advertise<std_msgs::String>("/matrix_shl100/receive", 10);
    ros::Publisher io_state = nh.advertise<std_msgs::Int8MultiArray>("/matrix_shl100/io_state", 10);
    ros::Publisher hb_pub = nh.advertise<std_msgs::Int8>("/matrix_shl100/heart_beat", 10);
    ros::Publisher state_pub = nh.advertise<std_msgs::String>("/matrix_shl100/state", 10);
    ros::Publisher lift_pose_pub = nh.advertise<std_msgs::String>("/matrix_shl100/pose", 10);
    ros::ServiceServer service = nh.advertiseService("serial_communication_srv", cbService);

    std::string port;
    // port = "/dev/ttyACM0";
    ros::param::param<std::string>("~port", port, "/dev/matrix_shl100");

start:

    try
    {
        ser.setPort(port);
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(3000);
        ser.setTimeout(to);
        ser.open();

        if (ser.isOpen())
        {
            ROS_INFO_STREAM("Serial Port Open success!!!");

            std::string str_obj, data_completed_;
            char *char_arr;
            bool H_ = false;
            bool E_ = false;
            bool C_ = false;

            // initial B-Bypass Mode for Digi XBee S2C XB24C27SITB003
            // ser.write("B");
            ROS_INFO_STREAM("Serial Port Open initialized success!!!");

            // initial lift
            ros::Duration(1).sleep();
            ser.write("#TxSHL100_DOWN$");
            ros::Duration(3).sleep();

            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                
                if (ser.waitReadable())
                {

                    std::string receive_xbee;

                    receive_xbee = ser.read(ser.available());

                    // ROS_INFO("%s", receive_xbee.c_str());

                    str_obj = receive_xbee;
                    char_arr = &str_obj[0];

                    for (int i = 0; i < str_obj.length(); i++)
                    {
                        // cout << char_arr[i];
                        // cout << "\n";
                        // if (H_ && char_arr[i] == 0x02)
                        if (H_ && char_arr[i] == '#')
                        {
                            data_completed_.clear();
                            // data_completed_.append(1, char_arr[i]);
                            H_ = false;
                        }

                        if (H_ && char_arr[i] != '$')
                        {
                            data_completed_.append(1, char_arr[i]);
                        }

                        if (char_arr[i] == '#')
                        {
                            H_ = true;
                            data_completed_.append(1, char_arr[i]);
                        }

                        if (H_ && char_arr[i] == '$')
                        {
                            E_ = true;
                            data_completed_.append(1, char_arr[i]);
                            // cout << "false\n";
                        }

                        if (H_ && E_)
                        {
                            C_ = true;
                            H_ = false;
                            E_ = false;
                        }

                        if (C_)
                        {
                            // if ((data_completed_.find('#') != -1) && (data_completed_.find('$') != -1) && (data_completed_.length() <= 80))
                            // {
                                
                                // [<char> - '0'] C++ Program For char to int Conversion https://www.geeksforgeeks.org/cpp-program-for-char-to-int-conversion/
                                // printf("%d", int(data_completed_[18] - '0'));
                                if(data_completed_.find('IOSTATE') != -1)
                                {
                                    std_msgs::Int8MultiArray i_msg;
                                    try
                                    {
                                        for(int i=18; i<29; i++)
                                        {
                                            i_msg.data.push_back((data_completed_[i] - '0'));
                                        }
                                        io_state.publish(i_msg);
                                    }
                                    catch(const std::exception& e)
                                    {
                                        std::cerr << e.what() << '\n';
                                    }
                                    
                                    


                                    std_msgs::String pose;
                                    if((i_msg.data[5] == 0) && (i_msg.data[6] == 1))
                                    {
                                        pose.data = "High";
                                    }
                                    else if ((i_msg.data[5] == 1) && (i_msg.data[6] == 0))
                                    {
                                        pose.data = "Low";
                                    }
                                    else
                                    {
                                        pose.data = "Between High-Low";
                                    }

                                    lift_pose_pub.publish(pose);
                                    
                                }
                                else
                                {
                                    // ROS_INFO("send control goal, feedback, results data %s", receive_xbee.c_str());
                                    std_msgs::String raw_read;
                                    raw_read.data = data_completed_;
                                    if((raw_read.data.find("IDL") == -1) && (raw_read.data.find("BUSY_AcCONTROL") == -1) && (raw_read.data.find("CURReNT") == -1))
                                    {
                                        raw_read_pub.publish(raw_read);
                                    // ROS_INFO("%s", data_completed_.c_str());
                                    }
                                    
                                    
                                    try
                                    {
                                        if(data_completed_.find('#RxSHL100_[HB]$') != -1)
                                        {
                                            std_msgs::Int8 msg;
                                            msg.data = 1;
                                            hb_pub.publish(msg);
                                        }
                                    }
                                    catch(...)
                                    {
                                        ;
                                    }

                                    

                                }
                                
                                
                            // }

                            // cout << "completed data is -->";
                            //         cout << data_completed_;
                            //         cout << "\n";

                            
                            data_completed_.clear();
                            C_ = false;
                        }
                    }

                    // ROS_INFO("Data coming %s", receive_xbee.c_str());
                    // if((receive_xbee.find(0x02) != -1) && (receive_xbee.find(0x03) != -1))
                    // {

                    //     receive_xbee.erase(0, receive_xbee.find(0x02));
                    //     receive_xbee.erase(receive_xbee.find(0x03)+1);

                    //     if((receive_xbee.find(0x02) != -1) && (receive_xbee.find(0x03) != -1) && (receive_xbee.length() <= 59))
                    //     {
                    //         ROS_INFO("data issssssssssssssssssss %s", receive_xbee.c_str());
                    //         std_msgs::String raw_read;
                    //         raw_read.data = receive_xbee;
                    //         raw_read_pub.publish(raw_read);
                    //     }
                    //     // ROS_INFO("----------");
                    // }
                    // // ser.flush();
                }

                if (isSendRepeats)
                {
                    if ((ros::Time::now().toSec() - last_time_send) > 1)
                    {
                        // ser.write(SendRepeats_str);
                        ROS_INFO("send repeats data: %s", SendRepeats_str.c_str());
                        last_time_send = ros::Time::now().toSec();
                    }
                    else
                        ;
                }
                else
                    ;

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        else
        {
            return -1;
        }
    }
    catch (serial::IOException &e)
    {
        printf("Port not found %s\n", port.c_str());
        ROS_ERROR_STREAM("Unable to open port ");
        // return -1;
        goto start;
    }
}
