#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <web_interface_msgs/Layout.h>
#include <matrix_msgs/XbeeCommuROHM.h>
#include <bits/stdc++.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <matrix_msgs/XbeeCommunicationAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Char.h>
#include <std_srvs/SetBool.h>
#include "sensor_msgs/BatteryState.h"
#include <matrix_msgs/RobotMode.h>
#include <matrix_msgs/MovmentStatus.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <iomanip> 
#include <stdio.h>
#include <iostream>  
#include <sstream>  
#include <string>  
#include <cstring>

#include <serial/serial.h>

using namespace matrix_msgs;
typedef actionlib::SimpleActionClient<matrix_msgs::XbeeCommunicationAction> MatrixMovementClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class XbeeCmdServer
{
    protected:
        ros::NodeHandle nh_;

        MatrixMovementClient matrix_movement_ac;
        MoveBaseClient move_base_ac;
        
        bool Matrix_MovingCompleted;
     
        

        ros::Subscriber sub_robot_info;
        
        ros::Publisher command_pub,
                        read_pub;

        ros::ServiceClient layouts_srv_;

        enum class Task_Sequence
            {
                Init            =   0,
                   
                OpenSerialPort,
                

                CheckXbeeConnect,
                XbeeDisconnect,
                XbeeConnect,

                DataComing,
                DecodeDataComing,
                ActionCmdSelect,

                MatrixMovement_NavGoal,
                MatrixMovement_NavGoal_WaitFinish,

                REACHED_TARGET_POS,
                MOVE_TARGET_FAIL,

                CancleAllMoveGoals,
                SendRespond,

                PubRobotInfo,

                Fisnish,
            };
        Task_Sequence task_seq_ = Task_Sequence::Init;

        serial::Serial ser;
        
        matrix_msgs::XbeeCommuROHM data_info_, period_info_;
        std::string serial_no = "ROHM020020220001A";

        // configuring parameters
        std::string map_frame, base_frame; 
        std::string receive_xbee;

        
        geometry_msgs::Quaternion orientation_;
        geometry_msgs::Pose current_pose_, period_pose_;

        double response_timepout_;
        std::string action_response;
        std::string xbee_msg;
        std::string get_info_;

    
    public:
        XbeeCmdServer(std::string name) : matrix_movement_ac("matrix_xbeemovement_server", true),
                                          move_base_ac("move_base", true)
        {

            //wait for the action server to come up
            while(!move_base_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base_ac action server to come up");
            }
            
            //wait for the action server to come up
            while(!matrix_movement_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the matrix_movement_ac action server to come up");
            }

            init_pub_();
            init_sub_();
            init_srv_();
            init_param_();
            
        }

        void init_pub_()
        {
            ;
        }

        void init_sub_()
        {
            sub_robot_info = nh_.subscribe("/remote_to_xbee", 1, &XbeeCmdServer::cbRobotInfo, this);
        }

        void init_srv_()
        {
            layouts_srv_ = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
        }

        void init_param_()
        {
            ros::param::param<std::string>("~serial_no", serial_no, "ROHM020020220001A");
            ros::param::param<std::string>("~map_frame", map_frame, "/map");
            ros::param::param<std::string>("~base_frame", base_frame, "/base_link");
        }

        void cbRobotInfo(const std_msgs::String msg)
        {
            get_info_ = msg.data;
            // ROS_INFO("info => %s", get_info_.c_str());
        }

        void matrixMovementDoneCb(const actionlib::SimpleClientGoalState &state, const XbeeCommunicationResultConstPtr &result)
        {
            //ROS_INFO("[Matrix_XbeeManagerServer]:matrix_movement DONECB: Finished in state [%s]", state.toString().c_str());
            Matrix_MovingCompleted = true;
            //ROS_INFO("result is %d", result->result);
        }

        void spin()
        {
            ros::Rate r(5);

            while (ros::ok())
            {
                
                task_manager();


                ros::spinOnce();
                r.sleep();
            }
        }

        void task_manager()
        {   
             
            // xbee_receive_.data = "#0x601{ROUTEgoPOI15}$";

            ROS_INFO("%d\n", task_seq_);

            switch(task_seq_)
            {
                case Task_Sequence::Init:
                    //ROS_INFO("Init");
                    current_pose_ = getRobotPose();
                    period_pose_ = current_pose_;
                    task_seq_ = Task_Sequence::OpenSerialPort;
                    break;
                
                case Task_Sequence::OpenSerialPort:
                    try
                    {
                        ser.setPort("/dev/matrix_xbee");
                        //ROS_INFO("Connecting Xbee at /dev/matrix_xbee");
                        ser.setBaudrate(9600);
                        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                        ser.setTimeout(to);
                        ser.open();
                        task_seq_ = Task_Sequence::CheckXbeeConnect;
                        //ROS_INFO("Connected Xbee at /dev/matrix_xbee");
                    }
                    catch (serial::IOException& e)
                    {
                        ROS_ERROR_STREAM("Unable to open port ");
                        // return -1;
                        task_seq_ = Task_Sequence::OpenSerialPort;
                    }
                    break;
                
                case Task_Sequence::CheckXbeeConnect:
                    //ROS_INFO("CheckXbeeConnect");
                    if(ser.isOpen())
                    {
                        
                        task_seq_ = Task_Sequence::XbeeConnect;
                    }
                    else
                    {
                        task_seq_ = Task_Sequence::OpenSerialPort;
                        ROS_ERROR_STREAM("Unable to open port ");
                    }
                    ser.flush();
                    break;
                
                case Task_Sequence::XbeeConnect:
                    //ROS_INFO("XbeeConnect");
                    task_seq_ = Task_Sequence::DataComing;
                    ser.flush();
                    break;
                
                case Task_Sequence::DataComing:
                    //ROS_INFO("DataComing");
                    task_seq_ = Task_Sequence::MatrixMovement_NavGoal_WaitFinish;
                    if(ser.waitReadable())
                    {
                        receive_xbee = ser.readline(ser.available(), "$");
                        ROS_INFO("DataComing ==> found new MSG:%s", receive_xbee.c_str());
                        if(receive_xbee.find("#0x601") != -1)
                        {
                            receive_xbee.erase(0, receive_xbee.find("#0x601"));
                            if((receive_xbee.find("#") != -1) && (receive_xbee.find("$") != -1) && (receive_xbee.length() <= 59))
                            {
                                ROS_INFO("#:%d, $:%d, size: %d, data: %s",receive_xbee.find("#"), receive_xbee.find("$"), receive_xbee.length(), receive_xbee.c_str());
                                // receive_xbee = receive_xbee;
                                // std_msgs::String result;
                                // result.data = receive_xbee;
                                // read_pub.publish(result);
                                task_seq_ = Task_Sequence::DecodeDataComing;
                                // task_seq_ = Task_Sequence::CheckXbeeConnect;

                            }
                        }
                    }
                    else
                    {
                        // ROS_WARN("No data");
                        // task_seq_ = Task_Sequence::PubRobotInfo; 
                    }
                    ser.flush();
                    
                    break;
                
                case Task_Sequence::DecodeDataComing:
                    ROS_INFO("DecodeDataComing");
                    if(matrix_movement_ac.getState() != actionlib::SimpleClientGoalState::ACTIVE)
                    {
                        //ROS_INFO("DecodeDataComing ==> matrix_movement_ac isnt ACTIVE");
                        if(checkCmdFormatMovement(receive_xbee))
                        {
                            //ROS_INFO("DecodeDataComing ==> cmd corrected form");
                            data_info_ = getCmdInfo(receive_xbee);
                            task_seq_ = Task_Sequence::MatrixMovement_NavGoal;
                        }
                        else
                        {
                            task_seq_ = Task_Sequence::MatrixMovement_NavGoal_WaitFinish;
                        }
                        
                        
                    }
                    else
                    {
                        //ROS_INFO("DecodeDataComing ==> matrix_movement_ac ACTIVE");
                        // found cancel movement
                        if(receive_xbee.find("CancelMovement") != -1)
                        {
                            //ROS_INFO("DecodeDataComing ==> Detect CacelMovement!!");
                            task_seq_ = Task_Sequence::CancleAllMoveGoals;
                        }
                        else
                        {
                            //ROS_INFO("DecodeDataComing ==> response ACTIVE_s");
                            action_response = XbeeCommunicationResult::ACTIVE_s;
                            task_seq_ = Task_Sequence::MatrixMovement_NavGoal_WaitFinish;
                        }
                        
                    }
                    ser.flush();         
                    break;
                
                case Task_Sequence::CancleAllMoveGoals:
                    //ROS_INFO("DecodeDataComing ==> CacelMovement Success!!");
                    matrix_movement_ac.cancelAllGoals();
                    move_base_ac.cancelAllGoals();
                    action_response="cancelAllGoalsSuccess";

                    task_seq_ = Task_Sequence::SendRespond;
                    ser.flush();
                    break;
                
                case Task_Sequence::MatrixMovement_NavGoal:
                {
                    //ROS_INFO("MatrixMovement_NavGoal!!");
                    XbeeCommunicationGoal goal;
                    goal.recieve_string = receive_xbee;
                    matrix_movement_ac.sendGoal(goal, boost::bind(&XbeeCmdServer::matrixMovementDoneCb, this, _1, _2));
                    task_seq_ = Task_Sequence::MatrixMovement_NavGoal_WaitFinish;
                    break;
                }

                case Task_Sequence::MatrixMovement_NavGoal_WaitFinish:
                {
                    
                    if (Matrix_MovingCompleted)
                    {
                        ROS_INFO("MatrixMovement_FINISH");
                        if(matrix_movement_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            // ROS_INFO("[Matrix_XbeeMovementServer]:move_result_->result = %s", move_result_->result ? "true" : "false");
                            task_seq_ = Task_Sequence::REACHED_TARGET_POS;
                        }
                        else
                        {
                            task_seq_ = Task_Sequence::MOVE_TARGET_FAIL;
                        }

                        //ROS_INFO("MatrixMovement_NavGoal_WaitFinish ==> move Finish");
                        auto result_ = matrix_movement_ac.getResult();
                        action_response = result_->text + result_->info.poi_name;
                        
   
                        Matrix_MovingCompleted =false;
                        // ROS_INFO("MatrixMovement_NavGoal_WaitFinish ==> %s", action_response.c_str());
                    }
                    else
                    {
                        // ROS_INFO("MatrixMovement_NavGoal_WaitFinish ==> isnt Finish");
                        task_seq_ = Task_Sequence::PubRobotInfo;
                    }
                    ROS_INFO("MatrixMovement_NavGoal_WaitFinish action_response ==> %s", action_response.c_str());
                }
                ser.flush();
                break;

                case Task_Sequence::REACHED_TARGET_POS:
                    task_seq_ = Task_Sequence::SendRespond;
                    ser.flush();
                    break;
                
                case Task_Sequence::MOVE_TARGET_FAIL:
                    task_seq_ = Task_Sequence::SendRespond;
                    ser.flush();
                    break;
                
                case Task_Sequence::SendRespond:
                {
                    
                    std::string open_res_("#0x581{");
                    std::string close_res_("}$");
                    // std::string msg = open_res_ + result_->text + result_->info.poi_name + close_res_;
                    xbee_msg = open_res_ + action_response + close_res_;
                    ROS_INFO(" SendRespond action_response ==> %s", action_response.c_str());
                    task_seq_ = Task_Sequence::PubRobotInfo;
                    ser.flush();
                    break;
                }
                
                case Task_Sequence::PubRobotInfo:
                {
                    //ROS_INFO("PubRobotInfo");
                    if(xbee_msg.length() >= 10)
                    {
                        ser.write(xbee_msg);
                        ROS_INFO("SendRespond ==> %s", xbee_msg.c_str());
                        xbee_msg = "";
                        action_response="";
                        ROS_INFO(" PubRobotInfo action_response ==> %s", action_response.c_str());
                    }

                    static double prev_send_info_ = 0;
                    if((ros::Time::now().toSec() - prev_send_info_) >= 3)
                    {
                        //ROS_INFO("Time to send");
                        std::string robot_info_;
                        robot_info_ = get_info_;
                        if(checkFormRobotInfo(robot_info_))
                        {
                            //ROS_INFO("Send info => %s", robot_info_.c_str());
                            // ser.write(robot_info_);
                        }

                        prev_send_info_ = ros::Time::now().toSec();
                    }

                    task_seq_ = Task_Sequence::Fisnish;
                    ser.flush();
                    break;
                }
                case Task_Sequence::Fisnish:
                    //ROS_INFO("Fisnish");
                    ROS_INFO(" ");
                    task_seq_ = Task_Sequence::CheckXbeeConnect;
                    ser.flush();
                    break;


            }

            
        }

        std::string getRobotInfo()
        {
            std::string data;
            return data;
        }

        bool checkFormRobotInfo(std::string robot_info)
        {
            bool corrected = false;
            if(robot_info.find("#0x581{INFO,M") == 0)
            {
                corrected = true;
            }
            return corrected;
        }

        bool checkCmdFormatMovement(std::string xbee_msg)
        {
            bool corrected_cmd = false;
            if(xbee_msg.find("#0x") == 0)
            {
                if(xbee_msg.find("{") == 6)
                {
                    if((xbee_msg.find("ROUTE") == 7) || (xbee_msg.find("POI") == 7))
                    {
                        if((xbee_msg.find("go") == 12) || (xbee_msg.find("go") == 10))
                        {
                            if(xbee_msg.find("}$") != -1)
                            {
                                corrected_cmd = true;
                            }
                            
                        }
                    }
                }
                
            }
            return corrected_cmd;
        }       

        // for convert String 8 byte (Hex 4 byte 0x00, 0x00, 0x00, 0x00)
        float HexString_float(std::string value, float divider)
        {
            unsigned int i;
            std::istringstream iss(value);
            iss >> std::hex >> i;
            auto j = static_cast<int>(i);
            float b = j;
            b = b/divider;
            // std::cout << b << std::endl; // 1000
            return b;
        }


         // get string between 2 delimer 
        std::string split_string(std::string string_cmd, std::string begin_pos, std::string end_pos)
        {
            size_t pos = 0;
            std::string token;
            while ((pos = string_cmd.find(begin_pos)) != std::string::npos) {
                string_cmd.erase(0, pos + begin_pos.length());
                string_cmd.erase(string_cmd.find(end_pos));
            }
            // std::cout << string_cmd << std::endl;
            return string_cmd.c_str();
        }

        matrix_msgs::XbeeCommuROHM getCmdInfo(std::string string_receive)
        {
            // xbee_receive_.data = "#0x601{ROUTEgoPOI15}$"; 

            matrix_msgs::XbeeCommuROHM info_;
            info_.robot_no = HexString_float(split_string(string_receive, "#0x", "{"), 1) - 0x600;
            info_.cmd_string = split_string(string_receive, "{", "go");
            info_.poi_name = split_string(string_receive, "go", "}$");

            //ROS_INFO("cmd_info robot no. %d, cmd %s, poi_name %s", info_.robot_no, info_.cmd_string.c_str(), info_.poi_name.c_str());
            return info_;
        }
        

        geometry_msgs::Pose getRobotPose()
        {
            geometry_msgs::Pose pose_;
            tf::StampedTransform transform;
            tf::TransformListener listener;

            try
            {
                listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                // continue;
            }

            pose_.position.x = transform.getOrigin().x();
            pose_.position.y = transform.getOrigin().y();
            pose_.orientation.z = transform.getRotation().getZ();
            pose_.orientation.w = transform.getRotation().getW();
            return pose_;
        }

        

        

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "xbee_management");

    XbeeCmdServer Xbee_Cmd_Server("xbee_management");
    Xbee_Cmd_Server.spin();

    return 0;
}
