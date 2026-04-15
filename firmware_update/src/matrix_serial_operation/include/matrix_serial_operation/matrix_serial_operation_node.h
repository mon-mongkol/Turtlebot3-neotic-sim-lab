#ifndef _MATRIX_SERIAL_NODE_H
#define _MATRIX_SERIAL_NODE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <matrix_msgs/SerialOperationAction.h>
#include <std_srvs/SetBool.h>
#include <matrix_msgs/SerialCommu.h>
#include <string>

#include <matrix_msgs/ActionController.h>

//for pause state
#include <matrix_msgs/RobotMode.h>

#include <autodock_core/AutoDockingAction.h>
typedef actionlib::SimpleActionClient<autodock_core::AutoDockingAction> ARTrack3OpClient;

enum SerialControlState
{
    INIT = 0,
    SELECT_CMD,

    CHECK_DEVICE,
    SEND_CMD,
    WAIT_SEND_CMD_FINISH,

    WAIT_SERIAL_COMING,
    SERIAL_COMING,

    TIMMER_SET,
    WAIT_TIMER_FINISH,

    CANCEL_ACTION,

    SR_SEND,
    SR_RECEIVE,

    INIT_MULTI_DATA,
    WAIT_MULTI_DATA_MATCH,

    LAUNCH_AUTODOCKING,
    WAIT_LAUNCH_FINISH,

    FIND_DATA_IN_PAYLOAD_INIT,
    FIND_DATA_IN_PAYLOAD_SEND,
    FIND_DATA_IN_PAYLOAD_CHECK,

    FINISH,
    ERROR,
    PAUSE
};

class SerialOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::SerialOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::SerialOperationFeedback feedback_;
        matrix_msgs::SerialOperationResult   results_;

        ros::Subscriber serial_receive_sub, robotmode_sub, rs785_receive_sub;
        ros::Publisher serial_send_pub, rs785_send_pub;
        ros::ServiceClient cancel_action_srv_, serial_commu_srv_, launch_controller_ac;

        ARTrack3OpClient artrack3_operation_ac;

        std::string serial_data_;
        std_msgs::String serial_msg;
        bool Matrix_MovingCompleted = false;
        bool isSimState = false;

        bool finished = false;
        bool success = false; 

        bool sr_init_timeout = false;

        // std::string wait_data = "";
        bool data_coming = false;
        bool sr_1st_send = false;

        double last_send_stmp=0;
        // std::vector<std::string> multi_data;
        matrix_msgs::SerialOperationGoal multi_data_;
        std::string isData = "";

        //for pause state
        int period_state = 0;
        int period_timeout_coutdown = 0;
        int current_robotmode = 0;
        int send_freq = 1;
        std::string period_text ="";
        
        std::string device_name="";
        std::string serial_receive_topic="";
        std::string serial_send_topic="";
        std::string serial_communication_srv="";
    
    public:
        SerialOperation(std::string name)
            : as_(nh_, name, boost::bind(&SerialOperation::executeCB, this, _1), false),
              action_name_(name),
              artrack3_operation_ac("autodock_action", true)
        {
            ros::NodeHandle private_nh("~");

            // //wait for the action server to come up
            // while(!move_base_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the move_base_ac action server to come up");
            // }
            
            // //wait for the action server to come up
            // while(!matrix_movement_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the matrix_movement_ac action server to come up");
            // }

            ros::param::param<std::string>("~device_name", device_name, "rs485");
            ros::param::param<std::string>("~serial_receive_topic", serial_receive_topic, "/rs485_receive");
            ros::param::param<std::string>("~serial_send_topic", serial_send_topic, "/rs485_send");
            ros::param::param<std::string>("~serial_communication_srv", serial_communication_srv, "/serial_communication_srv");

            as_.start();
            as_.registerPreemptCallback(boost::bind(&SerialOperation::preemptCB, this));
            ROS_INFO("%s server is ready!", name.c_str());

            // for pause state
            serial_receive_sub = nh_.subscribe(serial_receive_topic, 1, &SerialOperation::cbSerialRead, this);
            robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &SerialOperation::cbRobotMode, this);

            // rs785_receive_sub = nh_.subscribe("/rs485_receive", 1, &SerialOperation::cbRS785_SerialRead, this);
            // rs785_send_pub = nh_.advertise<std_msgs::String>("/rs485_send", 1);

            serial_send_pub = nh_.advertise<std_msgs::String>(serial_send_topic, 1);

            cancel_action_srv_ = nh_.serviceClient<std_srvs::SetBool>("/matrix_launch_controller/cancel_action");
            serial_commu_srv_ = nh_.serviceClient<matrix_msgs::SerialCommu>(serial_communication_srv);
            launch_controller_ac =nh_.serviceClient<matrix_msgs::ActionController>("/matrix_launch_controller/action_controller");

        }
    
    private:
        bool moveCompleted;
        int current_sequence = -1;
        double setTimeOut;
        bool executeCB(const matrix_msgs::SerialOperationGoalConstPtr &goal);
        void cbSerialRead(const std_msgs::String::ConstPtr &msg);
        // void cbRS785_SerialRead(const std_msgs::String::ConstPtr &msg);
        void SerialDataChecker(std::string data);
        // bool SerialDataSend(std::string data);
        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg);
        void preemptCB();
        bool LaunchController(std::string cmd, std::string action_name);
};

#endif // _MATRIX_SERIAL_NODE_H