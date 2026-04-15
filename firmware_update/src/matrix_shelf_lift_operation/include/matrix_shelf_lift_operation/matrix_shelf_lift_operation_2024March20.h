#ifndef _MATRIX_SHELF_LIFT_2024MARCH20_H
#define _MATRIX_SHELF_LIFT_2024MARCH20_H

#include <ros/ros.h>

// action_server 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <matrix_msgs/SHLOperationAction.h>
 

// action_client
#include <actionlib/client/simple_action_client.h>


// srv_client
#include <dynamic_reconfigure/Reconfigure.h>

// sub // pub 
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>


//math
#include <cmath>

//for pause state
#include <matrix_msgs/RobotMode.h>

enum SHLControlState
{
    INIT_PARM, //call lauch_controller for startup AR3_tracking

    CHECK_LIFT_STATE,
    WAIT_CHECK_LIFT_STATE_FINISH,

    CMD_SELECTOR,

    SEND_CMD,
    WAIT_RES_CMD,
    WAIT_RESULT,

    FINISH,
    ERROR,
    PAUSE

};

class SHLOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::SHLOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::SHLOperationFeedback feedback_;
        matrix_msgs::SHLOperationResult   results_;

        bool _isSim = false;

        bool isConditionMatch = false;

        std::string condition1="";
        std::string condition2="";
        std::string condition3="";
        std::string condition4="";
        std::string conditionStr = "";

        int state_condition_timeout = 2;

        bool SHL_completed = false;

        bool initPause = false;
        bool isLiftEmerActive = false;


        // ros::ServiceClient docking_mode_sc, set_ios_sc, launch_controller_ac, motor_control_sc;
        ros::Subscriber sub_shl100_recv, sub_shl100_io_state, robotmode_sub;
        ros::Publisher pub_shl100_write;


        bool finished = false;
        bool success = false;

        std_msgs::String cmd_msg;

        //for pause state
        int period_state = 0;
        int period_timeout_coutdown = 0;
        int current_robotmode = 0;

        std::string period_text ="";
        std::string data_send, data_receive;

        std::string serial_no="";
        std::string lift_model="";
        
        double setTimeOut;
        
        int current_sequence = 0;

        int retry_counter = 0;
        int max_retry = 3;

        bool disable_timeout = false;

    
    public:
        SHLOperation(std::string name)
            : as_(nh_, name, boost::bind(&SHLOperation::executeCB, this, _1), false),
              action_name_(name)
        {
            ros::NodeHandle private_nh("~");

            // for pause state
            robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &SHLOperation::cbRobotMode, this);
            sub_shl100_recv = nh_.subscribe("/matrix_shl100/receive", 10, &SHLOperation::cbSHLReceive, this);
            sub_shl100_io_state = nh_.subscribe("/matrix_shl100/io_state", 1, &SHLOperation::cbSHLIOstate, this);

            pub_shl100_write = nh_.advertise<std_msgs::String>("/matrix_shl100/write", 1);
            
            ros::param::param<std::string>("~serial_no", serial_no, "SDR01002023001DRB001");
            ros::param::param<std::string>("~lift_model", lift_model, "SHL100");
            
            //wait service server start
            // docking_mode_sc.waitForExistence();

            as_.start();
            as_.registerPreemptCallback(boost::bind(&SHLOperation::preemptCB, this));
            ROS_INFO("[matrix_shelf_lift_operation]: Matrix_shelf_lift_operation action server is ready!!!!!*********");

            ros::Rate rr(10);

        }
    
    private:
        
        
        bool executeCB(const matrix_msgs::SHLOperationGoalConstPtr &goal);
        void preemptCB();

        //subscribe callback
        //for pause sate
        void cbSHLReceive(std_msgs::String::ConstPtr msg);
        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg);
        void cbSHLIOstate(std_msgs::Int8MultiArray msg);

        //func()
        void clear_params(bool cancel_action_ar3, bool kill_action_ar3);

        std::string STRcondition_checking(std::string con1, std::string con2, std::string cmd);
        std::string STRcondition_checking(std::string con1, std::string con2, std::string con3, std::string con4, std::string cmd);
        

       
};

#endif // _MATRIX_SERIAL_NODE_H