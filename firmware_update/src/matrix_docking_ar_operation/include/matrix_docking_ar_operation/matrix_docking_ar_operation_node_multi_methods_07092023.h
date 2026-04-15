#ifndef _MATRIX_DOCKING_AR_OPERATION_NODE_MULTI_METHODS_H
#define _MATRIX_DOCKING_AR_OPERATION_NODE_MULTI_METHODS_H

#include <ros/ros.h>

// action_server 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <matrix_msgs/ARDockingOperationMultiMethodsAction.h> 

// action_client
#include <actionlib/client/simple_action_client.h>
#include <matrix_msgs/SerialOperationAction.h>
#include <matrix_msgs/ARTrackOperationAction.h>
#include <matrix_msgs/ScheduleAction.h>
#include <autodock_core/AutoDockingAction.h>
#include <matrix_msgs/CenterVShapeTrackingAction.h>

// srv_client
#include <std_srvs/SetBool.h> // set mode_controller to docking mode 
#include <matrix_msgs/SetIOs.h>
#include <matrix_msgs/ActionController.h>
#include <dynamic_reconfigure/Reconfigure.h>
// sub
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// pub 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

//math
#include <cmath>

//for pause state
#include <matrix_msgs/RobotMode.h>

enum ARDockingControlState
{
    INIT_PARM, //call lauch_controller for startup AR3_tracking

    CMD_SELECTOR,

    LAUNCH_AR_TRACK,
    WAIT_AR_TRACK_LAUNCH_FINISH,

    LAUNCH_CVSHAPE,
    WAIT_CVSHAPE_LAUNCH_FINISH,

    GOAL_CHECKER,

    ENABLE_DOCKING_MODE,

    SEND_CG_END,
    WAIT_CG_FINISH,
    SEND_CG_CHECK,
    WAIT_CG_OK,

    SELECT_TRACKING_METHOD,

    SEARCH_AR,
    INIT_AR_TRACKING,
    AR_TRACKING,
    WAIT_FINISH_AR_TRACKING,

    SEARCH_CV_SHAPE,
    INIT_CV_SHAPE,
    CV_SHPAE_TRACKING,
    WAIT_FINISH_CV_TRACKING,

    INTI_SKIP_TRACKING_DOCK,
    WAIT_DOCKING_CONNECT,

    WAIT_CG_CONFIRM_CONNECT,

    SEND_CG_REQ_ON,
    WAIT_CG_REQ_ON_SUCCESS,

    INIT_BMS_CONFIRM_STATE,
    WAIT_BMS_CONFIRM_STATE,

    SEND_CONFIRM_STATE_TO_DOCK,
    WAIT_CG_CONFIRM_STATE,

    INIT_CG_CONDITION_FINISH,

    CHECK_CONDITION_FINISH,

    DISABLE_DOCKING_MODE,
    FINISH,
    ERROR,
    PAUSE,

    DODCKING_NOT_SYNC

};



typedef actionlib::SimpleActionClient<matrix_msgs::SerialOperationAction> SerialOpClient;
typedef actionlib::SimpleActionClient<matrix_msgs::ARTrackOperationAction> ARTrackOpClient;
typedef actionlib::SimpleActionClient<matrix_msgs::ScheduleAction> ScheduleClient;
typedef actionlib::SimpleActionClient<autodock_core::AutoDockingAction> ARTrack3OpClient;
typedef actionlib::SimpleActionClient<matrix_msgs::CenterVShapeTrackingAction> CVshapeClient;

class ARDockingOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::ARDockingOperationMultiMethodsAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::ARDockingOperationMultiMethodsFeedback feedback_;
        matrix_msgs::ARDockingOperationMultiMethodsResult   results_;

        SerialOpClient serial_operation_rs485_ac;
        SerialOpClient serial_operation_xbee_ac;
        ARTrackOpClient artrack_operation_ac;
        ScheduleClient schedule_operation_ac;
        ARTrack3OpClient artrack3_operation_ac;
        CVshapeClient cvshape_operation_ac;

        bool Matrix_ARTrackCompleted = false;
        bool Matrix_ARTrack3Completed = false;
        bool Matrix_SerialCompleted = false;
        bool Matrix_ScheduleCompleted = false;
        bool Matrix_CVShapeCompleted = false;

        bool _isSim = false;

        bool isConditionMatch = false;

        std::string condition1="";
        std::string condition2="";


        ros::ServiceClient docking_mode_sc, set_ios_sc, launch_controller_ac, motor_control_sc;
        ros::Subscriber rs485_sub, serial_sub, batt_sub, odom_sub, robotmode_sub;
        ros::Publisher cmd_vel_pub, pause_dock_pub;
        sensor_msgs::BatteryState batt_state;

        int charging_state_timeout = 30;



        bool finished = false;
        bool success = false;

        //for pause state
        int period_state = 0;
        int period_timeout_coutdown = 0;
        int current_robotmode = 0;
        std::string period_text ="";
        std::string data_send, data_receive;
        std::string DG_FINISH="DG_FINISH";
        std::string CHARGE_OK="CHARGE_OK";
        std::string CHARGE_NOT_OK="CHARGE_NOT_OK";
        std::string CHARGE_CHECK="CHARGE_CHECK";
        std::string DG_REQ="DG_REQ";
        std::string RxRBICS="RxRBICS";
        std::string CHARGE_FINISH="CHARGE_FINISH";
        std::string RxREQSS="RxREQSS";
        std::string CG_CHARGING_STATE="CG_CHARGING_STATE";
        std::string DG_SN="";
        std::string RxREADY="RxREADY";
        std::string RxWRBREQ="RxWRBREQ";
        std::string DG_RBICS="DG_RBICS";
        double goal_timeout;
        double setTimeOut;
        double charging_time;
        
        int error_counter = 0;
        bool skip_confirm_state = false;

        int current_sequence = 0;

        bool _isDockingStateSync = false;
        std::string docking_state_str = "";

        matrix_msgs::CenterVShapeTrackingGoal cv_goal_yaml;
        bool cv_from_yaml = true;
        float cv_default_lin_vel_max = 0.0;
        float cv_default_travel_dist = 0.0;
        float cv_default_stop_dist = 0.0;
        float cv_default_right_offset = 0.0;
        float cv_default_left_offset = 0.0;
        int cv_default_timeout = 100;
    
    public:
        ARDockingOperation(std::string name)
            : as_(nh_, name, boost::bind(&ARDockingOperation::executeCB, this, _1), false),
              action_name_(name),
              serial_operation_rs485_ac("matrix_serial_operation_485", true),
              serial_operation_xbee_ac("matrix_serial_operation"),
              artrack_operation_ac("matrix_artrack_operation", true),
              schedule_operation_ac("matrix_scheldule_operation", true),
              artrack3_operation_ac("autodock_action", true),
              cvshape_operation_ac("matrix_cvshape_2dir_operation", true)

        {
            ros::NodeHandle private_nh("~");

            // // //wait for the matrix_serial_operation action server to come up
            // while(!serial_operation_rs485_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the matrix_serial_operation action server to come up");
            // }

            // while(!schedule_operation_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the matrix_schedule_operation action server to come up");
            // }
            
            // // //wait for the matrix_artrack_operation action server to come up
            // while(!artrack_operation_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the matrix_artrack_operation action server to come up");
            // }

            batt_sub = nh_.subscribe("/battery_state", 1, &ARDockingOperation::cbBatt, this);
            odom_sub = nh_.subscribe("/odom", 1, &ARDockingOperation::cbOdom, this);

            // for pause state
            robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &ARDockingOperation::cbRobotMode, this);
            serial_sub = nh_.subscribe("/raw_read", 1, &ARDockingOperation::cbSerialRead, this);
            rs485_sub = nh_.subscribe("/matrix_io/rs485_receive", 1, &ARDockingOperation::cbRS485Receive, this);

            docking_mode_sc = nh_.serviceClient<std_srvs::SetBool>("matrix_mode_controller/docking_mode");
            set_ios_sc = nh_.serviceClient<matrix_msgs::SetIOs>("matrix_io/service");
            motor_control_sc = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("matrix_canopen_motor_driver/set_parameters");

            launch_controller_ac =nh_.serviceClient<matrix_msgs::ActionController>("/matrix_launch_controller/action_controller");
            cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            pause_dock_pub = nh_.advertise<std_msgs::Bool>("/pause_dock", 1);

            
            ros::param::param<bool>("~skip_confirm_state", skip_confirm_state, false);
            ros::param::param<bool>("~cv_from_yaml", cv_from_yaml, true);
            ros::param::param<float>("~cv_lin_vel_max", cv_default_lin_vel_max, -0.2);
            ros::param::param<float>("~cv_travel_dist", cv_default_travel_dist, 2.0);
            ros::param::param<float>("~cv_stop_dist", cv_default_stop_dist, 0.4);
            ros::param::param<float>("~cv_right_offset", cv_default_right_offset, 0.0);
            ros::param::param<float>("~cv_left_offset", cv_default_left_offset, 0.0);
            ros::param::param<int>("~cv_timeout", cv_default_timeout, 60); 
            
            cv_goal_yaml.lin_vel_max = cv_default_lin_vel_max;
            cv_goal_yaml.travel_dist = cv_default_travel_dist;
            cv_goal_yaml.stop_dist = cv_default_stop_dist;
            cv_goal_yaml.right_offset = cv_default_right_offset;
            cv_goal_yaml.left_offset = cv_default_left_offset;
            cv_goal_yaml.timeout = cv_default_timeout;
            

            //wait service server start
            // docking_mode_sc.waitForExistence();

            as_.start();
            as_.registerPreemptCallback(boost::bind(&ARDockingOperation::preemptCB, this));
            ROS_INFO("[matrix_docking_multi_methods_operation]: Matrix_docking_multi_methods_operation action server is ready!!!!!");

            ros::Rate rr(10);

            // while(ros::ok())
            // {
            //     rr.sleep();
            // }
        }
    
    private:
        
        
        bool executeCB(const matrix_msgs::ARDockingOperationMultiMethodsGoalConstPtr &goal);
        void preemptCB();

        // void matrixSerialOpDoneCb(const actionlib::SimpleClientGoalState &state);
        // void matrixARTrackOpDoneCb(const actionlib::SimpleClientGoalState &state);
        // void SerialOperationAction(std::string cmd, std::string data);

        //subscribe callback
        void cbBatt(sensor_msgs::BatteryState msg);
        //for pause sate
        void cbSerialRead(std_msgs::String::ConstPtr msg);
        void cbRS485Receive(std_msgs::String::ConstPtr msg);
        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg);
        void cbOdom(nav_msgs::Odometry msg);

        //func()
        bool setDockingMode(std::string cmd);
        void clear_params(bool cancel_action_ar3, bool kill_action_ar3);
        // float MovingControl(MovingControlMode mode, MovingParams moving_params);
        void fnStop(void);

        //action DonceCb
        void matrixScheduleDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::ScheduleResultConstPtr &result);
        void matrixSerialDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SerialOperationResultConstPtr &result);
        void matrixARTrack3DonceCb(const actionlib::SimpleClientGoalState &state, const autodock_core::AutoDockingResultConstPtr &result);
        void matrixCVshapeDonceCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::CenterVShapeTrackingResultConstPtr &result);
        

        void SerialActionSend(std::string cmd, std::string send, std::string receive, double timeout, bool repeat);
        void SerialActionWaitFinish(int current_seq, ARDockingControlState pass_seq, ARDockingControlState fail_seq, double goal_timeout);

        void SerialActionSendMulti(std::string cmd, std::string send, std::string* receive_s, double timeout, bool repeat);

        bool LaunchController(std::string cmd, std::string action_name);

        bool setMotorMode(std::string cmd, std::string mode);

       
};

#endif // _MATRIX_SERIAL_NODE_H