#ifndef _MATRIX_DOCKING_OPERATION_NODE_H
#define _MATRIX_DOCKING_OPERATION_NODE_H

#include <ros/ros.h>

// action_server 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <matrix_msgs/ROHMChargerOperationAction.h> 

// action_client
#include <actionlib/client/simple_action_client.h>
#include <matrix_msgs/SerialOperationAction.h>
#include <matrix_msgs/ARTrackOperationAction.h>
#include <matrix_msgs/ScheduleAction.h>
// srv_client
#include <std_srvs/SetBool.h> // set mode_controller to docking mode 

// sub
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// pub 
#include <geometry_msgs/Twist.h>

//math
#include <cmath>

//for pause state
#include <matrix_msgs/RobotMode.h>

enum DockingControlState
{
    INIT = 0,

    CmdSelector,

    WAIT_CHARGING_STATE,
    CHARGING_STATE_COMEUP,
    SET_SCHEDULE,
    SET_TIMMER,
    CHARGING,

    BACKWARD_INIT,
    BACKWARD_CONTROL,

    END_CHARGE,

    WAIT_CHARGER_FINISH,

    FINISH,
    ERROR,
    PAUSE
};

enum MovingControlMode
{
    BACKWARD,
    FORWARD
};

struct MovingParams
{
    float start_pos_x;
    float start_pos_y;
    float linear_dis;
    float max_vel;
    double control_timeout;

};
struct MovingParams moving_params_;



typedef actionlib::SimpleActionClient<matrix_msgs::SerialOperationAction> SerialOpClient;
typedef actionlib::SimpleActionClient<matrix_msgs::ARTrackOperationAction> ARTrackOpClient;
typedef actionlib::SimpleActionClient<matrix_msgs::ScheduleAction> ScheduleClient;

class DockingOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::ROHMChargerOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::ROHMChargerOperationFeedback feedback_;
        matrix_msgs::ROHMChargerOperationResult   results_;

        SerialOpClient serial_operation_ac;
        ARTrackOpClient artrack_operation_ac;
        ScheduleClient schedule_operation_ac;

        bool Matrix_ARTrackCompleted = false;
        bool Matrix_SerialComplete = false;

        bool isSimState = false;

        bool isConditionMatch = false;
        std::string condition1="";
        std::string condition2="";


        ros::ServiceClient docking_mode_sc;
        ros::Subscriber serial_sub, batt_sub, odom_sub, robotmode_sub;
        ros::Publisher cmd_vel_pub;
        sensor_msgs::BatteryState bat_state;

        int charging_state_timeout = 30;

        // nav_msgs::Odometry odom_;
        double current_pos_x = 0.0;
        double current_pos_y = 0.0;

        float lastError = 0;

        bool finished = false;
        bool success = false;

        bool isScheduleCompleted = false;
        bool isTimmerCompleted = false;

        //for pause state
        int period_state = 0;
        int period_timeout_coutdown = 0;
        int current_robotmode = 0;
        std::string period_text ="";
        double setTimeOut;

        bool skip_timer = false;
        
    
    public:
        DockingOperation(std::string name)
            : as_(nh_, name, boost::bind(&DockingOperation::executeCB, this, _1), false),
              action_name_(name),
              serial_operation_ac("matrix_serial_operation", true),
              artrack_operation_ac("matrix_artrack_operation", true),
              schedule_operation_ac("matrix_scheldule_operation", true)

        {
            ros::NodeHandle private_nh("~");

            // // //wait for the matrix_serial_operation action server to come up
            while(!serial_operation_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the matrix_serial_operation action server to come up");
            }

            while(!schedule_operation_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the matrix_schedule_operation action server to come up");
            }
            
            // // //wait for the matrix_artrack_operation action server to come up
            // while(!artrack_operation_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the matrix_artrack_operation action server to come up");
            // }

            batt_sub = nh_.subscribe("/battery_state", 1, &DockingOperation::cbBatt, this);
            odom_sub = nh_.subscribe("/odom", 1, &DockingOperation::cbOdom, this);

            // for pause state
            robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &DockingOperation::cbRobotMode, this);
            serial_sub = nh_.subscribe("/raw_read", 1, &DockingOperation::cbSerialRead, this);

            docking_mode_sc = nh_.serviceClient<std_srvs::SetBool>("matrix_mode_controller/docking_mode");

            cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            //wait service server start
            docking_mode_sc.waitForExistence();

            as_.start();
            as_.registerPreemptCallback(boost::bind(&DockingOperation::preemptCB, this));
            ROS_INFO("matrix_docking_operation action server is ready!");
        }
    
    private:
        
        
        bool executeCB(const matrix_msgs::ROHMChargerOperationGoalConstPtr &goal);
        void preemptCB();

        // void matrixSerialOpDoneCb(const actionlib::SimpleClientGoalState &state);
        // void matrixARTrackOpDoneCb(const actionlib::SimpleClientGoalState &state);
        // void SerialOperationAction(std::string cmd, std::string data);

        //subscribe callback
        void cbBatt(sensor_msgs::BatteryState msg);
        //for pause sate
        void cbSerialRead(std_msgs::String::ConstPtr msg);
        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg);
        void cbOdom(nav_msgs::Odometry msg);

        //func()
        bool setDockingMode(std::string cmd);
        void clear_params(void);
        float MovingControl(MovingControlMode mode, MovingParams moving_params);
        void fnStop(void);

        //action DonceCb
        void matrixScheduleDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::ScheduleResultConstPtr &result);
        void matrixSerialDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SerialOperationResultConstPtr &result);

};

#endif // _MATRIX_SERIAL_NODE_H