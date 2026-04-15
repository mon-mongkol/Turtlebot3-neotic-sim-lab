#ifndef _MATRIX_DOCKING_OPERATION_NODE_H
#define _MATRIX_DOCKING_OPERATION_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>

// action_server 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <matrix_msgs/OfflineControlOperationAction.h> 

// action_client
// #include <actionlib/client/simple_action_client.h>
// #include <matrix_msgs/SerialOperationAction.h>
// #include <matrix_msgs/ARTrackOperationAction.h>
// #include <matrix_msgs/ScheduleAction.h>
// srv_client
// #include <std_srvs/SetBool.h> // set mode_controller to docking mode 

// sub
// #include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// pub 
#include <geometry_msgs/Twist.h>

//math
#include <cmath>

//for pause state
#include <matrix_msgs/RobotMode.h>

enum OfflineControlState
{
    INIT = 0,

    CmdSelector,

    // FORWARD,
    MOVEMENT_INIT,
    MOVEMENT,

    ROTATE_INIT,
    ROTATE,
    ROTATE_LEFT,
    ROTATE_RIGHT,

    FINISH,
    ERROR,
    PAUSE
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

enum MovingControlMode
{
    BACKWARD,
    FORWARD
};


class OfflineControlOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::OfflineControlOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::OfflineControlOperationFeedback feedback_;
        matrix_msgs::OfflineControlOperationResult   results_;

        ros::Subscriber odom_sub, robotmode_sub;
        ros::Publisher cmd_vel_pub;

        // nav_msgs::Odometry odom_;
        double current_pos_x = 0.0;
        double current_pos_y = 0.0;
        double current_theta = 0.0;
        double last_current_theta = 0.0;
        double desired_theta = 0.0;
        float lastError = 0;

        float math_pi = 3.141592653589793238463;

        float theta_travel_current, theta_now, theta_old, theta_travel_goal;
        int dir;

        

        bool finished = false;
        bool success = false;

        //for pause state
        int period_state = 0;
        int period_timeout_coutdown = 0;
        int current_robotmode = 0;
        std::string period_text ="";
        double setTimeOut;

        double offset_linear = 0.95;
        double offset_angular = 1.25;
        
    
    public:
        OfflineControlOperation(std::string name)
            : as_(nh_, name, boost::bind(&OfflineControlOperation::executeCB, this, _1), false),
              action_name_(name)

        {
            ros::NodeHandle private_nh("~");
            
            // // //wait for the matrix_artrack_operation action server to come up
            // while(!artrack_operation_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the matrix_artrack_operation action server to come up");
            // }

            odom_sub = nh_.subscribe("/odom", 1, &OfflineControlOperation::cbOdom, this);

            // for pause state
            robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &OfflineControlOperation::cbRobotMode, this);

            cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            //wait service server start
            // set_robot_mode_sc.waitForExistence();

            ros::param::param<double>("~offset_linear", offset_linear, 0.95);
            ros::param::param<double>("~offset_angular", offset_angular, 1.25);

            as_.start();
            as_.registerPreemptCallback(boost::bind(&OfflineControlOperation::preemptCB, this));
            ROS_INFO("matrix_docking_operation action server is ready!");
        }
    
    private:
        
        
        bool executeCB(const matrix_msgs::OfflineControlOperationGoalConstPtr &goal);
        void preemptCB();

        // void matrixSerialOpDoneCb(const actionlib::SimpleClientGoalState &state);
        // void matrixARTrackOpDoneCb(const actionlib::SimpleClientGoalState &state);
        // void SerialOperationAction(std::string cmd, std::string data);

        //subscribe callback

        //for pause sate
        // void cbSerialRead(std_msgs::String::ConstPtr msg);
        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg);
        void cbOdom(nav_msgs::Odometry msg);

        //func()
        // bool setRobotMode(std::string cmd);
        void clear_params(void);
        float MovingControl(MovingControlMode mode, MovingParams moving_params);
        void fnStop(void);
        float fnrotate(float max_vel, int dir);

        //action DonceCb
        // void matrixScheduleDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::ScheduleResultConstPtr &result);
        // void matrixSerialDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SerialOperationResultConstPtr &result);

};

#endif // _MATRIX_SERIAL_NODE_H