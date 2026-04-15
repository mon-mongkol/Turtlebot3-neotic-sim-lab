#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <matrix_msgs/MovmentStatus.h>
#include <matrix_msgs/Diagnostics.h>
#include <matrix_msgs/RobotMode.h>


class RobotState
{
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub_cmd_vel,
                        sub_robot_mode,
                        sub_odom,
                        sub_system_ready;

        ros::Publisher pub_robot_status;

        enum robot_mode{    INITIAL,
                            IDLE,
                            START_MOTOR,
                            SHUTDOWN_MOTOR,
                            SHUTDOWN_ROBOT,
                            EMERGENCY,
                            FUCN_1,
                            DOWN_STAIRS,
                            TABLET_LOSS_COMMU,
                            UVC_ON,
                            UVC_OFF,
                            READY_TO_START,
                            DOCKING_MODE_ON,
                            DOCKING_MODE_OFF,
                            ERROR_DEVICE,
                            CHARGER_ON,
                            CHARGER_OFF,
                            RESET
                            };
        
        int current_mode, current_mode_old;

        geometry_msgs::Twist current_cmd_vel;

        float current_x = 0;
        float current_z = 0;

        double time_now, time_period;
        bool _isInit;

        bool moving_li = false;
        bool moving_ang = false;
        bool cmd_hit = false;
        bool robot_moving = false;

        nav_msgs::Odometry period_state;
        int system_ready = 1;

        std::string mode_topic = "";
        std::string cmdvel_topic = "";
        std::string odom_topic = "";
        std::string systemready_topic = "";
        std::string pubstate_topic = "";



    public:
        RobotState()
        {
            ros::param::param<std::string>("~mode_topic", mode_topic, "/matrix_mode_controller/mode");
            ros::param::param<std::string>("~cmdvel_topic", cmdvel_topic, "/cmd_vel");
            ros::param::param<std::string>("~odom_topic", odom_topic, "/odom");
            ros::param::param<std::string>("~systemready_topic", systemready_topic, "/matrix_system/diagnostics/system_ready");
            ros::param::param<std::string>("~pubstate_topic", pubstate_topic, "/matrix_system/movement_status");
            
            sub_robot_mode = nh.subscribe(mode_topic, 10, &RobotState::cbRobotMode, this);
            sub_cmd_vel = nh.subscribe(cmdvel_topic, 10, &RobotState::cbCmdVel, this);
            sub_odom = nh.subscribe(odom_topic, 10, &RobotState::cbOdom, this);
            sub_system_ready = nh.subscribe(systemready_topic, 10, &RobotState::cbSystemReady, this);

            pub_robot_status = nh.advertise<matrix_msgs::MovmentStatus>(pubstate_topic, 10);

            ros::Rate loop_rate(10);

            
            while(ros::ok())
            {
                
                update_robot_status();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void cbOdom(const nav_msgs::Odometry &msg)
        {
            if((period_state.pose.pose.position.x != msg.pose.pose.position.x) || (period_state.pose.pose.position.y != msg.pose.pose.position.y))
            {
                moving_li = true;
                // printf("moving_li %d", moving_li );
                // printf("moving");
            }
            else
            {
                moving_li = false;
                // printf("stop");
            }

            period_state = msg;
        }
        void cbRobotMode(const matrix_msgs::RobotMode &robot_mode_msg)
        {
            current_mode = robot_mode_msg.robot_mode;
            // printf("recive mode %d\n", current_mode);
        }

        void cbSystemReady(const matrix_msgs::Diagnostics &msg)
        {
            system_ready = msg.system_ready;
            // printf("recive mode %d\n", system_ready);
        }

        void cbCmdVel(const geometry_msgs::Twist &cmd_vel_msg)
        {
            current_x = cmd_vel_msg.linear.x;
            current_z = cmd_vel_msg.angular.z;
            if((cmd_vel_msg.linear.x != 0.00) || (cmd_vel_msg.angular.z != 0.00)) 
            {
                cmd_hit = true;
                // printf("cmd hit %d", cmd_hit);
            }
            else
            {
                cmd_hit = false;
            }

            time_period = ros::Time::now().toSec();

        }
    
        bool move_ment()
        {
            if((moving_li == true) && (system_ready == 0) && (cmd_hit))
            {        
                robot_moving = true;
                // printf("True");
            }
            else
            {   
                robot_moving = false;
            }

            if((ros::Time::now().toSec() - time_period) >= 1.5)
            {
                cmd_hit = false;
            }
            else ;

            return robot_moving;
        }      

        void update_robot_status()
        {
            matrix_msgs::MovmentStatus robot_status_msg_;
            robot_status_msg_.robot_moving = move_ment();
            pub_robot_status.publish(robot_status_msg_);
        }
};



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "matrix_movement_state");

  //Create an object of class SubscribeAndPublish that will take care of everything
  RobotState current_robot_state;

  ros::spin();

  return 0;
}








