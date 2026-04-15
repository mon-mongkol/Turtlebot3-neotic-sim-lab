#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robot_status/RobotStatus.h>


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
        std_msgs::Bool system_ready;



    public:
        RobotState()
        {
            pub_robot_status = nh.advertise<robot_status::RobotStatus>("/aragorn/base_status", 10);
            
            sub_robot_mode = nh.subscribe("/aragorn/robot_mode", 10, &RobotState::cbRobotMode, this);
            sub_cmd_vel = nh.subscribe("/cmd_vel", 10, &RobotState::cbCmdVel, this);
            sub_odom = nh.subscribe("/zlac706/odom", 10, &RobotState::cbOdom, this);
            sub_system_ready = nh.subscribe("/aragorn/diagnostics/system_ready", 10, &RobotState::cbSystemReady, this);

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
                // printf("moving");
            }
            else
            {
                moving_li = false;
                // printf("stop");
            }

            period_state = msg;
        }
        void cbRobotMode(const std_msgs::Int8 &robot_mode_msg)
        {
            current_mode = robot_mode_msg.data;
            // printf("recive mode %d\n", current_mode);
        }

        void cbSystemReady(const std_msgs::Bool &msg)
        {
            system_ready.data = msg.data;
        }

        void cbCmdVel(const geometry_msgs::Twist &cmd_vel_msg)
        {
            current_x = cmd_vel_msg.linear.x;
            current_z = cmd_vel_msg.angular.z;
            if((cmd_vel_msg.linear.x != 0.0) && (cmd_vel_msg.angular.z != 0.0)) 
            {
                cmd_hit = true;
            }
            else
            {
                cmd_hit = false;
            }

            time_period = ros::Time::now().toSec();

        }
    
        bool move_ment()
        {
            if((moving_li == true) && (cmd_hit == true) && (system_ready.data == true))
            {        
                robot_moving = true;
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
            robot_status::RobotStatus robot_status_msg_;
            robot_status_msg_.robot_moving = move_ment();
            pub_robot_status.publish(robot_status_msg_);
        }
};



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "robot_state");

  //Create an object of class SubscribeAndPublish that will take care of everything
  RobotState current_robot_state;

  ros::spin();

  return 0;
}








