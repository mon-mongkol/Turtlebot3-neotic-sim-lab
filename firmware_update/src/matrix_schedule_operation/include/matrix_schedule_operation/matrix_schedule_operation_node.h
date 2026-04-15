#ifndef _Schedule_OPERATION_NODE_H
#define _Schedule_OPERATION_NODE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include <matrix_msgs/ScheduleAction.h>
#include <std_msgs/String.h>

#include <iostream>
#include <ctime>

class ScheduleOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::ScheduleAction> as_;
        std::string action_name_;

        matrix_msgs::ScheduleFeedback feedback_;
        matrix_msgs::ScheduleResult results_;

        bool finished = false;
        bool success = false; 

        ros::Subscriber serial_sub;

        // tm *goal_date;
        time_t goal_date_t;
        std::string mode;

    public:
        ScheduleOperation(std::string name)
            :   as_(nh_, name, boost::bind(&ScheduleOperation::executeCB, this, _1), false),
                action_name_(name)
        {
            as_.start();
            as_.registerPreemptCallback(boost::bind(&ScheduleOperation::preemptCB, this));
            ROS_INFO("matrix_schedule_operation action server is ready!");

            // serial_sub = nh_.subscribe("/raw_read", 1, &ScheduleOperation::cbSerialRead, this);
        }
    
    private:
        bool executeCB(const matrix_msgs::ScheduleGoalConstPtr &goal);
        void preemptCB(void);
        // void cbSerialRead(const std_msgs::String msg);
        time_t getTimeGMT7Now(time_t now);
        time_t getTimeForm(time_t now, const matrix_msgs::Date date, bool current_date);
        void clear_param(void);
}; 

#endif // 
