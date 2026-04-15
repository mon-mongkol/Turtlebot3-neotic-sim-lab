#ifndef _MATRIX_DOCKING_OPERATION_NODE_H
#define _MATRIX_DOCKING_OPERATION_NODE_H

#include <ros/ros.h>

// action_server 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <matrix_msgs/CommonCommandAction.h> 

// action_client
#include <actionlib/client/simple_action_client.h>
#include <matrix_msgs/SerialOperationAction.h>

// sub
#include <std_msgs/String.h>


typedef actionlib::SimpleActionClient<matrix_msgs::SerialOperationAction> SerialOpClient;

class DockingManagement
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::CommonCommandAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::CommonCommandFeedback feedback_;
        matrix_msgs::CommonCommandResult   results_;

        SerialOpClient serial_operation_rs485_ac;

        bool isSimState = false;

        bool finished = false;
        bool success = false;

        double setTimeOut;

        bool skip_timer = false;
        
    
    public:
        DockingManagement(std::string name)
            : as_(nh_, name, boost::bind(&DockingManagement::executeCB, this, _1), false),
              action_name_(name),
              serial_operation_rs485_ac("matrix_serial_operation_485", true)
        {
            ros::NodeHandle private_nh("~");

            // // //wait for the serial_operation_rs485_ac action server to come up
            while(!serial_operation_rs485_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the serial_operation_rs485_ac action server to come up");
            }

            as_.start();
            as_.registerPreemptCallback(boost::bind(&DockingManagement::preemptCB, this));
            ROS_INFO("matrix_docking_management action server is ready!");
        }
    
    private:
        
        
        bool executeCB(const matrix_msgs::CommonCommandGoalConstPtr &goal);
        void preemptCB();

        //func()
        void clear_params(void);

        //action DonceCb
        void matrixSerialDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SerialOperationResultConstPtr &result);

};

#endif // _MATRIX_SERIAL_NODE_H