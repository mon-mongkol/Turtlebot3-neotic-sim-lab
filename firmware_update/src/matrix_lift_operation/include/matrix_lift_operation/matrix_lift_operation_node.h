#ifndef _MATRIX_LIFT_NODE_H
#define _MATRIX_LIFT_NODE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <matrix_msgs/SwapEnviOperationAction.h>
#include <matrix_msgs/ROHMLiftOperationAction.h>
#include <std_msgs/String.h>
#include <matrix_msgs/XbeeCommunicationAction.h>

enum LiftControlState
{
    INIT = 0,
    WAIT_FOR_ROBOT_POSE,
    SEND_LIFT_REQ,
    WAIT_LIFT_READY,
    LIFT_READY,
    MOVE_IN_LIFT,
    WAIT_MOVE_IN_LIFT_FINISH,
    REACH_TARGET,
    MOVE_TARGET_FAIL,
    CALL_ENVI_SWAP,
    WAIT_ENVI_SWAP_FINISH,
    WAIT_LIFT_FINISH,
    LIFT_FINISH,
    LEFT_LIFT,
    WAIT_LEFT_LIFT_FINISH,
    SEND_CMD,
    FINISH,
    ERROR
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<matrix_msgs::XbeeCommunicationAction> MatrixMovementClient;
typedef actionlib::SimpleActionClient<matrix_msgs::SwapEnviOperationAction> MatrixSwapEnviClient;
class LiftOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::ROHMLiftOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::ROHMLiftOperationFeedback feedback_;
        matrix_msgs::ROHMLiftOperationResult   results_;

        MoveBaseClient movebase_ac;
        MatrixMovementClient matrix_movement_ac;
        MatrixSwapEnviClient matrix_swapenvi_ac;

        ros::Subscriber serial_sub;
        ros::Publisher serial_pub;

        std::string serial_data_;
        bool Matrix_MovingCompleted = false;
        bool Matrix_SeapenviCompleted = false;
        bool isSimState = false;
    
    public:
        LiftOperation(std::string name)
            : as_(nh_, name, boost::bind(&LiftOperation::executeCB, this, _1), false),
              action_name_(name),
              movebase_ac("move_base", true),
              matrix_swapenvi_ac("matrix_swapenvi_operation", true),
              matrix_movement_ac("matrix_xbeemovement_server", true)
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

            as_.start();
            as_.registerPreemptCallback(boost::bind(&LiftOperation::preemptCB, this));
            ROS_INFO("matrix_lift_operation action server is ready!");

            serial_sub = nh_.subscribe("/read", 1, &LiftOperation::cbSerialRead, this);
            serial_pub = nh_.advertise<std_msgs::String>("/remote_xbee", 1);
        }
    
    private:
        bool moveCompleted;
        int current_sequence = -1;
        double setTimeOut;
        bool executeCB(const matrix_msgs::ROHMLiftOperationGoalConstPtr &goal);
        void cbSerialRead(const std_msgs::String::ConstPtr &msg);

        // void MoveBaseActive();
        // void MoveBaseFeedback(const matrix_msgs::ROHMLiftOperationFeedbackConstPtr &feedback);
        void moveDoneCb(const actionlib::SimpleClientGoalState &state);
        void matrixMovementDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::XbeeCommunicationResultConstPtr &result);
        void matrixSwapEnviDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SwapEnviOperationResultConstPtr &result);
        void showGoal(matrix_msgs::ROHMLiftOperationGoalConstPtr &goal);
        bool matrix_movementSendGoal(std::string poi_name);
        matrix_msgs::XbeeCommunicationResult waitMatrixMovementFinish();
        void preemptCB();
};

#endif // _MATRIX_LIFT_NODE_H