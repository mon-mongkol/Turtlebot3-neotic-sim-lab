#ifndef _MATRIX_IO_OPERATION_NODE_H
#define _MATRIX_IO_OPERATION_NODE_H
#include  <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <matrix_msgs/IOOperationAction.h>

// service set IO at MCU01
#include <matrix_msgs/SetIO.h>
// state of input/output of MCU01
#include <std_msgs/Int32MultiArray.h>

//for pause state
#include <matrix_msgs/RobotMode.h>


using namespace std;
enum IOControlState
{
    INIT = 0,
    SELECT_FUN,

    WAIT_IO_MATCH_INIT,
    WAIT_IO_MATCH,

    SET_IO_INIT,
    SET_IO,

    FINISH,
    ERROR,
    PAUSE
};

enum ResultDataChecker
{
    MATCH,
    MISS_MATCH,
    OUT_OF_LEGHT,
    CORE_DUMP
};

enum PublishMode
{
    NOT_PUBLISH,
    LOW,
    RISING,
    FALLING,
    HIGH
};



class IOOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::IOOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::IOOperationFeedback feedback_;
        matrix_msgs::IOOperationResult   results_;

        ros::Subscriber input_sub, output_sub, robotmode_sub;
        ros::ServiceClient set_io_srv_;

        vector<int> MCUIO_input_data;
        vector<vector<int>>GOAL_input_data;

        bool isSimState = false;

        bool finished = false;
        bool success = false; 

        //for pause state
        int period_state = 0;
        int period_timeout_coutdown = 0;
        int current_robotmode = 0;
        std::string period_text ="";

        int mode = 0;

        struct {
            int input_address;
            int current_state;
            int last_state;
            int mode;
            std::string data;
        }x1, x2, x3, x4;


        
    
    public:
        IOOperation(std::string name)
            : as_(nh_, name, boost::bind(&IOOperation::executeCB, this, _1), false),
              action_name_(name)
        {
            ros::NodeHandle private_nh("~");

            


            as_.start();
            as_.registerPreemptCallback(boost::bind(&IOOperation::preemptCB, this));
            ROS_INFO("%s server is ready!", name.c_str()); 

            input_sub = nh_.subscribe("/matrix_io/input", 1 ,&IOOperation::cbMCU01Input, this);
            output_sub = nh_.subscribe("/matrix_io/output", 1 ,&IOOperation::cbMCU01Output, this);

            // for pause state
            robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &IOOperation::cbRobotMode, this);

            set_io_srv_ = nh_.serviceClient<matrix_msgs::SetIO>("/matrix_io/service");
        }
    
    private:
  
        int current_sequence = -1;
        double setTimeOut;
        bool executeCB(const matrix_msgs::IOOperationGoalConstPtr &goal);
        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg);
        void cbMCUInput(const std_msgs::Int32MultiArray msg);
        void cbMCUOutput(const std_msgs::Int32MultiArray msg);
        int IntDataChecker(std::vector<int> mcu_data, std::vector<std::vector<int>> goal_data);
        void preemptCB();
};

#endif // _MATRIX_SERIAL_NODE_H