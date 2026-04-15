#include <ros/ros.h>
#include <std_msgs/String.h>
#include <matrix_msgs/RobotMode.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>

// srv_client
#include <std_srvs/SetBool.h> // set mode_controller to Pause mode 

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#include <dynamic_reconfigure/server.h>
#include <matrix_state_controller/MatrixStateControllerConfig.h>

using namespace matrix_msgs;
class StateController
{
    protected:
        ros::NodeHandle nh_;
        // MoveBaseClient move_base_ac;
        ros::Subscriber serial_sub, robot_mode_sub;
        ros::Publisher state_pub, pause_dock_pub;
        ros::ServiceClient pause_mode_sc;

        std::string serial_data = "";
        int current_robotmode = 0;
        
        std::string pause_mode_detect = "";
        std::string resume_mode_detect = "";
        bool trigger_pause = false;
        bool trigger_resume = true;
        double resume_dock_time = 2;

        dynamic_reconfigure::Server<matrix_state_controller::MatrixStateControllerConfig> dyn_srv_;
    
    public:
        StateController()
        {
            //wait for the action server to come up
            // while(!move_base_ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the move_base_ac action server to come up");
            // }

            init_param();
            init_pub_();
            init_sub_();
            init_srv_();

            dyn_srv_.setCallback(boost::bind(&StateController::callback, this, _1, _2));

            ros::Rate loop_rate(10);

            while(ros::ok())
            {   
                check_current_state();
                ros::spinOnce();
                loop_rate.sleep();
            }

        }

        void callback(matrix_state_controller::MatrixStateControllerConfig &config, uint32_t level)
        {
            pause_mode_detect = config.pause_mode_detect;
            resume_mode_detect = config.resume_mode_detect;

            trigger_pause = config.trigger_pause;
            if(trigger_pause)
            {
                serial_data = pause_mode_detect;
                config.trigger_pause = false;
            }

            trigger_resume = config.trigger_resume;
            if(trigger_resume)
            {
                serial_data = resume_mode_detect;
                config.trigger_resume = false;
            }

            resume_dock_time = config.resume_dock_time;

            // ROS_INFO("trigger_pause issssssssssss", trigger_pause);
            
        }

        void init_param()
        {
            ros::param::param<std::string>("~pause_mode_detect", pause_mode_detect, "PAUSE");
            ros::param::param<std::string>("~resume_mode_detect", resume_mode_detect, "RESUME");
            ros::param::param<bool>("~trigger_pause", trigger_pause, false);
            ros::param::param<bool>("~trigger_resume", trigger_resume, false);
            ros::param::param<double>("~resume_dock_time", resume_dock_time, 2);
        }

        void init_pub_()
        {
            state_pub = nh_.advertise<matrix_msgs::RobotMode>("/matrix_state_controller/robot_state", 1);
            pause_dock_pub = nh_.advertise<std_msgs::Bool>("/pause_dock", 1);
        }

        void init_sub_()
        {
            serial_sub = nh_.subscribe("/raw_read", 1, &StateController::cbSerialRead, this);
            robot_mode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &StateController::cbRobotMode, this);
        }

        void init_srv_()
        {
            pause_mode_sc = nh_.serviceClient<std_srvs::SetBool>("matrix_mode_controller/pause_mode");
        }

        void cbSerialRead(const std_msgs::String::ConstPtr &msg)
        {
            serial_data = msg->data;
        }

        void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
        {
            current_robotmode = msg->robot_mode;
        }
        

        void check_current_state()
        {
            // ROS_INFO("serial_data -----> %s", serial_data.c_str());
            if((current_robotmode != RobotMode::READY_TO_START) && (current_robotmode != RobotMode::EMERGENCY_CHARGE) && (current_robotmode != RobotMode::ERROR_DEVICE) && (current_robotmode != RobotMode::EMERGENCY))
            {
                // ROS_INFO("CHECK word pause mode");
                if((serial_data.find("STOP") != -1) || (serial_data.find(pause_mode_detect) != -1))
                {
                    ROS_WARN("[matrix_state_controller]:trigger PAUSE mode");
                    setPauseMode("on");
                    serial_data.clear();
                    std_msgs::Bool pause_dock_msg;
                    pause_dock_msg.data = true;
                    pause_dock_pub.publish(pause_dock_msg);
                    
                    
                }

                if((serial_data.find("START") != -1) || (serial_data.find(resume_mode_detect) != -1))
                {
                    ROS_INFO("[matrix_state_controller]:trigger RESUME mode");
                    setPauseMode("off");
                    serial_data.clear();

                    ros::Duration(resume_dock_time).sleep();
                    std_msgs::Bool pause_dock_msg;
                    pause_dock_msg.data = false;
                    pause_dock_pub.publish(pause_dock_msg);
                }

            }
            else
            {
                if((serial_data.find("START") != -1) || (serial_data.find(resume_mode_detect) != -1))
                {
                    ROS_INFO("RESUME mode");
                    setPauseMode("off");
                    serial_data.clear();

                    ros::Duration(resume_dock_time).sleep();
                    std_msgs::Bool pause_dock_msg;
                    pause_dock_msg.data = false;
                    pause_dock_pub.publish(pause_dock_msg);
                }
            }
            serial_data.clear();
            
        }

        bool setPauseMode(std::string cmd)
        {
            std_srvs::SetBool pause_cmd;
            pause_cmd.request.data = (cmd == "on")? true:false;
            pause_mode_sc.call(pause_cmd);

            return pause_cmd.response.success;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_state_controller");

    StateController state_controller;

    ros::spin();

    return 0;


}