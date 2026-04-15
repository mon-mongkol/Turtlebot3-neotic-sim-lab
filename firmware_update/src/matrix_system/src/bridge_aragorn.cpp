#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <matrix_msgs/RobotMode.h>
enum robot_mode{  INITIAL,
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
                  RESET,
                  EMERGENCY_CASE_ACTIVE,
                  EMERGENCY_CHARGE
                  };

class bridge_aragorn {
    private:
    int counter;
    std_msgs::Int8 matrix_mode_;
    ros::Publisher pub, pub_robot_mode, pub_emer;
    ros::Subscriber number_subscriber, sub_matrix_mode;
    ros::ServiceServer reset_service;
    public:
    bridge_aragorn(ros::NodeHandle *nh) {
        // counter = 0;
        // pub = nh->advertise<std_msgs::Int64>("/number_count", 10);    
        // number_subscriber = nh->subscribe("/number", 1000, 
        //     &bridge_aragorn::callback_number, this);
        
        pub_robot_mode = nh->advertise<std_msgs::Int8>("/aragorn/robot_mode", 10);   
        pub_emer = nh->advertise<std_msgs::Int8>("/aragorn_io/emergency", 10);  
        sub_matrix_mode = nh->subscribe("/matrix_mode_controller/mode", 10, 
            &bridge_aragorn::callback_MatrixMode, this);

        // reset_service = nh->advertiseService("/reset_counter", 
        //     &bridge_aragorn::callback_reset_counter, this);

        ros::Rate loop_rate(10);

        while(ros::ok())
        {
            bridge_state();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    // void callback_number(const std_msgs::Int64& msg) {
    //     counter += msg.data;
    //     std_msgs::Int64 new_msg;
    //     new_msg.data = counter;
    //     pub.publish(new_msg);
    // }
    // bool callback_reset_counter(std_srvs::SetBool::Request &req, 
    //                             std_srvs::SetBool::Response &res)
    // {
    //     if (req.data) {
    //         counter = 0;
    //         res.success = true;
    //         res.message = "Counter has been successfully reset";
    //     }
    //     else {
    //         res.success = false;
    //         res.message = "Counter has not been reset";
    //     }
    //     return true;
    // }

    void callback_MatrixMode(const matrix_msgs::RobotMode& msg)
    {
        matrix_mode_.data = msg.robot_mode;
    }

    void bridge_state()
    {
        std_msgs::Int8 emer_msg, mode_msg;
        switch(matrix_mode_.data)
        {
            case robot_mode::EMERGENCY:
                emer_msg.data = 1;
                break;

            default:
                emer_msg.data = matrix_mode_.data;
                break;
        }
        mode_msg.data = matrix_mode_.data;
        
        pub_emer.publish(emer_msg);
        pub_robot_mode.publish(mode_msg);
    }
};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "bridge_aragorn");
    ros::NodeHandle nh;
    bridge_aragorn nc = bridge_aragorn(&nh);
    ros::spin();
}