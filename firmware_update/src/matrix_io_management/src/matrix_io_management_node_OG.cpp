#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <matrix_io_management/IOManagementConfig.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <matrix_msgs/RobotMode.h>
#include <std_msgs/Int8MultiArray.h>
#include <matrix_msgs/SetIOs.h>

#include <string> 

class ManagementIO {

    private:
        ros::NodeHandle nh_;
        int counter;
        ros::Publisher pub_raw_read, pub_bumper_state, pub_usr_pin_x, pub_usr_pin_y, pub_io_output;
        ros::Subscriber input_sub, output_sub, robotmode_sub ;
        // ros::ServiceServer reset_service;
        ros::ServiceClient set_mode_sc, set_io_sc;

        enum PublishMode
        {
            NOT_PUBLISH,
            INACTIVE,
            CHANGE,
            RISING,
            FALLING,
            ACTIVE
        };

        enum ActionMode
        {
            NONE,
            STOP_MOVEMENT,
            STOP_ALL,
            PAUSE,
            RESUME,
            EMERGENCY_CASE,
            DOCKING
        };

        struct ISRdata {
            int input_address;
            int current_state;
            int last_state;
            int mode;
            std::string data;
            int action;
        };

        ISRdata x1_prev, x1_de, x1, x2_prev, x2_de, x2, x3_prev, x3_de, x3, x4_prev, x4_de, x4, xGD_button, xRD_button, xOG_button;
        ISRdata x5_prev, x5_de, x5, x6_prev, x6_de, x6, x7_prev, x7_de, x7, x8_prev, x8_de, x8, x9_prev, x9_de, x9, x10_prev, x10_de, x10;
        ISRdata xLimitSw, xBumper;

        int active_state = 0;
        int inactive_state = 1;

        int active_state_prev = 0;
        int inactive_state_prev = 1;

        int active_state_de = 0;
        int inactive_state_de = 1;

        int active_state_button = 0;
        int inactive_state_button = 1;



        bool use_input_botton_state = false;

        dynamic_reconfigure::Server<matrix_io_management::IOManagementConfig> dyn_srv_;

        bool init_dy_default = false;

        int hex_header = 0x02;
        int hex_footer = 0x03;
        std::string str_header = "";
        std::string str_footer = "";

        int hex_header_prev = 0x02;
        int hex_footer_prev = 0x03;
        std::string str_header_prev = "";
        std::string str_footer_prev = "";

        int hex_header_de = 0x02;
        int hex_footer_de = 0x03;
        std::string str_header_de = "";
        std::string str_footer_de = "";

        int hex_header_button = 0x02;
        int hex_footer_button = 0x03;
        std::string str_header_button = "";
        std::string str_footer_button = "";

        bool use_STX_ETX = true;
        bool use_newline = true;

        bool use_STX_ETX_de = true;
        bool use_newline_de = true;

        bool use_STX_ETX_prev = true;
        bool use_newline_prev = true;

        bool use_STX_ETX_button = true;
        bool use_newline_button = true;
        bool use_input_bumper_state = true;

        int current_robotmode= 0;

        bool output_state_prev[4] = {0,0,0,0};
        bool y_cfg[4][4];
        std_msgs::Int32MultiArray output_state;
        int output[4];

        bool wait_data_comeup = false;



    public:
    ManagementIO() {

        ros::param::param<bool>("~y1_enabled", y_cfg[0][0], false);
        ros::param::param<bool>("~y2_enabled", y_cfg[1][0], false);
        ros::param::param<bool>("~y3_enabled", y_cfg[2][0], false);
        ros::param::param<bool>("~y4_enabled", y_cfg[3][0], false);
        // ros::param::param<bool>("~y1_on", y1_cfg[1], false);

        ros::param::param<bool>("~use_input_botton_state", use_input_botton_state, false);
        ros::param::param<bool>("~use_input_bumper_state", use_input_bumper_state, false);

        ros::param::param<bool>("~use_STX_ETX", use_STX_ETX, true);
        ros::param::param<bool>("~use_newline", use_newline, false);

        ros::param::param<int>("~active_state", active_state, 0);
        ros::param::param<int>("~inactive_state", inactive_state, 1);

        ros::param::param<int>("~hex_header", hex_header, 0x02);
        ros::param::param<int>("~hex_footer", hex_footer, 0x03);

        ros::param::param<std::string>("~str_header", str_header, "");
        ros::param::param<std::string>("~str_footer", str_footer, "");

        ros::param::param<int>("~x1_input_address", x1.input_address, 10);
        ros::param::param<int>("~x1_mode", x1.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x1_data", x1.data, "x1");
        ros::param::param<int>("~x1_action", x1.action, 0);

        ros::param::param<int>("~x2_input_address", x2.input_address, 11);
        ros::param::param<int>("~x2_mode", x2.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x2_data", x2.data, "x2");
        ros::param::param<int>("~x2_action", x2.action, 0);

        ros::param::param<int>("~x3_input_address", x3.input_address, 12);
        ros::param::param<int>("~x3_mode", x3.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x3_data", x3.data, "x3");
        ros::param::param<int>("~x3_action", x3.action, 0);

        ros::param::param<int>("~x4_input_address", x4.input_address, 13);
        ros::param::param<int>("~x4_mode", x4.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x4_data", x4.data, "x4");
        ros::param::param<int>("~x4_action", x4.action, 0);

        ros::param::param<int>("~x5_input_address", x5.input_address, 10);
        ros::param::param<int>("~x5_mode", x5.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x5_data", x5.data, "x5");
        ros::param::param<int>("~x5_action", x5.action, 0);

        ros::param::param<int>("~x6_input_address", x6.input_address, 10);
        ros::param::param<int>("~x6_mode", x6.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x6_data", x6.data, "x6");
        ros::param::param<int>("~x6_action", x6.action, 0);

        ros::param::param<int>("~x7_input_address", x7.input_address, 10);
        ros::param::param<int>("~x7_mode", x7.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x7_data", x7.data, "x7");
        ros::param::param<int>("~x7_action", x7.action, 0);

        ros::param::param<int>("~x8_input_address", x8.input_address, 10);
        ros::param::param<int>("~x8_mode", x8.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x8_data", x8.data, "x8");
        ros::param::param<int>("~x8_action", x8.action, 0);

        ros::param::param<int>("~x9_input_address", x9.input_address, 10);
        ros::param::param<int>("~x9_mode", x9.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x9_data", x9.data, "x9");
        ros::param::param<int>("~x9_action", x9.action, 0);

        ros::param::param<int>("~x10_input_address", x10.input_address, 10);
        ros::param::param<int>("~x10_mode", x10.mode, PublishMode::RISING);
        ros::param::param<std::string>("~x10_data", x10.data, "x10");
        ros::param::param<int>("~x10_action", x10.action, 0);

        ros::param::param<int>("~xLimitSw_input_address", xLimitSw.input_address, 10);
        ros::param::param<int>("~xLimitSw_mode", xLimitSw.mode, PublishMode::RISING);
        ros::param::param<std::string>("~xLimitSw_data", xLimitSw.data, "xLimitSw");
        ros::param::param<int>("~xLimitSw_action", xLimitSw.action, 0);

        ros::param::param<int>("~xBumper_input_address", xBumper.input_address, 10);
        ros::param::param<int>("~xBumper_mode", xBumper.mode, PublishMode::RISING);
        ros::param::param<std::string>("~xBumper_data", xBumper.data, "xBumper");
        ros::param::param<int>("~xBumper_action", xBumper.action, 0);

        ros::param::param<int>("~xGD_button_input_address", xGD_button.input_address, 10);
        ros::param::param<int>("~xGD_button_mode", xGD_button.mode, PublishMode::RISING);
        ros::param::param<std::string>("~xGD_button_data", xGD_button.data, "START");
        ros::param::param<int>("~xGD_button_action", xGD_button.action, 0);

        ros::param::param<int>("~xRD_button_input_address", xRD_button.input_address, 10);
        ros::param::param<int>("~xRD_button_mode", xRD_button.mode, PublishMode::RISING);
        ros::param::param<std::string>("~xRD_button_data", xRD_button.data, "STOP");
        ros::param::param<int>("~xRD_button_action", xRD_button.action, 0);

        ros::param::param<int>("~xOG_button_input_address", xOG_button.input_address, 10);
        ros::param::param<int>("~xOG_button_mode", xOG_button.mode, PublishMode::RISING);
        ros::param::param<std::string>("~xOG_button_data", xOG_button.data, "RESET");
        ros::param::param<int>("~xOG_button_action", xOG_button.action, 0);




        use_STX_ETX_de = use_STX_ETX;
        use_newline_de = use_newline;

        x1_de = x1;
        x2_de = x2;
        x3_de = x3;
        x4_de = x4;
        x5_de = x5;
        x6_de = x6;
        x7_de = x7;
        x8_de = x8;
        x9_de = x9;
        x10_de = x10;

        x1_prev = x1;
        x2_prev = x2;
        x3_prev = x3;
        x4_prev = x4;
        x5_prev = x5;
        x6_prev = x6;
        x7_prev = x7;
        x8_prev = x8;
        x9_prev = x9;
        x10_prev = x10;

        hex_header_prev = hex_header;
        hex_footer_prev = hex_footer;

        hex_header_de = hex_header;
        hex_footer_de = hex_footer;

        str_header_de = str_header;
        str_footer_de = str_footer;


        hex_footer_button = 0x03;
        hex_header_button = 0x02;
        str_footer_button = "";
        str_header_button = "";
        use_STX_ETX_button = true;
        use_newline_button = false;
        

        input_sub = nh_.subscribe("/matrix_io/input", 1 ,&ManagementIO::cbMCUInput, this);
        output_sub = nh_.subscribe("/matrix_io/output", 1 ,&ManagementIO::cbMCUOutput, this);
        robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &ManagementIO::cbRobotMode, this);

        

        pub_raw_read = nh_.advertise<std_msgs::String>("/raw_read", 1);
        pub_bumper_state = nh_.advertise<std_msgs::Bool>("/matrix_io_management/bumper", 1);

        pub_usr_pin_x = nh_.advertise<std_msgs::Int8MultiArray>("/matrix_io_management/user_cfg_state/input", 1);
        pub_usr_pin_y = nh_.advertise<std_msgs::Int8MultiArray>("/matrix_io_management/user_cfg_state/output", 1);

        dyn_srv_.setCallback(boost::bind(&ManagementIO::callback, this, _1, _2));

        set_mode_sc = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/matrix_mode_controller/set_parameters");
        set_io_sc = nh_.serviceClient<matrix_msgs::SetIOs>("matrix_io/service");

        pub_io_output = nh_.advertise<std_msgs::Int32MultiArray>("/matrix_io_management/user_ctrl_output", 1);


        while(!set_io_sc.waitForExistence())
        {
            break;
        }

        ROS_INFO("[matrix_io_management]: Ready!!");

        ros::Rate loop_rate(10);

        while(ros::ok())
        {
            // try
            // {
            //     scan_time_control();
            // }
            // catch(...)
            // {
            //     ROS_ERROR("Error");
            // }
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }

    void cbRobotMode(const matrix_msgs::RobotMode msg)
    {
        current_robotmode = msg.robot_mode;
    }

    bool scan_time_control()
    {
        auto success = false;
        for(int i= 0; i<4; i++)
        {
            matrix_msgs::SetIOs srv_setios;
            srv_setios.request.cmd = "set_user_output";
            if(y_cfg[i][0])
            {
                // ROS_INFO("AA");
                try{
                    if(y_cfg[i][1] != output_state.data[10+i])
                    {
                        srv_setios.request.arg0 = i;
                        srv_setios.request.arg1 = (y_cfg[i][1] == true)? 1:0;
                        ROS_INFO("[matrix_io_management]: action at user_output_chanel:%d, with sate: %d", i+1, srv_setios.request.arg1);
                        try
                        {
                            // if(set_io_sc.call(srv_setios))
                            // {
                            //     auto s = std::to_string(srv_setios.request.arg1);
                            //     ROS_INFO("%s, %s", s.c_str(), srv_setios.response.text.c_str());
                        
                            //     if(srv_setios.response.text.find("1") != -1)
                            //     {
                            //         y_cfg[i][2] = 1;
                            //         // ROS_INFO("[matrix_io_management]: Set IOs action at user_output_chanel:%d, with sate: %d Success!!!", i+1, srv_setios.request.arg1);
                            //         // success = true;
                            //     }
                            //     else{
                            //         y_cfg[i][2] = 0;
                            //         // ROS_WARN("[matrix_io_management]: Set IOs action at user_output_chanel:%d, with sate: %d Fail!!!", i+1, srv_setios.request.arg1);
                            //         // success = false;
                            //     }
                            // }
                            std_msgs::Int32MultiArray op_data;
                            if(i == 0)
                            {
                                op_data.data.push_back(srv_setios.request.arg1);
                                op_data.data.push_back(output_state.data[11]);
                                op_data.data.push_back(output_state.data[12]);
                                op_data.data.push_back(output_state.data[13]);
                            }
                            if(i == 1)
                            {
                                op_data.data.push_back(output_state.data[10]);
                                op_data.data.push_back(srv_setios.request.arg1);
                                op_data.data.push_back(output_state.data[12]);
                                op_data.data.push_back(output_state.data[13]);
                            }
                            if(i == 2)
                            {
                                op_data.data.push_back(output_state.data[10]);
                                op_data.data.push_back(output_state.data[11]);
                                op_data.data.push_back(srv_setios.request.arg1);
                                op_data.data.push_back(output_state.data[13]);
                            }
                            if(i == 3)
                            {
                                op_data.data.push_back(output_state.data[10]);
                                op_data.data.push_back(output_state.data[11]);
                                op_data.data.push_back(output_state.data[12]);
                                op_data.data.push_back(srv_setios.request.arg1);
                            }
                            pub_io_output.publish(op_data);

                            // ros::Duration d(0.1);

                            y_cfg[i][2] = srv_setios.request.arg1;

                        }
                        catch (...)
                        {
                            ROS_ERROR("[matrix_io_management]: cannot call service_server /matrix_io/services please check servicer is available");
                        }
                        

                    }else{
                        ;
                    }
                }
                catch (...)
                {
                    ROS_WARN("[matrix_io_management]: wait data coming!!");
                }
            }
            else
            {
                ;
            }
    
        }
        return success;
    }

    bool scan_time_control_intCase()
    {
        auto success = false;
        for(int i= 0; i<4; i++)
        {
            matrix_msgs::SetIOs srv_setios;
            srv_setios.request.cmd = "set_user_output";
            // check enabled
            // if(y_cfg[i][0])
            if(output_state.data[10+i] != 2) // 0 = off, 1 = on, 2 = disabled
            {
                // ROS_INFO("AA");
                try{
                    if(y_cfg[i][1] != output_state.data[10+i])
                    {
                        srv_setios.request.arg0 = 1;
                        srv_setios.request.arg1 = (y_cfg[i][1] == true)? 1:0;
                        ROS_INFO("[matrix_io_management]: action at user_output_chanel:%d, with sate: %d", i+1, srv_setios.request.arg1);
                        try
                        {
                            if(set_io_sc.call(srv_setios))
                            {
                                auto s = std::to_string(srv_setios.request.arg1);
                                ROS_INFO("%s, %s", s.c_str(), srv_setios.response.text.c_str());
                        
                                if(srv_setios.response.text.find("1") != -1)
                                {
                                    y_cfg[i][2] = 1;
                                    // ROS_INFO("[matrix_io_management]: Set IOs action at user_output_chanel:%d, with sate: %d Success!!!", i+1, srv_setios.request.arg1);
                                    // success = true;
                                }
                                else{
                                    y_cfg[i][2] = 0;
                                    // ROS_WARN("[matrix_io_management]: Set IOs action at user_output_chanel:%d, with sate: %d Fail!!!", i+1, srv_setios.request.arg1);
                                    // success = false;
                                }
                            }
                        }
                        catch (...)
                        {
                            ROS_ERROR("[matrix_io_management]: cannot call service_server /matrix_io/services please check servicer is available");
                        }
                        

                    }else{
                        ;
                    }
                }
                catch (...)
                {
                    ROS_WARN("[matrix_io_management]: wait data coming!!");
                }
            }
            else
            {
                ;
            }
    
        }
        return success;
    }

    void callback(matrix_io_management::IOManagementConfig &config, uint32_t level)
    {
        // matrix_msgs::SetIOs srv_setios;
        // srv_setios.request.cmd = "set_output";
        // if(config.y1_enabled)
        // {
        //     config.y1_input_address = 10;
        //     if(config.y1_on != output_state.data[0])
        //     {
        //         srv_setios.request.cmd = "set_output";
        //         srv_setios.request.arg0 = 1;
        //         srv_setios.request.arg1 = (config.y1_on == true)? 1:0;
        //     }
        //     config.y1_pin_cfg = -1;
        // }
        if(wait_data_comeup)
        {
            y_cfg[0][0] = config.y1_enabled;
            if(!config.y1_enabled)
            {
                config.y1_on = false;
                // ("aaaa");
            }
            else
            {
                y_cfg[0][1] = config.y1_on;
            }


            y_cfg[1][0] = config.y2_enabled;
            if(!config.y2_enabled)
            {
                config.y2_on = false;
            }
            else
            {
                y_cfg[1][1] = config.y2_on;
            }
          

            y_cfg[2][0] = config.y3_enabled;
            if(!config.y3_enabled)
            {
                config.y3_on = false;
            }
            else
            {
                y_cfg[2][1] = config.y3_on;
            }

            y_cfg[3][0] = config.y4_enabled;
            if(!config.y4_enabled)
            {
                config.y4_on = false;
            }
            else
            {
                y_cfg[3][1] = config.y4_on;
            }

            scan_time_control();



            
            // ROS_INFO("10: %d", y_cfg[0][2]);
            config.y1_on = (config.y1_enabled)? y_cfg[0][2]:false;
            config.y2_on = (config.y2_enabled)? y_cfg[1][2]:false;
            config.y3_on = (config.y3_enabled)? y_cfg[2][2]:false;
            config.y4_on = (config.y4_enabled)? y_cfg[3][2]:false;
        }
        



        if(config.restore_default)
        {
            config.active_state = active_state_de;
            config.inactive_state = inactive_state_de;
            config.hex_header = hex_header_de;
            config.hex_footer = hex_footer_de;
            config.str_header = str_header_de;
            config.str_footer = str_footer_de;
            config.use_STX_ETX = use_STX_ETX_de;
            config.use_newline = use_newline_de;

            config.x1_input_address = x1_de.input_address;
            config.x1_mode = x1_de.mode;
            config.x1_data = x1_de.data;
            config.x1_action = x1_de.action;
            ROS_INFO("[matrix_io_management]:X1 address: %d, mode: %d, data: %s", x1.input_address, x1.mode, x1.data.c_str());

            config.x2_input_address = x2_de.input_address;
            config.x2_mode = x2_de.mode;
            config.x2_data = x2_de.data;
            config.x2_action = x2_de.action;
            ROS_INFO("[matrix_io_management]:X2 address: %d, mode: %d, data: %s", x2.input_address, x2.mode, x2.data.c_str());

            config.x3_input_address = x3_de.input_address;
            config.x3_mode = x3_de.mode;
            config.x3_data = x3_de.data;
            config.x3_action = x3_de.action;
            ROS_INFO("[matrix_io_management]:X3 address: %d, mode: %d, data: %s", x3.input_address, x3.mode, x3.data.c_str());

            config.x4_input_address = x4_de.input_address;
            config.x4_mode = x4_de.mode;
            config.x4_data = x4_de.data;
            config.x4_action = x4_de.action;
            ROS_INFO("[matrix_io_management]:X4 address: %d, mode: %d, data: %s", x4.input_address, x4.mode, x4.data.c_str());

            config.x5_input_address = x5_de.input_address;
            config.x5_mode = x5_de.mode;
            config.x5_data = x5_de.data;
            config.x5_action = x5_de.action;
            ROS_INFO("[matrix_io_management]:x5 address: %d, mode: %d, data: %s", x5.input_address, x5.mode, x5.data.c_str());

            config.x6_input_address = x6_de.input_address;
            config.x6_mode = x6_de.mode;
            config.x6_data = x6_de.data;
            config.x6_action = x6_de.action;
            ROS_INFO("[matrix_io_management]:x6 address: %d, mode: %d, data: %s", x6.input_address, x6.mode, x6.data.c_str());

            config.x7_input_address = x7_de.input_address;
            config.x7_mode = x7_de.mode;
            config.x7_data = x7_de.data;
            config.x7_action = x7_de.action;
            ROS_INFO("[matrix_io_management]:x7 address: %d, mode: %d, data: %s", x7.input_address, x7.mode, x7.data.c_str());

            config.x8_input_address = x8_de.input_address;
            config.x8_mode = x8_de.mode;
            config.x8_data = x8_de.data;
            config.x8_action = x8_de.action;
            ROS_INFO("[matrix_io_management]:x8 address: %d, mode: %d, data: %s", x8.input_address, x8.mode, x8.data.c_str());

            config.x9_input_address = x9_de.input_address;
            config.x9_mode = x9_de.mode;
            config.x9_data = x9_de.data;
            config.x9_action = x9_de.action;
            ROS_INFO("[matrix_io_management]:x9 address: %d, mode: %d, data: %s", x9.input_address, x9.mode, x9.data.c_str());

            config.x10_input_address = x10_de.input_address;
            config.x10_mode = x10_de.mode;
            config.x10_data = x10_de.data;
            config.x10_action = x10_de.action;
            ROS_INFO("[matrix_io_management]:x10 address: %d, mode: %d, data: %s", x10.input_address, x10.mode, x10.data.c_str());
            

            init_dy_default = true;

            config.restore_default = false;
        }
        else
        {
            if(config.use_input_botton_state)
            {
                config.active_state = active_state_button;
                config.inactive_state = inactive_state_button;
                config.hex_header = hex_header_button;
                config.hex_footer = hex_footer_button;
                config.str_header = str_header_button;
                config.str_footer = str_footer_button;
                config.use_STX_ETX = use_STX_ETX_button;
                config.use_newline = use_newline_button;


                config.x1_input_address = xGD_button.input_address;
                config.x1_mode = xGD_button.mode;
                config.x1_data = xGD_button.data;
                config.x1_action = xGD_button.action;
                // ROS_INFO("[matrix_io_management]:X1 address: %d, mode: %d, data: %s", x1.input_address, x1.mode, x1.data.c_str());

                config.x2_input_address = xRD_button.input_address;
                config.x2_mode = xRD_button.mode;
                config.x2_data = xRD_button.data;
                config.x2_action = xRD_button.action;
                // ROS_INFO("[matrix_io_management]:X2 address: %d, mode: %d, data: %s", x2.input_address, x2.mode, x2.data.c_str());

                config.x3_input_address = xOG_button.input_address;
                config.x3_mode = xOG_button.mode;
                config.x3_data = xOG_button.data;
                config.x3_action = xOG_button.action;
                // ROS_INFO("[matrix_io_management]:X3 address: %d, mode: %d, data: %s", x3.input_address, x3.mode, x3.data.c_str());

                // x4.input_address = config.x4_input_address;
                // x4.mode = config.x4_mode;
                // x4.data = config.x4_data;
                // ROS_INFO("[matrix_io_management]:X4 address: %d, mode: %d, data: %s", x4.input_address, x4.mode, x4.data.c_str());

                config.x4_input_address = xLimitSw.input_address;
                config.x4_mode = xLimitSw.mode;
                config.x4_data = xLimitSw.data;
                config.x4_action = xLimitSw.action;

                config.x5_input_address = xBumper.input_address;
                config.x5_mode = xBumper.mode;
                config.x5_data = xBumper.data;
                config.x5_action = xBumper.action;

                init_dy_default = false;
            }
            else
            {
                if(!init_dy_default)
                {
                    config.active_state = active_state_prev;
                    config.inactive_state = inactive_state_prev;
                    config.hex_header = hex_header_prev;
                    config.hex_footer = hex_footer_prev;
                    config.str_header = str_header_prev;
                    config.str_footer = str_footer_prev;
                    config.use_STX_ETX = use_STX_ETX_prev;
                    config.use_newline = use_newline_prev;

                    config.x1_input_address = x1_prev.input_address;
                    config.x1_mode = x1_prev.mode;
                    config.x1_data = x1_prev.data;
                    config.x1_action =x1_prev.action;
                    // ROS_INFO("[matrix_io_management]:X1 address: %d, mode: %d, data: %s", x1.input_address, x1.mode, x1.data.c_str());

                    config.x2_input_address = x2_prev.input_address;
                    config.x2_mode = x2_prev.mode;
                    config.x2_data = x2_prev.data;
                    config.x2_action =x2_prev.action;
                    // ROS_INFO("[matrix_io_management]:X2 address: %d, mode: %d, data: %s", x2.input_address, x2.mode, x2.data.c_str());

                    config.x3_input_address = x3.input_address;
                    config.x3_mode = x3_prev.mode;
                    config.x3_data = x3_prev.data;
                    config.x3_action =x3_prev.action;
                    // ROS_INFO("[matrix_io_management]:X3 address: %d, mode: %d, data: %s", x3.input_address, x3.mode, x3.data.c_str());

                    config.x4_input_address = x4_prev.input_address;
                    config.x4_mode = x4_prev.mode;
                    config.x4_data = x4_prev.data;
                    config.x4_action =x4_prev.action;
                    // ROS_INFO("[matrix_io_management]:X4 address: %d, mode: %d, data: %s", x4.input_address, x4.mode, x4.data.c_str());

                    config.x5_input_address = x5_prev.input_address;
                    config.x5_mode = x5_prev.mode;
                    config.x5_data = x5_prev.data;
                    config.x5_action =x5_prev.action;

                    config.x6_input_address = x6_prev.input_address;
                    config.x6_mode = x6_prev.mode;
                    config.x6_data = x6_prev.data;
                    config.x6_action =x6_prev.action;

                    config.x7_input_address = x7_prev.input_address;
                    config.x7_mode = x7_prev.mode;
                    config.x7_data = x7_prev.data;
                    config.x7_action =x7_prev.action;

                    config.x8_input_address = x8_prev.input_address;
                    config.x8_mode = x8_prev.mode;
                    config.x8_data = x8_prev.data;
                    config.x8_action =x8_prev.action;

                    config.x9_input_address = x9_prev.input_address;
                    config.x9_mode = x9_prev.mode;
                    config.x9_data = x9_prev.data;
                    config.x9_action =x9_prev.action;

                    config.x10_input_address = x10_prev.input_address;
                    config.x10_mode = x10_prev.mode;
                    config.x10_data = x10_prev.data;
                    config.x10_action =x10_prev.action;

                    init_dy_default = true;
                }
                else
                {
                    ;
                }
            }
        }

        
        x1_prev = x1;
        x2_prev = x2;
        x3_prev = x3;
        x4_prev = x4;
        x5_prev = x5;
        x6_prev = x6;
        x7_prev = x7;
        x8_prev = x8;
        x9_prev = x9;
        x10_prev = x10;

        hex_header_prev = hex_header;
        hex_footer_prev = hex_footer;

        use_STX_ETX_prev = use_STX_ETX;
        use_newline_prev = use_newline;


        

        active_state = config.active_state;
        inactive_state = config.inactive_state;
        hex_header = config.hex_header;
        hex_footer = config.hex_footer;
        str_header = config.str_header;
        str_footer = config.str_footer;
        use_STX_ETX = config.use_STX_ETX;
        use_newline = config.use_newline;
        
        x1.input_address = config.x1_input_address;
        x1.mode = config.x1_mode;
        x1.data = config.x1_data;
        x1.action = config.x1_action;
        ROS_INFO("[matrix_io_management]:X1 address: %d, mode: %d, data: %s", x1.input_address, x1.mode, x1.data.c_str());

        x2.input_address = config.x2_input_address;
        x2.mode = config.x2_mode;
        x2.data = config.x2_data;
        x2.action = config.x2_action;
        ROS_INFO("[matrix_io_management]:X2 address: %d, mode: %d, data: %s", x2.input_address, x2.mode, x2.data.c_str());

        x3.input_address = config.x3_input_address;
        x3.mode = config.x3_mode;
        x3.data = config.x3_data;
        x3.action = config.x3_action;
        ROS_INFO("[matrix_io_management]:X3 address: %d, mode: %d, data: %s", x3.input_address, x3.mode, x3.data.c_str());

        x4.input_address = config.x4_input_address;
        x4.mode = config.x4_mode;
        x4.data = config.x4_data;
        x4.action = config.x4_action;
        ROS_INFO("[matrix_io_management]:X4 address: %d, mode: %d, data: %s", x4.input_address, x4.mode, x4.data.c_str());

        x5.input_address = config.x5_input_address;
        x5.mode = config.x5_mode;
        x5.data = config.x5_data;
        x5.action = config.x5_action;
        ROS_INFO("[matrix_io_management]:X5 address: %d, mode: %d, data: %s", x5.input_address, x5.mode, x5.data.c_str());

        x6.input_address = config.x6_input_address;
        x6.mode = config.x6_mode;
        x6.data = config.x6_data;
        x6.action = config.x6_action;
        ROS_INFO("[matrix_io_management]:X6 address: %d, mode: %d, data: %s", x6.input_address, x6.mode, x6.data.c_str());

        x7.input_address = config.x7_input_address;
        x7.mode = config.x7_mode;
        x7.data = config.x7_data;
        x7.action = config.x7_action;
        ROS_INFO("[matrix_io_management]:X7 address: %d, mode: %d, data: %s", x7.input_address, x7.mode, x7.data.c_str());

        x8.input_address = config.x8_input_address;
        x8.mode = config.x8_mode;
        x8.data = config.x8_data;
        x8.action = config.x8_action;
        ROS_INFO("[matrix_io_management]:X8 address: %d, mode: %d, data: %s", x8.input_address, x8.mode, x8.data.c_str());

        x9.input_address = config.x9_input_address;
        x9.mode = config.x9_mode;
        x9.data = config.x9_data;
        x9.action = config.x9_action;
        ROS_INFO("[matrix_io_management]:X9 address: %d, mode: %d, data: %s", x9.input_address, x9.mode, x9.data.c_str());

        x10.input_address = config.x10_input_address;
        x10.mode = config.x10_mode;
        x10.data = config.x10_data;
        x10.action = config.x10_action;
        ROS_INFO("[matrix_io_management]:X10 address: %d, mode: %d, data: %s", x10.input_address, x10.mode, x10.data.c_str());
        

        ROS_INFO("---------------------");
    }
    
    void cbMCUInput(const std_msgs::Int32MultiArray msg)
    {
        x1.current_state = msg.data[x1.input_address];
        x2.current_state = msg.data[x2.input_address];
        x3.current_state = msg.data[x3.input_address];
        x4.current_state = msg.data[x4.input_address];
        x5.current_state = msg.data[x5.input_address];
        x6.current_state = msg.data[x6.input_address];
        x7.current_state = msg.data[x7.input_address];
        x8.current_state = msg.data[x8.input_address];
        x9.current_state = msg.data[x9.input_address];
        x10.current_state = msg.data[x10.input_address];
        attachInterrupt(x1);
        attachInterrupt(x2);
        attachInterrupt(x3);
        attachInterrupt(x4);
        attachInterrupt(x5);
        attachInterrupt(x6);
        attachInterrupt(x7);
        attachInterrupt(x8);
        attachInterrupt(x9);
        attachInterrupt(x10);
        x1.last_state = msg.data[x1.input_address];
        x2.last_state = msg.data[x2.input_address];
        x3.last_state = msg.data[x3.input_address];
        x4.last_state = msg.data[x4.input_address];
        x5.last_state = msg.data[x5.input_address];
        x6.last_state = msg.data[x6.input_address];
        x7.last_state = msg.data[x7.input_address];
        x8.last_state = msg.data[x8.input_address];
        x9.last_state = msg.data[x9.input_address];
        x10.last_state = msg.data[x10.input_address];

        std_msgs::Int8MultiArray msg_pub;
        msg_pub.data.push_back(msg.data[10]);
        msg_pub.data.push_back(msg.data[11]);
        msg_pub.data.push_back(msg.data[12]);
        msg_pub.data.push_back(msg.data[13]);
        pub_usr_pin_x.publish(msg_pub);

    }

    void cbMCUOutput(const std_msgs::Int32MultiArray msg)
    {
        output_state = msg;
        output[0] = msg.data[0];
        output[1] = msg.data[1];
        output[2] = msg.data[2];
        output[3] = msg.data[3];
        std_msgs::Int8MultiArray msg_pub;
        msg_pub.data.push_back(msg.data[10]);
        msg_pub.data.push_back(msg.data[11]);
        msg_pub.data.push_back(msg.data[12]);
        msg_pub.data.push_back(msg.data[13]);
        pub_usr_pin_y.publish(msg_pub);

        wait_data_comeup = true;
    }

    void attachInterrupt(ISRdata x)
    {
        if(x.input_address != -1)
        {
            if (x.mode == PublishMode::CHANGE)
            {
                rise(x);
                fall(x);
            }
            else if (x.mode == PublishMode::RISING)
            {
                rise(x);
            }
            else if (x.mode == PublishMode::FALLING)
            {
                fall(x);
            }
            else if (x.mode == PublishMode::INACTIVE)
            {
                inactive(x);
            }
            else if (x.mode == PublishMode::ACTIVE)
            {
                active(x);
            }
            else if (x.mode == PublishMode::NOT_PUBLISH)
            {
                ;
            }
            else
            {
                ;
            }
        }else;

        pub_bumper(x);
        
    }

    void rise(ISRdata x)
    {
        if(x.current_state != x.last_state)
        {
            if(x.current_state == active_state)
            {
                ROS_INFO("add %d", x.input_address);
                ROS_INFO("Rise Detect");
                set_robot_mode(x.action);
                pub_data(x.data);
                
            }
        }
        
    }

    void fall(ISRdata x)
    {
        if(x.current_state != x.last_state)
        {
            if(x.current_state == inactive_state)
            {
                ROS_INFO("add %d", x.input_address);
                ROS_INFO("Fall Detect");
                set_robot_mode(x.action);
                pub_data(x.data);
            }
        }

        // pub_bumper(x.data, false);
    }

    void inactive(ISRdata x)
    {
        if(x.current_state == inactive_state)
        {
            ROS_INFO("add %d", x.input_address);
            set_robot_mode(x.action);
            pub_data(x.data);
        }

        // pub_bumper(x.data, false);
    }

    void active(ISRdata x)
    {
        // bool bumper_force = false;
        if(x.current_state == active_state)
        {
            // pub_bumper(x.data, true);
            ROS_INFO("add %d", x.input_address);
            set_robot_mode(x.action);
            pub_data(x.data);
        }
        // else
        // {
        //     pub_bumper(x.data, false);
        // }
        
    }

    bool pub_bumper(ISRdata x)
    {
        std_msgs::Bool msg;
        if(use_input_bumper_state)
        {
            if(x.data.find("Bumper") != -1)
            {
                if(x.current_state == active_state)
                {
                    msg.data = true;
                }
                else
                {
                    msg.data = false;
                }   
                pub_bumper_state.publish(msg);
            }
        }
        else
        {
            msg.data = false;
            pub_bumper_state.publish(msg);
        }
        
        
        
    }

    void pub_data(std::string data)
    {
        // int num = 123;
        // std::string str = std::to_string(num);
        char STX='\x02';
        char ETX='\x03';
        char NL = '\n';
        std_msgs::String msg;

        
        if(use_STX_ETX && use_newline)
        {
            msg.data = STX + str_header + data + str_footer + ETX + NL;
        }
        else if(use_STX_ETX && !use_newline)
        {
            msg.data = STX + str_header + data + str_footer + ETX;
        }
        else if(!use_STX_ETX && use_newline)
        {
            msg.data = str_header + data + str_footer + NL;
        }
        else
        {
             msg.data = str_header + data + str_footer;
        }
    
        pub_raw_read.publish(msg);
    }

    void set_robot_mode(int mode)
    {
            bool no_action = false;
            dynamic_reconfigure::Reconfigure dyc;
            dynamic_reconfigure::IntParameter set_robot_mode; 
            dynamic_reconfigure::BoolParameter trig_stop_all;
            dynamic_reconfigure::BoolParameter trig_stop_movement;
            ROS_INFO("mode %d", mode);
            
            if(mode == ActionMode::STOP_MOVEMENT)
            {
                // dynamic_reconfigure::BoolParameter trig_stop_movement;
                trig_stop_movement.name = "stop_movement";
                trig_stop_movement.value = true;
                dyc.request.config.bools.push_back(trig_stop_movement);
            }
            else if(mode == ActionMode::STOP_ALL)
            {
                // dynamic_reconfigure::BoolParameter trig_stop_all;
                trig_stop_all.name = "stop_all";
                trig_stop_all.value = true;
                dyc.request.config.bools.push_back(trig_stop_all);
            }
            else if(mode == ActionMode::PAUSE)
            {
                // ROS_INFO("GoPause");
                // dynamic_reconfigure::IntParameter set_robot_mode; 
                set_robot_mode.name = "robot_mode";
                set_robot_mode.value = matrix_msgs::RobotMode::PAUSE; //clear all state 
                dyc.request.config.ints.push_back(set_robot_mode);
            }
            else if(mode == ActionMode::RESUME)
            {
                // dynamic_reconfigure::IntParameter set_robot_mode; 
                set_robot_mode.name = "robot_mode";
                set_robot_mode.value = 0; //clear all state 
                dyc.request.config.ints.push_back(set_robot_mode);
            }
            else if(mode == ActionMode::EMERGENCY_CASE)
            {
                // dynamic_reconfigure::IntParameter set_robot_mode; 
                set_robot_mode.name = "robot_mode";
                set_robot_mode.value = matrix_msgs::RobotMode::EMERGENCY_CASE_ACTIVE;
                dyc.request.config.ints.push_back(set_robot_mode);
            }
            else if(mode == ActionMode::DOCKING)
            {
                // dynamic_reconfigure::IntParameter set_robot_mode; 
                set_robot_mode.name = "robot_mode";
                set_robot_mode.value = matrix_msgs::RobotMode::DOCKING_MODE_ON;
                dyc.request.config.ints.push_back(set_robot_mode);
            }
            else
            {
                no_action = true;
                ROS_INFO("[NONE 0] no_action");
            }

            if(!no_action)
            {
                if(set_robot_mode.name == "robot_mode")
                {
                    ROS_INFO("set robot_mode to --> %d", set_robot_mode.value);
                    if(current_robotmode != set_robot_mode.value)
                    {
                        set_mode_sc.call(dyc);
                        ROS_INFO("set_mode success");
                    }
                    else
                    {
                        ROS_INFO("robot mode is already");
                    }
                }
                else
                {
                    ROS_INFO("set action_mode to --> %s", (mode == ActionMode::STOP_MOVEMENT)? "stop_movement": "stop_all");
                    set_mode_sc.call(dyc);
                }
            }
            else
            {
                ;
            }
            
        
        
        
    }




};

int main (int argc, char **argv)
{

    ros::init(argc, argv, "matrix_management_io");

    ManagementIO managementIO;

    ros::spin();

    return 0;
}