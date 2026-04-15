// use with matrix IO v6.0 

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

#include <matrix_msgs/Diagnostics.h>
#include <matrix_msgs/RobotMode.h>
#include <sensor_msgs/BatteryState.h>
#include <iostream>
#include <matrix_msgs/ActionController.h>
#include <matrix_msgs/SetIOs.h>



#include <dynamic_reconfigure/server.h>
#include <matrix_system/MatrixModeControllerConfig.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/Odometry.h>

#define MCU_SIGNAL_ACTIVE 0

using namespace matrix_msgs;

class MatrixModeController {

    private:
        ros::NodeHandle nh_;
        int counter;
        ros::Publisher pub_mode, pub_allowed_master_on;
        ros::Subscriber diagnostics_subscriber, 
                        sub_emer_charge,
                        sub_emer_state,
                        sub_batt_state,
                        sub_input,
                        sub_output,
                        sub_raw_read,
                        sub_emergency_charge,
                        sub_bumper,
                        sub_odom;
        ros::ServiceServer docking_service, pause_service, set_mode_service;
        ros::ServiceClient set_ios_sc, action_launch_sc, action_launch2_sc;

        int _isSystemReady;

        // bool _isDocking_mode;
        bool _isEmerCharge_mode;
        
        int _isEmerState_mode;
        // bool _isCharging = false;
        // bool _isPause = false;
        // bool _isOn = false;

        bool _isErrorDevice_state = false;
        // bool _isEmerCharge_state = false;
        // bool _isEmergency_state = false;
        // bool _isEmergency_case_active = false;




        matrix_msgs::RobotMode mode_msg, current_mode;

        std::string serial_no, model;

        bool _isMCUMaster_on = false;
        bool _isMCUMaster_on_done = false;
        bool    _isMCUEmergency = false;
        bool    _isMCUEmergencyPlugin = false;
        bool    _isMCUEmergency_charge = false;
        bool    _isMCUComputer_ready = false;
        bool    _isMCUShutdown_robot= false;
        bool    _isMCUEnable_charger = false;
        bool    _isMCUBumper = false;

        bool _isSOFTWAREEmergency_charge = false;

        bool _isInit_state = false;
        bool _isInit_motor = false;
        bool _isSystemReady_comeup = false;
        bool _isErrorDevice = false;
        bool _isEmergency_state = false;
        bool _isEmergency_state_prev = false;
        bool _isEmerCharge_state = false;
        bool _isEmergency_case_active = false;
        bool enable_emergency_charge_state = false;
        bool _isDocking_mode = false;
        bool _isOperating = false;
        bool _isShutdown = false;
        bool _isCharging = false;
        bool _isPause = false;
        bool _isOn = false;
        bool _isMasterOnSys = false;
        bool _isBumperPush = false;

        int emergency_charge_active_value = 0;
        int emergency_plugin_active_value = 0;
        int emergency_base_active_value = 0;
        int prev_robot_mode = 0;
        int before_robot_mode = 0;

        bool permissive_on_when_bumper_detect = true;
        geometry_msgs::Twist twist;

        // bool check_error_device();
        // void mode_manager3();
        // void check_ButtonState();

        enum operation_sequence{  
                                init_io,
                                check_sysready,
                                check_button_state,
                                check_emer_charge,
                                pub_botton_state,
                                control_state
        };
        operation_sequence operation_sequence_ = init_io;

        dynamic_reconfigure::Server<matrix_system::MatrixModeControllerConfig> dyn_srv_;
        std_msgs::Bool allowed_master_on_msg;

    public:
    MatrixModeController() {
        counter = 0;

        pub_mode = nh_.advertise<matrix_msgs::RobotMode>("/matrix_mode_controller/mode", 1); 
        pub_allowed_master_on =  nh_.advertise<std_msgs::Bool>("/matrix_mode_controller/allowed_master_on", 1);  

        diagnostics_subscriber = nh_.subscribe("/matrix_system/diagnostics/system_ready", 1, 
            &MatrixModeController::cbDiag, this);
        sub_emer_charge = nh_.subscribe("/matrix_system/diagnostics/emer_charge", 1, 
            &MatrixModeController::cbEmerCharge, this);
        sub_emer_state = nh_.subscribe("/matrix_io/emergency", 1, 
            &MatrixModeController::cbEmerState, this);
        sub_batt_state = nh_.subscribe("battery_state", 1,
            &MatrixModeController::cbBatteryState, this);
        
        sub_input = nh_.subscribe("/matrix_io/input", 1,&MatrixModeController::cbInput, this);
        
        sub_output = nh_.subscribe("matrix_io/output", 1,
            &MatrixModeController::cbOutput, this);

        sub_emer_charge = nh_.subscribe("/matrix_system/diagnostics/emergency_charge", 1,
            &MatrixModeController::cbEmerChargeState, this);

        // sub_emer_charge = nh_.subscribe("/matrix_io_management/bumper", 1,
        //     &MatrixModeController::cbBumper, this);
        
        sub_bumper = nh_.subscribe("/matrix_io_management/bumper", 1, &MatrixModeController::cbBumper, this);

        // sub_raw_read = nh_.subscribe("/raw_read", 1, &MatrixModeController::cbRawRead, this);

        sub_odom = nh_.subscribe("/odom", 1, &MatrixModeController::cbOdom, this);

        

        docking_service = nh_.advertiseService("/matrix_mode_controller/docking_mode", 
            &MatrixModeController::cbDocking, this);

        pause_service = nh_.advertiseService("/matrix_mode_controller/pause_mode", 
            &MatrixModeController::cbPause, this);

        set_mode_service = nh_.advertiseService("/matrix_mode_controller/set_mode",
            &MatrixModeController::cbSetMode, this);
        
        set_ios_sc = nh_.serviceClient<matrix_msgs::SetIOs>("matrix_io/service");
        action_launch_sc = nh_.serviceClient<matrix_msgs::ActionController>("matrix_launch_controller/action_controller");
        action_launch2_sc = nh_.serviceClient<std_srvs::SetBool>("matrix_launch_controller/cancel_action");
        
        ros::param::param<std::string>("~serial_no", serial_no, "DELI010020220001A");
        ros::param::param<std::string>("~model", model, "delivery");

        

        ros::param::param<bool>("~enable_emergency_charge_state", enable_emergency_charge_state, true);
        ros::param::param<int>("~emergency_charge_active_value", emergency_charge_active_value, 0);
        ros::param::param<int>("~emergency_plugin_active_value", emergency_plugin_active_value, 0);
        ros::param::param<int>("~emergency_base_active_value", emergency_base_active_value, 0);
        ros::param::param<bool>("~permissive_on_when_bumper_detect", permissive_on_when_bumper_detect, true);
        

        //init data 
        _isSystemReady = matrix_msgs::Diagnostics::SYSTEM_READY_WAIT_COMEUP;
        _isEmerCharge_mode = false;
        _isEmerState_mode = 1;
        _isDocking_mode = false;
        _isPause = false;

        ROS_INFO("[MatrixModeController]: wait matrix_io comeup... ");
        while(!set_ios_sc.waitForExistence())
        {
            break;
        }

        ros::Duration(1).sleep();

        ROS_INFO("[MatrixModeController]: matrix_io ready !!!");




        dyn_srv_.setCallback(boost::bind(&MatrixModeController::callback, this, _1, _2));



        ros::Rate loop_rate(25);

        


        while(ros::ok())
        {   
            // if(model == "rohm")
            // {
            //     mode_manager();
            // }
            // else
            // {
            //     // mode manager for modern Matrix [delivery, deliverTrue]
            //     mode_manager2();
            // }

            // ROS_INFO("[MatrixModeController]: goo !!!");

            state_controller_sequence();

            ros::spinOnce();
            loop_rate.sleep();
        }

    }

    void callback(matrix_system::MatrixModeControllerConfig &config, uint32_t level)
    {
        enable_emergency_charge_state = config.enable_emergency_charge_state;
        emergency_charge_active_value = config.emergency_charge_active_value;
        emergency_plugin_active_value = config.emergency_plugin_active_value;
        emergency_base_active_value = config.emergency_base_active_value;

        matrix_msgs::ActionController srv;

        if(config.stop_movement)
        {
            srv.request.cmd = "STOP_MOVEMENT";
            cbSetMode(srv.request, srv.response);
            config.stop_movement = false;
        }

        if(config.stop_all)
        {
            srv.request.cmd = "STOP_ALL";
            cbSetMode(srv.request, srv.response);
            config.stop_all = false;
        }
        

        if(config.robot_mode == 0)
        {
            srv.request.cmd = "NORMAL";
            srv.request.arg0 = "ACTIVE";
        }
        else if(config.robot_mode == matrix_msgs::RobotMode::DOCKING_MODE_ON)
        {
            srv.request.cmd = "DOCKING";
            srv.request.arg0 = "ACTIVE";
        }
        else if(config.robot_mode == matrix_msgs::RobotMode::PAUSE)
        {
            srv.request.cmd = "PAUSE";
            srv.request.arg0 = "ACTIVE";
        }
        else if(config.robot_mode == matrix_msgs::RobotMode::RESUME)
        {
            srv.request.cmd = "PAUSE";
            srv.request.arg0 = "INACTIVE";
        }
        else if(config.robot_mode == matrix_msgs::RobotMode::EMERGENCY_CASE_ACTIVE)
        {
            srv.request.cmd = "EMERGENCY_CASE";
            srv.request.arg0 = "ACTIVE";
        }
        else;
        cbSetMode(srv.request, srv.response);

    }

    void cbOdom(const nav_msgs::Odometry msg)
    {
        twist = msg.twist.twist;
    }

    void cbBumper(const std_msgs::Bool msg)
    {
        _isBumperPush = msg.data;
    }

    bool invertMCUState(int signal, int active_signal = MCU_SIGNAL_ACTIVE)
    {   
        if(active_signal == -1)
        {
            return false;
        }
        else
        {
            if(signal == active_signal)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        
    }

    void cbEmerChargeState(const std_msgs::Bool &msg)
    {
        _isSOFTWAREEmergency_charge = msg.data;
    }

    void cbRawRead(const std_msgs::String &msg)
    {
        if(msg.data.find("Bumper") != -1)
        {
            _isBumperPush = true;
        }
    }

    void cbInput(const std_msgs::Int32MultiArray &x)
    {
        // if((model == "delivery") || (model == "deliveryTrue"))
        // {
            _isMCUEmergency = invertMCUState(x.data[0], emergency_base_active_value);
            _isMCUMaster_on = invertMCUState(x.data[1]);
            _isMCUMaster_on_done = invertMCUState(x.data[2]);
            _isMCUEmergency_charge = (enable_emergency_charge_state)? invertMCUState(x.data[3], emergency_charge_active_value) : false;
            _isMCUEmergencyPlugin = invertMCUState(x.data[4], emergency_plugin_active_value);
        // }else;

        // ROS_INFO("emer state %d", x.data[0]);
    }

    void cbOutput(const std_msgs::Int32MultiArray &y)
    {
        // if((model == "delivery") || (model == "deliveryTrue"))
        // {
            _isMCUComputer_ready = invertMCUState(y.data[0], 1);
            _isMCUShutdown_robot = invertMCUState(y.data[1], 1);
            _isMCUEnable_charger = invertMCUState(y.data[3], 1);
        // }else;
    }

    void cbBatteryState(const sensor_msgs::BatteryState &msg)
    {
        if(msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
        {
            _isCharging = true;
            
        }
        else
        {
            _isCharging = false;
            
        }
    }

    void cbEmerState(const std_msgs::Int8& msg)
    {
        _isEmerState_mode = msg.data;
    }

    void cbDiag(const matrix_msgs::Diagnostics& msg) {
        _isSystemReady = msg.system_ready;
        // ROS_INFO("_isSystemReady %d", _isSystemReady);
        // if(_isSystemReady == matrix_msgs::RobotMode::SYSTEM_READY_STATUS_FAULT)
        // {
        //     _isErrorDevice = true;
        // }
        // else if(_isSystemReady == matrix_msgs)
        _isSystemReady_comeup = true;
    }

    void cbEmerCharge(const std_msgs::Bool& msg){
        _isEmerCharge_mode = msg.data;
    }


    bool cbSetMode(matrix_msgs::ActionController::Request &req, matrix_msgs::ActionController::Response &res)
    {
        if(req.cmd == "NORMAL")
        {
            if(req.arg0 == "ACTIVE") 
            {
                _isPause = false;
                _isDocking_mode = false;
                // _isEmerCharge_mode = false;
                _isEmergency_case_active = false;
                _isOn = false;
                res.success = true;
                res.message = "[MatrixModeController]:Normal mode";
                _isInit_state = false;
            }
            else 
            {
                // _isPause = false;
                res.success = true;
                res.message = "[MatrixModeController]:Normal mode disable";

                //for mode_manager2() modern Matrix
                // _isInit_state = false;
            }
        }
        else if(req.cmd == "STOP_MOVEMENT")
        {
            matrix_msgs::ActionController srv;
            srv.request.cmd = "action_cancel";
            srv.request.action_name = "movement";
            if(action_launch_sc.call(srv))
            {
                if(srv.response.success)
                {
                    res.success = true;
                }
                else
                {
                    res.success = false;
                }
                res.message = srv.response.message;
            }

        }
        else if(req.cmd == "STOP_ALL")
        {
            matrix_msgs::ActionController srv;
            srv.request.cmd = "action_cancel";
            srv.request.action_name = "STOP_ALL";
            if(action_launch_sc.call(srv))
            {
                if(srv.response.success)
                {
                    res.success = true;
                }
                else
                {
                    res.success = false;
                }
                res.message = srv.response.message;
            }

            std_srvs::SetBool srv2;
            srv2.request.data = true;
            if(action_launch2_sc.call(srv2))
            {
                if(srv2.response.success)
                {
                    res.success = true;
                }
                else
                {
                    res.success = false;
                }
                res.message = srv2.response.message;
            }

            ROS_WARN("[MatrixModeController]:cancel");
        }
        else if(req.cmd == "PAUSE")
        {
            if(req.arg0 == "ACTIVE") 
            {
                _isPause = true;
                // _isDocking_mode = false;
                // _isEmerCharge_mode = false;
                res.success = true;
                res.message = "[MatrixModeController]:Pause enable";
            }
            else 
            {
                // _isPause = false;
                res.success = false;
                res.message = "[MatrixModeController]:Pause disable";

                // //for mode_manager2() modern Matrix
                // _isInit_state = false;
            }
        }
        else if(req.cmd == "RESUME")
        {   
            if(req.arg0 == "ACTIVE") 
            {
                if(permissive_on_when_bumper_detect)
                {
                    _isPause = false;
                    // _isDocking_mode = false;
                    // _isEmerCharge_mode = false;
                    _isInit_state = false;
                    res.success = true;
                    _isOn = false;
                    res.message = "[MatrixModeController]:Resume enable";
                }
                else
                {
                    if(!_isBumperPush)
                    {
                        _isPause = false;
                        // _isDocking_mode = false;
                        // _isEmerCharge_mode = false;
                        _isInit_state = false;
                        res.success = true;
                        _isOn = false;
                        res.message = "[MatrixModeController]:Resume enable";
                    }
                }
                
            }
            else 
            {
                // _isPause = true;
                res.success = true;
                res.message = "[MatrixModeController]:Resume disable";

                //for mode_manager2() modern Matrix
                _isInit_state = false;
            }
        }
        else if(req.cmd == "DOCKING")
        {
            if(req.arg0 == "ACTIVE")
            {
                if(_isMCUMaster_on_done)
                {
                    if(allowed_master_on_msg.data)
                    {
                        _isDocking_mode = true;
                        // _isEmerCharge_mode = true;
                        res.success = true;
                        res.message = "[MatrixModeController]:Docking enable";

                        ROS_INFO("Docking mode trigger");

                        //for mode_manager2() modern Matrix
                        current_mode.robot_mode = matrix_msgs::RobotMode::DOCKING_MODE_ON;
                    }else;
                }
                else 
                {
                    _isDocking_mode = false;
                    _isOn = false;
                    // _isEmerCharge_mode = false;
                    res.success = false;
                    res.message = "[MatrixModeController]:Docking disable";
                    

                    //for mode_manager2() modern Matrix
                    // current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
                    _isInit_state = false;

                    ROS_ERROR("Press master on done");
                }
            }
            else
            {
                _isDocking_mode = false;
                _isOn = false;
                // _isEmerCharge_mode = false;
                res.success = true;
                res.message = "[MatrixModeController]:Docking disable";


                //for mode_manager2() modern Matrix
                // current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
                _isInit_state = false;
            }
            
            
        }
        else if(req.cmd == "EMERGENCY_CASE")
        {
            
            if(req.arg0 == "ACTIVE") 
            {
                _isEmergency_case_active = true;
                _isPause = false;
                _isDocking_mode = false;
                // _isEmerCharge_mode = false;
                _isInit_state = false;
                res.success = true;
                // _isOn = false;
                res.message = "[MatrixModeController]:EMERGENCY_CASE enable";
            }
            else 
            {
                // _isPause = true;
                res.success = true;
                res.message = "[MatrixModeController]:EMERGENCY_CASE not accept";

                //for mode_manager2() modern Matrix
                // _isInit_state = false;
            }

        }
        else 
        {
            ;
        }

        return true;
    }

    

    bool cbDocking(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data) {

            if(_isMCUMaster_on_done)
            {
                _isDocking_mode = true;
                // _isEmerCharge_mode = true;
                res.success = true;
                res.message = "[MatrixModeController]:Docking enable";

                ROS_INFO("Docking mode trigger");

                //for mode_manager2() modern Matrix
                current_mode.robot_mode = matrix_msgs::RobotMode::DOCKING_MODE_ON;
            }
            else
            {
                _isDocking_mode = false;
                _isOn = false;
                // _isEmerCharge_mode = false;
                res.success = false;
                res.message = "[MatrixModeController]:Docking disable";
                

                //for mode_manager2() modern Matrix
                // current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
                _isInit_state = false;

                ROS_ERROR("Press master on done");
            }
            
            
        }
        else {
            _isDocking_mode = false;
            _isOn = false;
            // _isEmerCharge_mode = false;
            res.success = true;
            res.message = "[MatrixModeController]:Docking disable";


            //for mode_manager2() modern Matrix
            // current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
            _isInit_state = false;
        }
        return true;
    }

    bool cbPause(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data) {
            _isPause = true;
            _isDocking_mode = false;
            _isEmerCharge_mode = false;
            res.success = true;
            res.message = "[MatrixModeController]:Pause enable";
        }
        else {
            _isPause = false;
            res.success = true;
            res.message = "[MatrixModeController]:Pause disable";

            //for mode_manager2() modern Matrix
            _isInit_state = false;
        }
        return true;
    }

    void clear_state()
    {
        _isPause = false;
        // _isDocking_mode = false;
        // _isEmerCharge_mode = false;
    }

    // void mode_manager()
    // {
    //     using namespace matrix_msgs;
    //     if(_isEmerState_mode == 1)
    //     {
    //         if(!_isEmerCharge_mode)
    //         {
    //             printf("!_isEmerCharge_mode\n");
    //             switch(_isSystemReady)
    //             {
    //                 case Diagnostics::SYSTEM_READY_STATUS_OK:
    //                     // printf("SYSTEM_READY_STATUS_OK\n");
    //                     if(_isEmerState_mode == 1)
    //                     {
    //                         if(!_isPause)
    //                         {
    //                             // printf("MOTOR\n");
    //                             mode_msg.robot_mode = RobotMode::START_MOTOR;
    //                         }
    //                         else
    //                         {
    //                             mode_msg.robot_mode = RobotMode::PAUSE;
    //                         }
                            
    //                     }
    //                     else
    //                     {
    //                         mode_msg.robot_mode = RobotMode::EMERGENCY;
    //                         clear_state();
    //                     }
    //                     break;
                    
    //                 case Diagnostics::SYSTEM_READY_NOT_USE:
    //                     clear_state();
    //                     break;
                    
    //                 case Diagnostics::SYSTEM_READY_STATUS_FAULT:
    //                     mode_msg.robot_mode = RobotMode::ERROR_DEVICE;
    //                     clear_state();
    //                     break;

    //                 case Diagnostics::SYSTEM_READY_WAIT_COMEUP:
    //                     mode_msg.robot_mode = RobotMode::IDLE;
    //                     clear_state();
    //                     break;
    //             }
    //         }
    //         else
    //         {
    //             mode_msg.robot_mode = RobotMode::EMERGENCY_CHARGE;
    //             clear_state();
    //         }
    //     }
    //     else
    //     {
    //         mode_msg.robot_mode = RobotMode::EMERGENCY;
    //         clear_state();
    //     }
        

    //     pub_mode.publish(mode_msg);
    // }

    // mode manager for modern Matrix [delivery, deliverTrue]
    // void mode_manager2()
    // {
    //     using namespace matrix_msgs;

    //     // init robot state
    //     if(!_isInit_state)
    //     {
    //         if(init_state_func())
    //         {
    //             _isInit_state = true;
    //         }
    //         else
    //         {
    //             _isInit_state = false;
    //         }
    //     }

    //     check_MasaterOn();

    //     // ROS_INFO("_isMCUEmergency %d | _isMCUMaster_on_done %d | _isMCUEmergency_charge %d", _isMCUEmergency, _isMCUMaster_on_done, _isMCUEmergency_charge);
    //     if(_isSystemReady == Diagnostics::SYSTEM_READY_STATUS_OK)
    //     {
    //         if(!_isMCUEmergency_charge)
    //         {
    //             if(!_isMCUEmergency)
    //             {
    //                 if(!_isEmergency_case_active)
    //                 {
    //                     if(!_isPause)
    //                     {
    //                         mode_msg.robot_mode = current_mode.robot_mode;
    //                     }
    //                     else
    //                     {
    //                         mode_msg.robot_mode = RobotMode::PAUSE;
    //                         _isInit_state = false;
    //                     }
    //                 }
    //                 else
    //                 {
    //                     mode_msg.robot_mode = RobotMode::EMERGENCY_CASE_ACTIVE;
    //                     _isInit_state = false;
    //                 }
    //             }
    //             else
    //             {
    //                 clear_emer_state();
    //                 // ROS_INFO("RobotMode::EMERGENCY");
    //                 mode_msg.robot_mode = RobotMode::EMERGENCY;
    //                 _isInit_state = false;
    //             }
    //         }else
    //         {
    //             mode_msg.robot_mode = RobotMode::EMERGENCY_CHARGE;
    //             _isInit_state = false;
    //         }
    //     }
    //     else if(_isSystemReady == Diagnostics::SYSTEM_READY_WAIT_COMEUP)
    //     {
    //         ;
    //     }
    //     else
    //     {
    //         // ROS_INFO("error device main");
    //         mode_msg.robot_mode = RobotMode::ERROR_DEVICE;
    //         _isInit_state = false;
    //     }

    //     // ROS_INFO("_isSystemReady %d", _isSystemReady);
    //     // ROS_INFO("mode_msg %d", mode_msg);
    //     pub_mode.publish(mode_msg);

    // }

    void clear_emer_state()
    {
        _isEmergency_case_active = false;
        _isPause = false;
        _isDocking_mode = false;

        if(_isMCUMaster_on_done && _isMasterOnSys)
        {
            matrix_msgs::SetIOs srv;
            srv.request.cmd = "circuit_reset";
            srv.request.arg0 = 1;
            set_ios_sc.call(srv);
        }
    }

    bool fall(bool current_state, bool prev_state)
    {
        bool detect = false;
        if(current_state != prev_state)
        {
            if(!current_state)
            {
                ROS_INFO("Fall Detect");
                detect = true;
            }
        }
        return detect;
    }



    bool init_state_func()
    {
        bool success =false;
        if(!_isMCUMaster_on_done)
        {
            if(!_isMCUMaster_on)
            {
                if(_isSystemReady == matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_OK)
                {
                    current_mode.robot_mode = matrix_msgs::RobotMode::READY_TO_START;
                    // ROS_INFO("system ready");
                    success = true;
                }else
                {
                    // ROS_INFO("error device");
                    success = false;
                }
            }else;
        }
        else
        {
            current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
            success = true;
        }

        return success;
    }
    

    bool init_state()
    {
        ;
    }

    void check_MasaterOn()
    {
        if(mode_msg.robot_mode == matrix_msgs::RobotMode::READY_TO_START)
        {
            if(_isMCUMaster_on_done)
            {
                // ROS_INFO("set master on mode");
                current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
            }
            
        }
        
    }

    void state_controller_sequence()
    {
        // ROS_INFO("[MatrixModeController]:state_controller_sequence!!!");

        switch(operation_sequence_)
        {
            case operation_sequence::init_io:
                static bool smr200_turnon_fan = false;
                ROS_INFO("[MatrixModeController]:set fan!!!");
                if((!smr200_turnon_fan))
                {
                    if(serial_no.find("SMR0200") != -1)
                    {
                        setIO("cooling_fan", 1);
                        smr200_turnon_fan = true;
                    }
                    

                    try {
                        matrix_msgs::SetIOs srv;
                        srv.request.cmd = "use_master_on_system";
                        if(set_ios_sc.call(srv))
                        {
                            if(srv.response.text == "0")
                            {
                                _isMasterOnSys = false;
                            }
                            else
                            {
                                _isMasterOnSys = true;
                            }
                        }
                    }
                    catch (...) {
                        ROS_ERROR("Error call cmd: use_master_on_system");
                        _isMasterOnSys = true;
                    }

                }

                // ROS_INFO("[MatrixModeController]:get_version!!!");

                

                // ROS_INFO("[MatrixModeController]: use_master_on_system : %d", _isMasterOnSys);
                    

                operation_sequence_ = operation_sequence::check_sysready;
                break;
            
            case operation_sequence::check_sysready:
                if(!check_error_device())
                {
                    operation_sequence_ = operation_sequence::check_emer_charge;
                }
                else
                {
                    operation_sequence_ = operation_sequence::check_emer_charge;
                }
                break;
            
            case operation_sequence::check_emer_charge:
                check_EmerCharge();
                if (!_isMasterOnSys)
                {
                    //Clear state in Robot doesn't have master on system
                    if(fall(_isEmergency_state, _isEmergency_state_prev) && _isMasterOnSys)
                    {
                        matrix_msgs::SetIOs srv;
                        srv.request.cmd = "circuit_reset";
                        srv.request.arg0 = 0;
                        set_ios_sc.call(srv);
                    }
                    _isEmergency_state_prev = _isEmergency_state;
                }
                operation_sequence_ = operation_sequence::check_button_state;
                break;
            
            case operation_sequence::check_button_state:
                if(_isSystemReady_comeup)
                {
                   check_ButtonState();
                }
                else;
                operation_sequence_ = operation_sequence::pub_botton_state;
                break;
            
            case operation_sequence::pub_botton_state:
                // get_io_state();
                operation_sequence_ = operation_sequence::control_state;
                break;
            
            case operation_sequence::control_state:
                mode_manager3();
                operation_sequence_ = operation_sequence::check_sysready;
                break;
        }
    }

    bool check_error_device()
    {
        // if(_isSystemReady_comeup)
        // {
        //     if(_isSystemReady == Diagnostics::SYSTEM_READY_STATUS_OK)
        //     {
        //         _isErrorDevice = false;
        //     }
        //     else
        //     { 
        //         // master on inactive
        //         // if(digitalRead(MASTER_ON_DONE_X02) == HIGH)
        //         // {
        //         //     ;
        //         // }
        //         // else
        //         // {
        //         //     emergency_mode();
        //         // }

        //         _isInit_state = false;
        //         _isErrorDevice = true;
        //         current_mode.robot_mode = RobotMode::ERROR_DEVICE;
        //     }
        // }else;

        return _isErrorDevice;
    }


    void check_EmerCharge()
    {
        if(_isMCUEmergency_charge || _isSOFTWAREEmergency_charge)
        {
            _isEmerCharge_state = true;
            if(_isMCUMaster_on_done && _isMasterOnSys)
            {
                matrix_msgs::SetIOs srv;
                srv.request.cmd = "circuit_reset";
                srv.request.arg0 = 1;
                set_ios_sc.call(srv);
            }
        }
        else
        {
            if(_isEmerCharge_state && _isMasterOnSys)
            {
                ROS_INFO("off circuit reset");
                matrix_msgs::SetIOs srv;
                srv.request.cmd = "circuit_reset";
                srv.request.arg0 = 0;
                set_ios_sc.call(srv);
                // _isEmerCharge_state = false;
            }
            _isEmerCharge_state = false;
            
        }
    }
    
    void check_ButtonState()
    {
        if(_isMCUEmergency || _isMCUEmergencyPlugin)
        {
            ROS_INFO("emerpush");
            _isInit_state = false;
            _isEmergency_case_active = false;
            _isOperating = false;
            _isEmergency_state = true;
            _isOn = false;
            current_mode.robot_mode = matrix_msgs::RobotMode::EMERGENCY;
        } 
        else
        {   
            if(!_isMasterOnSys)
            {
                _isEmergency_state = false;
            }
            
            if(!_isMCUMaster_on_done)
            {
                _isEmergency_state = false;

                // if(!_isInit_state)
                // {
                //     init_state();
                //     _isInit_state = true;
                // }else;
                
                // if(_isMCUMaster_on)
                // {
                //     ;
                // }
                // else
                // {
                //     if((_isSystemReady_comeup) && (!_isShutdown) && (!_isOn))
                //     {
                        if(_isSystemReady == Diagnostics::SYSTEM_READY_STATUS_OK)
                        {
                            if(_isMasterOnSys)
                            {
                                current_mode.robot_mode = matrix_msgs::RobotMode::READY_TO_START;
                            }
                            else;
                            _isOn = false;
                        }else;
                //     }else;
                // }
            }
            else
            {
                // if(mode_msg.robot_mode == matrix_msgs::RobotMode::READY_TO_START)
                // {
                    // ROS_INFO("robot mode %d", current_mode.robot_mode);

                    if((_isMCUMaster_on_done) && (!_isOn))
                    {
                        ROS_INFO("set master on mode");
                        current_mode.robot_mode = matrix_msgs::RobotMode::START_MOTOR;
                        _isOn = true;
                        _isInit_state = true;
                    }
                    else
                    {
                        ;
                    }
                    
                // }
            }
        }

        
    
    }

    void mode_manager3()
    {
        using namespace matrix_msgs;

        // check_ButtonState();

        
        allowed_master_on_msg.data = false;

        before_robot_mode = mode_msg.robot_mode;

        // ROS_INFO("_isMCUEmergency %d | _isMCUMaster_on_done %d | _isMCUEmergency_charge %d", _isMCUEmergency, _isMCUMaster_on_done, _isMCUEmergency_charge);
        if(_isSystemReady == Diagnostics::SYSTEM_READY_STATUS_OK)
        {
            // if(!_isErrorDevice_state)
            // {
                if(!_isEmerCharge_state)
                {
                    if(!_isEmergency_state)
                    {
                        if(!_isEmergency_case_active)
                        {
                            if(!_isPause)
                            {
                                ROS_INFO("LastMode---------***********---->>>");
                                
                                mode_msg.robot_mode = current_mode.robot_mode;
                                allowed_master_on_msg.data = true;
                                
                            }
                            else
                            {
                                mode_msg.robot_mode = RobotMode::PAUSE;
                                _isInit_state = false;
                            }
                        }
                        else
                        {
                            mode_msg.robot_mode = RobotMode::EMERGENCY_CASE_ACTIVE;
                            _isInit_state = false;
                        }
                    }
                    else
                    {
                        clear_emer_state();
                        // ROS_INFO("RobotMode::EMERGENCY");
                        mode_msg.robot_mode = RobotMode::EMERGENCY;
                        _isInit_state = false;
                    }
                    
                }
                else
                {
                    clear_emer_state();
                    // ROS_INFO("RobotMode::EMERGENCY");
                    mode_msg.robot_mode = RobotMode::EMERGENCY_CHARGE;
                    _isInit_state = false;
                }
            // }
            // else
            // {
            //     mode_msg.robot_mode = RobotMode::EMERGENCY_CHARGE;
            //     _isInit_state = false;
            // }
        }
        else if(_isSystemReady == Diagnostics::SYSTEM_READY_WAIT_COMEUP)
        {
            ;
        }
        else
        {
            clear_emer_state();
            ROS_INFO("error device main");
            mode_msg.robot_mode = RobotMode::ERROR_DEVICE;
            _isInit_state = false;
        }


        if(before_robot_mode != mode_msg.robot_mode)
        {
            prev_robot_mode = before_robot_mode;
        }
        

        if((mode_msg.robot_mode == RobotMode::READY_TO_START))
        {

                    // ROS_INFO("linear x:%f, angular z:%f %d", abs(twist.linear.x), abs(twist.angular.z), (abs(twist.linear.x) <= 0.01)? true:false);
                if((abs(twist.linear.x) <= 0.001) && (abs(twist.angular.z) <= 0.001))
                // if(abs(twist.linear.x) <= 0.00)
                {
                    
                    ROS_INFO("Allowed", twist.linear.x, twist.angular.z);
                    allowed_master_on_msg.data = true;
                }
                else
                {
                    allowed_master_on_msg.data = false;
                 
                }
            
        }
        else if((mode_msg.robot_mode == RobotMode::START_MOTOR) && (prev_robot_mode == RobotMode::PAUSE)) 
        {
            if(!_isInit_motor)
            {

                if((abs(twist.linear.x) <= 0.001) && (abs(twist.angular.z) <= 0.001))
                // if(abs(twist.linear.x) <= 0.00)
                {
                    
                    ROS_INFO("Allowed", twist.linear.x, twist.angular.z);
                    allowed_master_on_msg.data = true;
                    _isInit_motor = true;
                }
                else
                {
                    allowed_master_on_msg.data = false;
                    ROS_INFO("WAIT CMD == 0.000");
                }
            }
            else
            {
                ROS_INFO("_isInit_motor True;");
            }
        }
        else
        {
            _isInit_motor = false;
            // allowed_master_on_msg.data = true;
            // _isInit_motor = false;
        }




        // ROS_INFO("_isSystemReady %d", _isSystemReady);
        // ROS_INFO("mode_msg %d", mode_msg);
        ROS_INFO("allowed_master_on_msg %d", allowed_master_on_msg);
        pub_mode.publish(mode_msg);
        pub_allowed_master_on.publish(allowed_master_on_msg);
        
        

        ROS_INFO("prev_robot mode %d", prev_robot_mode);
        ROS_INFO("robot mode %d", mode_msg.robot_mode);

        ROS_INFO("", mode_msg.robot_mode);
        ROS_INFO("", mode_msg.robot_mode);
        ROS_INFO("", mode_msg.robot_mode);
        

        

        
    }


    bool setIO(std::string cmd, int value)
    {
        matrix_msgs::SetIOs srv;
        srv.request.cmd = cmd;
        srv.request.arg0 = value;
        bool result = set_ios_sc.call(srv);
        return result;
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "matrix_mode_controller");
    ros::NodeHandle nh;
    MatrixModeController nc;
    ros::spin();
}