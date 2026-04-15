// use with matrix IO v6.0 

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32MultiArray.h>

#include <matrix_msgs/Diagnostics.h>
#include <matrix_msgs/RobotMode.h>
#include <sensor_msgs/BatteryState.h>
#include <iostream>

#define MCU_SIGNAL_ACTIVE 0
class MatrixModeController {

    private:
    int counter;
    ros::Publisher pub_mode;
    ros::Subscriber diagnostics_subscriber, 
                    sub_emer_charge,
                    sub_emer_state,
                    sub_batt_state,
                    sub_input,
                    sub_output;
    ros::ServiceServer docking_service, pause_service;

    int _isSystemReady;

    bool _isDocking_mode;
    bool _isEmerCharge_mode;
    int _isEmerState_mode;
    bool _isCharging = false;
    bool _isPause = false;

    matrix_msgs::RobotMode mode_msg, current_mode;

    std::string serial_no, model;

    bool _isMCUMaster_on,
        _isMCUMaster_on_done,
        _isMCUEmergency,
        _isMCUEmergency_charge,
        _isMCUComputer_ready,
        _isMCUShutdown_robot,
        _isMCUEnable_charger,
        _isMCUBumper;

    bool _isInit_state = false;
    bool _isSystemReady_comeup = false;
    bool _isErrorDevice = false;
    bool _isEmergency_case_active = false;
    bool _isEnableEmerCharg = true;

    public:
    MatrixModeController(ros::NodeHandle *nh) {
        counter = 0;

        pub_mode = nh->advertise<matrix_msgs::RobotMode>("/matrix_mode_controller/mode", 1);   

        diagnostics_subscriber = nh->subscribe("/matrix_system/diagnostics/system_ready", 1, 
            &MatrixModeController::cbDiag, this);
        sub_emer_charge = nh->subscribe("/matrix_system/diagnostics/emer_charge", 1, 
            &MatrixModeController::cbEmerCharge, this);
        sub_emer_state = nh->subscribe("/matrix_io/emergency", 1, 
            &MatrixModeController::cbEmerState, this);
        sub_batt_state = nh->subscribe("battery_state", 1,
            &MatrixModeController::cbBatteryState, this);
        
        sub_input = nh->subscribe("/matrix_io/input", 1,&MatrixModeController::cbInput, this);
        
        sub_output = nh->subscribe("matrix_io/output", 1,
            &MatrixModeController::cbOutput, this);


        

        docking_service = nh->advertiseService("/matrix_mode_controller/docking_mode", 
            &MatrixModeController::cbDocking, this);

        pause_service = nh->advertiseService("/matrix_mode_controller/pause_mode", 
            &MatrixModeController::cbPause, this);

        
        ros::param::param<std::string>("~serial_no", serial_no, "DELI010020220001A");
        ros::param::param<std::string>("~model", model, "delivery");

        ros::param::param<bool>("~enable_emergency_charge_state", _isEnableEmerCharg, true);

        //init data 
        _isSystemReady = matrix_msgs::Diagnostics::SYSTEM_READY_WAIT_COMEUP;
        _isEmerCharge_mode = false;
        _isEmerState_mode = 1;
        _isDocking_mode = false;
        _isPause = false;



        ros::Rate loop_rate(10);


        while(ros::ok())
        {   
            if(model == "rohm")
            {
                mode_manager();
            }
            else
            {
                // mode manager for modern Matrix [delivery, deliverTrue]
                mode_manager2();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

    }

    bool invertMCUState(int signal, int active_signal = MCU_SIGNAL_ACTIVE)
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

    void cbInput(const std_msgs::Int32MultiArray &x)
    {
        if((model == "delivery") || (model == "deliveryTrue"))
        {
            _isMCUEmergency = invertMCUState(x.data[0]);
            _isMCUMaster_on = invertMCUState(x.data[1]);
            _isMCUMaster_on_done = invertMCUState(x.data[2]);
            _isMCUEmergency_charge = (_isEnableEmerCharg)? invertMCUState(x.data[3], 1) : false;
        }else;

        // ROS_INFO("emer state %d", x.data[0]);
    }

    void cbOutput(const std_msgs::Int32MultiArray &y)
    {
        if((model == "delivery") || (model == "deliveryTrue"))
        {
            _isMCUComputer_ready = invertMCUState(y.data[0], 1);
            _isMCUShutdown_robot = invertMCUState(y.data[1], 1);
            _isMCUEnable_charger = invertMCUState(y.data[3], 1);
        }else;
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

    bool cbDocking(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data) {
            _isDocking_mode = true;
            _isEmerCharge_mode = true;
            res.success = true;
            res.message = "[MatrixModeController]:Docking enable";

            //for mode_manager2() modern Matrix
            current_mode.robot_mode = matrix_msgs::RobotMode::DOCKING_MODE_ON;
            
        }
        else {
            _isDocking_mode = false;
            _isEmerCharge_mode = false;
            res.success = true;
            res.message = "[MatrixModeController]:Docking disable";

            //for mode_manager2() modern Matrix
            current_mode.robot_mode = matrix_msgs::RobotMode::DOCKING_MODE_OFF;
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

    void mode_manager()
    {
        using namespace matrix_msgs;
        if(_isEmerState_mode == 1)
        {
            if(!_isEmerCharge_mode)
            {
                // printf("!_isEmerCharge_mode\n");
                switch(_isSystemReady)
                {
                    case Diagnostics::SYSTEM_READY_STATUS_OK:
                        // printf("SYSTEM_READY_STATUS_OK\n");
                        if(_isEmerState_mode == 1)
                        {
                            if(!_isPause)
                            {
                                // printf("MOTOR\n");
                                mode_msg.robot_mode = RobotMode::START_MOTOR;
                            }
                            else
                            {
                                mode_msg.robot_mode = RobotMode::PAUSE;
                            }
                            
                        }
                        else
                        {
                            mode_msg.robot_mode = RobotMode::EMERGENCY;
                            clear_state();
                        }
                        break;
                    
                    case Diagnostics::SYSTEM_READY_NOT_USE:
                        clear_state();
                        break;
                    
                    case Diagnostics::SYSTEM_READY_STATUS_FAULT:
                        mode_msg.robot_mode = RobotMode::ERROR_DEVICE;
                        clear_state();
                        break;

                    case Diagnostics::SYSTEM_READY_WAIT_COMEUP:
                        mode_msg.robot_mode = RobotMode::IDLE;
                        clear_state();
                        break;
                }
            }
            else
            {
                mode_msg.robot_mode = RobotMode::EMERGENCY_CHARGE;
                clear_state();
            }
        }
        else
        {
            mode_msg.robot_mode = RobotMode::EMERGENCY;
            clear_state();
        }
        

        pub_mode.publish(mode_msg);
    }

    // mode manager for modern Matrix [delivery, deliverTrue]
    void mode_manager2()
    {
        using namespace matrix_msgs;

        // init robot state
        if(!_isInit_state)
        {
            if(init_state_func())
            {
                _isInit_state = true;
            }
            else
            {
                _isInit_state = false;
            }
        }

        check_MasaterOn();

        // ROS_INFO("_isMCUEmergency %d | _isMCUMaster_on_done %d | _isMCUEmergency_charge %d", _isMCUEmergency, _isMCUMaster_on_done, _isMCUEmergency_charge);
        if(_isSystemReady == Diagnostics::SYSTEM_READY_STATUS_OK)
        {
            if(!_isMCUEmergency_charge)
            {
                if(!_isMCUEmergency)
                {
                    if(!_isEmergency_case_active)
                    {
                        if(!_isPause)
                        {
                            mode_msg.robot_mode = current_mode.robot_mode;
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
            }else
            {
                mode_msg.robot_mode = RobotMode::EMERGENCY_CHARGE;
                _isInit_state = false;
            }
        }
        else if(_isSystemReady == Diagnostics::SYSTEM_READY_WAIT_COMEUP)
        {
            ;
        }
        else
        {
            // ROS_INFO("error device main");
            mode_msg.robot_mode = RobotMode::ERROR_DEVICE;
            _isInit_state = false;
        }

        // ROS_INFO("_isSystemReady %d", _isSystemReady);
        // ROS_INFO("mode_msg %d", mode_msg);
        pub_mode.publish(mode_msg);

    }

    void clear_emer_state()
    {
        _isEmergency_case_active = false;
        _isPause = false;
        _isDocking_mode = false;
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
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "matrix_mode_controller");
    ros::NodeHandle nh;
    MatrixModeController nc = MatrixModeController(&nh);
    ros::spin();
}