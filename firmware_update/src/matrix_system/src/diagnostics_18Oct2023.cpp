#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>
#include <matrix_msgs/Diagnostics.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <matrix_msgs/Diagnostics_modern.h>
#include <stdio.h>

#include <dynamic_reconfigure/server.h>
#include <matrix_system/MatrixDiagnosticsConfig.h>

// #include <iostream.h>

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
                  EMERGENCY_CHARGE,
                  MANUAL_DOCKING,
                  ROBOT_OPERATION_BEGIN,
                  ROBOT_OPERATION_FINISH
                  };


enum DIAGNOSTIC_SEQUENCE{ INIT_HARDWARE_TYPE,
                          WAIT_ALL_DEVICES_COMEUP,
                          DIAGNOSTIC_DEVICE,
                          PUB_SYSTEM_READY,
                          DIAGNOSTIC_CHARGE,
                          DIAGNOSTIC_UVC_LIGHT,
                          DIAGNOSTIC_MOTOR_HEAT
                        };

class Diagnostic
{
    protected:
        ros::NodeHandle nh_;
        // ros::ServiceClient crsv_reset, csrv_uvc_mode, csrv_tablet_discon_mode, crsv_setCE, crsv_shutdown;
        ros::Publisher pub_diagnostics,
                       pub_diag_emer,
                       pub_diagnostics_modern;

        ros::Subscriber sub_lds_f,
                        sub_lds_r,
                        sub_imu,
                        sub_cam1,
                        sub_cam2,
                        sub_cam3,
                        sub_camrear,
                        sub_bms_base,
                        sub_bms_plugin,
                        sub_mcu1,
                        sub_mcu2,
                        sub_motor,
                        sub_zigbee,
                        sub_rs485_wireless,
                        sub_robot_mode,
                        sub_robot_status;

        int  _isLds_f,
             _isLds_r,
             _isMcu_1,
             _isMcu_2,
             _isIMU,
             _isBattery,
             _isMotordrive,
             _isCam1,
             _isCam2;
        
        bool _is_output = false;
        // robot_status::RobotStatus robot_status_;
        
        // diagnostic_msgs::DiagnosticArray  smr_diagnostics;
        // diagnostic_msgs::DiagnosticStatus imu_state,
        //                                   lds_f_state,
        //                                   lds_r_state
        //                                   mcu1_state,
        //                                   mcu2_state,
        //                                   batt_state,
        //                                   motor_state,
        //                                   cam1_state,
        //                                   cam2_state;
        
        double imu_recive_stamp_old_,
                lds_f_recive_stamp_old_,
                lds_r_recive_stamp_old_,
                mcu1_recive_stamp_old_,
                mcu2_recive_stamp_old_,
                bmsbase_receive_stamp_old_,
                motor_recive_stamp_old_,
                cam1_recive_stamp_old_,
                cam2_recive_stamp_old_,
                imu_recive_stamp_,
                lds_f_recive_stamp_,
                lds_r_recive_stamp_,
                mcu1_recive_stamp_,
                mcu2_recive_stamp_,
                bmsbase_receive_stamp_,
                motor_recive_stamp_,
                cam1_recive_stamp_,
                cam2_recive_stamp_;

        diagnostic_msgs::DiagnosticStatus   diag_imu,
                                            diag_lif,
                                            diag_lir,
                                            diag_mcu01,
                                            diag_mcu02,
                                            diag_bmsbase,
                                            diag_bmsplugin,
                                            diag_motor,
                                            diag_cam1,
                                            diag_cam2,
                                            diag_cam3,
                                            diag_camrear,
                                            diag_xbee,
                                            diag_wireless485;

        
        sensor_msgs::BatteryState battery_state;

        bool use_automatic_shutdown = true;
        int timeout_error_device = 10;
        int timeout_error_system = 20;
        // int TIMEOUT_RATE_CURRENT_HEAT = 60;
        int timeout_noaction_shutdown = 10800; //sec
        int timeout_full_charge_shutdown = 600; //sec
        // float MOTOR_HEAT_CURRENT = 8.0;
        std::string serial_no = "20210001B";
        std::string model= "smr200";

        std::string ldsf_topic = "";
        std::string ldsr_topic = "";
        std::string mcu1_topic = "";
        std::string mcu2_topic = "";
        std::string imu_topic = "";
        std::string bms_base_topic = "";
        std::string cam1_topic = "";
        std::string cam2_topic = "";
        std::string motor_topic = "";
        std::string bms_plugin_topic = "";
        std::string cam3_topic = "";
        std::string camrear_topic = "";
        std::string xbee_topic = "";
        std::string wireless485_topic = "";

        bool check_ldsf = true;
        bool check_ldsr = true;
        bool check_mcu1 = true;
        bool check_mcu2 = true;
        bool check_imu = true;
        bool check_bms_base = true;
        bool check_cam1 = true;
        bool check_cam2 = true;
        bool check_motor = true;

        bool use_diagnostics = true;
        bool debug = false;

        double wait_error_device_;
        double saving_begin = 600; //sec 3
        int current_diagnostic_step = DIAGNOSTIC_SEQUENCE::INIT_HARDWARE_TYPE;

        // bool system_ready_1st = false;
        matrix_msgs::Diagnostics diag_msg;

        bool check_wireless485, 
                check_xbee,
                check_bms_plugin,
                check_camrear,
                check_cam3;
        int _isWireLess485, 
            _isXbee,
            _isBatteryPlugin,
            _isCamrear,
            _isCam3;
        double wireless485_recive_stamp_,
            zigbee_recive_stamp_,
            bmsplugin_receive_stamp_,
            camrear_recive_stamp_,
            cam3_recive_stamp_,
            wireless485_recive_stamp_old_,
            zigbee_recive_stamp_old_,
            bmsplugin_receive_stamp_old_,
            camrear_recive_stamp_old_,
            cam3_recive_stamp_old_;

        sensor_msgs::BatteryState battplugin_state;


        dynamic_reconfigure::Server<matrix_system::MatrixDiagnosticsConfig> dyn_srv_;

    
    public:
        Diagnostic()
        {

            
            ros::param::param<std::string>("~serial_no", serial_no, "20210001B");
            ros::param::param<std::string>("~model", model, "smr200");
            ros::param::param<std::string>("~ldsf_topic", ldsf_topic, "/scan_front");
            ros::param::param<std::string>("~ldsr_topic", ldsr_topic, "/scan_rear");
            ros::param::param<std::string>("~mcu1_topic", mcu1_topic, "/mcu1");
            ros::param::param<std::string>("~mcu2_topic", mcu2_topic, "/mcu2");
            ros::param::param<std::string>("~imu_topic", imu_topic, "/imu");
            ros::param::param<std::string>("~bms_base_topic", bms_base_topic, "/bms");
            ros::param::param<std::string>("~bms_plugin_topic", bms_plugin_topic, "/bms_plugin");
            ros::param::param<std::string>("~cam1_topic", cam1_topic, "/cam1");
            ros::param::param<std::string>("~cam2_topic", cam2_topic, "/cam2");
            ros::param::param<std::string>("~cam3_topic", cam3_topic, "/cam3");
            ros::param::param<std::string>("~camrear_topic", camrear_topic, "/camrear");
            ros::param::param<std::string>("~motor_topic", motor_topic, "/motor");
            ros::param::param<std::string>("~xbee_topic", xbee_topic, "/xbee");
            ros::param::param<std::string>("~wireless485_topic", wireless485_topic, "/wireless485");
            
            ros::param::param<bool>("~check_ldsf", check_ldsf, true);
            ros::param::param<bool>("~check_ldsr", check_ldsr, true);
            ros::param::param<bool>("~check_mcu1", check_mcu1, true);
            ros::param::param<bool>("~check_mcu2", check_mcu2, true);
            ros::param::param<bool>("~check_imu", check_imu, true);
            ros::param::param<bool>("~check_bms_base", check_bms_base, false);
            ros::param::param<bool>("~check_bms_plugin", check_bms_plugin, false);
            ros::param::param<bool>("~check_cam1", check_cam1, true);
            ros::param::param<bool>("~check_cam2", check_cam2, true);
            ros::param::param<bool>("~check_cam3", check_cam3, false);
            ros::param::param<bool>("~check_camrear", check_camrear, false);
            ros::param::param<bool>("~check_motor", check_motor, true);
            ros::param::param<bool>("~check_xbee", check_xbee, false);
            ros::param::param<bool>("~check_wireless485", check_wireless485, false);

            ros::param::param<bool>("~debug", debug, false);
            ros::param::param<bool>("~use_diagnostics", use_diagnostics, true);
            ros::param::param<bool>("~use_automatic_shutdown", use_automatic_shutdown, true);
            ros::param::param<int>("~timeout_error_device", timeout_error_device, 10);
            ros::param::param<int>("~timeout_error_system", timeout_error_system, 20);

            if(model != "uvc")
            {
                ros::param::param<int>("~timeout_noaction_shutdown", timeout_noaction_shutdown, 10800);
                ros::param::param<int>("~timeout_full_charge_shutdown", timeout_full_charge_shutdown, 600);
                // ros::param::param<int>("~timeout_rate_current_heat", TIMEOUT_RATE_CURRENT_HEAT, 60);
                // ros::param::param<float>("~motor_heat_current", MOTOR_HEAT_CURRENT, 8.0);
            }else;

            // pub_diagnostics = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
            pub_diagnostics = nh_.advertise<matrix_msgs::Diagnostics>("/matrix_system/diagnostics/system_ready", 1);
            pub_diagnostics_modern = nh_.advertise<matrix_msgs::Diagnostics_modern>("/matrix_system/diagnostics", 1);
            // pub_diag_port = nh_.advertise<aragorn_bringup::DiagPort>("/diagnostics",1);
            // pub_diag_emer = nh_.advertise<std_msgs::Bool>("/emer_charge", 1);
            //pub_aragorn_robot_mode = nh_.advertise<std_msgs::Int8>("/aragorn/robot_mode", 10);

            sub_lds_f = nh_.subscribe(ldsf_topic, 1, &Diagnostic::cbLDS_f, this);
            sub_lds_r = nh_.subscribe(ldsr_topic, 1, &Diagnostic::cbLDS_r, this);
            sub_imu = nh_.subscribe(imu_topic, 1, &Diagnostic::cbIMU, this);
            sub_cam1 = nh_.subscribe(cam1_topic, 10, &Diagnostic::cbCam1, this);
            sub_cam2 = nh_.subscribe(cam2_topic, 10, &Diagnostic::cbCam2, this);
            sub_cam3 = nh_.subscribe(cam3_topic, 10, &Diagnostic::cbCam3, this);
            sub_camrear = nh_.subscribe(camrear_topic, 10, &Diagnostic::cbCamrear, this);
            sub_bms_base = nh_.subscribe(bms_base_topic, 10, &Diagnostic::cbBmsbase, this);
            sub_bms_plugin = nh_.subscribe(bms_plugin_topic, 10, &Diagnostic::cbBmsplugin, this);
            sub_mcu1 = nh_.subscribe(mcu1_topic, 1, &Diagnostic::cbMCU1, this);
            sub_mcu2 = nh_.subscribe(mcu2_topic, 1, &Diagnostic::cbMCU2, this);
            sub_motor = nh_.subscribe(motor_topic, 10, &Diagnostic::cbMotor, this);
            sub_zigbee = nh_.subscribe(xbee_topic, 10, &Diagnostic::cbZigbee, this);
            sub_rs485_wireless = nh_.subscribe(wireless485_topic, 10, &Diagnostic::cb485, this); 
            
            
             dyn_srv_.setCallback(boost::bind(&Diagnostic::callback, this, _1, _2));


            
            diag_msg.system_ready = 3;

            ros::Rate loop_rate(10);


            while(ros::ok())
            {   
                step_diagnostics();
                ros::spinOnce();
                loop_rate.sleep();
            }


        }

        void callback(matrix_system::MatrixDiagnosticsConfig &config, uint32_t level)
        {
            check_ldsf = config.check_ldsf;
            check_ldsr = config.check_ldsr;
            check_imu = config.check_imu;
            check_bms_base = config.check_bms_base;
            check_bms_plugin = config.check_bms_plugin;
            check_cam1 = config.check_cam1;
            check_cam2 = config.check_cam2;
            check_cam3 = config.check_cam3;
            check_camrear = config.check_camrear;
            check_motor = config.check_motor;
            check_mcu1 = config.check_mcu1;
            check_mcu2 = config.check_mcu2;
            check_xbee = config.check_xbee;
            check_wireless485 = config.check_wireless485;

            timeout_error_system = config.timeout_error_system;
            timeout_error_device = config.timeout_error_device;
            use_diagnostics = config.use_diagnostics;
        }

        void step_diagnostics()
        {
            switch(current_diagnostic_step)
            {
                case DIAGNOSTIC_SEQUENCE::INIT_HARDWARE_TYPE:
                    // init_hardware_state();
                    init_parameter();
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_DEVICE;
                    break;

                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_DEVICE:
                    diagnostics_device();
                    diagnostics_pub();
                    // current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_CHARGE;
                    break;
                
                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_CHARGE:
                    // diagnostic_charger_state();
                    // if(use_automatic_shutdown)
                    // {
                    //     diagnostic_automatics_shutdown_();
                    // }
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_MOTOR_HEAT;
                    break;
                
                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_MOTOR_HEAT:
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_UVC_LIGHT;
                    break;
                
                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_UVC_LIGHT:
                    // if(USE_ACC_TABLET_SENSE)
                    // {
                    //     diagnostic_acc_uvc();
                    // }else;

                    // diagnostic_uvc_timeout();
            
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::PUB_SYSTEM_READY;
                    break;

                case DIAGNOSTIC_SEQUENCE::PUB_SYSTEM_READY:
                    // system_ready_pub();
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_DEVICE;
                    break;
            };
        }

        void init_parameter()
        {
            wait_error_device_ = ros::Time::now().toSec();
            
        }


        void cbCam1(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            cam1_recive_stamp_ = ros::Time::now().toSec();
            // ROS_INFO("cam11111_ %d\n  %d\n", &cam1_recive_stamp_);
        }

        void cbCam2(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            cam2_recive_stamp_ = ros::Time::now().toSec();
            // ROS_INFO("cam222_ %d \n", ros::Time::now().toSec());
        }

        void cbCam3(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            cam3_recive_stamp_ = ros::Time::now().toSec();
            // ROS_INFO("cam222_ %d \n", ros::Time::now().toSec());
        }

        void cbCamrear(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            camrear_recive_stamp_ = ros::Time::now().toSec();
            // ROS_INFO("cam222_ %d \n", ros::Time::now().toSec());
        }

        void cbBmsbase(const sensor_msgs::BatteryState &msg)
        {
            bmsbase_receive_stamp_ = ros::Time::now().toSec();
            battery_state = msg;
        }

        void cbBmsplugin(const sensor_msgs::BatteryState &msg)
        {
            // bmsbase_receive_stamp_ = ros::Time::now().toSec();
            // battery_state = msg;
            ;
        }

        void cbIMU(const sensor_msgs::Imu::ConstPtr &msg)
        {
            imu_recive_stamp_ = ros::Time::now().toSec();
        }

        void cbMotor(const std_msgs::Int8::ConstPtr &msg)
        {
            motor_recive_stamp_ = ros::Time::now().toSec();
        }

        void cbLDS_f(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
            lds_f_recive_stamp_ = ros::Time::now().toSec();
        }

        void cbLDS_r(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
            lds_r_recive_stamp_ = ros::Time::now().toSec();
        }

        void cbMCU1(const std_msgs::Int8::ConstPtr &msg)
        {
            mcu1_recive_stamp_ = ros::Time::now().toSec();
            // ROS_INFO("mcu1\n");
        }

        void cbMCU2(const std_msgs::Int8::ConstPtr &msg)
        {
            mcu2_recive_stamp_ = ros::Time::now().toSec();
        }

        void cbZigbee(const std_msgs::Int8::ConstPtr &msg)
        {
            zigbee_recive_stamp_ = ros::Time::now().toSec();
        }

        void cb485(const std_msgs::Int8::ConstPtr &msg)
        {
            wireless485_recive_stamp_ = ros::Time::now().toSec();
        }



        void init_hardware_state()
        {
            imu_recive_stamp_ = ros::Time::now().toSec();
            lds_f_recive_stamp_ = ros::Time::now().toSec();
            lds_r_recive_stamp_ = ros::Time::now().toSec();
            mcu1_recive_stamp_ = ros::Time::now().toSec();
            mcu2_recive_stamp_ = ros::Time::now().toSec();
            bmsbase_receive_stamp_ = ros::Time::now().toSec();
            motor_recive_stamp_ = ros::Time::now().toSec();
            cam1_recive_stamp_ = ros::Time::now().toSec();
            cam2_recive_stamp_ = ros::Time::now().toSec();
            cam3_recive_stamp_ = ros::Time::now().toSec();
            camrear_recive_stamp_ = ros::Time::now().toSec();
            bmsplugin_receive_stamp_ = ros::Time::now().toSec();
            zigbee_recive_stamp_ = ros::Time::now().toSec();
            wireless485_recive_stamp_ = ros::Time::now().toSec();


            imu_recive_stamp_old_ = imu_recive_stamp_;
            lds_f_recive_stamp_old_ = lds_f_recive_stamp_;
            lds_r_recive_stamp_old_ = lds_r_recive_stamp_;
            mcu1_recive_stamp_old_ = mcu1_recive_stamp_;
            mcu2_recive_stamp_old_ = mcu2_recive_stamp_;
            bmsbase_receive_stamp_old_ = bmsbase_receive_stamp_;
            motor_recive_stamp_old_ = motor_recive_stamp_;
            cam1_recive_stamp_old_ = cam1_recive_stamp_;
            cam2_recive_stamp_old_ = cam2_recive_stamp_;
            cam3_recive_stamp_old_ = cam3_recive_stamp_;
            camrear_recive_stamp_old_ = camrear_recive_stamp_;
            bmsplugin_receive_stamp_old_ = bmsplugin_receive_stamp_;
            zigbee_recive_stamp_old_ = zigbee_recive_stamp_;
            wireless485_recive_stamp_old_ = wireless485_recive_stamp_;
        }

        void diagnostics_device()
        {
            // double time_secs_now_ = ros::Time::now().toSec();;
            //imu
            // _isIMU = ((imu_recive_stamp_-imu_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_imu)
            {
                _isIMU = ((ros::Time::now().toSec() - imu_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isIMU = 1;
            }
            
            diag_imu.level = _isIMU;
            diag_imu.name = "IMU";
    
            //lidar_f
            // _isLds_f = ((lds_f_recive_stamp_old_-lds_f_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_ldsf)
            {
                _isLds_f = ((ros::Time::now().toSec() - lds_f_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isLds_f = 1;
            }
            diag_lif.level = _isLds_f;
            diag_lif.name = "Lidar Front";
            

            //lidar_r
            // _isLds_r = ((lds_r_recive_stamp_old_-lds_r_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_ldsr)
            {
                _isLds_r = ((ros::Time::now().toSec() - lds_r_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else    
            {
                _isLds_r = 1;
            }
            diag_lir.level = _isLds_r;
            diag_lir.name = "Lidar Rear";
            
            
            //mcu1
            // _isMcu_1 = ((mcu1_recive_stamp_-mcu1_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_mcu1)
            {
                _isMcu_1 = ((ros::Time::now().toSec() - mcu1_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isMcu_1 = 1;
            }
            diag_mcu01.level = _isMcu_1;
            diag_mcu01.name = "MCU01";
            

            //mcu2
            // _isMcu_2 = ((mcu2_recive_stamp_-mcu2_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_mcu2)
            {
                _isMcu_2 = ((ros::Time::now().toSec() - mcu2_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isMcu_2 = 1;
            }
            diag_mcu02.level = _isMcu_2;
            diag_mcu02.name = "MCU02";
            

            //Motordrive
            // _isMotordrive = ((motor_recive_stamp_-motor_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_motor)
            {
                _isMotordrive = ((ros::Time::now().toSec() - motor_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isMotordrive = 1;
            }
            diag_motor.level = _isMotordrive;
            diag_motor.name = "MOTOR";
            

            //cam1 
            // _isCam1 = ((cam1_recive_stamp_-cam1_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            // ROS_INFO("cam2_ %d\n  %d\n", &cam1_recive_stamp_, &cam1_recive_stamp_old_);
            if(check_cam1)
            {
                _isCam1 = ((ros::Time::now().toSec() - cam1_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isCam1 = 1;
            }
            diag_cam1.level = _isCam1;
            diag_cam1.name = "Camera1";

            // ROS_INFO("cam1_ %d\n", _isCam1);

            //cam2
            // _isCam2 = ((cam2_recive_stamp_-cam2_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_cam2)
            {
                _isCam2 = ((ros::Time::now().toSec() - cam2_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isCam2 = 1;
            }
            diag_cam2.level = _isCam2;
            diag_cam2.name = "Camera2";

            

            //cam3
            // _isCam3 = ((cam3_recive_stamp_-cam3_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_cam3)
            {
                _isCam3 = ((ros::Time::now().toSec() - cam3_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isCam3 = 1;
            }
            diag_cam3.level = _isCam3;
            diag_cam3.name = "Camera3";


            

            //camrear
            // _isCamRear = ((cam3_recive_stamp_-cam3_recive_stamp_old_) > timeout_error_device)? 2 :  0;
            if(check_camrear)
            {
                _isCamrear = ((ros::Time::now().toSec() - camrear_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isCamrear = 1;
            }
            diag_camrear.level = _isCamrear;
            diag_camrear.name = "Camera Rear";

            
            
            // ROS_INFO("cam2_ %d\n  %d\n", &cam2_recive_stamp_, &cam2_recive_stamp_old_);

            if(check_bms_base)
            {
                //battery
                _isBattery = ((ros::Time::now().toSec() - bmsbase_receive_stamp_) > timeout_error_device)? 2 : 0;

                // if(_isBattery != 2)
                // {
                //     _isBattery = (battery_state.percentage < 0.20)? 2 : 0;
                // }else;
            }
            else
            {
                _isBattery = 1;
            }
            diag_bmsbase.level = _isBattery;
            diag_bmsbase.name = "BMS Base";



            if(check_bms_plugin)
            {
                //battery
                _isBatteryPlugin = ((ros::Time::now().toSec() - bmsplugin_receive_stamp_) > timeout_error_device)? 2 : 0;

                if(_isBatteryPlugin != 2)
                {
                    _isBatteryPlugin = (battplugin_state.percentage < 0.20)? 2 : 0;
                }else;
            }
            else
            {
                _isBatteryPlugin = 1;
            }
            diag_bmsplugin.level = _isBatteryPlugin;
            diag_bmsplugin.name = "BMS Plugin";

            


            if(check_xbee)
            {
                _isXbee = ((ros::Time::now().toSec() - zigbee_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isXbee = 1;
            }
            diag_xbee.level = _isXbee;
            diag_xbee.name = "Xbee";


            

            if(check_wireless485)
            {
                _isWireLess485 = ((ros::Time::now().toSec() - wireless485_recive_stamp_) > timeout_error_device)? 2 : 0;
            }
            else
            {
                _isWireLess485 = 1;
            }
            diag_wireless485.level = _isWireLess485;
            diag_wireless485.name = "Wireless RS485";

            


            // imu_recive_stamp_old_ = imu_recive_stamp_;
            // lds_f_recive_stamp_old_ = lds_f_recive_stamp_;
            // lds_r_recive_stamp_old_ = lds_r_recive_stamp_;
            // mcu1_recive_stamp_old_ = mcu1_recive_stamp_;
            // mcu2_recive_stamp_old_ = mcu2_recive_stamp_;
            // bmsbase_receive_stamp_old_ = bmsbase_receive_stamp_;
            // motor_recive_stamp_old_ = motor_recive_stamp_;
            // cam1_recive_stamp_old_ = cam1_recive_stamp_;
            // cam2_recive_stamp_old_ = cam2_recive_stamp_;

            

        }

        void diagnostics_pub()
        {
            static bool output_once_ = false;
            
            diag_msg.device_state.lidar_front = diag_lif.level;
            diag_msg.device_state.lidar_rear = diag_lir.level;
            diag_msg.device_state.imu = diag_imu.level;
            diag_msg.device_state.camera_1 = diag_cam1.level;
            diag_msg.device_state.camera_2 = diag_cam2.level;
            diag_msg.device_state.camera_3 = diag_cam3.level;
            diag_msg.device_state.camera_rear = diag_camrear.level;
            diag_msg.device_state.bms= diag_bmsbase.level;
            diag_msg.device_state.bms_base= diag_bmsbase.level;
            diag_msg.device_state.bms_plugin= diag_bmsplugin.level;
            diag_msg.device_state.mcu_1 = diag_mcu01.level;
            diag_msg.device_state.mcu_2 = diag_mcu02.level;
            diag_msg.device_state.motor = diag_motor.level;
            diag_msg.device_state.zigbee = diag_xbee.level;

            diagnostic_msgs::DiagnosticArray diag;
            diag.status.push_back(diag_lif);
            diag.status.push_back(diag_lir);
            diag.status.push_back(diag_imu);
            diag.status.push_back(diag_cam1);
            diag.status.push_back(diag_cam2);
            diag.status.push_back(diag_cam3);
            diag.status.push_back(diag_camrear);
            diag.status.push_back(diag_bmsbase);
            diag.status.push_back(diag_bmsplugin);
            diag.status.push_back(diag_mcu01);
            diag.status.push_back(diag_mcu02);
            diag.status.push_back(diag_motor);
            diag.status.push_back(diag_xbee);
            diag.status.push_back(diag_wireless485);

            matrix_msgs::Diagnostics_modern diag_modern;
            diag_modern.status = diag.status;
            diag_modern.system_ready = diagnostic_msgs::DiagnosticStatus::OK;

            if(use_diagnostics)
            {
                for(int i =0; i<diag_modern.status.size(); i++)
                {
                    if(diag_modern.status[i].level == diagnostic_msgs::DiagnosticStatus::ERROR)
                    {
                        // ROS_INFO("diag_modern.status.size() %d", diag_modern.status.size());
                        // ROS_ERROR("Error device is %s", diag_modern.status[i].name.c_str());
                        diag_modern.system_ready = diagnostic_msgs::DiagnosticStatus::ERROR;
                        // break;
                    }
                }

                if(diag_modern.system_ready == diagnostic_msgs::DiagnosticStatus::OK)
                {
                    diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_OK;
                    diag_modern.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_OK;
                    if(!output_once_)
                    {   
                        ROS_INFO("[matrix_diagnostics]:****************************");
                        ROS_INFO("[matrix_diagnostics]:");
                        ROS_INFO("[matrix_diagnostics]:");
                        ROS_INFO("[matrix_diagnostics]: matrix_robot ready to start");
                        ROS_INFO("[matrix_diagnostics]:");
                        ROS_INFO("[matrix_diagnostics]:");
                        ROS_INFO("[matrix_diagnostics]:****************************");
                        output_once_ = true;
                    }
                    wait_error_device_ = ros::Time::now().toSec();
                }
                else
                {
                    output_once_ = false;
                    double time_now = ros::Time::now().toSec();
                    if((abs(time_now - wait_error_device_)) > timeout_error_system)
                    {
                        diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_FAULT;
                        diag_modern.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_FAULT;
                        ROS_ERROR("[matrix_diagnostics]:some device error");
                        for(int i =0; i<diag_modern.status.size(); i++)
                        {
                            if(diag_modern.status[i].level == diagnostic_msgs::DiagnosticStatus::ERROR)
                            {
                                // ROS_INFO("diag_modern.status.size() %d", diag_modern.status.size());
                                ROS_ERROR("[matrix_diagnostics]:Error device is %s", diag_modern.status[i].name.c_str());
                                // diag_modern.system_ready = diagnostic_msgs::DiagnosticStatus::ERROR;
                                // break;
                            }
                        }
                    }
                    else
                    {
                        diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_WAIT_COMEUP;
                        diag_modern.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_WAIT_COMEUP;
                        // ROS_WARN("[matrix_diagnostics]:wait to connect device");

                        for(int i =0; i<diag_modern.status.size(); i++)
                        {
                            if(diag_modern.status[i].level == diagnostic_msgs::DiagnosticStatus::ERROR)
                            {
                                // ROS_INFO("diag_modern.status.size() %d", diag_modern.status.size());
                                ROS_WARN("[matrix_diagnostics]:Waiting device comeup %s", diag_modern.status[i].name.c_str());
                                // diag_modern.system_ready = diagnostic_msgs::DiagnosticStatus::ERROR;
                                // break;
                            }
                        }
                    }
                }

                // if((_isLds_f != 2) && (_isLds_r != 2) && (_isIMU != 2 ) && (_isCam1 != 2) && (_isCam2 != 2) && (_isMcu_1 != 2) && (_isMcu_2 != 2) && (_isBattery != 2) && (_isMotordrive != 2) )
                // {

                //     diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_OK;
                //     if(!output_once_)
                //     {   
                //         ROS_INFO("[matrix_diagnostics]:****************************");
                //         ROS_INFO("[matrix_diagnostics]:");
                //         ROS_INFO("[matrix_diagnostics]:");
                //         ROS_INFO("[matrix_diagnostics]: matrix_robot ready to start");
                //         ROS_INFO("[matrix_diagnostics]:");
                //         ROS_INFO("[matrix_diagnostics]:");
                //         ROS_INFO("[matrix_diagnostics]:****************************");
                //         output_once_ = true;
                //     }
                //     wait_error_device_ = ros::Time::now().toSec();
                // }
                // else
                // {
                //     double time_now = ros::Time::now().toSec();
                //     if((abs(time_now - wait_error_device_)) > timeout_error_system)
                //     {
                //         diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_STATUS_FAULT;
                //         ROS_ERROR("[matrix_diagnostics]:some device error");
                //     }
                //     else
                //     {
                //         diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_WAIT_COMEUP;
                //         ROS_WARN("[matrix_diagnostics]:wait to connect device");
                //     }
                    
                // }

            }
            else
            {
                diag_msg.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_NOT_USE;
                diag_modern.system_ready = matrix_msgs::Diagnostics::SYSTEM_READY_NOT_USE;
            }
            
            if(debug)
            {
                ROS_INFO("ldf:%d, ldr:%d, imu:%d, cam_left:%d, cam_right:%d, batt:%d, mcu1:%d", _isLds_f, _isLds_r, _isIMU, _isCam1, _isCam2, _isBattery, _isMcu_1);
                ROS_INFO("results %d\n", diag_msg.system_ready);
            }
            
            pub_diagnostics.publish(diag_msg);
            pub_diagnostics_modern.publish(diag_modern);

        }

        // void diagnostic_charger_state()
        // {
        //     // ROS_INFO("*********diagnostic charger*********");
        //     if(system_status.data == true)
        //     {
        //         if(battery_state.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
        //         {
        //             if((current_robot_mode.data != robot_mode::CHARGER_ON) && (!_isAutodocking))
        //             {
        //                 if((state_change == false) && (current_robot_mode.data != robot_mode::READY_TO_START) && (current_robot_mode.data != robot_mode::EMERGENCY))
        //                 {

        //                     while(1)
        //                     {
        //                         ROS_INFO("*****reset charing state*********");
        //                         std_srvs::SetBool reset_srv;
        //                         reset_srv.request.data = true;
        //                         if(crsv_reset.call(reset_srv))
        //                         {
        //                             break;
        //                         }
        //                     }

        //                     state_change = true;
        //                 }
        //                 else
        //                 {
        //                     ROS_INFO("********manual_mode*********");
        //                 }
        //                 emer_charge_msg.data = true; 
        //             }
        //             else
        //             {
        //                 if(current_robot_mode.data == robot_mode::CHARGER_ON)
        //                 {
        //                     ROS_INFO("*******autodocking mode*********");
        //                     emer_charge_msg.data = false; 
        //                     _isAutodocking = true;
        //                 }
        //                 else;
        //             } 
        //         }
        //         else
        //         {
        //             _isAutodocking = false;
        //             emer_charge_msg.data = false;
        //             state_change = false;
        //             // ROS_INFO("battery sate discharge");
        //         }
        //         // ROS_INFO("current robot mode %d", current_robot_mode.data);
        //     }
        //     else
        //     {
        //         emer_charge_msg.data = false;
        //     }
        //     pub_diag_emer.publish(emer_charge_msg);
        // }

   

        


        


        
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_diagnostics");

    Diagnostic aragorn_dia;

    ros::spin();

    return 0;


}