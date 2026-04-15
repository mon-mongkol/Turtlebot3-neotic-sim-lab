#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <string>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/console.h> 
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <aragorn_bringup/DiagPort.h>
#include <aragorn_bringup/Cautions.h>
#include <std_srvs/SetBool.h>
#include <time.h>
#include <robot_status/RobotStatus.h>

#define VOLTAGE_THRESHOLD_UVC_CUTOFF 22 //volt
// #define TIMEOUT_ACC_TABLET 50

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

enum UVC_STATE{ UVC_LAMP_OFF,
                UVC_LAMP_ON
              };

enum DIAGNOSTIC_SEQUENCE{ INIT_HARDWARE_TYPE,
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
        ros::ServiceClient crsv_reset, csrv_uvc_mode, csrv_tablet_discon_mode, crsv_setCE;
        ros::Publisher pub_aragorn_diagnostics,
                       pub_aragorn_system_ready,
                       pub_diag_port,
                       pub_aragorn_robot_mode, // only pub when IO disconnected
                       pub_diag_emer;

        ros::Subscriber sub_lds,
                        sub_io,
                        sub_uvc_interlock,
                        sub_imu,
                        sub_battery,
                        sub_motor,
                        sub_cam,
                        sub_cam2,
                        sub_robot_mode,
                        sub_robot_staus,
                        sub_tablet_acc,
                        sub_uvc_state;

        bool _isRPlidar,
             _isIO,
             _isIMU,
             _isBattery,
             _isMotordrive,
             _isCam,
             _isCam2,
             _isInit_uvc_timeout;
        
        bool _is_output = false;
        robot_status::RobotStatus robot_status_;
        
        diagnostic_msgs::DiagnosticArray  aragorn_diagnostics;
        diagnostic_msgs::DiagnosticStatus imu_state,
                                          lds_state,
                                          io_state,
                                          batt_state,
                                          motor_state,
                                          cam_state,
                                          cam2_state;
        
        std_msgs::Bool system_status, emer_charge_msg;
        std_msgs::Int8 robot_mode, interlock_state, uvc_state;

        int imu_recive_stamp_,
                      lds_recive_stamp_,
                      io_recive_stamp_,
                      batt_recive_stamp_,
                      motor_recive_stamp_,
                      cam_recive_stamp_,
                      cam2_recive_stamp_;
        
        int imu_recive_stamp_old_,
                      lds_recive_stamp_old_,
                      io_recive_stamp_old_,
                      batt_recive_stamp_old_,
                      motor_recive_stamp_old_,
                      cam_recive_stamp_old_,
                      cam2_recive_stamp_old_;

        unsigned long ldss_recive_stamp_old_ = 0.0;
        unsigned long ldss_recive_stamp_ = 0.0;

        double imuu_recive_stamp_, imuu_recive_stamp_old_;

        int ioo_recive_stamp_ = 1;
        int ioo_recive_stamp_old_ = 1;

        float SOC_theshold = 30.0;
        // float battery_state.percentage;
        
        double batt_time_coming_ , acc_time_stamp_, warning_current_stamp_, uvc_timeout_stamp_;
        bool _isBattComing = false;
        bool _isInit_heat_stmp = false;
        bool _isAutodocking = false;
        bool _isCharging = false;
        bool _init_saving = false;
        bool init_tablet_discon_ = false;
        int batt_wait_recive = 0;
        int counter_wait_recive = 0;
        std_msgs::Int8 current_robot_mode, prev_robot_mode;
        bool state_change = false;

        aragorn_bringup::Cautions system_cautions;
        sensor_msgs::BatteryState battery_state;

        bool USE_ACC_TABLET_SENSE = true;
        bool USE_AUTOMATIC_SHUTDOWN = true;
        int TIMEOUT_ACC_TABLET = 40;
        int TIMEOUT_ERROR_DEVICE = 10;
        int TIMEOUT_RATE_CURRENT_HEAT = 60;
        int TIMEOUT_UVC_ON_B = 1800;
        int TIMEOUT_UVC_ON_A = 900;
        int TIMEOUT_UVC_ON = 900;
        int TIMEOUT_NOACTION_BEFORE_SHUTDOWN = 600; //sec
        float MOTOR_HEAT_CURRENT = 8.0;
        std::string SERIAL_NO = "20210001B";

        double saving_begin = 600; //sec 

        int current_diagnostic_step = DIAGNOSTIC_SEQUENCE::INIT_HARDWARE_TYPE;

    public:
        Diagnostic()
        {
            ros::param::param<bool>("~use_acc_teblet_sense", USE_ACC_TABLET_SENSE, true);
            ros::param::param<bool>("~use_automatic_shutdown", USE_AUTOMATIC_SHUTDOWN, false);
            ros::param::param<int>("~timeout_noaction_before_shutdown", TIMEOUT_NOACTION_BEFORE_SHUTDOWN, 10);
            ros::param::param<int>("~timeout_acc_tablet", TIMEOUT_ACC_TABLET, 40);
            ros::param::param<int>("~timeout_error_device", TIMEOUT_ERROR_DEVICE, 10);
            ros::param::param<int>("~timeout_rate_current_heat", TIMEOUT_RATE_CURRENT_HEAT, 60);
            ros::param::param<int>("~timeout_uvc_on_a", TIMEOUT_UVC_ON_A, 900);
            ros::param::param<int>("~timeout_uvc_on_b", TIMEOUT_UVC_ON_B, 1800);
            ros::param::param<float>("~motor_heat_current", MOTOR_HEAT_CURRENT, 8.0);
            ros::param::param<std::string>("~serial_no", SERIAL_NO, "20210001B");

            pub_aragorn_diagnostics = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
            pub_aragorn_system_ready = nh_.advertise<std_msgs::Bool>("/aragorn/diagnostics/system_ready", 10);
            pub_diag_port = nh_.advertise<aragorn_bringup::DiagPort>("/aragorn/diagnostics", 10);
            pub_diag_emer = nh_.advertise<std_msgs::Bool>("/aragorn/diagnostics/emer_charge", 10);
            //pub_aragorn_robot_mode = nh_.advertise<std_msgs::Int8>("/aragorn/robot_mode", 10);

            sub_imu = nh_.subscribe("/imu/data_raw", 10, &Diagnostic::cbIMU, this);
            sub_lds = nh_.subscribe("/scan", 10, &Diagnostic::cbLDS, this);
            sub_io = nh_.subscribe("/aragorn_io/emergency", 10, &Diagnostic::cbIOboard, this);
            sub_battery = nh_.subscribe("/battery_state", 10, &Diagnostic::cbBatteryState, this);
            sub_motor = nh_.subscribe("/zlac706/odom", 10, &Diagnostic::cbMotor, this);
            sub_cam = nh_.subscribe("/camera/depth/color/points", 10, &Diagnostic::cbCam, this);
            sub_cam2 = nh_.subscribe("/camera2/depth/color/points", 10, &Diagnostic::cbCam2, this);
            sub_uvc_interlock = nh_.subscribe("/aragorn_io/interlock_state", 1, &Diagnostic::cbUVCInterlock, this);
            sub_robot_mode = nh_.subscribe("/aragorn/robot_mode", 1, &Diagnostic::cbRobotMode, this);
            sub_tablet_acc = nh_.subscribe("/accelerometer", 1, &Diagnostic::cbTabletAcc, this);
            sub_uvc_state = nh_.subscribe("/aragorn_io/uvc_state", 1, &Diagnostic::cbUvcState, this);
            sub_robot_staus = nh_.subscribe("/aragorn/base_status", 1, &Diagnostic::cbStatus, this);

            crsv_reset = nh_.serviceClient<std_srvs::SetBool>("/aragorn_io/reset");
            csrv_uvc_mode = nh_.serviceClient<std_srvs::SetBool>("/aragorn_io/uvc");
            csrv_tablet_discon_mode = nh_.serviceClient<std_srvs::SetBool>("/aragorn_io/tablet_discon");
            crsv_setCE = nh_.serviceClient<std_srvs::SetBool>("/aragorn_io/ce_ver");
            

            ros::service::waitForService("/aragorn_io/ce_ver", -1);

            ros::Rate loop_rate(10);

            _isBattery = true;
             

            system_status.data = false;

            while(ros::ok())
            {   
                step_diagnostics();
                ros::spinOnce();
                loop_rate.sleep();
            }


        }

        void step_diagnostics()
        {
            switch(current_diagnostic_step)
            {
                case DIAGNOSTIC_SEQUENCE::INIT_HARDWARE_TYPE:
                    if(SERIAL_NO == "20210009A")
                    {
                        ROS_INFO("set CE version");
                        while(1)
                        {
                            std_srvs::SetBool ce_srv;
                            ce_srv.request.data = true;
                            if(crsv_setCE.call(ce_srv))
                            {
                                break;
                            }
                        }
                    }else;
                    {
                        ROS_INFO("%s", SERIAL_NO);
                    }
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_DEVICE;
                    break;

                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_DEVICE:
                    diagnostic_pub();
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_CHARGE;
                    break;
                
                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_CHARGE:
                    diagnostic_charger_state();
                    // if(USE_AUTOMATIC_SHUTDOWN)
                    // {
                    //     diagnostic_automatics_shutdown_();
                    // }
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_MOTOR_HEAT;
                    break;
                
                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_MOTOR_HEAT:
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_UVC_LIGHT;
                    break;
                
                case DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_UVC_LIGHT:
                    if(USE_ACC_TABLET_SENSE)
                    {
                        diagnostic_acc_uvc();
                    }else;

                    // diagnostic_uvc_timeout();
            
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::PUB_SYSTEM_READY;
                    break;

                case DIAGNOSTIC_SEQUENCE::PUB_SYSTEM_READY:
                    system_ready_pub();
                    current_diagnostic_step = DIAGNOSTIC_SEQUENCE::DIAGNOSTIC_DEVICE;
                    break;
            };
        }

        void cbStatus(const robot_status::RobotStatus &msg)
        {
            robot_status_ = msg;
        }

        void cbUvcState(const std_msgs::Int8 &msg)
        {
            uvc_state = msg;
        }

        void cbTabletAcc(const geometry_msgs::Vector3 &msg)
        {
            acc_time_stamp_ = ros::Time::now().toSec();
        }

        void cbRobotMode(const std_msgs::Int8 &msg)
        {
            current_robot_mode = msg;
            ROS_INFO("aa %d ", current_robot_mode.data);
        }

        void cbUVCInterlock(const std_msgs::Int8 &msg)
        {
            interlock_state = msg;
        }
        void cbCam(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            if(cam_recive_stamp_ >= 10000)
            {
                cam_recive_stamp_ = 0;
            }
            else
            {
                cam_recive_stamp_++;
            }
            
        }

        void cbCam2(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            if(cam2_recive_stamp_ >= 10000)
            {
                cam2_recive_stamp_ = 0;
            }
            else
            {
                cam2_recive_stamp_++;
            }
        }

        void cbBatteryState(const sensor_msgs::BatteryState &msg)
        {
            battery_state = msg;
            _isBattComing = true;
            batt_time_coming_ = ros::Time::now().toSec();
            
        }

        void cbIMU(const sensor_msgs::Imu::ConstPtr &msg)
        {
            // imu_recive_stamp_ = (imu_recive_stamp_ >= 1000) ? 0 : imu_recive_stamp_++;
            // if(imu_recive_stamp_ >= 1000)
            // {
            //     imu_recive_stamp_ = 0;
            // }
            // else
            // {
            //     imu_recive_stamp_++;
            // } 
            imuu_recive_stamp_ = msg->orientation.x;
        }

        void cbMotor(const nav_msgs::Odometry::ConstPtr &msg)
        {
            // motor_recive_stamp_ = (motor_recive_stamp_ >= 1000) ? 0 : motor_recive_stamp_++; 
            if(motor_recive_stamp_ >= 1000)
            {
                motor_recive_stamp_ = 0;
            }
            else
            {
                motor_recive_stamp_++;
            }
        }

        void cbLDS(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
            // ldss_recive_stamp_ = (ldss_recive_stamp_ >= 1000) ? 0 : ldss_recive_stamp_++; 
            if(ldss_recive_stamp_ >= 1000)
            {
                ldss_recive_stamp_ = 0;
            }
            else
            {
                ldss_recive_stamp_++;
            }
            // ldss_recive_stamp_+=1;
            // ROS_INFO("stamp lidar  %f", ldss_recive_stamp_ );
        }

        void cbIOboard(const std_msgs::Int8::ConstPtr &msg)
        {
            // io_recive_stamp_ = (io_recive_stamp_ >= 1000) ? 0 : io_recive_stamp_++; 
            if(io_recive_stamp_ >= 1000)
            {
                io_recive_stamp_ = 0;
            }
            else
            {
                io_recive_stamp_++;
            }
            // ioo_recive_stamp_ += 1;
            // ROS_INFO("%d", ioo_recive_stamp_);
            // ROS_INFO("goooooooooooooo");

        }



        void setDiagnostic()
        {
            int ok_ = diagnostic_msgs::DiagnosticStatus::OK;
            int warn_ = diagnostic_msgs::DiagnosticStatus::WARN;
            int error_ = diagnostic_msgs::DiagnosticStatus::ERROR;

            double time_secs_now_ = ros::Time::now().toSec();
            

            //imu
            if(imuu_recive_stamp_ != imuu_recive_stamp_old_)
            {
                setDiagnosisMsg(&imu_state, ok_, "IMU", "USB IMU connected", "ICM20948");
                _isIMU = true;
            }else
            {
                setDiagnosisMsg(&imu_state, error_, "IMU", "Please check USB IMU connection", "ICM20948");
                _isIMU = false;
            }
            
            //lidar
            if(ldss_recive_stamp_ != ldss_recive_stamp_old_)
            {
                setDiagnosisMsg(&lds_state, ok_, "LIDAR", "USB Lidar connected", "RPLidar A3");
                _isRPlidar = true;
            }else
            {
                setDiagnosisMsg(&lds_state, error_, "LIDAR", "Please check USB Lidar connection", "RPLidar A3");
                _isRPlidar = false;
            }
            // ROS_INFO("%d", _isRPlidar);

            //lidar
            if(io_recive_stamp_ != io_recive_stamp_old_)
            {
                setDiagnosisMsg(&io_state, ok_, "LIDAR", "USB Lidar connected", "RPLidar A3");
                _isIO = true;
            }else
            {
                setDiagnosisMsg(&io_state, error_, "LIDAR", "Please check USB Lidar connection", "RPLidar A3");
                _isIO = false;
            }
            // ROS_INFO("%d", _isIO);
            // ROS_INFO("%d", io_recive_stamp_old_);


            //battery
            if(_isBattComing)
            {
                //check percent of battery 
                if(0.00 >= battery_state.percentage <= 0.05)
                {
                    setDiagnosisMsg(&batt_state, warn_, "Power System", "Lower than 5", "Battery");
                    _isBattery = true;
                    system_cautions.caution_code = aragorn_bringup::Cautions::BATTERY_LOWER_5_PERCENT_CUT_UVC;
                }
                else if(0.00 >= battery_state.percentage <= 0.10)
                {
                    setDiagnosisMsg(&batt_state, warn_, "Power System", "Lower than 10.0", "Battery");
                    _isBattery = true;
                    system_cautions.caution_code = aragorn_bringup::Cautions::BATTERY_LOWER_10_PERCENT_CUT_UVC;
                }
                else if(0.00 >= battery_state.percentage <= 0.20)
                {
                    setDiagnosisMsg(&batt_state, warn_, "Power System", "Lower than 20.0", "Battery");
                    _isBattery = true;
                    system_cautions.caution_code = aragorn_bringup::Cautions::BATTERY_LOWER_20_PERCENT_CUT_UVC;
                }
                else if(0.20 > battery_state.percentage <= 1.0)
                {
                    setDiagnosisMsg(&batt_state, ok_, "Power System", "Good Condition", "Battery");
                    _isBattery = true;
                    system_cautions.caution_code = aragorn_bringup::Cautions::BATTERY_OK;
                }
                else
                {
                    setDiagnosisMsg(&batt_state, error_, "Power System", "Please check USB IO Board connection", "Battery");
                    _isBattery = false;
                    system_cautions.caution_code = aragorn_bringup::Cautions::BATTERY_UNKNOWS;
                }


                system_cautions.current_battery_percentage = battery_state.percentage;

                //reset state Battery and trig by new new battery state coming 
                _isBattComing = false;
                
            // check state when battery state doesn't coming in 5sec cause Battery loop spin on 1HZ
            }else if((!_isBattComing) && ((ros::Time::now().toSec() - batt_time_coming_) >= 5.0))
            {
                _isBattery = false;
            }
            else
            {
                ;
            }




            //motor
            if(motor_recive_stamp_ != motor_recive_stamp_old_)
            {
                setDiagnosisMsg(&motor_state, ok_, "Motor driver", "USB IO board connected", "ZLAC706-CAN");
                _isMotordrive = true;
            }else
            {
                setDiagnosisMsg(&motor_state, error_, "Motor driver", "Please check USB IO Board connection", "ZLAC706-CAN");
                _isMotordrive = false;
            }

            //cam
            if(cam_recive_stamp_ != cam_recive_stamp_old_)
            {
                setDiagnosisMsg(&cam_state, ok_, "Camera1", "USB camera1 connected", "Realsense d455");
                _isCam = true;
            }else
            {
                setDiagnosisMsg(&cam_state, error_, "Camera1", "Please check USB camera1 connection", "Realsense d455");
                _isCam = false;
            }

            //cam2
            if(cam2_recive_stamp_ != cam2_recive_stamp_old_)
            {
                setDiagnosisMsg(&cam2_state, ok_, "Camera1", "USB camera2 connected", "Realsense d455");
                _isCam2 = true;
            }else
            {
                setDiagnosisMsg(&cam2_state, error_, "Camera1", "Please check USB camera2 connection", "Realsense d455");
                _isCam2 = false;
            }

            imuu_recive_stamp_old_ = imuu_recive_stamp_;
            ldss_recive_stamp_old_ = ldss_recive_stamp_;
            io_recive_stamp_old_ = io_recive_stamp_;
            batt_recive_stamp_old_ = batt_recive_stamp_;
            motor_recive_stamp_old_ = motor_recive_stamp_;
            cam_recive_stamp_old_ = cam_recive_stamp_;
            cam2_recive_stamp_old_ = cam2_recive_stamp_;


        }


        void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
        {
            diag->level = level;
            diag->name  = name;
            diag->message = message;
            diag->hardware_id = hardware_id;
        }

        void diagnostic_pub()
        {
            setDiagnostic();

            diagnostic_msgs::DiagnosticArray  aragorn_diagnostics;

            aragorn_diagnostics.header.stamp = ros::Time::now();

            aragorn_diagnostics.status.clear();
            aragorn_diagnostics.status.push_back(imu_state);
            aragorn_diagnostics.status.push_back(lds_state);
            aragorn_diagnostics.status.push_back(io_state);
            aragorn_diagnostics.status.push_back(batt_state);
            aragorn_diagnostics.status.push_back(motor_state);
            aragorn_diagnostics.status.push_back(cam_state);
            aragorn_diagnostics.status.push_back(cam2_state);

            pub_aragorn_diagnostics.publish(aragorn_diagnostics);
        }


        void diagnostic_motor_heat()
        {
            ROS_INFO("*********diagnostic motor heat*********");

            if(system_status.data == true)
            {
                if(battery_state.current >= MOTOR_HEAT_CURRENT)
                {
                    if(battery_state.power_supply_status != sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
                    {
                        if(current_robot_mode.data != robot_mode::UVC_ON)
                        {
                            if(!_isInit_heat_stmp)
                            {
                                warning_current_stamp_ = ros::Time::now().toSec() + TIMEOUT_RATE_CURRENT_HEAT;
                                _isInit_heat_stmp = true;
                            }
                            else;

                            if((warning_current_stamp_ - ros::Time::now().toSec()) <= 0.0)
                            {
                                ;
                            }
                        }else;
                    }else;
                }
                else
                {
                    _isInit_heat_stmp = false;
                }
            }else;
        }

        void diagnostic_uvc_timeout()
        {
           if(system_status.data == true)
           {
               if(current_robot_mode.data == robot_mode::UVC_ON)
               {
                   if(!_isInit_uvc_timeout)
                   {
                       if(SERIAL_NO[8] == 'A')
                       {
                           TIMEOUT_UVC_ON = TIMEOUT_UVC_ON_A;
                       }
                       else if(SERIAL_NO[8] == 'B')
                       {
                           TIMEOUT_UVC_ON = TIMEOUT_UVC_ON_B;
                       }
                       uvc_timeout_stamp_ = ros::Time::now().toSec() + TIMEOUT_UVC_ON;
                       _isInit_uvc_timeout = true;
                   }else;

                   if(ros::Time::now().toSec() >= uvc_timeout_stamp_)
                   {
                       // turn off uvc lamp 
                       _isInit_uvc_timeout = false;
                   }
               }
               else
               {
                   _isInit_uvc_timeout = false;
               }
           }else;
        }

        // void diagnostic_automatics_shutdown_()
        // {
        //     if(battery_state.power_supply_status != sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
        //     {
        //         if(!robot_status_.robot_moving)
        //         {
        //             ROS_INFO("[aragorn_diagnostics]:robot doest moving");
        //             if(!_init_saving)
        //             {
        //                 saving_begin = ros::Time::now().toSec();
        //                 ROS_INFO("[aragorn_diagnostics]:init saving power time %d ", saving_begin);
        //                 _init_saving = true;
        //             }
        //             else
        //             {
        //                 double timmer_ = ros::Time::now().toSec() - saving_begin;
        //                 if(abs(timmer_) > TIMEOUT_NOACTION_BEFORE_SHUTDOWN)
        //                 {
        //                     //do something
        //                     _init_saving = false;
        //                 }
        //                 else;
        //             }
        //         }
        //         else
        //         {
        //             ROS_INFO("[aragorn_diagnostics]:robot moving");
        //             _init_saving = false;
        //         }            
        //     }else
        //     {
        //         _init_saving = false;
        //     }
        // }


        void diagnostic_charger_state()
        {
            ROS_INFO("*********diagnostic charger*********");
            if(system_status.data == true)
            {
                if(battery_state.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
                {
                    if((current_robot_mode.data != robot_mode::CHARGER_ON) && (!_isAutodocking))
                    {
                        if((state_change == false) && (current_robot_mode.data != robot_mode::READY_TO_START) && (current_robot_mode.data != robot_mode::EMERGENCY))
                        {

                            while(1)
                            {
                                ROS_INFO("*****aaaaaaaaa*********");
                                std_srvs::SetBool reset_srv;
                                reset_srv.request.data = true;
                                if(crsv_reset.call(reset_srv))
                                {
                                    break;
                                }
                            }

                            state_change = true;
                        }
                        else
                        {
                            ROS_INFO("********manual_mode*********");
                        }
                        emer_charge_msg.data = true; 
                    }
                    else
                    {
                        if(current_robot_mode.data == robot_mode::CHARGER_ON)
                        {
                            ROS_INFO("*******autodocking mode*********");
                            emer_charge_msg.data = false; 
                            _isAutodocking = true;
                        }
                        else;
                    } 
                }
                else
                {
                    _isAutodocking = false;
                    emer_charge_msg.data = false;
                    state_change = false;
                    ROS_INFO("battery sate discharge");
                }
                ROS_INFO("current robot mode %d", current_robot_mode.data);
            }
            else
            {
                emer_charge_msg.data = false;
            }
            pub_diag_emer.publish(emer_charge_msg);
        }

        void diagnostic_acc_uvc()
        {
            if(uvc_state.data == UVC_STATE::UVC_LAMP_ON)
            {
                ROS_INFO("*******Diagnostic Tablet acc && UVC lamp*********");
                if((ros::Time::now().toSec() - acc_time_stamp_) > TIMEOUT_ACC_TABLET)
                {
                    if(!init_tablet_discon_)
                    {
                        // turn off uvc lamp
                        while(1)
                        {
                            std_srvs::SetBool tablet_discon_srv;
                            tablet_discon_srv.request.data = true;
                            if(csrv_tablet_discon_mode.call(tablet_discon_srv))
                            {
                                break;
                            }
                        }
                        init_tablet_discon_ = true;
                    }else;
                    
                }else;
            }
            else;
            {
                init_tablet_discon_ = false;
            }
            
        }

        void system_ready_pub()
        {
            
            if(( _isRPlidar && _isIO && _isIMU && _isCam && _isCam2 && _isBattery) == true)
            // if(( _isRPlidar && _isIO && _isIMU && _isCam && _isCam2) == true)
            // if(( _isRPlidar && _isIO && _isIMU && _isCam ) == true)
            // if(( _isRPlidar && _isIO && _isIMU) == true)
            {
                system_status.data = true;
                pub_aragorn_system_ready.publish(system_status);
                if(_is_output == false)
                {   
                    ROS_INFO("***********************");
                    ROS_INFO("");
                    ROS_INFO("");
                    ROS_INFO("aragorn ready to start");
                    ROS_INFO("");
                    ROS_INFO("");
                    ROS_INFO("***********************");
                    _is_output = true;
                }
                counter_wait_recive = 0;
            }
            else
            {
                
                
                if(counter_wait_recive >= TIMEOUT_ERROR_DEVICE)
                {
                    system_status.data = false;
                    pub_aragorn_system_ready.publish(system_status);

                    if(_isRPlidar == false)
                    {
                        ROS_ERROR("[Diagnostic]: Failed to connect RPlidar");
                    }
                    else ;

                    if(_isCam == false)
                    {
                        ROS_ERROR("[Diagnostic]: Failed to connect Camera1");
                    }
                    else ;

                    if(_isCam2 == false)
                    {
                        ROS_ERROR("[Diagnostic]: Failed to connect Camera2");
                    }
                    else ;
                    
                    if(_isIO == false)
                    {
                        ROS_ERROR("[Diagnostic]: Failed to connect IO board");
                        robot_mode.data = robot_mode::ERROR_DEVICE;
                        pub_aragorn_robot_mode.publish(robot_mode);
                    }
                    else ;

                    if(_isBattery == false)
                    {
                        ROS_ERROR("[Diagnostic]: Failed to connect BMS");
                    }
                    else ;

                    if(_isIMU == false)
                    {
                        ROS_ERROR("[Diagnostic]: Failed to connect IMU");
                    }
                    else ;

                    ROS_INFO("");

                    _is_output = false;
                }else 
                {
                    ROS_WARN("[Diagnostic]: wait to connect device");
                    counter_wait_recive++;
                }
                
                
            }


            // Access UVC tower.not allowed to use uvc when battery state  lower than 0.2 percentage
            if(system_cautions.caution_code == aragorn_bringup::Cautions::BATTERY_OK)
            {
                // check voltage of battery(cell balance) 
                if(battery_state.voltage >= VOLTAGE_THRESHOLD_UVC_CUTOFF)
                {
                    system_cautions.allowed_uvc = aragorn_bringup::Cautions::UVC_ALLOWED_ON;
                    system_cautions.message = "allow to turn on UVC";
                }
                else
                {
                    system_cautions.allowed_uvc = aragorn_bringup::Cautions::UVC_NOT_ALLOWED_ON_VOLTAGGE_BALANCE;
                    system_cautions.message = "voltage of battery isn't balance";
                }
            }
            else
            {
                system_cautions.allowed_uvc = aragorn_bringup::Cautions::UVC_NOT_ALLOWED_ON_LOW_BATTERY;
                system_cautions.message = "Not allowed to turn on UVC cause battery lower than 20 percent";
            }

            aragorn_bringup::DiagPort port_check;
            port_check.bms = _isBattery;
            port_check.cam1 = _isCam;
            port_check.cam2 = _isCam2;
            port_check.imu = _isIMU;
            port_check.io = _isIO;
            port_check.uvc_interlock = interlock_state.data;
            port_check.lidar = _isRPlidar;
            port_check.w_left = true;
            port_check.w_right = true;
            port_check.system_cuations = system_cautions;
            pub_diag_port.publish(port_check);
            
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aragorn_diagnostics");

    Diagnostic aragorn_dia;

    ros::spin();

    return 0;


}


