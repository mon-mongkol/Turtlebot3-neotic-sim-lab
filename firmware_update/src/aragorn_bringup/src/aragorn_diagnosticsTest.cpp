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
// #include <deligreencs_bms/BatteryData.h>


class Diagnostic
{
    protected:
        ros::NodeHandle nh_;
        ros::Publisher pub_aragorn_diagnostics,
                       pub_aragorn_system_ready,
                       pub_diag_port; 

        ros::Subscriber sub_lds,
                        sub_io,
                        sub_imu,
                        sub_battery,
                        sub_motor,
                        sub_cam,
                        sub_cam2;

        bool _isRPlidar,
             _isIO,
             _isIMU,
             _isBattery,
             _isMotordrive,
             _isCam,
             _isCam2;
        
        bool _is_output = false;
        
        diagnostic_msgs::DiagnosticArray  aragorn_diagnostics;
        diagnostic_msgs::DiagnosticStatus imu_state,
                                          lds_state,
                                          io_state,
                                          batt_state,
                                          motor_state,
                                          cam_state,
                                          cam2_state;
        
        std_msgs::Bool system_status;

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
        float SOC_percent;
        double batt_time_coming_;
        bool _isBattComing = false;
        int batt_wait_recive = 0;
        int counter_wait_recive = 0;

    public:
        Diagnostic()
        {
            pub_aragorn_diagnostics = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
            pub_aragorn_system_ready = nh_.advertise<std_msgs::Bool>("/aragorn/diagnostics/system_ready", 10);
            pub_diag_port = nh_.advertise<aragorn_bringup::DiagPort>("/aragorn/diagnostics", 10);
           
            sub_imu = nh_.subscribe("/imu/data_raw", 10, &Diagnostic::cbIMU, this);
            sub_lds = nh_.subscribe("/scan", 10, &Diagnostic::cbLDS, this);
            sub_io = nh_.subscribe("/aragorn_io/emergency", 10, &Diagnostic::cbIOboard, this);
            sub_battery = nh_.subscribe("/battery_state", 10, &Diagnostic::cbBatteryState, this);
            sub_motor = nh_.subscribe("/zlac706/odom", 10, &Diagnostic::cbMotor, this);
            sub_cam = nh_.subscribe("/camera/depth/color/points", 10, &Diagnostic::cbCam, this);
            sub_cam2 = nh_.subscribe("/camera2/depth/color/points", 10, &Diagnostic::cbCam2, this);
            

            ros::Rate loop_rate(0.5);

            _isBattery = true;

            while(ros::ok())
            {   
                diagnostic_pub();
                system_ready_pub();
                ros::spinOnce();
                loop_rate.sleep();
            }


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

        void cbBatteryState(const sensor_msgs::BatteryState::ConstPtr &msg)
        {
            SOC_percent = msg->percentage * 100;
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

            double time_secs_now_ =ros::Time::now().toSec();
            

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
                if(SOC_percent >= 0.0)
                {
                    setDiagnosisMsg(&batt_state, ok_, "Power System", "Good Condition", "Battery");
                    _isBattery = true;
                }
                else
                {
                    setDiagnosisMsg(&batt_state, error_, "Power System", "Please check USB IO Board connection", "Battery");
                    _isBattery = false;
                }

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

        void system_ready_pub()
        {
            
            if(( _isRPlidar && _isIO && _isIMU && _isCam && _isCam2 && _isBattery) == true)
            // if(( _isRPlidar && _isIO && _isIMU && _isCam ) == true)
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
                
                
                if(counter_wait_recive >= 10)
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

            aragorn_bringup::DiagPort port_check;
            port_check.bms = _isBattery;
            port_check.cam1 = _isCam;
            port_check.cam2 = _isCam2;
            port_check.imu = _isIMU;
            port_check.io = _isIO;
            port_check.lidar = _isRPlidar;
            port_check.w_left = true;
            port_check.w_right = true;
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


