#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <aragorn_io_action/ScanMotionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <aragorn_io_action/UVCAction.h>

#define TIME_OUT_WAIT_PIR_READY 5 //sec
#define PIR_UNDETECT_STATE 1
#define NUMBER_PIR 8
#define WAIT_ALL_UVC_ON 30 //sec

typedef actionlib::SimpleActionClient<aragorn_io_action::UVCAction> clientUVC;

enum robot_mode{  INITIAL           = 0,
                  IDLE              = 1,
                  START_MOTOR       = 2,
                  SHUTDOWN_MOTOR    = 3,
                  SHUTDOWN_ROBOT    = 4,
                  EMERGENCY         = 5,
                  FUCN_1            = 6,
                  DOWN_STAIRS       = 7,
                  TABLET_LOSS_COMMU = 8,
                  UVC_ON            = 9,
                  UVC_OFF           = 10,
                  READY_TO_START    = 11,
                  DOCKING_MODE_ON   = 12,
                  DOCKING_MODE_OFF  = 13,
                  ERROR_DEVICE      = 14,
                  CHARGER_ON        = 15,
                  CHARGER_OFF       = 16,
                  RESET             = 17,
                  EMERGENCY_CASE_ACTIVE = 18,
                  EMERGENCY_CHARGE = 19
                  };

enum MOTION_VERSION
 {
     NONE_VERSION = 0,
     CHECK_ONLY_BEFORE_START_VERSION, 
     ALONGSIDE_UVC_ON_VERSION
};

enum UVC_ModeState
{
    INIT_MODE=0,
    ON_MODE,
    OFF_MODE,
    EMERGENCY_CUT_OFF,
    UVC_INTERLOCK_NOT_AVAILABLE,
    TABLET_LOSS_COMMUNICATE
};

enum SequenceState
{
    INIT=0,
    START_SCAN,
    WAIT_PIR_READY,
    PIR_READY,
    INIT_PIR,
    SCANING,
    SCAN_FINISH
};

enum ScanResult
{
    SOMETHING_MOVE, 
    SAFETY,
    INTERLOCK_NOT_AVAILABLE,
    NONE_STATUS
};

class scan_motion_server
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<aragorn_io_action::ScanMotionAction> as_;
        
        std::string action_name_;
        ros::Subscriber sub_motion, sub_interlock, sub_robot_mode;


        aragorn_io_action::ScanMotionFeedback feedback_;
        aragorn_io_action::ScanMotionResult result_;
        
        std_msgs::Int8MultiArray from_pir_, init_pir_, pir_ready_state_;
        std_msgs::Int8 from_interlock, robot_state;

        aragorn_io_action::UVCGoal uvc_goal;
        clientUVC ac_uvc;
        bool uvcCompleted = false;


        bool success = true; 
        bool task_completed = false;
        bool pir_ready = false;
       
        int scan_result = ScanResult::NONE_STATUS;
        
        int couter_timeout = 0;

        int number_motion_ = 8;

        double scan_time_out, time_now_secs, timeout_wait_ready;

        std::string SERIAL_NO = "20210001B";
        int pir_undetect_state_ = 0;
        int scan_detected_counter_ = 0;

    public:
        scan_motion_server(std::string name) : as_(nh_, name, boost::bind(&scan_motion_server::executeCB, this, _1), false),
                                               action_name_(name),
                                               ac_uvc("/uvc_server", true)
    {
        as_.start();

        while(!ac_uvc.waitForServer())
        {
            ROS_INFO("[scanmotion_server]: Waiting for the uvc_server action server to come up");
        }

        as_.registerPreemptCallback(boost::bind(&scan_motion_server::preemptCB, this));

        init_sub();
    }

    void init_sub()
    {
        sub_motion = nh_.subscribe("/aragorn_io/pir_motion", 1, &scan_motion_server::cbPIR, this);
        sub_interlock = nh_.subscribe("/aragorn_io/interlock_state", 1, &scan_motion_server::cbInterlock, this);
        sub_robot_mode = nh_.subscribe("/aragorn/robot_mode", 1, &scan_motion_server::cbRobot_mode, this);
        ros::param::param<std::string>("~serial_no", SERIAL_NO, "20210001B");

        number_motion_ = (SERIAL_NO == "20210009A")? 4 : 8;
        pir_undetect_state_ = (SERIAL_NO == "20210009A")? 0 : 1;
        ROS_INFO("number of motion: %d", number_motion_);
        ROS_INFO("pir undetect state: %d", pir_undetect_state_);
    }

    void cbInterlock(const std_msgs::Int8 &msg)
    {
        from_interlock = msg;
    }

    void cbRobot_mode(const std_msgs::Int8 &msg)
    {
        robot_state = msg;
    }


    void cbPIR(const std_msgs::Int8MultiArray &msg)
    {
        from_pir_ = msg;
        // for (int i = 0; i < 9; i++) 
        // {
        //  ROS_WARN("%d", msg.data[i]);
        // }
        // ROS_INFO(" ");
    }

    void uvcDoneCb(const actionlib::SimpleClientGoalState &state)
    {
        ROS_INFO("DONECB: Finish in state [%s]", state.toString().c_str());
        uvcCompleted = true;
    }

    ~scan_motion_server(void)
    {
        
        
    }

    void preemptCB()
    {
        ROS_WARN("%s got preempted!", action_name_.c_str());

        success = true; 
        task_completed = true;
        feedback_.current_time = 0;
        feedback_.sequence = SequenceState::INIT;
        result_.result = ScanResult::SAFETY;
        scan_result = ScanResult::SAFETY;

        // as_.setPreempted(result_);
    }

    void executeCB(const aragorn_io_action::ScanMotionGoalConstPtr &goal)
    {
        ros::Rate rate(5);

        success = true; 
        task_completed = false;
        int time_out;
        int couter_timeout = 0;

        feedback_.sequence = SequenceState::INIT;

        if(!as_.isActive() || as_.isPreemptRequested()) return;

        while(!task_completed)
        {
            //Check for ros
            if(!ros::ok())
            {
                // result.final_count = progress;
                // as_.setAborted(result,"I failed !");
                ROS_INFO("%s Shutting down",action_name_.c_str());
                break;
            }

            if(!as_.isActive() || as_.isPreemptRequested())
            {
                return;
            }

            switch(feedback_.sequence)
            {
                case SequenceState::INIT:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. *** Scan motion server ***", feedback_.sequence);
                    //check interlock state 
                    if(from_interlock.data == 1)
                    {
                        success = true;
                        task_completed = true;
                        scan_result = ScanResult::INTERLOCK_NOT_AVAILABLE; //interlock not avilable
                        result_.result_message = "INTERLOCK_NOT_AVAILABLE";
                    } 
                    else
                    {
                        feedback_.sequence = SequenceState::START_SCAN;
                    }
                    break;
                
                case SequenceState::START_SCAN:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Trigger start scan", feedback_.sequence);
                    timeout_wait_ready = ros::Time::now().toSec() + TIME_OUT_WAIT_PIR_READY;
                    feedback_.sequence = SequenceState::WAIT_PIR_READY;
                    break;
                
                case SequenceState::WAIT_PIR_READY:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. wait PIR ready", feedback_.sequence);
                    
                    for(int i=0; i<number_motion_ ; i++)
                    {
                        if(from_pir_.data[i] != pir_undetect_state_)
                        {
                            pir_ready = false;
                            // i=999;
                            break;
                        }
                        else
                        {
                            pir_ready = true;
                        }
                        ROS_INFO("status_pir: %d = %d",i ,from_pir_.data[i]);
                    }

                    if(pir_ready == true)
                    {
                        feedback_.sequence = SequenceState::PIR_READY;
                    }
                    else    
                    {
                        feedback_.sequence = SequenceState::WAIT_PIR_READY;
                        if(ros::Time::now().toSec() >= timeout_wait_ready)
                        {
                            ROS_ERROR("[Scan_motion ActionServer)]: Something still in the room!!");
                            task_completed = true;
                            success = true;
                            // feedback_.sequence = SequenceState::SCAN_FINISH;
                            scan_result = ScanResult::SOMETHING_MOVE;
                            result_.result_message = "SOMETHING_MOVE";
                        }
                    }   

                    // ros::Duration(2.5).sleep();
                    break;

                case SequenceState::PIR_READY:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d.PIR ready", feedback_.sequence);
                    scan_result = ScanResult::SAFETY;
                    feedback_.sequence = SequenceState::INIT_PIR;
                    break;

                case SequenceState::INIT_PIR:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Initial state PIR sensor", feedback_.sequence);
                    //initial state of pir sensor when robot in the room 
                    init_pir_ = from_pir_;
                    //set scan time out
                    if(goal->scan_version == MOTION_VERSION::ALONGSIDE_UVC_ON_VERSION)
                    {
                        scan_time_out = ros::Time::now().toSec() + goal->scan_time + WAIT_ALL_UVC_ON;
                    }
                    else
                    {
                        scan_time_out = ros::Time::now().toSec() + goal->scan_time;
                    }
                    ROS_INFO("[Scan_motion ActionServer]: use version %d", goal->scan_version);
                    ROS_INFO("[Scan_motion ActionServer]: scan_time_out %d", scan_time_out);
                    feedback_.sequence = SequenceState::SCANING;
                    break;

                case SequenceState::SCANING:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Scanning......", feedback_.sequence);

                    if((ros::Time::now().toSec() >= scan_time_out) || (scan_result != ScanResult::SAFETY))
                    {
                        if(scan_result != ScanResult::SAFETY)
                        {
                            ROS_ERROR("[Scan_motion ActionServer)]: Something still in the room!!");
                            // uvc_goal.uvc_mode = UVC_ModeState::OFF_MODE;
                            // ac_uvc.sendGoal(uvc_goal);
                            task_completed = true;
                            success = true;
                            feedback_.sequence = SequenceState::SCAN_FINISH;
                            scan_result = ScanResult::SOMETHING_MOVE;
                            result_.result_message = "SOMETHING_MOVE";
                        }
                        else
                        {
                            feedback_.sequence = SequenceState::SCAN_FINISH;
                            scan_result = ScanResult::SAFETY;
                            result_.result_message = "SAFETY";
                        }
                    }
                    else
                    {
                        if(robot_state.data == robot_mode::EMERGENCY_CASE_ACTIVE)
                        {
                            scan_result = ScanResult::SOMETHING_MOVE;
                            result_.result_message = "SOMETHING_MOVE CUTOFF BY PIR";
                        } 

                        for(int i=0; i<NUMBER_PIR ; i++)
                        {
                            if((init_pir_.data[i] != from_pir_.data[i]))
                            {
                                scan_result = ScanResult::SOMETHING_MOVE;
                                i=9;
                            }
                            else
                            {
                                scan_result = ScanResult::SAFETY;
                            }
                        }
                    }
                    
                    break;

                case SequenceState::SCAN_FINISH:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Finish scan", feedback_.sequence);
                    task_completed = true;
                    break;
            }   

            //check emergency case && tablet loss communicate
            // if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
            // {
            //     task_completed = true;
            //     success = true;
            //     scan_result = ScanResult::SOMETHING_MOVE;
            //     ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp problem: %d ", feedback_.sequence, scan_result);
            // }else if(robot_state.data == robot_mode::EMERGENCY_CHARGE)
            // {
            //     scan_result = ScanResult::SOMETHING_MOVE;
            //     task_completed = true;
            //     success = true;
            //     ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp problem: %d ", feedback_.sequence, scan_result);
            // }else;

            // ROS_INFO(" ");
            time_now_secs = ros::Time::now().toSec();
            // ROS_INFO("time_now_secs %d", ros::Time::now().toSec());
            as_.publishFeedback(feedback_);
            rate.sleep();
            // set the action state to succeeded
            // as_.setSucceeded(result_);
        }

        
        result_.result = scan_result;
        as_.setSucceeded(result_);
        ROS_INFO("[Scan_motion ActionServer]: Result of motion scan is %d", result_.result);
        ROS_INFO("%s: %s", action_name_.c_str() ,(success)? "Succeeded":"Fail" );
        // ROS_INFO(" ");
        ROS_INFO("*************************");

    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_motion_server");
  ROS_INFO("Start scan motion");

  scan_motion_server missions("scan_motion_server");
  ros::spin();

  return 0;
}