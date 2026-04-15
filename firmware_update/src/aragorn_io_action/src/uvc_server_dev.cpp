#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int8.h>
#include <aragorn_io_action/UVCAction.h>
#include <std_srvs/SetBool.h>
#include <aragorn_bringup/DiagPort.h>

enum SequenceState
{
    INIT                = 0,
    TURN_ON_UVC         = 1,
    TURN_OFF_UVC        = 2,   
    WAIT_ALL_UVC_ON     = 3,
    WARM_UVC_LAMP       = 4,
    FINISH              = 5,
    COMMAND_NOT_FOUND   = 6
};

enum UVC_VERSION   
{
    NONE_VERSION        = 0,
    ALWAYS_ON_VERSION   = 1,
    POINT_ON_VERSION    = 2
};

enum LAMP_STATE
{
    ALL_LAMP_OFF    = 0,
    ALL_LAMP_ON     = 1
};

enum ModeState
{
    INIT_MODE_STATE = 0,
    ON_MODE         = 1,
    OFF_MODE        = 2
};

enum UVC_STATE
{
    UVC_RESULT_INIT                         = 0,
    UVC_SUCCESS_TURN_ON                     = 1,
    UVC_NOT_ALLOWED_ON_VOLTAGGE_BALANCE     = 2,
    UVC_NOT_ALLOWED_ON_LOW_BATTERY          = 3,
    UVC_UNKNOW                              = 4,
    UVC_SERVICE_ERROR                       = 5,
    UVC_HARDWARE_ERROR                      = 6,
    UVC_INTERLOCK_INACTIVE                  = 7,
    UVC_CANNT_TURN_OFF                      = 8,
    UVC_SUCCESS_TURN_OFF                    = 9,
    UVC_TABLET_LOSS_COMMUNICATE             = 10,
    UVC_PIR_CUTOFF_CE_VER                   = 11,
    UVC_PREEMPT_OFF_SUCCESS                 = 12,
    UVC_PREEMPT_CANNT_OFF                   = 13
};

enum RESULT
{
    RESULT_PROCESS_FAIL     = 0,
    RESULT_PROCESS_SUCCESS  = 1,
    RESULT_PREEMPT          = 2
};

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
                  EMERGENCY_CASE_ACTIVE
                  };

class uvc_server
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<aragorn_io_action::UVCAction> as_;
        std::string action_name_;

        ros::Subscriber sub_uvc_state, sub_robot_mode, sub_interlock, sub_diag;
        ros::ServiceClient csrv_uvc_mode;

        std_srvs::SetBool uvc_srv;

        aragorn_io_action::UVCFeedback feedback_;
        aragorn_io_action::UVCResult result_;

        aragorn_bringup::DiagPort robot_diag;
        
        std_msgs::Int8 uvc_state, robot_state, interlock_state;

        bool success = true; 
        bool task_completed = false;

        // int uvc_state;
       
        int uvc_result = 0;
        int couter_timeout = 0;
        int counter_warmimg = 0;
        int uvc_on_order = 0;

        double time_now_secs, on_time_secs, warm_time_secs, wait_uvc_on_secs, last_time_feedback;
        float pub_period = 1;

    public:
        uvc_server(std::string name) :
            as_(nh_, name, boost::bind(&uvc_server::executeCB, this, _1), false),
            action_name_(name)
    {
        ros::service::waitForService("/aragorn_io/uvc", -1);

        as_.start();
        as_.registerPreemptCallback(boost::bind(&uvc_server::preemptCB, this));

        init_sub();
        init_srv();

        
    }

    void init_sub()
    {
        sub_uvc_state = nh_.subscribe("/aragorn_io/uvc_state", 1, &uvc_server::cbUVC_state, this);
        sub_robot_mode = nh_.subscribe("/aragorn/robot_mode", 1, &uvc_server::cbRobot_mode, this);
        sub_interlock = nh_.subscribe("/aragorn_io/interlock_state", 1, &uvc_server::cbInterlock_state, this);
        sub_diag = nh_.subscribe("/aragorn/diagnostics", 1, &uvc_server::cbDiag, this);
    }

    void init_srv()
    {
        csrv_uvc_mode = nh_.serviceClient<std_srvs::SetBool>("/aragorn_io/uvc");
    }

    void cbDiag(const aragorn_bringup::DiagPort &msg)
    {
        robot_diag = msg;
    }

    void cbRobot_mode(const std_msgs::Int8 &msg)
    {
        robot_state = msg;
    }

    void cbUVC_state(const std_msgs::Int8 &msg)
    {
        uvc_state = msg;
    }

    void cbInterlock_state(const std_msgs::Int8 &msg)
    {
        interlock_state = msg;
    }

    ~uvc_server(void)
    {
        
        
    }

    void preemptCB()
    {
        ROS_WARN("%s got preempted!", action_name_.c_str());
        // as_.setPreempted();

        task_completed = true;
        feedback_.sequence = feedback_.sequence;
        success = RESULT::RESULT_PROCESS_SUCCESS;

        if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
        {
            ;
        }
        else
        {
            // turn off lamp
            uvc_srv.request.data = false;
            if(csrv_uvc_mode.call(uvc_srv))
            {
                ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken", feedback_.sequence);
                //wait turn off lamp
                ros::Duration(1).sleep();
            }
        }

        //check lamp state after turn off
        if(uvc_state.data == LAMP_STATE::ALL_LAMP_ON) 
        {
            uvc_result = UVC_STATE::UVC_PREEMPT_CANNT_OFF;
        }
        else if(uvc_state.data == LAMP_STATE::ALL_LAMP_OFF)
        {
            uvc_result = UVC_STATE::UVC_PREEMPT_OFF_SUCCESS;
        }
        
        
    }

    void executeCB(const aragorn_io_action::UVCGoalConstPtr &goal)
    {
        ros::Rate rate(50);

        success = true; 
        task_completed = false;
        wait_uvc_on_secs = 25;
        feedback_.warming_countdown = 0.0;
        uvc_on_order = 0;
        ROS_INFO("[UVC_lamp ActionServer]: Set warm time from goal %d s", goal->uvc_warm_time_secs);

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
                case SequenceState::TURN_OFF_UVC:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. *** Trigger turn off uvc ***", feedback_.sequence);
                    uvc_srv.request.data = false;
                    if(csrv_uvc_mode.call(uvc_srv))
                    {
                        //wait turn off lamp
                        ros::Duration(2).sleep();

                        //check lamp state after turn off
                        if(uvc_state.data == LAMP_STATE::ALL_LAMP_ON) 
                        {
                            task_completed = true;
                            success = false;
                            uvc_result = UVC_STATE::UVC_SUCCESS_TURN_ON;
                            result_.result_message = ":UVC_SUCCESS_TURN_OFF";
                        }
                        else if(uvc_state.data == LAMP_STATE::ALL_LAMP_OFF)
                        {
                            feedback_.warming_countdown = 0.0;
                            uvc_result = UVC_STATE::UVC_SUCCESS_TURN_OFF;
                            result_.result_message = "UVC_SUCCESS_TURN_OFF";
                            feedback_.sequence = SequenceState::FINISH;
                            ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Turn off uvc complete", feedback_.sequence);
                        }
                        
                    }
                    else
                    {
                        ROS_ERROR("[UVC_lamp ActionServer]: Failed to call service uvc_srv");
                        task_completed = true;
                        success = false;
                        uvc_result = UVC_STATE::UVC_SERVICE_ERROR;
                        result_.result_message = "UVC_SERVICE_ERROR";
                    }

                    uvc_on_order = 0;
                    break;

                case SequenceState::INIT:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. *** Trigger turn on uvc ***", feedback_.sequence);

                    //mode select
                    if((goal->uvc_mode) == ModeState::ON_MODE)
                    {
                        //check interlock circuit before turn on UVC
                        ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Check Interlock button ..", feedback_.sequence);
                        if(robot_diag.uvc_interlock == aragorn_bringup::DiagPort::UVC_INTERLOCK_ACTIVE)
                        {
                            ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Interlock button active ", feedback_.sequence);
                            // check allowed turn on uvc lamp from diagnostics
                            if(robot_diag.system_cuations.allowed_uvc == aragorn_bringup::Cautions::UVC_ALLOWED_ON)
                            {
                                ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Diagnostics allowed turn on lamp ", feedback_.sequence);
                                uvc_on_order = 1;
                                feedback_.warming_countdown = 0.0;
                                feedback_.sequence = SequenceState::TURN_ON_UVC;
                            }
                            // if cannt turn on, check caution code
                            else
                            {   
                                if(robot_diag.system_cuations.caution_code == aragorn_bringup::Cautions::UVC_NOT_ALLOWED_ON_LOW_BATTERY)
                                {
                                    uvc_result = UVC_STATE::UVC_NOT_ALLOWED_ON_LOW_BATTERY;
                                    result_.result_message = "UVC_NOT_ALLOWED_ON_LOW_BATTERY";
                                    task_completed = true;
                                    success = false;
                                }
                                else if(robot_diag.system_cuations.caution_code == aragorn_bringup::Cautions::UVC_NOT_ALLOWED_ON_VOLTAGGE_BALANCE)
                                {
                                    uvc_result = UVC_STATE::UVC_NOT_ALLOWED_ON_VOLTAGGE_BALANCE;
                                    result_.result_message = "UVC_NOT_ALLOWED_ON_VOLTAGGE_BALANCE";
                                    task_completed = true;
                                    success = false;
                                }
                                else
                                {
                                    task_completed = true;
                                    success = false;
                                    uvc_result = UVC_STATE::UVC_UNKNOW;
                                    result_.result_message = "UVC_UNKNOW";
                                }
                                ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Diagnostics not allowed turn on, uvc_result: %d", feedback_.sequence, uvc_result);
                                
                            }
                            
                        }
                        else
                        {
                            task_completed = true;
                            success = false;
                            uvc_result = UVC_STATE::UVC_INTERLOCK_INACTIVE;
                            result_.result_message = "UVC_INTERLOCK_INACTIVE";
                            ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Interlock inactive, uvc_result: %d", feedback_.sequence, uvc_result);
                        }
                    }
                    else if((goal->uvc_mode) == ModeState::OFF_MODE)
                    {
                        feedback_.sequence = SequenceState::TURN_OFF_UVC;
                    }
                    else 
                    {
                        task_completed = true;
                        success = false;
                        uvc_result = UVC_STATE::UVC_UNKNOW;
                        result_.result_message = "UVC_COMMAND_NOT_FOUND";
                        ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Not found mode, uvc_result: %d", feedback_.sequence, uvc_result);

                    }

                    //check emergency case && tablet loss communicate
                    check_system_error();
                    break;
                
                case SequenceState::TURN_ON_UVC:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Turn on uvc lamp service", feedback_.sequence);
                    // turn on uvc 
                    uvc_srv.request.data = true;
                    if(csrv_uvc_mode.call(uvc_srv))
                    {
                        // check response of service
                        if(uvc_srv.response.success == true)
                        {
                            on_time_secs = ros::Time::now().toSec() + wait_uvc_on_secs; 
                            feedback_.sequence = SequenceState::WAIT_ALL_UVC_ON;
                        }
                        //inter lock and/or master on inactive
                        else 
                        {
                            task_completed = true;
                            success = false;
                            uvc_result = UVC_STATE::UVC_INTERLOCK_INACTIVE;
                            result_.result_message = "UVC_INTERLOCK_INACTIVE";
                            ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Interlock inactive, uvc_result: %d", feedback_.sequence, uvc_result);

                        }
                    }
                    // service inactive
                    else
                    {
                        task_completed = true;
                        success = false;
                        uvc_result = UVC_STATE::UVC_SERVICE_ERROR;
                        result_.result_message = "UVC_SERVICE_ERROR";
                        ROS_ERROR("[UVC_lamp ActionServer]: Failed to call service uvc_srv");
                    }

                    check_system_error();
                    feedback_.warming_countdown = 0.0;
                    break;
                
                case SequenceState::WAIT_ALL_UVC_ON:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Wait all uvc lamp on..", feedback_.sequence);
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. uvc_on_order %d", feedback_.sequence, uvc_on_order);
                    //wait for uvc turn on OR time out
                    if((time_now_secs >= on_time_secs) || (uvc_state.data == LAMP_STATE::ALL_LAMP_ON))
                    {
                        //all uvc lamp turn on
                        if(uvc_state.data == LAMP_STATE::ALL_LAMP_ON)  
                        {
                            // check operation version
                            if(goal->uvc_version == UVC_VERSION::POINT_ON_VERSION) // CE mark ver.
                            {
                                // uvc warm time use with 1st start (first point)
                                if(goal->point_no == 0)
                                {
                                    warm_time_secs = ros::Time::now().toSec() + goal->uvc_warm_time_secs;
                                    counter_warmimg = goal->uvc_warm_time_secs;
                                    
                                }
                                else
                                {
                                    warm_time_secs = ros::Time::now().toSec() + 0;
                                    counter_warmimg = 0;
                                }

                                feedback_.sequence = SequenceState::WARM_UVC_LAMP;
                            }
                            else
                            {
                                ROS_INFO("[UVC_lamp ActionServer]: Current step %d. All uvc lamp turn on", feedback_.sequence);
                                warm_time_secs = ros::Time::now().toSec() + goal->uvc_warm_time_secs;
                                counter_warmimg = goal->uvc_warm_time_secs;
                                feedback_.sequence = SequenceState::WARM_UVC_LAMP;
                            }

                        }
                        else
                        {
                            //check order turn on uvc lamp for 1st turn fail and retry
                            if(uvc_on_order == 1)
                            {
                                ROS_WARN("[UVC_lamp ActionServer]: Current step %d. 1st turn on fail.Retry turn on uvc lamp", feedback_.sequence);
                                feedback_.sequence = SequenceState::TURN_ON_UVC;
                                uvc_on_order++;
                            }
                            else 
                            {
                                
                                //off uvc lamp system
                                uvc_on_order = 0;
                                uvc_srv.request.data = false;
                                if(!csrv_uvc_mode.call(uvc_srv))
                                {
                                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken", feedback_.sequence);
                                }
                                task_completed = true;
                                success = false;
                                uvc_result = UVC_STATE::UVC_HARDWARE_ERROR;
                                result_.result_message = "UVC_HARDWARE_ERROR";
                                ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken, uvc_result: %d", feedback_.sequence, uvc_result);
                            }
			                
                        }
                    }
                    else
                    {
                        ;
                    }

                    check_system_error();
                    feedback_.warming_countdown = 0.0;
                    break;

                case SequenceState::WARM_UVC_LAMP:
                    ROS_INFO("[UVC_lamp ActionServer]: Currentt step %d. Warming uvc lamp success in %d sec ", feedback_.sequence, counter_warmimg);
                    uvc_on_order = 0;
                    
                    if(time_now_secs >= warm_time_secs) //motion scan timeout
                    {
                        feedback_.sequence = SequenceState::FINISH;
                    }
                    else
                    {
                        if((time_now_secs - last_time_feedback) > pub_period)
                        {
                            counter_warmimg--;
                            feedback_.warming_countdown = counter_warmimg;
                            last_time_feedback = time_now_secs;
                        }
                        else
                        {
                            ;
                        }
                        // couter_timeout++;
                        // ros::Duration(0.001).sleep();
                        // feedback_.sequence = SequenceState::WARM_UVC_LAMP;
                        // ;
                    }

                    check_system_error();

                    break;

                case SequenceState::FINISH:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Finish turn on and warm uvc lamp", feedback_.sequence);
                    uvc_result = UVC_STATE::UVC_SUCCESS_TURN_ON;
                    result_.result_message = "UVC_SUCCESS_TURN_ON";
                    success = true;
                    feedback_.warming_countdown = 0.0;
                    task_completed = true;
                    break;
            }   
            time_now_secs = ros::Time::now().toSec();
            as_.publishFeedback(feedback_);
            rate.sleep();
        }

        
        result_.result = success;
        result_.uvc_state = uvc_result;
        as_.setSucceeded(result_);
        ROS_INFO("[UVC_lamp ActionServer]: Result of uvc lamp process is : %d and uvc_result is : %d", result_.result, uvc_result);
        ROS_INFO("%s: %s", action_name_.c_str() ,(success)? "Succeeded":"Fail" );
        ROS_INFO("*************************");

    }

    void check_system_error()
    {
        //check emergency case && tablet loss communicate
        if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
        {
            ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp Tablet loss communicate", feedback_.sequence);
            task_completed = true;
            success = false;
            uvc_result = UVC_STATE::UVC_TABLET_LOSS_COMMUNICATE;
        }
        else if(robot_state.data == robot_mode::EMERGENCY_CASE_ACTIVE)
        {
            ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp PIR cut off (CE ver.)", feedback_.sequence);
            task_completed = true;
            success = false;
            uvc_result = UVC_STATE::UVC_PIR_CUTOFF_CE_VER;
        }
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uvc_server");
  ROS_INFO("Start uvc_server");

  uvc_server missions("uvc_server");
  ros::spin();

  return 0;
}
