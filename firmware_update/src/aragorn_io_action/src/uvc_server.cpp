#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int8.h>
#include <aragorn_io_action/UVCAction.h>
#include <std_srvs/SetBool.h>

enum SequenceState
{
    INIT=0,
    TURN_ON_UVC,
    TURN_OFF_UVC,
    WAIT_ALL_UVC_ON,
    WARM_UVC_LAMP,
    FINISH,
    COMMAND_NOT_FOUND
};

enum UVC_VERSION
{
    NONE_VERSION=0,
    ALWAYS_ON_VERSION,
    POINT_ON_VERSION
};

enum ModeState
{
    INIT_MODE=0,
    ON_MODE,
    OFF_MODE,
    EMERGENCY_CUT_OFF,
    INTERLOCK_NOT_AVAILABLE,
    TABLET_LOSS_COMMUNICATE,
    SOME_UVC_BROKEN
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
                  EMERGENCY_CASE_ACTIVE,
                  EMERGENCY_CHARGE
                  };

class uvc_server
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<aragorn_io_action::UVCAction> as_;
        std::string action_name_;

        ros::Subscriber sub_uvc_state, sub_robot_mode, sub_interlock;
        ros::ServiceClient csrv_uvc_mode;

        std_srvs::SetBool uvc_srv;

        aragorn_io_action::UVCFeedback feedback_;
        aragorn_io_action::UVCResult result_;
        
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
        bool _isWarm_finish = false;

    public:
        uvc_server(std::string name) :
            as_(nh_, name, boost::bind(&uvc_server::executeCB, this, _1), false),
            action_name_(name)
    {
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
    }

    void init_srv()
    {
        csrv_uvc_mode = nh_.serviceClient<std_srvs::SetBool>("/aragorn_io/uvc");
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
        as_.setPreempted();

        success = false; 
        task_completed = true;
        // feedback_.current_time = 0;
        feedback_.sequence = SequenceState::INIT;
        result_.result = false;

        if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
        {
            ;
        }
        else
        {
            uvc_srv.request.data = false;
            if(csrv_uvc_mode.call(uvc_srv))
            {
                ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken", feedback_.sequence);
            }
        }
        // task_completed = true;
        // success = false;
        
    }

    void executeCB(const aragorn_io_action::UVCGoalConstPtr &goal)
    {
        ros::Rate rate(50);

        success = true; 
        task_completed = false;
        wait_uvc_on_secs = 25;
        feedback_.warming_countdown = 0.0;
        uvc_on_order = 0;
        // int counter_time = 5;
        // int time_out = goal->uvc_warm_time;
        // int couter_timeout = 0;
        ROS_INFO("[UVC_lamp ActionServer]: Set warm time from goal %d s", goal->uvc_warm_time_secs);

        //mode select
        if((goal->uvc_mode) == ModeState::ON_MODE)
        {
            feedback_.sequence = SequenceState::INIT;
        }
        else if((goal->uvc_mode) == ModeState::OFF_MODE)
        {
            feedback_.sequence = SequenceState::TURN_OFF_UVC;
        }
        else 
        {
            task_completed = true;
            feedback_.sequence = SequenceState::COMMAND_NOT_FOUND;
            uvc_result = true;
        }
        

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
                        
                        if(uvc_state.data == 1) //uvc on
                        {
                            task_completed = true;
                            success = false;
                        }
                        else if(uvc_state.data == 0)
                        {
                            feedback_.warming_countdown = 0.0;
                            uvc_result = ModeState::OFF_MODE;
                            feedback_.sequence = SequenceState::FINISH;
                            ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Turn off uvc complete", feedback_.sequence);
                        }
                        
                    }
                    else
                    {
                        ROS_ERROR("[UVC_lamp ActionServer]: Failed to call service uvc_srv");
                        task_completed = true;
                        success = false;
                    }
                    uvc_on_order = 0;

                    // ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Uvc state is : %d ", uvc_result);
                    // task_completed = true;
                    // feedback_.sequence = SequenceState::FINISH;
                    break;

                case SequenceState::INIT:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. *** Trigger turn on uvc ***", feedback_.sequence);

                    //check circuit before turn on UVC
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Check Interlock button ..", feedback_.sequence);
                    if(interlock_state.data == 0)
                    {
                        ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Interlock button active ", feedback_.sequence);
                        uvc_on_order = 1;
                        feedback_.warming_countdown = 0.0;
                        feedback_.sequence = SequenceState::TURN_ON_UVC;
                    }
                    else
                    {
                        ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Interlock not avialable ", feedback_.sequence);
                        task_completed = true;
                        success = false;
                        uvc_result = ModeState::INTERLOCK_NOT_AVAILABLE;
                    }
                    break;
                
                case SequenceState::TURN_ON_UVC:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Turn on uvc lamp service", feedback_.sequence);
                    uvc_srv.request.data = true;
                    if(csrv_uvc_mode.call(uvc_srv))
                    {

                        if(uvc_srv.response.success == true)
                        {
                            on_time_secs = ros::Time::now().toSec() + wait_uvc_on_secs; 
                            feedback_.sequence = SequenceState::WAIT_ALL_UVC_ON;
                        }
                        else //inter lock and/or master on not available
                        {
                            task_completed = true;
                            success = false;
                        }
                    }
                    else
                    {
                        ROS_ERROR("[UVC_lamp ActionServer]: Failed to call service uvc_srv");
                        task_completed = true;
                        success = false;
                    }

                    //check emergency case && tablet loss communicate
                    // if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
                    // {
                    //     ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp Tablet loss communicate", feedback_.sequence);
                    //     task_completed = true;
                    //     success = false;
                    //     uvc_result = ModeState::TABLET_LOSS_COMMUNICATE;
                    // }
                    feedback_.warming_countdown = 0.0;
                    break;
                
                case SequenceState::WAIT_ALL_UVC_ON:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Wait all uvc lamp on..", feedback_.sequence);
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. uvc_on_order %d", feedback_.sequence, uvc_on_order);
                    if(time_now_secs >= on_time_secs) //wait 30s after trig uvc on
                    {
                        if(uvc_state.data == 1) //all uvc lamp turn on 
                        {
                            // check operation version
                            if(goal->uvc_version == UVC_VERSION::POINT_ON_VERSION)
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
                                //off uvc lamp system
                                uvc_srv.request.data = false;
                                if(!csrv_uvc_mode.call(uvc_srv))
                                {
                                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken", feedback_.sequence);
                                }
                                ros::Duration(3.0).sleep();
                            }
                            else 
                            {
                                ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken", feedback_.sequence);
                                //off uvc lamp system
                                uvc_on_order = 0;
                                uvc_srv.request.data = false;
                                if(!csrv_uvc_mode.call(uvc_srv))
                                {
                                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Some uvc lamp has broken", feedback_.sequence);
                                }
                                task_completed = true;
                                success = false;
                                uvc_result = ModeState::SOME_UVC_BROKEN;
                            }
			                
                        }
                    }
                    else
                    {
                        ;
                    }

                    //check emergency case && tablet loss communicate
                    // if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
                    // {
                    //     ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp Tablet loss communicate", feedback_.sequence);
                    //     task_completed = true;
                    //     success = false;
                    //     uvc_result = ModeState::TABLET_LOSS_COMMUNICATE;
                    // }
                    feedback_.warming_countdown = 0.0;
                    break;

                case SequenceState::WARM_UVC_LAMP:
                    ROS_INFO("[UVC_lamp ActionServer]: Currentt step %d. Warming uvc lamp success in %d sec ", feedback_.sequence, counter_warmimg);
                    uvc_on_order = 0;
                    ROS_INFO("[UVC_lamp ActionServer]: point number %d ", goal->point_no);
                    

                    if(time_now_secs >= warm_time_secs) //motion scan timeout
                    {
                        // uvc_result = ModeState::ON_MODE;
                        // feedback_.sequence = SequenceState::FINISH;
                        _isWarm_finish = true;
                        
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

                    //check uvc state while uvc warming 
                    if(uvc_state.data == 1)
                    {
                        if(_isWarm_finish)
                        {
                            uvc_result = ModeState::ON_MODE;
                            feedback_.sequence = SequenceState::FINISH;
                        }else;
                    }
                    else
                    {
                        task_completed = true;
                        success = false;
                        uvc_result = ModeState::SOME_UVC_BROKEN;
                        feedback_.sequence = SequenceState::FINISH;
                    }

                    //check emergency case && tablet loss communicate
                    // if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
                    // {
                    //     ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp Tablet loss communicate", feedback_.sequence);
                    //     task_completed = true;
                    //     success = false;
                    //     uvc_result = ModeState::TABLET_LOSS_COMMUNICATE;
                    // }

                    break;

                case SequenceState::FINISH:
                    ROS_INFO("[UVC_lamp ActionServer]: Current step %d. Finish turn on and warm uvc lamp", feedback_.sequence);
                    feedback_.warming_countdown = 0.0;
                    task_completed = true;
                    break;
            };

            //check emergency case && tablet loss communicate
            if(robot_state.data == robot_mode::TABLET_LOSS_COMMU)
            {
                task_completed = true;
                success = false;
                uvc_result = ModeState::TABLET_LOSS_COMMUNICATE;
                ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp problem: %d ", feedback_.sequence, uvc_result);
            }else if(robot_state.data == robot_mode::EMERGENCY_CASE_ACTIVE)
            {
                uvc_result = ModeState::EMERGENCY_CUT_OFF;
                task_completed = true;
                success = false;
                ROS_ERROR("[UVC_lamp ActionServer]: Current step %d. Cut off uvc lamp problem: %d ", feedback_.sequence, uvc_result);
            }   
            
            //clear order 
            // uvc_on_order = 0;
            time_now_secs = ros::Time::now().toSec();
            as_.publishFeedback(feedback_);
            rate.sleep();
            // set the action state to succeeded
            // as_.setSucceeded(result_);
        }
        
        result_.result = success;
        result_.uvc_state = uvc_result;
        as_.setSucceeded(result_);
        ROS_INFO("[UVC_lamp ActionServer]: Result of uvc lamp process is : %d and uvc_state is : %d", result_.result, uvc_result);
        ROS_INFO("%s: %s", action_name_.c_str() ,(success)? "Succeeded":"Fail" );
        // ROS_INFO(" ");
        ROS_INFO("*************************");

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
