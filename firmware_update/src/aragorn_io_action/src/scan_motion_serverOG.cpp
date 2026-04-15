#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int8MultiArray.h>
#include <aragorn_io_action/ScanMotionAction.h>


enum SequenceState
{
    INIT=0,
    START_SCAN,
    INIT_PIR,
    SCANING,
    SCAN_FINISH
};

class scan_motion_server
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<aragorn_io_action::ScanMotionAction> as_;
        std::string action_name_;
        ros::Subscriber sub_motion;
        aragorn_io_action::ScanMotionFeedback feedback_;
        aragorn_io_action::ScanMotionResult result_;
        
        std_msgs::Int8MultiArray from_pir_, init_pir_;

        bool success = true; 
        bool task_completed = false;
       
        bool scan_result = false;
        int couter_timeout = 0;

    public:
        scan_motion_server(std::string name) :
            as_(nh_, name, boost::bind(&scan_motion_server::executeCB, this, _1), false),
            action_name_(name)
    {
        as_.start();
        as_.registerPreemptCallback(boost::bind(&scan_motion_server::preemptCB, this));

        init_sub();
    }

    void init_sub()
    {
        sub_motion = nh_.subscribe("/aragorn_io/pir_motion", 1, &scan_motion_server::cbPIR, this);
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

    ~scan_motion_server(void)
    {
        
        
    }

    void preemptCB()
    {
        ROS_WARN("%s got preempted!", action_name_.c_str());
        as_.setPreempted();

        success = false; 
        task_completed = true;
        feedback_.current_time = 0;
        feedback_.sequence = SequenceState::INIT;
        result_.result = scan_result;
    }

    void executeCB(const aragorn_io_action::ScanMotionGoalConstPtr &goal)
    {
        ros::Rate rate(5);

        success = true; 
        task_completed = false;
        int counter_time = 5;
        int time_out = goal->scan_time * 1000;
        int couter_timeout = 0;
        ROS_INFO("[Scan_motion ActionServer]: Set scan time from goal %d ms", time_out);

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
                    feedback_.sequence = SequenceState::START_SCAN;
                    break;
                
                case SequenceState::START_SCAN:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Trigger start scan", feedback_.sequence);
                    feedback_.sequence = SequenceState::INIT_PIR;
                    break;

                case SequenceState::INIT_PIR:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Initial state PIR sensor", feedback_.sequence);
                    //initial state of pir sensor when robot in the room 
                    init_pir_ = from_pir_;
                    // for (int i = 0; i < 9; i++) 
                    // {
                    //  ROS_WARN("%d", init_pir_.data[i]);
                    // }
                    feedback_.sequence = SequenceState::SCANING;
                    break;

                case SequenceState::SCANING:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Scanning......", feedback_.sequence);
                    //start scan pir time = depending on action goal
                    while(1)
                    {
                        if(!as_.isActive() || as_.isPreemptRequested())
                        {
                            return;
                        }

                        for(int i=0; i<8 ; i++)
                        {
                            if((init_pir_.data[i] != from_pir_.data[i]))
                            {
                                scan_result = false;
                                i=9;
                            }
                            else
                            {
                                scan_result = true;
                            }
                        }
                        
                        // ROS_INFO("%d", couter_timeout);
                        // ROS_INFO("scan result %d", scan_result);

                        if((couter_timeout >= time_out)  || (scan_result == false)) //motion scan timeout
                        {
                            // success = false;
                            // task_completed = true;
                            break;
                        }
                        else
                        {
                            couter_timeout++;
                            ros::Duration(0.001).sleep();
                        }
                        // feedback_.current_time = couter_timeout;
                    }
                    feedback_.sequence = SequenceState::SCAN_FINISH;
                    break;

                case SequenceState::SCAN_FINISH:
                    ROS_INFO("[Scan_motion ActionServer]: Current step %d. Finish scan", feedback_.sequence);
                    task_completed = true;
                    break;
            }   

            // ROS_INFO(" ");
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