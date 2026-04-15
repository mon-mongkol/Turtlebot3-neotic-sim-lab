#include "matrix_schedule_operation/matrix_schedule_operation_node.h"

bool ScheduleOperation::executeCB(const matrix_msgs::ScheduleGoalConstPtr &goal)
{
    enum ScheduleControlState
    {
        INIT = 0,
        GET_GOAL,
        CHECK_GOAL,
        SET_TIMMER,
        WAIT_TIMMER_FINISH,
        FINISH,
        ERROR
    };

    ros::Rate r(1);
    feedback_.sequence = ScheduleControlState::INIT;

    ROS_INFO("GET GOAL");

    finished = false;
    success = false;

    while(!finished)
    {
        if(!ros::ok())
        {
            ROS_INFO("%s Shutting down", action_name_.c_str());
            break;
        }

        if(!as_.isActive() || as_.isPreemptRequested())
        {
            ;

        }

        time_t now = time(0);
        // getTimeGMT7Now(now);
        // time_t now;

        switch (feedback_.sequence)
        {
            case ScheduleControlState::INIT:
                ROS_INFO("[Matrix_scheldule_operation]: INIT");
                // std::cout << getTimeGMT7Now(now) << std::endl;
                feedback_.sequence = ScheduleControlState::GET_GOAL;
                break;
        
            case ScheduleControlState::GET_GOAL:
            {
                ROS_INFO("[Matrix_scheldule_operation]: GET_GOAL mode --> %s",goal->date.cmd.c_str());
                bool error = false;
                if(goal->date.cmd == matrix_msgs::ScheduleGoal::check_date)
                {
                    goal_date_t = getTimeForm(now, goal->date, false);
                }
                else if(goal->date.cmd == matrix_msgs::ScheduleGoal::check_time)
                {
                    ROS_INFO("[Matrix_scheldule_operation]: check_time");
                    goal_date_t = getTimeForm(now, goal->date, true);
                }
                else
                {
                    error = true;
                }

                if(error)
                {
                    ROS_ERROR("[Matrix_scheldule_operation]:  GET_GOAL mode --> Command fode not found");
                    results_.result = matrix_msgs::ScheduleResult::CMD_NOTFOUND;
                    results_.text = goal->date.cmd + "command not found";
                    feedback_.sequence = ScheduleControlState::ERROR;
                }else
                {
                    feedback_.sequence = ScheduleControlState::CHECK_GOAL;
                    ROS_INFO("[Matrix_scheldule_operation]: GET_GOAL goal timmer is  --> %ld", goal_date_t);
                }
                break;
            }

            case ScheduleControlState::CHECK_GOAL:
            {   
                ROS_INFO("[Matrix_scheldule_operation]: CHECK_GOAL");
                double diff_value = difftime(goal_date_t, getTimeGMT7Now(now));
                // std::cout << getTimeGMT7Now() << std::endl;
                // timmer set in past error case
                if(diff_value < 0)
                {
                    ROS_ERROR("[Matrix_scheldule_operation]: CHECK_GOAL --> Goal timmer is run out!!");
                    results_.result = matrix_msgs::ScheduleResult::GOAL_RUN_OUT;
                    results_.text = "Goal timmer is run out!!";
                    feedback_.sequence = ScheduleControlState::ERROR;
                }
                else if(diff_value == 0)
                {
                    ROS_INFO("[Matrix_scheldule_operation]: CHECK_GOAL --> Goal timmer match current time!!");
                    results_.result = matrix_msgs::ScheduleResult::SUCCESS;
                    results_.text = "Goal timmer match current time!!";
                    feedback_.sequence = ScheduleControlState::FINISH;
                }
                else if(diff_value > 0)
                {
                    ROS_INFO("[Matrix_scheldule_operation]: CHECK_GOAL --> Wait goal timmer match current time in %lf sec!!", diff_value);
                    // feedback_.sequence = ScheduleControlState::WAIT_TIMMER_FINISH;
                }
                break;
            }

            case ScheduleControlState::WAIT_TIMMER_FINISH:
            {
                
                // std::cout << getTimeGMT7Now() - goal_date_t << std::endl;
                // std::cout << getTimeGMT7Now() << std::endl;
                // long diff_value = difftime(goal_date_t, getTimeGMT7Now());
                // ROS_INFO("[Matrix_scheldule_operation]: WAIT_TIMMER_FINISH --> Timmer countdown: %d !!", diff_value);
                // if((diff_value < 0) || (diff_value == 0))
                // {
                //     ROS_INFO("[Matrix_scheldule_operation]: WAIT_TIMMER_FINISH --> Goal timmer match current time!!");
                //     results_.result = matrix_msgs::ScheduleResult::SUCCESS;
                //     results_.text = "Goal timmer match current time!!";
                //     feedback_.sequence = ScheduleControlState::FINISH;
                // }
                break;
            }

            case ScheduleControlState::FINISH:
                finished = true;
                break;
            
            case ScheduleControlState::ERROR:
                feedback_.sequence = ScheduleControlState::FINISH;
                break;
                
            // default:
            //     results_.text = "ScheduleControlState Fail please check in code";
            //     results_.result = matrix_msgs::ScheduleResult::CMD_NOTFOUND;
            //     finished = true;
            //     success = false;
            //     break;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
    }
    results_.result = results_.result;
    ROS_INFO("%s: %s", action_name_.c_str(), (success) ? "Succeeded" : "Fail");
    as_.setSucceeded(results_);

    ROS_INFO("********************************");
    ROS_INFO(" ");

}

void ScheduleOperation::preemptCB()
{
    ROS_WARN("%s got preempted!", action_name_.c_str());
    finished = true;
    success = false;
    // cancelMoveAction();
    // step_complete();
    // success = false;
    // task_completed = true;
    // results_.info = feedback_.info;
    // results_.result = XbeeCommunicationResult::GOT_PREEMPTED;
    // results_.text = XbeeCommunicationResult::GOT_PREEMPTED_s;
    // as_.setSucceeded(results_);
    as_.setPreempted();
}

time_t ScheduleOperation::getTimeForm(time_t now, const matrix_msgs::Date date, bool current_date)
{
    
    tm goal_time;
    if(current_date)
    {
        tm *ltm = localtime(&now);
        goal_time.tm_year = 1900 + ltm->tm_year;
        goal_time.tm_mon  = 1 + ltm->tm_mon;
        goal_time.tm_mday = ltm->tm_mday;

        goal_time.tm_wday = ltm->tm_wday;
        goal_time.tm_yday = ltm->tm_yday;
        goal_time.tm_isdst = ltm->tm_isdst;

        // std::cout << goal_time->tm_year << std::endl;
        // std::cout << goal_time->tm_mon << std::endl;
        // std::cout << goal_time->tm_mday << std::endl;
    }
    else
    {
        goal_time.tm_year = date.year;
        goal_time.tm_mon = date.month;
        goal_time.tm_mday = date.day;
    }
    
    goal_time.tm_hour = date.hour;
    goal_time.tm_min = date.min;
    goal_time.tm_sec = date.sec;

    // std::cout << goal_time->tm_hour << std::endl;
    // std::cout << goal_time->tm_min << std::endl;

    time_t t_form = mktime(&goal_time);
    // std::cout << mktime(goal_time) << std::endl;

    // std::cout << now <<std::endl;

    

    return t_form;
}

time_t ScheduleOperation::getTimeGMT7Now(time_t now)
{
    // ref https://www.tutorialspoint.com/cplusplus/cpp_date_time.htm
    // current date/time based on current system
    // time_t now = time(0);


    // std::cout << "Number of sec since January 1,1970 is:: " << now << std::endl;
    
    tm *ltm = localtime(&now);
    // std::cout << asctime(ltm) << std::endl;

    tm *aa;
    aa->tm_year = 1900 + ltm->tm_year;
    aa->tm_mon = 1 + ltm->tm_mon;
    aa->tm_mday = ltm->tm_mday;
    aa->tm_hour = ltm->tm_hour;
    aa->tm_min = ltm->tm_min;
    aa->tm_sec = ltm->tm_sec;

    time_t time_now = mktime(aa);
    return time_now;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_scheldule_operation");

    ScheduleOperation ScheduleOperation("matrix_scheldule_operation");
    ros::spin();

    return 0;
}