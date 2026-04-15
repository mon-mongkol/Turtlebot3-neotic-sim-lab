#include "matrix_io_operation/matrix_io_operation_node.h"

using namespace matrix_msgs;
using namespace std;
bool IOOperation::executeCB(const matrix_msgs::IOOperationGoalConstPtr &goal)
{
    

    ros::Rate r(10);
    feedback_.sequence = IOControlState::INIT;
    feedback_.text = "";
    feedback_.timeout = 0;

    finished = false;
    success = false;


    while(!finished)
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
            ;
        }

        

        switch(feedback_.sequence)
        {
            case IOControlState::INIT:
                feedback_.text = "[matrix_io_operation]: current_step ---> INIT";
                ROS_INFO("%s", feedback_.text);
                feedback_.sequence = IOControlState::SELECT_FUN;
                break;              
            
            case IOControlState::SELECT_FUN:
                feedback_.text = "[matrix_io_operation]: current_step ---> SELECT_FUN";
                ROS_INFO("%s", feedback_.text);
                if(goal->fun == IOOperationGoal::FUN_SET_OUTPUT)
                {
                    feedback_.sequence = IOControlState::SET_IO_INIT;
                }
                else if(goal->fun == IOOperationGoal::FUN_WAIT_INPUT)
                {
                    feedback_.sequence = IOControlState::WAIT_IO_MATCH_INIT;
                }
                else
                {
                    feedback_.text = "[matrix_io_operation]: Goal function not found.Please check function available in matrix_msgs/IOOperationGoal";
                    ROS_ERROR("%s", feedback_.text);
                    feedback_.sequence = IOControlState::ERROR;
                }
                break;   
            
            case IOControlState::SET_IO_INIT:
            {
                feedback_.text = "[matrix_io_operation]: current_step ---> SET_IO_INIT";
                ROS_INFO("%s", feedback_.text.c_str());
                dynamic_reconfigure::Reconfigure srv;
                for(int i = 0; i < goal->pin_state.size(); i++)
                {
                    std::string str = std::to_string(goal->pin_state[i].pin);
                    dynamic_reconfigure::BoolParameter data;
                    data.name = "y"+str+"_on";
                    data.value = (goal->pin_state[i].state ==1)? true:false;
                    srv.request.config.bools.push_back(data);
                }
                
                if(set_io_management.call(srv))
                {
                    for(auto srv : srv.request.config.bools)
                    {
                        ROS_INFO("[matrix_io_operation]: set IO follow --> %s:%d", srv.name.c_str(), srv.value);
                    }
                }

                
                feedback_.sequence = IOControlState::SET_IO;
                break;
            }
            case IOControlState::SET_IO:
            {
                feedback_.text = "[matrix_io_operation]: current_step ---> SET_IO";
                ROS_INFO("%s", feedback_.text);
                // matrix_msgs::SetIO set_io;
                // set_io.request.fun = SetIO::Request::FUN_SET_OUTPUT;
                // set_io.request.pin_details = goal->pin_state;
                // if(set_io_srv_.call(set_io))
                // {
                        success = true;
                        feedback_.sequence = IOControlState::FINISH;
                //     feedback_.text = "[matrix_io_operation]: current_step ---> SET_IO ----- Set IO success";
                //     results_.result = IOOperationResult::SUCCESS;
                //     results_.text = feedback_.text;
                // }
                // else
                // {
                //     success = false;
                //     feedback_.sequence = IOControlState::ERROR;
                //     feedback_.text = "[matrix_io_operation]: current_step ---> SET_IO ----- Goal function not found.Please check function available in matrix_msgs/IOOperationGoal";
                //     results_.result = IOOperationResult::UNKNOW_ERROR;
                //     results_.text = feedback_.text;
                //     ROS_ERROR("%s", feedback_.text);
                // }
                break;
            }

            case IOControlState::WAIT_IO_MATCH_INIT:
                feedback_.text = "[matrix_io_operation]: current_step ---> WAIT_IO_MATCH_INIT";
                ROS_INFO("%s", feedback_.text);

                GOAL_input_data.clear();
                for(auto i: goal->pin_state)
                {  
                    GOAL_input_data.push_back({i.pin, i.state});
                }
                setTimeOut = ros::Time::now().toSec() + goal->timeout;
                feedback_.sequence = IOControlState::WAIT_IO_MATCH;
                break;
            
                
                

            case IOControlState::ERROR:
                feedback_.text = "[matrix_io_operation]: current_step ---> ERROR";
                ROS_INFO("%s", feedback_.text);
                success = false;
                feedback_.sequence = IOControlState::FINISH;
                break;

            case IOControlState::PAUSE:
                ROS_WARN("[matrix_io_operation]: Robot state in Pausing mode");
                if(current_robotmode != matrix_msgs::RobotMode::PAUSE)
                {
                    feedback_.sequence = period_state;
                    feedback_.text = period_text;
                    period_text.clear();
                    setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
                }
                break;
                
            case IOControlState::FINISH:
                ROS_INFO("[matrix_io_operation]: FINISH");
                finished = true;
                break;

            

        }

        //**********************************************************//
        //                                                          //
        //                      check pause state                   //
        //                                                          //
        //**********************************************************//
        if((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != IOControlState::PAUSE))
        {
            // store period control state
            period_state = feedback_.sequence;
            period_text = feedback_.text;
            // set new control sate to PAUSE 
            feedback_.sequence = IOControlState::PAUSE;
            //get countdown time out
            period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

            feedback_.text = period_text + "  [Pausing_mode]";
            
        }

        as_.publishFeedback(feedback_);
        r.sleep();
    }
    
    results_.sequence = feedback_.sequence;
    results_.result = results_.result;
    ROS_INFO("%s: %s", action_name_.c_str(), (success) ? "Succeeded" : "Fail");
    as_.setSucceeded(results_);

    ROS_INFO("********************************");
    ROS_INFO(" ");
}




void IOOperation::cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
{
    current_robotmode = msg->robot_mode;
    
}


// int IOOperation::IntDataChecker(vector<int> mcu_data, vector<vector<int>> goal_data)
// {
//     int result = ResultDataChecker::MATCH;

//     try{
//         if(goal_data.size() == mcu_data.size())
//         {
//             ;
//         //     for(int i; i < goal_data.size(); i++)
//         //     {
//         //         if(goal_data[i] != matrix_msgs::IOOperationGoal::STATE_IGNORE)
//         //         {
//         //             if(goal_data[i] != mcu_data[i])
//         //             {
//         //                 result = ResultDataChecker::MISS_MATCH;
//         //                 break;
//         //             }else;
//         //         }else;
//         //     }
//         }
//         else
//         {
//             ROS_ERROR("[matrix_io_operation]: array error in function IOOperation::IntDataChecker lenght goal_data miss match mcu_data");
//             result = ResultDataChecker::OUT_OF_LEGHT;
//         }
//     }
//     catch(...)
//     {
//         ROS_ERROR("[matrix_io_operation]: array error in function IOOperation::IntDataChecker");
//         result = ResultDataChecker::CORE_DUMP;
//     }

//     return result;

// }

void IOOperation::preemptCB()
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

void IOOperation::cbMCU01Input(const std_msgs::Int8MultiArray msg)
{
    // for(std::vector<int>::const_iterator it = msg.data.begin(); it != msg.data.end(); ++it)
    // {
    //     MCUIO_input_data.push_back(*it);
    // }

}

void IOOperation::cbMCU01Output(const std_msgs::Int8MultiArray msg)
{
    ;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_io_operation");
    
    IOOperation IOOperation("matrix_io_operation");
    ros::spin();

    return 0;
}