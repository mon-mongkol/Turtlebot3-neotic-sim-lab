#include "matrix_serial_operation/matrix_serial_operation_node.h"

using namespace matrix_msgs;

bool SerialOperation::executeCB(const matrix_msgs::SerialOperationGoalConstPtr &goal)
{
    

    ros::Rate r(10);
    feedback_.sequence = SerialControlState::INIT;
    feedback_.text = "";
    feedback_.timeout = 0;

    finished = false;
    success = false;
    sr_init_timeout = false;
    isData = "";
    sr_1st_send = false;
    data_coming = false;
    

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
            case SerialControlState::INIT:
                ROS_INFO("[matrix_serial_operation]: INIT");
                //clear data receive before start process
                serial_data_.clear();
                feedback_.sequence = SerialControlState::SELECT_CMD;
                break;

            case SerialControlState::SELECT_CMD:
                ROS_INFO("[matrix_serial_operation]: SELECT_CMD");
                if(goal->cmd == matrix_msgs::SerialOperationGoal::serial_send)
                {
                    feedback_.sequence = SerialControlState::CHECK_DEVICE;
                }
                else if(goal->cmd == matrix_msgs::SerialOperationGoal::serial_receive)
                {
                    feedback_.sequence = SerialControlState::SERIAL_COMING;
                }
                else if(goal->cmd == matrix_msgs::SerialOperationGoal::timmer_set)
                {
                    feedback_.sequence = SerialControlState::TIMMER_SET;
                }
                else if(goal->cmd == matrix_msgs::SerialOperationGoal::cancel_action)
                {
                    feedback_.sequence = SerialControlState::CANCEL_ACTION;
                }else if(goal->cmd == matrix_msgs::SerialOperationGoal::send_receive)
                {
                    feedback_.sequence = SerialControlState::SR_SEND;
                }else if(goal->cmd == matrix_msgs::SerialOperationGoal::send_repeats)
                {
                    feedback_.sequence = SerialControlState::CHECK_DEVICE;
                }else if(goal->cmd == matrix_msgs::SerialOperationGoal::serial_receive_multi)
                {
                    feedback_.sequence = SerialControlState::INIT_MULTI_DATA;
                }
                else if(goal->cmd == "launch_autodocking")
                {
                    feedback_.sequence = SerialControlState::LAUNCH_AUTODOCKING;
                }
                else if(goal->cmd == "find_data_in_payload")
                {
                    feedback_.sequence = SerialControlState::FIND_DATA_IN_PAYLOAD_INIT;
                }
                else
                {
                    ;
                }
                break;
                


            case SerialControlState::CHECK_DEVICE:
                ROS_INFO("[matrix_serial_operation]: CHECK_DEVICE");    
                feedback_.sequence = SerialControlState::SEND_CMD;
                break;

            case SerialControlState::SEND_CMD:
            {
                if(goal->cmd == matrix_msgs::SerialOperationGoal::send_repeats)
                {

                    std::string data_send;
                    data_send = goal->arg_string;
                    //data format -->  #E[enable]E$,#S[cmd_sendString]S$
                    matrix_msgs::SerialCommu srv_;
                    srv_.request.cmd = matrix_msgs::SerialCommuRequest::send_repeats;
                    if(data_send.find(matrix_msgs::SerialCommuRequest::enable) != -1)
                    {
                        
                        try
                        {
                            data_send.erase(0, data_send.find("#S[") + 3);
                            data_send.erase(data_send.find("]S$"));
                            ROS_INFO("[matrix_serial_operation]: SEND_CMD_REPEATS --> %s", data_send.c_str());
            
                            srv_.request.arg0 = matrix_msgs::SerialCommuRequest::enable;
                            srv_.request.arg1 = data_send;
                        }
                        catch(const std::exception& e)
                        {
                            std::cerr << e.what() << '\n';
                            ROS_ERROR("[matrix_serial_operation]: SEND_CMD_REPEATS format error!!");
                            goto disable_repeats;
                        }
                    }
                    else
                    {
                        disable_repeats:
                        ROS_INFO("[matrix_serial_operation]: DISABLE_CMD_REPEATS");
                        srv_.request.arg0 = "disable";
                        srv_.request.arg1 = "";
                    }
                    serial_commu_srv_.call(srv_);
                    feedback_.sequence = SerialControlState::WAIT_SEND_CMD_FINISH;
                }
                else
                {
                    ROS_INFO("[matrix_serial_operation]: SEND_CMD --> %s", goal->arg_string.c_str());
                    //send request lift 
                    serial_msg.data = goal->arg_string;
                    serial_send_pub.publish(serial_msg);
                    feedback_.sequence = SerialControlState::WAIT_SEND_CMD_FINISH;
                }
                break;
            }

            case SerialControlState::WAIT_SEND_CMD_FINISH:
                ROS_INFO("[matrix_serial_operation]: WAIT_SEND_CMD_FINISH");
                results_.result = matrix_msgs::SerialOperationResult::SUCCESS;
                results_.text = "send command success";
                success = true;
                feedback_.sequence = SerialControlState::FINISH;
                break;
            
            case SerialControlState::SERIAL_COMING:
                ROS_INFO("[matrix_serial_operation]: SERIAL_COMING");
                setTimeOut = ros::Time::now().toSec() + goal->timeout;
                feedback_.sequence = SerialControlState::WAIT_SERIAL_COMING;
                break;

            case SerialControlState::WAIT_SERIAL_COMING:
                feedback_.text = "wait data --> " + goal->arg_string;
                ROS_INFO("[matrix_serial_operation]: WAIT_SERIAL_COMING --> %s", feedback_.text.c_str());

                if(serial_data_ == goal->arg_string)
                {
                    feedback_.sequence = SerialControlState::FINISH;
                    results_.result = matrix_msgs::SerialOperationResult::SUCCESS;
                    results_.text = "serial match";
                    success = true;
                }else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal->timeout != 0)
                    {
                        ROS_INFO("[matrix_serial_operation]: Check timeout");
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.result = matrix_msgs::SerialOperationResult::TIME_OUT;
                            results_.text = "wait serial timeout";
                            feedback_.sequence = SerialControlState::ERROR;
                        }
                    }
                }
                break;
            



            case SerialControlState::TIMMER_SET:
                ROS_INFO("[matrix_serial_operation]: TIMMER_SET");
                setTimeOut = ros::Time::now().toSec() + goal->arg_int;
                feedback_.sequence =SerialControlState::WAIT_TIMER_FINISH;
                feedback_.text = "wait timmer";
                feedback_.timeout = setTimeOut;
                break;
            
            case SerialControlState::WAIT_TIMER_FINISH:
                ROS_INFO("[matrix_serial_operation]: WAIT_TIMER_FINISH");
                //check time out
                if(ros::Time::now().toSec() > setTimeOut)
                {
                    success = true;
                    results_.result = matrix_msgs::SerialOperationResult::SUCCESS;
                    results_.text = "timmer success";
                    feedback_.sequence =SerialControlState::FINISH;
                }
                feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);
                feedback_.text = "wait timmer";
                break;
            
            case SerialControlState::CANCEL_ACTION:
            {
                ROS_INFO("[matrix_serial_operation]: CANCEL_ACTION");
                std_srvs::SetBool srv_msg;
                srv_msg.request.data = true;
                if(cancel_action_srv_.call(srv_msg))
                {
                    feedback_.sequence =SerialControlState::FINISH;
                }
                else
                {
                    feedback_.sequence =SerialControlState::ERROR;
                }
                break;
            }

            case SerialControlState::SR_SEND:
                ROS_INFO("[matrix_serial_operation]: SR_SEND");
                
                
                if(!sr_1st_send)
                {
                    ROS_INFO("[matrix_serial_operation]: SEND_CMD --> %s", goal->arg_sr_send.c_str());
                    isData = goal->arg_sr_receive;
                    data_coming = false;
                    //send request lift 
                    serial_msg.data = goal->arg_sr_send;
                    serial_send_pub.publish(serial_msg);
                    
                    // if(!sr_init_timeout)
                    // {
                        setTimeOut = ros::Time::now().toSec() + goal->timeout;
                        sr_init_timeout = true;
                    // }
                    sr_1st_send = true;
                    last_send_stmp = ros::Time::now().toSec();
                }
                else
                {
                    if(goal->arg_sr_send_repeats)
                    {
                        if((ros::Time::now().toSec() - last_send_stmp) > 1)
                        {
                            ROS_INFO("[matrix_serial_operation]: SEND_CMD_REPEATS --> %s", goal->arg_sr_send.c_str());
                            serial_msg.data = goal->arg_sr_send;
                            serial_send_pub.publish(serial_msg);
                            last_send_stmp = ros::Time::now().toSec();
                        }else;
                    }else;
                }
                
                feedback_.text = "send data --> " + goal->arg_sr_receive +" | wait data --> " + goal->arg_sr_receive + " | repeats --> " + ((goal->arg_sr_send_repeats)? "true" : "false");
                feedback_.sequence = SerialControlState::SR_RECEIVE;
                break;
            
            case SerialControlState::SR_RECEIVE:
                ROS_INFO("[matrix_serial_operation]: WAIT_SR_RECEIVE --> %s", feedback_.text.c_str());
                if((serial_data_ == goal->arg_sr_receive) || (data_coming))
                {
                    feedback_.sequence = SerialControlState::FINISH;
                    results_.result = matrix_msgs::SerialOperationResult::SUCCESS;
                    results_.text = "send_receive serial match";
                    success = true;
                }else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal->timeout != 0)
                    {
                        ROS_INFO("[matrix_serial_operation]: Check timeout");
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.result = matrix_msgs::SerialOperationResult::TIME_OUT;
                            results_.text = "wait serial timeout";
                            feedback_.sequence = SerialControlState::ERROR;
                        }
                        else
                        {
                            feedback_.sequence = SerialControlState::SR_SEND;
                        }
                    }
                    else
                    {
                        feedback_.sequence = SerialControlState::SR_SEND;
                    }

                    if(feedback_.sequence == SerialControlState::SR_SEND)
                    {
                        if(!goal->arg_sr_send_repeats)
                        {
                            feedback_.sequence = SerialControlState::SR_RECEIVE;
                        }
                    }
                }
                break;
            



            case SerialControlState::INIT_MULTI_DATA:
            {
                ROS_INFO("[matrix_serial_operation]: INIT_MULTI_DATA -->");
                multi_data_.arg_strings = goal->arg_strings;

                std::string string_from_array="wait data --> ";
                for(auto data : multi_data_.arg_strings)
                {
                    string_from_array = string_from_array + "," + data;
                }
                feedback_.text = string_from_array;
                setTimeOut = ros::Time::now().toSec() + goal->timeout;
                feedback_.sequence = SerialControlState::WAIT_MULTI_DATA_MATCH;
                break;
            }
                

            case SerialControlState::WAIT_MULTI_DATA_MATCH:
            {
                ROS_INFO("[matrix_serial_operation]: WAIT_MULTI_DATA_MATCH -->%s", feedback_.text.c_str());
                
                if(data_coming)
                {
                    results_.data_bypass = isData;
                    results_.result = matrix_msgs::SerialOperationResult::SUCCESS;
                    results_.text = "multi command match success -->" + isData;
                    success = true;
                    feedback_.sequence = SerialControlState::FINISH;
                }else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal->timeout != 0)
                    {
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.result = matrix_msgs::SerialOperationResult::TIME_OUT;
                            results_.text = "wait serial multi data timeout";
                            results_.data_bypass="";
                            feedback_.sequence = SerialControlState::ERROR;

                        }
                    }
                }
                break;
            }



            case SerialControlState::LAUNCH_AUTODOCKING:
                ROS_INFO("[matrix_serial_operation]: LAUNCH_AUTODOCKING");
                if(goal->arg_string == "on")
                {
                    LaunchController(ActionControllerRequest::CMD_ACTION_START, ActionControllerRequest::AC_AR3_REAR);
                    feedback_.sequence = SerialControlState::WAIT_LAUNCH_FINISH;
                }
                else
                {
                    LaunchController(ActionControllerRequest::CMD_ACTION_KILL, ActionControllerRequest::AC_AR3_REAR);
                    feedback_.sequence =SerialControlState::FINISH;
                    success = true;
                }
                
                break;
            
            case SerialControlState::WAIT_LAUNCH_FINISH:
                ROS_INFO("[matrix_serial_operation]: WAIT_LAUNCH_FINISH");

                if(artrack3_operation_ac.waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the matrix_autodock action server to come up");
                    success = true;
                    feedback_.sequence =SerialControlState::FINISH;
                }
                else
                {
                    feedback_.text = "WAIT_LAUNCH_FINISH";
                }
                break;
            

            case SerialControlState::FIND_DATA_IN_PAYLOAD_INIT:
            {
                ROS_INFO("[matrix_serial_operation]: FIND_DATA_IN_PAYLOAD_INIT");

                multi_data_.arg_strings = goal->arg_strings;

                std::string string_from_array="find data --> ";
                for(auto data : multi_data_.arg_strings)
                {
                    string_from_array = string_from_array + "," + data;
                }
                feedback_.text = string_from_array;
                setTimeOut = ros::Time::now().toSec() + goal->timeout;
                feedback_.sequence = SerialControlState::FIND_DATA_IN_PAYLOAD_CHECK;
                break;
            }
                
            
            case SerialControlState::FIND_DATA_IN_PAYLOAD_CHECK:
                ROS_INFO("[matrix_serial_operation]: WAIT_MULTI_DATA_MATCH -->%s", feedback_.text.c_str());
                
                if(data_coming)
                {
                    results_.data_bypass = isData;
                    results_.result = matrix_msgs::SerialOperationResult::SUCCESS;
                    results_.text = "multi command match success -->" + isData;
                    success = true;
                    feedback_.sequence = SerialControlState::FINISH;
                }else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal->timeout != 0)
                    {
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.result = matrix_msgs::SerialOperationResult::TIME_OUT;
                            results_.text = "wait serial multi data timeout";
                            results_.data_bypass="";
                            feedback_.sequence = SerialControlState::ERROR;

                        }
                    }
                }
                break;
                


            case SerialControlState::ERROR:
                ROS_INFO("[matrix_serial_operation]: ERROR");
                success = false;
                feedback_.sequence = SerialControlState::FINISH;
                break;

            case SerialControlState::PAUSE:
                ROS_WARN("[matrix_serial_operation]: Robot state in Pausing mode");
                if(current_robotmode != matrix_msgs::RobotMode::PAUSE)
                {
                    feedback_.sequence = period_state;
                    feedback_.text = period_text;
                    period_text.clear();
                    setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
                }
                break;
                
            case SerialControlState::FINISH:
                ROS_INFO("[matrix_serial_operation]: FINISH");
                finished = true;
                break;

            

        }

        //**********************************************************//
        //                                                          //
        //                      check pause state                   //
        //                                                          //
        //**********************************************************//
        if((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != SerialControlState::PAUSE))
        {
            // store period control state
            period_state = feedback_.sequence;
            period_text = feedback_.text;
            // set new control sate to PAUSE 
            feedback_.sequence = SerialControlState::PAUSE;
            //get countdown time out
            period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

            feedback_.text = period_text + "  [Pausing_mode]";
            
        }

        as_.publishFeedback(feedback_);
        r.sleep();
    }
    results_.cmd = goal->cmd;
    results_.sequence = feedback_.sequence;
    results_.result = results_.result;
    ROS_INFO("%s: %s", action_name_.c_str(), (success) ? "Succeeded" : "Fail");
    as_.setSucceeded(results_);

    ROS_INFO("********************************");
    ROS_INFO(" ");
}

void SerialOperation::SerialDataChecker(std::string data)
{
    if(serial_data_ == isData)
    {
        data_coming = true;
    }else;

    if((feedback_.sequence == SerialControlState::INIT_MULTI_DATA) || (feedback_.sequence == SerialControlState::WAIT_MULTI_DATA_MATCH))
    {
        for(auto data : multi_data_.arg_strings)
        {
            if(serial_data_ == data)
            {
                isData = data;
                data_coming = true;
                break;
            }else;
        }
    }else if((feedback_.sequence == SerialControlState::FIND_DATA_IN_PAYLOAD_INIT) || (feedback_.sequence == SerialControlState::FIND_DATA_IN_PAYLOAD_CHECK))
    {
        bool foud_all_element = true;
        for(auto data : multi_data_.arg_strings)
        {
            if(serial_data_.find(data) == -1) // if .find return -1 mean not found in payload
            {
                foud_all_element = false;
                break;
            }else;
        }

        if(foud_all_element)
        {
            isData = data;
            data_coming = true;
        }

    }
    else;
}

void SerialOperation::cbSerialRead(const std_msgs::String::ConstPtr &msg)
{
    serial_data_ = msg->data;
    SerialDataChecker(serial_data_);

    // if(serial_data_ == isData)
    // {
    //     data_coming = true;
    // }

    // if((feedback_.sequence == SerialControlState::INIT_MULTI_DATA) || (feedback_.sequence == SerialControlState::WAIT_MULTI_DATA_MATCH))
    // {
    //     for(auto data : multi_data_.arg_strings)
    //     {
    //         if(serial_data_ == data)
    //         {
    //             isData = data;
    //             data_coming = true;
    //             break;
    //         }
    //     }
    // }
}

// void SerialOperation::cbRS785_SerialRead(const std_msgs::String::ConstPtr &msg)
// {
//     serial_data_ = msg->data;
//     SerialDataChecker(serial_data_);
// }

void SerialOperation::cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
{
    current_robotmode = msg->robot_mode;
}

bool SerialOperation::LaunchController(std::string cmd, std::string action_name)
{
    bool success = false;
    matrix_msgs::ActionController srv;
    srv.request.cmd = cmd;
    srv.request.action_name = action_name;
    if(launch_controller_ac.call(srv))
    {
        success = true;
    }
    return success;
}

void SerialOperation::preemptCB()
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_serial_operation");
    
    SerialOperation SerialOperation("matrix_serial_operation");
    ros::spin();

    return 0;
}