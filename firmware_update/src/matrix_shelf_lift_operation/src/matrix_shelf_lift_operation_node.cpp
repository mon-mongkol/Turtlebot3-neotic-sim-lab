#include "matrix_shelf_lift_operation/matrix_shelf_lift_operation.h"

using namespace matrix_msgs;

bool SHLOperation::executeCB(const matrix_msgs::SHLOperationGoalConstPtr &goal)
{
    ros::Rate r(20);

    feedback_.sequence = SHLControlState::INIT_PARM;
    results_.result = SHLOperationResult::pending;
    finished = false;
    success = false;
    int _isAborted = false;
    STRcondition_checking("","", "clear");

    retry_counter = 0;
    
    

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

        current_sequence = feedback_.sequence;

        switch(feedback_.sequence)
        {
            case SHLControlState::INIT_PARM:
                ROS_INFO("[matrix_shelf_lift_operation]: INIT_PARM");
                feedback_.text = "INIT_PARM";
                // feedback_.sequence = SHLControlState::CHECK_LIFT_STATE;
                feedback_.sequence = SHLControlState::CMD_SELECTOR;
                break;

            case SHLControlState::CHECK_LIFT_STATE:
                ROS_INFO("[matrix_shelf_lift_operation]: CHECK_LIFT_STATE");
                feedback_.text = "CHECK_LIFT_STATE";
                STRcondition_checking("IDL", "BUSY", "check");
                setTimeOut = ros::Time::now().toSec() + state_condition_timeout;
                feedback_.sequence = SHLControlState::WAIT_CHECK_LIFT_STATE_FINISH;
                break;
            
            case SHLControlState::WAIT_CHECK_LIFT_STATE_FINISH:
            {
                std::string SHLState = STRcondition_checking("", "", "match_state");
                feedback_.text = "CHECK_LIFT_STATE";
                if(SHLState.find("IDL") != -1)
                {
                    // ROS_INFO("MATCH");
                    feedback_.sequence = SHLControlState::CMD_SELECTOR;
                }
                else if(SHLState.find("BUSY_Ac") != -1)
                {
                    feedback_.sequence = SHLControlState::CMD_SELECTOR;
                }
                else if(SHLState.find("BUSY_Ma") != -1)
                {
                    results_.result = SHLOperationResult::liftBusy;
                    results_.text = "control sequence timeout -->" + feedback_.text + "| DATA--> " + SHLState;
                    results_.sequence = feedback_.sequence;
                    feedback_.sequence = SHLControlState::ERROR;
                    ROS_WARN("%s", results_.text.c_str());
                }
                else
                {      
                    //check time out
                    if(ros::Time::now().toSec() > setTimeOut)
                    {
                        retry_counter++;
                        if(retry_counter > max_retry)
                        {
                            results_.result = SHLOperationResult::waitDataTimeout;
                            results_.text = "control sequence timeout -->" + feedback_.text;
                            results_.sequence = feedback_.sequence;
                            feedback_.sequence = SHLControlState::ERROR;
                            ROS_WARN("%s", results_.text.c_str());
                        }
                        else{
                            feedback_.text = "retry wait state IDL retry";
                            ROS_WARN("[matrix_shelf_lift_operation]:retry wait state IDL retry counter:%d", retry_counter);
                            feedback_.sequence = SHLControlState::INIT_PARM;
                        }
                    }
                }
                break;
            
            }
            case SHLControlState::CMD_SELECTOR:
                ROS_INFO("[matrix_shelf_lift_operation]: CMD_SELECTOR");
                feedback_.text = "CMD_SELECTOR";
                retry_counter = 0;
                feedback_.sequence = SHLControlState::SEND_CMD;
                break;
            
            case SHLControlState::SEND_CMD:
            {
                ROS_INFO("[matrix_shelf_lift_operation]: SEND_CMD"); 
                feedback_.text = "SEND_CMD";

                
                cmd_msg.data = "";
                ROS_INFO("%s", goal->cmd.c_str());
                if(goal->cmd == "up_check_obj")
                {
                    // cmd_msg.data = "#TxSHL100_UP$";
                    cmd_msg.data = "#Tx" + lift_model + "_UP$";
                }
                // else if(goal->cmd == SHLOperationGoal::up_skip_obj)
                else if(goal->cmd == "up_skip_obj")
                {
                    // cmd_msg.data = "#TxSHL100_UP_SkipCheckOBJ$";
                    cmd_msg.data = "#Tx" + lift_model + "_UP_SkipCheckOBJ$";
                }
                else if(goal->cmd == "down")
                {
                    // cmd_msg.data = "#TxSHL100_DOWN$";
                    cmd_msg.data = "#Tx" + lift_model + "_DOWN$";
                }
                else if(goal->cmd == "stop")
                {
                    // cmd_msg.data = "#TxSHL100_STOP$";
                    cmd_msg.data = "#Tx" + lift_model + "_STOP$";
                }
                else if(goal->cmd == "rst")
                {
                    // cmd_msg.data = "#TxSHL100_RST$";
                    cmd_msg.data = "#Tx" + lift_model + "_RST$";
                }
                else
                {
                    results_.result = SHLOperationResult::cmd_not_found;
                    results_.text = "control sequence -->" + feedback_.text + "goal cmd not found --> " + goal->cmd.c_str();
                    results_.sequence = feedback_.sequence;
                    feedback_.sequence = SHLControlState::ERROR;
                    ROS_WARN("%s", results_.text.c_str());
                }

                
                pub_shl100_write.publish(cmd_msg);
                

                               

                if(results_.result != SHLOperationResult::cmd_not_found)
                {
                    // feedback_.sequence = SHLControlState::WAIT_RESULT;
                    setTimeOut = ros::Time::now().toSec() + 3;
                    feedback_.sequence = SHLControlState::WAIT_RES_CMD;
                }

                if(goal->cmd != "rst")
                {
                    STRcondition_checking("AcResult", "AcResult", "check");
                }
                else
                {
                    STRcondition_checking("INIT_Success", "INIT_FINISH", "check");
                } 
                
                break;
            }


            case SHLControlState::WAIT_RES_CMD:
                feedback_.text = "WAIT_RES_CMD";
                ROS_WARN("WAIT_RES_CMD");
                if((data_receive.find(cmd_msg.data) != -1) || (data_receive.find('BUSY_AcCONTROL') != -1) || (data_receive.find('AcRecvGoal') != -1))
                {
                    if(goal->timeout != 0)
                    {
                        setTimeOut = ros::Time::now().toSec() + goal->timeout;
                    }
                    else
                    {
                        setTimeOut = ros::Time::now().toSec() + 20;
                    }

                    // if(goal->cmd != "rst")
                    // {
                    //     STRcondition_checking("AcResult", "AcResult", "check");
                    // }
                    // else
                    // {
                    //     STRcondition_checking("INIT_Success", "INIT_FINISH", "check");
                    // } 
                    ROS_WARN("WAIT_RES_CMD Success!!!");
                    feedback_.sequence = SHLControlState::WAIT_RESULT;
                    retry_counter = 0;
                }
                else
                {
                    if(ros::Time::now().toSec() > setTimeOut)
                    {
                        retry_counter++;
                        if(retry_counter > max_retry)
                        {
                            std_msgs::String cmd_stop;
                            // cmd_stop.data = "#TxSHL100_STOP$";
                            cmd_stop.data = "#Tx" + lift_model + "_STOP$";
                            pub_shl100_write.publish(cmd_stop);

                            results_.result = 97;
                            results_.text = "control sequence timeout in WAIT_RES_CMD, wait --> BUSY_AcCONTROL, AcRecvGoal, " + cmd_msg.data;
                            results_.sequence = feedback_.sequence;
                            feedback_.sequence = SHLControlState::ERROR;
                        }
                        else{
                            feedback_.text = "retry wait state IDL retry";
                            ROS_WARN("[matrix_shelf_lift_operation]:retry send cmd to lift module and wait state BUSY_AcCONTROL, AcRecvGoal. retry counter:%d", retry_counter);
                            feedback_.sequence = SHLControlState::SEND_CMD;
                        }
                    }
                }

                
            case SHLControlState::WAIT_RESULT:
            {
                ROS_INFO("[matrix_shelf_lift_operation]: WAIT_RESULT");
                feedback_.text = "WAIT_RESULT";
                
                std::string SHLResult = STRcondition_checking("", "", "match_state");
                ROS_INFO("%s", SHLResult.c_str());
                if(SHLResult.find('Result') != -1)
                {   
                    // ROS_INFO("%s", SHLResult[SHLResult.length()-2]);
                    ROS_INFO("%c", SHLResult[SHLResult.length()-3]);
                    int result = SHLResult[SHLResult.length()-3] - '0';
                    ROS_INFO("%d", result);
                    results_.result = result;
                    feedback_.sequence = SHLControlState::FINISH;
                }
                else
                {
                    if(goal->cmd == "rst")
                    {
                        // if(SHLResult == "#RxSHL100_[INIT_FINISH]$")
                        if(SHLResult == ("#Rx"+lift_model+"_[INIT_FINISH]$"))
                        {
                            ROS_INFO("RESTART FINISH");
                            results_.result = 111;
                            feedback_.sequence = SHLControlState::FINISH;
                        }
                        if(SHLResult.find('INIT_Success') != -1)
                        {
                            ROS_INFO("%c", SHLResult[SHLResult.length()-3]);
                            int result = SHLResult[SHLResult.length()-3] - '0';
                            ROS_INFO("%d", result);
                            results_.result = result;
                            feedback_.sequence = SHLControlState::FINISH;
                        }
                    }
                    //if timeout set 0 sec --> ignore timeout
                    // if(goal->communication_timeout != 0)
                    // {
                    //     ROS_INFO("[matrix_shelf_lift_operation]: Check timeout");
                    //     feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                    //     ROS_INFO("[matrix_shelf_lift_operation]: Check timeout countdown %d", feedback_.timeout);
                    //     //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            std_msgs::String cmd_stop;
                            // cmd_stop.data = "#TxSHL100_STOP$";
                            cmd_stop.data = "#Tx"+lift_model+"_STOP$";
                            pub_shl100_write.publish(cmd_stop);

                            results_.result = SHLOperationResult::controlTimeout;
                            results_.text = "control sequence timeout inWAIT_RESULT, wait -->" + data_receive;
                            results_.sequence = feedback_.sequence;
                            feedback_.sequence = SHLControlState::ERROR;
                        }
                    // }
                }
                break;
            }
            case SHLControlState::ERROR:
                ROS_INFO("[matrix_shelf_lift_operation]: ERROR");
                feedback_.text = "ERROR" + feedback_.text;
                ROS_ERROR("[Matrix_docking_operation]: ERROR is %s", results_.text.c_str());
                feedback_.sequence = SHLControlState::FINISH;
                break;

            //**********************************************************//
            //                                                          //
            //                      Pause Control state                 //
            //                                                          //
            //**********************************************************//
            case SHLControlState::PAUSE:
                ROS_INFO("[matrix_shelf_lift_operation]: PAUSE");
                feedback_.text = "PAUSE";
                //special for docking_operation
                if(!initPause)
                {
                    ROS_WARN("SEND PAUSE LIFT");
                    std_msgs::String cmd_stop;
                    // cmd_stop.data = "#TxSHL100_STOP$";
                    cmd_stop.data = "#Tx"+lift_model+"_STOP$";
                    pub_shl100_write.publish(cmd_stop);
                    period_state = SHLControlState::SEND_CMD;
                    initPause = true;
                }

                

                if(current_robotmode != matrix_msgs::RobotMode::PAUSE)
                {
                    feedback_.sequence = period_state;
                    feedback_.text = period_text;
                    period_text.clear();
                    setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
                    initPause = false;
                }
                break;
            //***********************************************************//

            case SHLControlState::FINISH:
                ROS_INFO("[matrix_shelf_lift_operation]: FINISH");
                // feedback_.text = "FINISH";  
                
                clear_params(false, true);
                finished = true;
                break;
                
        }
        //**********************************************************//
        //                                                          //
        //                      check Docking sysn                  //
        //                                                          //
        //**********************************************************//

        // if((!_isDockingStateSync) && (feedback_.sequence != SHLControlState::ERROR))
        // {
        //     ROS_ERROR("_isDockingStateSync");
        //     feedback_.sequence = SHLControlState::DODCKING_NOT_SYNC;
        //     _isDockingStateSync = true;
        // }

        //**********************************************************//
        //                                                          //
        //                      check pause state                   //
        //                                                          //
        //**********************************************************//
        if((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != SHLControlState::PAUSE))
        {
            // store period control state
            period_state = feedback_.sequence;
            period_text = feedback_.text;
            // set new control sate to PAUSE 
            feedback_.sequence = SHLControlState::PAUSE;
            //get countdown time out
            period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

            feedback_.text = period_text + " [Pausing_mode]";  
        }

        if(current_robotmode == matrix_msgs::RobotMode::EMERGENCY)
        {
            ROS_WARN("%s Emergency state setAborted!", action_name_.c_str());
            finished = true;
            success = false;
            clear_params(false, true);
            _isAborted = true;
            std_msgs::String cmd_stop;
            // cmd_stop.data = "#TxSHL100_STOP$";
            cmd_stop.data = "#Tx"+lift_model+"_STOP$";
            pub_shl100_write.publish(cmd_stop);
        }
        else
        {
            ROS_INFO("Robot mode %d", current_robotmode);
        }

        as_.publishFeedback(feedback_);
        r.sleep();
    }

    if(!as_.isPreemptRequested() && !_isAborted)
    {
        results_.result = results_.result;
        ROS_INFO("%s: %s", action_name_.c_str(), (results_.result) ? "Succeeded" : "Fail");
        as_.setSucceeded(results_);
    }
    

    ROS_INFO("********************************");
    ROS_INFO(" ");

}


 
void SHLOperation::preemptCB()
{
    ROS_WARN("%s got preempted!", action_name_.c_str());
    finished = true;
    success = false;
    clear_params(false, true);
    as_.setPreempted();
}

void SHLOperation::clear_params(bool cancel_action_ar3, bool kill_action_ar3)
{
    finished = true;
    success = false;
    // feedback_.sequence = SHLControlState::INIT_PARM;
    // results_.result = SHLOperationResult::pending;
    STRcondition_checking("","", "clear");
    // std_msgs::String cmd_stop;
    // cmd_stop.data = "#TxSHL100_STOP$";
    // pub_shl100_write.publish(cmd_stop);
}


std::string SHLOperation::STRcondition_checking(std::string con1, std::string con2, std::string cmd)
{
    std::string data_bypass = "";
    if(cmd == "check")
    {   
        isConditionMatch = false;
        condition1 = con1;
        condition2 = con2;
    }
    else if(cmd == "clear")
    {
        isConditionMatch = false;
        condition1 = "";
        condition2 = "";
    }
    else if(cmd == "match_state")
    {
        if(isConditionMatch)
        {
            data_bypass = conditionStr;
        }
        
    }
    else
    {
        ;
    }
    return data_bypass;
}





void SHLOperation::cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
{
    current_robotmode = msg->robot_mode;
}

void SHLOperation::cbSHLReceive(std_msgs::String::ConstPtr msg)
{
    data_receive = msg->data;
    // if((condition1 != "") || (condition2 != ""))
    // {
    //     if((msg->data == condition1) || (msg->data == condition2))
    //     {
    //         conditionStr = msg->data;
    //         isConditionMatch = true;
    //     }
    // }
    if((condition1 != "") || (condition2 != ""))
    {
        if(!isConditionMatch )
        {
            ROS_INFO("condition checking.... ");
            if((msg->data.find(condition1) != -1) || (msg->data.find(condition2) != -1 ))
            {
                conditionStr = msg->data;
                isConditionMatch = true;
            }
        }
        
    }
}

void SHLOperation::cbSHLIOstate(std_msgs::Int8MultiArray msg)
{
    ;
}

// void SHLOperation::cbRS485Receive(std_msgs::String::ConstPtr msg)
// {
//     ROS_INFO("GetData");
//      std::string a;
//     a = msg->data;
//     ROS_INFO("%s %d", a.c_str(), current_sequence);
//     // std::string cg_id_str = std::to_string(goal->charger_id);
//     if((current_sequence > SHLControlState::SEARCH_AR) && (current_sequence <= SHLControlState::WAIT_CG_CONFIRM_STATE))
//     {
//         ROS_INFO("SSSSS");
//         if(_isDockingStateSync)
//         {
//             // check docking cancel
//            ROS_INFO("1111");
//             if(a.find("RxRBDCN") != -1)
//             {
//                 _isDockingStateSync = false;
//                 docking_state_str=msg->data;
//             }
//             else if(a.find("PLS_PRESS_BUSBAR") != -1)
//             {
//                 _isDockingStateSync = false;
//                 docking_state_str=msg->data;
//             }
//             else if(a.find("CHARGE_NOT_OK") != -1)
//             {
//                 _isDockingStateSync = false;
//                 docking_state_str=msg->data;
//             }
//             else if(a.find("RxEND") != -1)
//             {
//                 _isDockingStateSync = false;
//                 docking_state_str=msg->data;
//             }
//             else
//             {
//                 ROS_INFO("4444");
//             }
//         }
        
//     }

// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_shelf_lift_operation");

    SHLOperation SHLOperation("matrix_shelf_lift_operation");
    ros::spin();

    return 0;
}