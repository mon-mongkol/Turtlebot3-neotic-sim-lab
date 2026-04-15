#include "matrix_docking_operation/matrix_docking_operation_node.h"

bool DockingOperation::executeCB(const matrix_msgs::ROHMChargerOperationGoalConstPtr &goal)
{
    ros::Rate r(10);

    feedback_.sequence = DockingControlState::INIT;
    finished = false;
    success = false;

    isTimmerCompleted = false;

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
            case DockingControlState::INIT:
                ROS_INFO("[Matrix_docking_operation]: INIT");
                feedback_.text = "DockingControlSate::INIT";
                //clear parameters before execute goal
                clear_params();
                feedback_.sequence = DockingControlState::CmdSelector;
                
                break;

            case DockingControlState::CmdSelector:
                if(goal->cmd == matrix_msgs::ROHMChargerOperationGoal::check_charger_end)
                {
                    ROS_INFO("[Matrix_docking_operation]: CMD_SELECTOR --> CHECK_CHARGER_END");
                    condition1 = goal->arg0_str;
                    condition2 = goal->arg1_str;
                    setTimeOut = ros::Time::now().toSec() + charging_state_timeout;
                    feedback_.sequence = DockingControlState::WAIT_CHARGING_STATE;
                }
                else if(goal->cmd == matrix_msgs::ROHMChargerOperationGoal::backward)
                {
                    ROS_INFO("[Matrix_docking_operation]: CMD_SELECTOR --> BACKWARD");
                    feedback_.sequence = DockingControlState::BACKWARD_INIT;
                }
                else
                {
                    ROS_ERROR("[Matrix_docking_operation]: CMD_SELECTOR --> ERROR");
                    results_.result = matrix_msgs::ROHMChargerOperationResult::CMD_NOTFOUND;
                    results_.text = "command not found";
                    feedback_.sequence = DockingControlState::ERROR;
                }
                break;
            
            case DockingControlState::WAIT_CHARGING_STATE:
                ROS_INFO("[Matrix_docking_operation]: WAIT_CHARGING_STATE from /battery_state");
                if(bat_state.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
                {
                    ROS_INFO("[Matrix_docking_operation]: Supply state charging....");
                    feedback_.sequence = DockingControlState::CHARGING_STATE_COMEUP;
                    
                }
                else
                {
                    if(charging_state_timeout != 0)
                    {
                        //get timeout counter
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            ROS_ERROR("[Matrix_docking_operation]: WAIT_CHARGING_STATE --> Timeout");
                            results_.result = matrix_msgs::ROHMChargerOperationResult::TIMEOUT;
                            results_.text = "timeout wait charging state";
                            feedback_.sequence = DockingControlState::ERROR;
                        }
                    }
                }
                feedback_.text = "wait charging state from /battery_state [timeout in --> " + std::to_string(feedback_.timeout) + "s]";
                break;
            
            case DockingControlState::CHARGING_STATE_COMEUP:
                setDockingMode("on");
                //select set timmer or set schedule
                if(goal->arg0_bool)
                {
                    feedback_.sequence = DockingControlState::SET_SCHEDULE;
                }
                else
                {
                    feedback_.sequence = DockingControlState::SET_TIMMER;
                }
                break;
            
            case DockingControlState::SET_SCHEDULE:
            {
                ROS_INFO("[Matrix_docking_operation]: SET_SCHEDULE");
                matrix_msgs::ScheduleGoal goal_schedule;
                goal_schedule.date.cmd = matrix_msgs::ScheduleGoal::check_time;
                goal_schedule.date.hour = goal->arg0_uint;
                goal_schedule.date.min  = goal->arg1_uint;
                schedule_operation_ac.sendGoal(goal_schedule, boost::bind(&DockingOperation::matrixScheduleDoneCb, this, _1, _2));
                feedback_.sequence = DockingControlState::CHARGING;
                break;
            }

            case DockingControlState::SET_TIMMER:
            {
                ROS_INFO("[Matrix_docking_operation]: SET_TIMMER");
                if((goal->arg0_uint == 0) && (goal->arg1_uint == 0))
                {
                    skip_timer = true;
                }
                else
                {
                    matrix_msgs::SerialOperationGoal goal_serial;
                    goal_serial.cmd = matrix_msgs::SerialOperationGoal::timmer_set;
                    goal_serial.arg_int = goal->arg0_uint*60*60 + goal->arg1_uint*60;
                    ROS_INFO("%d %d", goal->arg0_uint, goal->arg1_uint);
                    serial_operation_ac.sendGoal(goal_serial, boost::bind(&DockingOperation::matrixSerialDoneCb, this, _1, _2));
                }
                feedback_.sequence = DockingControlState::CHARGING;
                break;
            }


            
            case DockingControlState::CHARGING:
            {
                ROS_INFO("[Matrix_docking_operation]: CHARGING.. ");
                ROS_INFO("[Matrix_docking_operation]: wati %s OR Power supply change OR Serial data --> %s, %s", (goal->arg0_bool)? "Schedule":"Timmer",condition1.c_str(), condition2.c_str());
                // START, CHARGE_FINISH

                bool error = false;

                

                

                if(isConditionMatch)
                {
                    ROS_INFO("[Matrix_docking_operation]: CHARGING ---> CHARGE_END force by anther robot");
                    feedback_.sequence = DockingControlState::FINISH;
                    results_.result = matrix_msgs::ROHMChargerOperationResult::SERIAL_CON_MATCH;
                    results_.text = "Success from Xbee";
                    success = true;
                }

                
                if(!skip_timer)
                {
                    if(bat_state.power_supply_status != sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
                    {   
                        ROS_INFO("[Matrix_docking_operation]: CHARGING ---> Charger disable");
                        feedback_.sequence = DockingControlState::FINISH;
                        results_.result = matrix_msgs::ROHMChargerOperationResult::BATTERY_STATE_DIS;
                        results_.text = "success from Battery State Discon";
                        success = true;
                    }

                    if(isScheduleCompleted)
                    {
                        ROS_INFO("[Matrix_docking_operation]: CHARGING ---> Timmer stop charger");
                        auto ac_Results = schedule_operation_ac.getResult();
                        if(ac_Results->result == matrix_msgs::ScheduleResult::SUCCESS)
                        {
                            feedback_.sequence = DockingControlState::FINISH;
                            results_.result = matrix_msgs::ROHMChargerOperationResult::SCHEDULE_FINISH;
                            results_.text = "Success from Schedule";
                            success = true;
                        }
                        else if(ac_Results->result == matrix_msgs::ScheduleResult::GOAL_RUN_OUT)
                        {
                            feedback_.sequence = DockingControlState::FINISH;
                            results_.result = matrix_msgs::ROHMChargerOperationResult::TIMMER_LATE;
                            results_.text = "TIMMER_LATE";
                            success = true;
                        }
                        else
                        {   
                            ROS_ERROR("[Matrix_docking_operation]: SCHEDULE_ERROR --> Timeout");
                            results_.result = ac_Results->result;
                            results_.text = ac_Results->text;
                            feedback_.sequence = DockingControlState::ERROR;
                        }
                    }
                    
                    if(isTimmerCompleted)
                    {
                        ROS_INFO("[Matrix_docking_operation]: CHARGING ---> Timmer finish");
                        feedback_.sequence = DockingControlState::FINISH;
                        results_.result = matrix_msgs::ROHMChargerOperationResult::TIMMER_FINISH;
                        results_.text = "Sucess from Timer (Serial Operation)";
                        success = true;
                    }

                }
                
                
                break;
            }

            case DockingControlState::BACKWARD_INIT:
                moving_params_.linear_dis = goal->arg0_float;
                moving_params_.max_vel = goal->arg1_float;
                moving_params_.control_timeout = ros::Time::now().toSec() + (moving_params_.linear_dis/abs(moving_params_.max_vel)) + 10;
                setTimeOut = moving_params_.control_timeout;
                moving_params_.start_pos_x = current_pos_x;
                moving_params_.start_pos_y = current_pos_y;
                feedback_.sequence = DockingControlState::BACKWARD_CONTROL;
                feedback_.timeout = setTimeOut;
                break;
            
            case DockingControlState::BACKWARD_CONTROL:
            {
                double error_control = abs(MovingControl(MovingControlMode::BACKWARD, moving_params_));
                if(error_control < 0.005)
                {
                    
                    fnStop();
                    ROS_INFO("[Matrix_docking_operation]: BACKWARD_CONTROL --> FINISH");
                    ROS_INFO("[Matrix_docking_operation]: start x:%f, y%f | end x:%f, y:%f", moving_params_.start_pos_x, moving_params_.start_pos_y, current_pos_x, current_pos_y);
                    results_.result = matrix_msgs::ROHMChargerOperationResult::SUCCESS;
                    results_.text = "success";
                    feedback_.sequence = DockingControlState::FINISH;
                    success = true;
                }
                else
                {
                    //get timeout counter
                    feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                    ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);

                    if(ros::Time::now().toSec() > setTimeOut)
                    {
                        ROS_ERROR("[Matrix_docking_operation]: WAIT_CHARGING_STATE --> Timeout");
                        results_.result = matrix_msgs::ROHMChargerOperationResult::TIMEOUT;
                        results_.text = "timeout control backward please increase timeout parameter";
                        feedback_.sequence = DockingControlState::ERROR;
                    }
                }
                feedback_.text = "backward --> current error = " + std::to_string(error_control) + " m";
                break;
            }
                
            
            case DockingControlState::ERROR:
                ROS_ERROR("[Matrix_docking_operation]: ERROR is %s", results_.text.c_str());
                feedback_.sequence = DockingControlState::FINISH;
                break;

            //**********************************************************//
            //                                                          //
            //                      Pause Control state                 //
            //                                                          //
            //**********************************************************//
            case DockingControlState::PAUSE:
                ROS_WARN("[Matrix_docking_operation]: Robot state in Pausing mode");

                //special for docking_operation
                if(period_state == DockingControlState::BACKWARD_CONTROL)
                {
                    fnStop();
                }

                

                if(current_robotmode != matrix_msgs::RobotMode::PAUSE)
                {
                    if((period_state == DockingControlState::CHARGING) || (period_state == DockingControlState::SET_SCHEDULE) || (period_state == DockingControlState::SET_TIMMER))
                    {
                        setDockingMode("on");
                    }
                    feedback_.sequence = period_state;
                    feedback_.text = period_text;
                    period_text.clear();
                    setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
                }
                break;
            //***********************************************************//

            case DockingControlState::FINISH:
                ROS_INFO("[Matrix_docking_operation]: FINISH");   
                setDockingMode("off");
                schedule_operation_ac.cancelAllGoals();
                serial_operation_ac.cancelAllGoals();
                isScheduleCompleted = false;
                isTimmerCompleted = false;
                finished = true;
                break;
                
        }

        //**********************************************************//
        //                                                          //
        //                      check pause state                   //
        //                                                          //
        //**********************************************************//
        if((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != DockingControlState::PAUSE))
        {
            // store period control state
            period_state = feedback_.sequence;
            period_text = feedback_.text;
            // set new control sate to PAUSE 
            feedback_.sequence = DockingControlState::PAUSE;
            //get countdown time out
            period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

            feedback_.text = period_text + " [Pausing_mode]";
            
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

// void DockingOperation::SerialOperationAction(std::string cmd, std::string data)
// {
//     matrix_msgs::SerialOperationGoal goal_serial_;
//     goal_serial_.cmd = cmd;
//     goal_serial_.arg_string = data;
//     serial_operation_ac.sendGoal(goal_serial_, boost::bind(&DockingOperation::matrixSerialOpDoneCb, this, _1, _2));
// }
void DockingOperation::matrixSerialDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SerialOperationResultConstPtr &result)
{
    isTimmerCompleted = true;
    ROS_WARN("%d, %s", result->result, result->text.c_str());
    ROS_WARN("Timer_DONE");
}

void DockingOperation::matrixScheduleDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::ScheduleResultConstPtr &result)
{
    isScheduleCompleted = true;
}

void DockingOperation::preemptCB()
{
    ROS_WARN("%s got preempted!", action_name_.c_str());
    finished = true;
    success = false;
    clear_params();
    fnStop();
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

void DockingOperation::clear_params(void)
{
    //charger check state
    isConditionMatch = false;
    condition1 = "";
    condition2 = "";
    //moving params
    lastError = 0.0;
    moving_params_.linear_dis = 0.0;
    moving_params_.max_vel = 0.0;
    moving_params_.control_timeout = 0.0;
    moving_params_.start_pos_x = 0.0;
    moving_params_.start_pos_y = 0.0;

    setDockingMode("off");
    schedule_operation_ac.cancelAllGoals();
    serial_operation_ac.cancelAllGoals();
    isScheduleCompleted = false;
    isTimmerCompleted = false;
}

void DockingOperation::fnStop(void)
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
}
float DockingOperation::MovingControl(MovingControlMode mode, MovingParams moving_params)
{
    geometry_msgs::Twist twist;
    float err_pos = 0.0;
    switch(mode)
    {
        case MovingControlMode::BACKWARD:
        {
            if(moving_params.linear_dis >= 0)
            {
                err_pos = moving_params.linear_dis - sqrt(pow(current_pos_x - moving_params_.start_pos_x, 2) + pow(current_pos_y - moving_params_.start_pos_y, 2));
            }
            else
            {
                err_pos = sqrt(pow(current_pos_x - moving_params_.start_pos_x, 2) + pow(current_pos_y - moving_params_.start_pos_y, 2)) + moving_params.linear_dis;
            }
            ROS_INFO("[Matrix_docking_operation]: Moving control :%d, error_pos: %f", mode, err_pos);

            // float Kp = 0.4;
            // float Kd = 0.05;

            if(err_pos < 0)
            {
                twist.linear.x = -(moving_params.max_vel);
            }
            else
            {
                twist.linear.x = moving_params.max_vel;
            }
            
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub.publish(twist);
            break;
        }   
            
        case MovingControlMode::FORWARD:
            break;
        
        default:
            break;
    }
    return err_pos;
}


void DockingOperation::cbOdom(nav_msgs::Odometry msg)
{
current_pos_x = msg.pose.pose.position.x;
current_pos_y = msg.pose.pose.position.y;
}

void DockingOperation::cbBatt(sensor_msgs::BatteryState msg)
{
    bat_state = msg;
}

void DockingOperation::cbSerialRead(std_msgs::String::ConstPtr msg)
{
    if((condition1 != "") || (condition2 != ""))
    {
        if((msg->data == condition1) || (msg->data == condition2))
        {
            isConditionMatch = true;
            ROS_WARN("Conditaion Serial");
        }
    }
    
}

void DockingOperation::cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
{
    current_robotmode = msg->robot_mode;
}

bool DockingOperation::setDockingMode(std::string cmd)
{
    std_srvs::SetBool docking_cmd;
    docking_cmd.request.data = (cmd == "on")? true:false;
    docking_mode_sc.call(docking_cmd);

    return docking_cmd.response.success;
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_docking_operation");

    DockingOperation DockingOperation("matrix_docking_operation");
    ros::spin();

    return 0;
}