#include "matrix_docking_ar_operation/matrix_docking_ar_operation_node_multi_methods.h"

using namespace matrix_msgs;

bool ARDockingOperation::executeCB(const matrix_msgs::ARDockingOperationMultiMethodsGoalConstPtr &goal)
{
    ros::Rate r(10);

    feedback_.sequence = ARDockingControlState::INIT_PARM;
    finished = false;
    success = false;

    // std::string cg_id_str = std::to_string(goal->charger_id);
    std::string cg_id_str = goal->charger_id_str;
    DG_SN="[" +goal->charger_id_str+"]";

    error_counter = 0;

    _isDockingStateSync = true;

    

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
            case ARDockingControlState::INIT_PARM:
                ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_PARM");
                feedback_.text = "INIT_PARM";
                
                //clear parameters before execute goal
                clear_params(false, true);
                ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_PARM--->Cleara param successs");


                
                feedback_.sequence = ARDockingControlState::CMD_SELECTOR;
                
                break;

            case ARDockingControlState::CMD_SELECTOR:
                ROS_INFO("[matrix_docking_multi_methods_operation]: CMD_SELECTOR");
                feedback_.text = "CMD_SELECTOR";
                feedback_.sequence = ARDockingControlState::GOAL_CHECKER;

                // if(goal->method == "cvshape_method")
                // {
                //     feedback_.sequence = ARDockingControlState::LAUNCH_CVSHAPE;
                // }
                // else if(goal->method == "ar3track_method")
                // {
                //     feedback_.sequence = ARDockingControlState::LAUNCH_AR_TRACK;
                // }
                // else
                // {
                //     ;
                // }
                break;
            
            
            case ARDockingControlState::GOAL_CHECKER:
                ROS_INFO("[matrix_docking_multi_methods_operation]: GOAL_CHECKER");
                // setDockingMode("on");
                feedback_.text = "GOAL_CHECKER";
                feedback_.sequence = ARDockingControlState::SEND_CG_END;
                break;
            
            case ARDockingControlState::SEND_CG_END:
            {
                ROS_INFO("[matrix_docking_multi_methods_operation]: SEND_CG_END"); 

                data_send = DG_FINISH + DG_SN;
                data_receive = RxREADY + DG_SN;
                goal_timeout = goal->communication_timeout;

                SerialActionSend(SerialOperationGoal::send_receive, data_send, data_receive, goal_timeout, true);

                setTimeOut = ros::Time::now().toSec() + goal_timeout + 5;

                feedback_.text = "SEND_DGr_END";
                feedback_.sequence = ARDockingControlState::WAIT_CG_FINISH;
                break;
            }
                
            case ARDockingControlState::WAIT_CG_FINISH:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_CG_FINISH");
                feedback_.text = "WAIT_CG_FINISH";

                if(Matrix_SerialCompleted)
                {   
                    auto serial_results = serial_operation_rs485_ac.getResult();
                    if(serial_results->result == SerialOperationResult::SUCCESS)
                    {
                        // feedback_.sequence = ARDockingControlState::SEND_CG_CHECK;
                        // feedback_.sequence = ARDockingControlState::SEARCH_AR;
                        feedback_.sequence = ARDockingControlState::ENABLE_DOCKING_MODE;
                    }
                    else
                    {
                        results_.result = ARDockingOperationMultiMethodsResult::RESULT_DOCKING_NOT_RESPONSE;
                        results_.success = false;
                        results_.text = "fail communication with docking in WAIT_CG_FINISH, wait -->" + data_receive;
                        results_.sequence = ARDockingControlState::WAIT_CG_FINISH;
                        feedback_.sequence = ARDockingControlState::ERROR;
                    }
                }
                else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal->communication_timeout != 0)
                    {
                        ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout");
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.success = false;
                            results_.result = ARDockingOperationMultiMethodsResult::RESULT_CONTROL_SEQUENCE_TIMEOUT;
                            results_.text = "control sequence timeout in WAIT_CG_FINISH, wait -->" + data_receive;
                            results_.sequence = ARDockingControlState::WAIT_CG_FINISH;
                            feedback_.sequence = ARDockingControlState::ERROR;
                        }
                    }
                }
                break;
            
            case ARDockingControlState::SEND_CG_CHECK:
            {
                ROS_INFO("[matrix_docking_multi_methods_operation]: SEND_CG_CHECK");
                feedback_.text = "SEND_CG_CHECK";

                goal_timeout = goal->communication_timeout;
                Matrix_SerialCompleted = false;

                SerialOperationGoal rs485_goal;
                rs485_goal.cmd = SerialOperationGoal::serial_receive_multi;
                rs485_goal.arg_sr_send = "#" + CHARGE_CHECK + DG_SN + "$";
                rs485_goal.arg_strings.push_back(CHARGE_OK + DG_SN);
                rs485_goal.arg_strings.push_back(CHARGE_NOT_OK + DG_SN);
                rs485_goal.timeout = goal_timeout;
                // rs485.arg_sr_send_repeats = true;
                serial_operation_rs485_ac.sendGoal(rs485_goal, boost::bind(&ARDockingOperation::matrixSerialDoneCb, this, _1, _2));

                setTimeOut = ros::Time::now().toSec() + goal_timeout + 5;
                feedback_.sequence = ARDockingControlState::WAIT_CG_OK;
                break;
            }
            
            case ARDockingControlState::WAIT_CG_OK:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_CG_OK");
                feedback_.text = "communication with docking in WAIT_CG_OK, wait -->" + (CHARGE_OK + DG_SN) + ", " + (CHARGE_NOT_OK + DG_SN);

                if(Matrix_SerialCompleted)
                {   
                    auto serial_results = serial_operation_rs485_ac.getResult();
                    if(serial_results->result == SerialOperationResult::SUCCESS)
                    {
                        if(serial_results->data_bypass == (CHARGE_OK + DG_SN))
                        {
                            // feedback_.sequence = ARDockingControlState::SEARCH_AR;
                            feedback_.sequence = ARDockingControlState::ENABLE_DOCKING_MODE;
                        }
                        else
                        {
                            results_.success = false;
                            results_.sequence = ARDockingControlState::WAIT_CG_OK;
                            results_.text = "charger return " + serial_results->data_bypass;
                            results_.result = ARDockingOperationMultiMethodsResult::RESULT_DOCKING_NOT_OK;
                            ROS_ERROR("[matrix_docking_multi_methods_operation]: %s", serial_results->data_bypass.c_str());
                            feedback_.sequence = ARDockingControlState::ERROR;
                        }
                        
                    }
                    else
                    {
                        results_.result = ARDockingOperationMultiMethodsResult::RESULT_DOCKING_NOT_RESPONSE;
                        results_.success = false;
                        results_.text = "fail " + feedback_.text;
                        results_.sequence = ARDockingControlState::WAIT_CG_OK;
                        ROS_ERROR("[matrix_docking_multi_methods_operation]: %s", results_.text.c_str());
                        feedback_.sequence = ARDockingControlState::ERROR;
                    }
                }
                else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal->communication_timeout != 0)
                    {
                        ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout");
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.success = false;
                            results_.result = ARDockingOperationMultiMethodsResult::RESULT_CONTROL_SEQUENCE_TIMEOUT;
                            results_.text = "control sequence timeout -->" + feedback_.text;
                            results_.sequence = feedback_.sequence;
                            feedback_.sequence = ARDockingControlState::ERROR;
                        }
                    }
                }

                
                break;

            


            case ARDockingControlState::ENABLE_DOCKING_MODE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: ENABLE_DOCKING_MODE");
                setDockingMode("on");
                feedback_.sequence = ARDockingControlState::SELECT_TRACKING_METHOD;
                // if(setDockingMode("on"))
                // {
                //     feedback_.text = "SELECT_TRACKING_METHOD";
                //     feedback_.sequence = ARDockingControlState::SELECT_TRACKING_METHOD;
                // }
                // else
                // {
                //     ROS_ERROR("[matrix_docking_multi_methods_operation]: Please Press master on");
                //     feedback_.text = "Please Press master on";
                //     feedback_.sequence = ARDockingControlState::ERROR;
                // }
                break;
            


            case ARDockingControlState::SELECT_TRACKING_METHOD:
                ROS_INFO("[matrix_docking_multi_methods_operation]: SELECT_TRACKING_METHOD");
                if(!goal->skip_tracking_to_dock)
                {
                    if(goal->method == "cvshape_method")
                    {
                        feedback_.sequence = ARDockingControlState::LAUNCH_CVSHAPE;
                    }
                    else if(goal->method == "ar3track_method")
                    {
                        feedback_.sequence = ARDockingControlState::LAUNCH_AR_TRACK;
                    }
                    else
                    {
                        ROS_ERROR("[matrix_docking_multi_methods_operation]: Not found tracking methods");
                        feedback_.text = "Not found tracking method";
                        feedback_.sequence = ARDockingControlState::ERROR;
                    }
                }
                else
                {
                    setMotorMode("Unlock_Motor","active");
                    feedback_.sequence = ARDockingControlState::INTI_SKIP_TRACKING_DOCK;
                }
                break;
            

            case ARDockingControlState::INTI_SKIP_TRACKING_DOCK:
                ROS_INFO("[matrix_docking_multi_methods_operation]: INTI_SKIP_TRACKING_DOCK");
                data_send = "";
                data_receive = RxWRBREQ + DG_SN;
                goal_timeout = goal->communication_timeout+60;
                SerialActionSend(SerialOperationGoal::send_receive, data_send, data_receive, goal_timeout, false);
                feedback_.text = "INTI_SKIP_TRACKING_DOCK";
                feedback_.sequence = ARDockingControlState::WAIT_CG_CONFIRM_CONNECT;
                break;

            case ARDockingControlState::LAUNCH_AR_TRACK:
                ROS_INFO("[matrix_docking_multi_methods_operation]: LAUNCH_AR_TRACK");
                LaunchController(ActionControllerRequest::CMD_ACTION_START, ActionControllerRequest::AC_AR3_REAR);
                feedback_.sequence = ARDockingControlState::WAIT_AR_TRACK_LAUNCH_FINISH;
                break;
            
            case ARDockingControlState::WAIT_AR_TRACK_LAUNCH_FINISH:
                if(!artrack3_operation_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the matrix_schedule_operation action server to come up");
                }
                else
                {
                    ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_PARM--->Launch AR3 successs");
                    feedback_.text = "WAIT_AR_TRACK_LAUNCH_FINISH";
                    feedback_.sequence = ARDockingControlState::SEARCH_AR;
                }
                break;
            
            case ARDockingControlState::SEARCH_AR:
                ROS_INFO("[matrix_docking_multi_methods_operation]: SEARCH_AR--->[SKIP]");
                feedback_.sequence = ARDockingControlState::INIT_AR_TRACKING;
                break;
            
            case ARDockingControlState::INIT_AR_TRACKING:
                ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_AR_TRACKING");

                // set Serial action rs485 for wait data --> #ROBOT_CONNECTED_CG + CG_ID$
                data_send = "";
                // data_receive = "WAIT_ROBOT_REQ_CG" + DG_SN;
                data_receive = RxWRBREQ + DG_SN;
                goal_timeout = goal->communication_timeout+60;
                SerialActionSend(SerialOperationGoal::send_receive, data_send, data_receive, goal_timeout, false);
                Matrix_ARTrack3Completed = false;
                feedback_.text = "INIT_AR_TRACKING";
                feedback_.sequence = ARDockingControlState::AR_TRACKING;
                break;
            
            case ARDockingControlState::AR_TRACKING:
            {
                ROS_INFO("[matrix_docking_multi_methods_operation]: SEND_CG_END");
                // control until connect charger
                feedback_.text = "SEND_CG_END and AR Tracking";
                if(_isSim)
                {
                    feedback_.sequence = ARDockingControlState::WAIT_FINISH_AR_TRACKING;
                }
                else
                {
                    autodock_core::AutoDockingGoal artrack3_goal;
                    artrack3_goal.use_cfg_yaml = true;
                    artrack3_operation_ac.sendGoal(artrack3_goal, boost::bind(&ARDockingOperation::matrixARTrack3DonceCb, this, _1, _2));
                    setTimeOut = ros::Time::now().toSec() + goal->tracking_timeout; 
                    feedback_.sequence = ARDockingControlState::WAIT_FINISH_AR_TRACKING;
                }
                
                break;
            }
            case ARDockingControlState::WAIT_FINISH_AR_TRACKING:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_FINISH_AR_TRACKING");
                feedback_.text = "WAIT_FINISH_AR_TRACKING tracking..." ;

                if(_isSim)
                {
                    feedback_.sequence = ARDockingControlState::WAIT_CG_CONFIRM_CONNECT;
                }
                else
                {
                    if(Matrix_ARTrack3Completed)
                    {
                        auto artrack3_results = artrack3_operation_ac.getResult();
                        if(artrack3_results->is_success)
                        {
                            feedback_.sequence = ARDockingControlState::WAIT_CG_CONFIRM_CONNECT;
                        }else
                        {
                            results_.result = ARDockingOperationMultiMethodsResult::RESULT_FAIL_AR_TRACK;
                            results_.success = false;
                            results_.text = "fail " + feedback_.text;
                            results_.sequence = feedback_.sequence;
                            ROS_ERROR("[matrix_docking_multi_methods_operation]: %s", results_.text.c_str());
                            feedback_.sequence = ARDockingControlState::ERROR;
                        }

                        LaunchController(ActionControllerRequest::CMD_ACTION_KILL, ActionControllerRequest::AC_AR3_REAR);
                    }
                    else
                    {
                        //if timeout set 0 sec --> ignore timeout
                        if(goal->tracking_timeout != 0)
                        {
                            ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout");
                            feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                            ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout countdown %d", feedback_.timeout);
                            //check time out
                            if(ros::Time::now().toSec() > setTimeOut)
                            {
                                results_.success = false;
                                results_.result = ARDockingOperationMultiMethodsResult::RESULT_CONTROL_SEQUENCE_TIMEOUT;
                                results_.text = "control sequence timeout -->" + feedback_.text;
                                results_.sequence = feedback_.sequence;
                                feedback_.sequence = ARDockingControlState::ERROR;
                            }
                        }
                    }
                }
                
                break;

            case ARDockingControlState::LAUNCH_CVSHAPE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: LAUNCH_CVSHAPE");
                // LaunchController(ActionControllerRequest::CMD_ACTION_START, ActionControllerRequest::AC_AR3_REAR);
                feedback_.sequence = ARDockingControlState::WAIT_CVSHAPE_LAUNCH_FINISH;
                break;
            
            case ARDockingControlState::WAIT_CVSHAPE_LAUNCH_FINISH:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_CVSHAPE_LAUNCH_FINISH");
                if(!cvshape_operation_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the matrix_schedule_operation action server to come up");
                }
                else
                {
                    ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_PARM--->Launch CV_SHAPE successs");
                    feedback_.text = "WAIT_CVSHAPE_LAUNCH_FINISH";
                    feedback_.sequence = ARDockingControlState::SEARCH_CV_SHAPE;
                }
                break;
            
            case ARDockingControlState::SEARCH_CV_SHAPE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: SEARCH_CV_SHAPE --> [SKIP]");
                feedback_.sequence = ARDockingControlState::INIT_CV_SHAPE;
                break;
            
            case ARDockingControlState::INIT_CV_SHAPE:
                // set Serial action rs485 for wait data --> #ROBOT_CONNECTED_CG + CG_ID$
                data_send = "";
                // data_receive = "WAIT_ROBOT_REQ_CG" + DG_SN;
                data_receive = RxWRBREQ + DG_SN;
                goal_timeout = goal->communication_timeout+60;
                SerialActionSend(SerialOperationGoal::send_receive, data_send, data_receive, goal_timeout, false);
                Matrix_CVShapeCompleted = false;
                feedback_.text = "INIT_CV_SHAPE";
                feedback_.sequence = ARDockingControlState::CV_SHPAE_TRACKING;
                break;
            
            case ARDockingControlState::CV_SHPAE_TRACKING:
                ROS_INFO("[matrix_docking_multi_methods_operation]: CV_SHPAE_TRACKING");
                // control until connect charger
                feedback_.text = "CV_SHPAE_TRACKING";
                if(_isSim)
                {
                    feedback_.sequence = ARDockingControlState::WAIT_FINISH_CV_TRACKING;
                }
                else
                {
                    matrix_msgs::CenterVShapeTrackingGoal cv_goal;
                    cv_goal = (goal->cv_from_yaml)? cv_goal_yaml:goal->cv_shape_param;
                    cvshape_operation_ac.sendGoal(cv_goal, boost::bind(&ARDockingOperation::matrixCVshapeDonceCb, this, _1, _2));
                    setTimeOut = ros::Time::now().toSec() + goal->tracking_timeout; 
                    feedback_.sequence = ARDockingControlState::WAIT_FINISH_CV_TRACKING;
                }
                
                break;
            
            case ARDockingControlState::WAIT_FINISH_CV_TRACKING:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_FINISH_CV_TRACKING");
                feedback_.text = "WAIT_FINISH_CV_TRACKING tracking..." ;
                if(_isSim)
                {
                    feedback_.sequence = ARDockingControlState::WAIT_CG_CONFIRM_CONNECT;
                }
                else
                {
                    if(Matrix_CVShapeCompleted)
                    {
                        auto cvshape_results = cvshape_operation_ac.getResult();
                        if(cvshape_results->result == matrix_msgs::CenterVShapeTrackingResult::SUCCESS)
                        {
                            feedback_.sequence = ARDockingControlState::WAIT_CG_CONFIRM_CONNECT;
                        }else
                        {
                            results_.result = ARDockingOperationMultiMethodsResult::RESULT_FAIL_CVSHAPE;
                            results_.success = false;
                            results_.text = "fail " + feedback_.text;
                            results_.sequence = feedback_.sequence;
                            ROS_ERROR("[matrix_docking_multi_methods_operation]: %s", results_.text.c_str());
                            feedback_.sequence = ARDockingControlState::ERROR;
                        }
                    }
                    else
                    {
                        //if timeout set 0 sec --> ignore timeout
                        if(goal->tracking_timeout != 0)
                        {
                            ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout");
                            feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                            ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout countdown %d", feedback_.timeout);
                            //check time out
                            if(ros::Time::now().toSec() > setTimeOut)
                            {
                                results_.success = false;
                                results_.result = ARDockingOperationMultiMethodsResult::RESULT_CONTROL_SEQUENCE_TIMEOUT;
                                results_.text = "control sequence timeout -->" + feedback_.text;
                                results_.sequence = feedback_.sequence;
                                feedback_.sequence = ARDockingControlState::ERROR;
                            }
                        }
                    }
                }
                break;
            

            case ARDockingControlState::WAIT_CG_CONFIRM_CONNECT:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_CG_CONFIRM_CONNECT");
                feedback_.text = "communication with docking in WAIT_CG_FINISH, wait -->" + data_receive;
                
                SerialActionWaitFinish(feedback_.sequence, 
                                        ARDockingControlState::SEND_CG_REQ_ON, 
                                        ARDockingControlState::ERROR, 
                                        goal->communication_timeout);
                // feedback_.sequence = ARDockingControlState::SEND_CG_REQ_ON;
                break;
            
            case ARDockingControlState::SEND_CG_REQ_ON:
            {

                ROS_INFO("[matrix_docking_multi_methods_operation]: SEND_CG_REQ_ON");
                feedback_.text = "SEND_CG_REQ_ON";

                setMotorMode("Unlock_Motor","inactive");
                
                matrix_msgs::SetIOs set_ios_srv;
                set_ios_srv.request.cmd = "enable_busbar";
                set_ios_srv.request.arg0 = 1;
                set_ios_srv.request.arg1 = 0;
                set_ios_sc.call(set_ios_srv);

                // set Serial action rs485 for send data for request charger on --> #ROBOT_CONNECTED_CG + CG_ID$
                data_send = DG_REQ + DG_SN;
                // data_receive = CG_REQ_SUCCESS + DG_SN;
                data_receive = RxREQSS + DG_SN;
                goal_timeout = goal->communication_timeout;

                SerialActionSend(SerialOperationGoal::send_receive, 
                                    data_send, 
                                    data_receive, 
                                    goal_timeout, 
                                    true);

                feedback_.sequence = ARDockingControlState::WAIT_CG_REQ_ON_SUCCESS;
                // feedback_.sequence = ARDockingControlState::INIT_BMS_CONFIRM_STATE;
                break;
            }
                
            
            case ARDockingControlState::WAIT_CG_REQ_ON_SUCCESS:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_CG_REQ_ON_SUCCESS");
                feedback_.text = "communication with docking in WAIT_CG_REQ_ON_SUCCESS, wait -->" + data_receive;

                SerialActionWaitFinish(feedback_.sequence, 
                                        ARDockingControlState::INIT_BMS_CONFIRM_STATE, 
                                        ARDockingControlState::ERROR, 
                                        goal->communication_timeout);
                break;
            
            case ARDockingControlState::INIT_BMS_CONFIRM_STATE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_BMS_CONFIRM_STATE");
                feedback_.text = "INIT_BMS_CONFIRM_STATE";
                
                goal_timeout = goal->charging_state_timeout;
                setTimeOut = ros::Time::now().toSec() + goal_timeout;
                feedback_.sequence = ARDockingControlState::WAIT_BMS_CONFIRM_STATE;
                break;
            
            case ARDockingControlState::WAIT_BMS_CONFIRM_STATE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_BMS_CONFIRM_STATE");
                feedback_.text = "WAIT_BMS_CONFIRM_STATE";
                // if(_isSim)
                // {
                //     batt_state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
                // }

                // wait until battery supply state change to charging
                if( batt_state.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
                {
                    if(!skip_confirm_state)
                    {
                        feedback_.sequence = ARDockingControlState::SEND_CONFIRM_STATE_TO_DOCK;
                    }
                    else
                    {
                        feedback_.sequence =  ARDockingControlState::INIT_CG_CONDITION_FINISH;
                    }
                }
                else
                {
                    //if timeout set 0 sec --> ignore timeout
                    if(goal_timeout != 0)
                    {
                        ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout");
                        feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                        ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout countdown %d", feedback_.timeout);
                        //check time out
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.success = false;
                            results_.result = ARDockingOperationMultiMethodsResult::RESULT_CONTROL_SEQUENCE_TIMEOUT;
                            results_.text = "control sequence timeout -->" + feedback_.text;
                            results_.sequence = feedback_.sequence;
                            feedback_.sequence = ARDockingControlState::ERROR;
                        }
                    }
                }
                break;
            
            case ARDockingControlState::SEND_CONFIRM_STATE_TO_DOCK:
                ROS_INFO("[matrix_docking_multi_methods_operation]: SEND_CONFIRM_STATE_TO_DOCK");
                feedback_.text = "SEND_CONFIRM_STATE_TO_DOCK";
                // set Serial action rs485 for send data for request charger on --> #ROBOT_CONNECTED_CG + CG_ID$
                data_send = DG_RBICS + DG_SN;
                data_receive = RxRBICS + DG_SN;
                goal_timeout = goal->communication_timeout;
                setTimeOut = ros::Time::now().toSec() + goal_timeout;
                SerialActionSend(SerialOperationGoal::send_receive, 
                                    data_send, 
                                    data_receive, 
                                    goal_timeout, 
                                    true);

                feedback_.sequence = ARDockingControlState::WAIT_CG_CONFIRM_STATE;
                break;
            
            case ARDockingControlState::WAIT_CG_CONFIRM_STATE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: WAIT_CG_CONFIRM_STATE");
                feedback_.text = "communication with docking in WAIT_CG_CONFIRM_STATE, wait -->" + data_receive;

                SerialActionWaitFinish(feedback_.sequence, 
                                        ARDockingControlState::INIT_CG_CONDITION_FINISH, 
                                        ARDockingControlState::ERROR, 
                                        goal->communication_timeout);
                break;
            
            case ARDockingControlState::INIT_CG_CONDITION_FINISH:
            {   ROS_INFO("[matrix_docking_multi_methods_operation]: INIT_CG_CONDITION_FINISH");
                feedback_.text = "INIT_CG_CONDITION_FINISH";

                // goal_timeout = goal->communication_timeout;
                Matrix_SerialCompleted = false;
                Matrix_ScheduleCompleted = false;

                // check serial data for finish charge
                if(sizeof(goal->wait_serial) != 0)
                {
                    SerialOperationGoal rs485_goal;
                    rs485_goal.cmd = SerialOperationGoal::serial_receive_multi;
                    rs485_goal.arg_strings = goal->wait_serial;
                    // rs485_goal.arg_sr_send = CHARGE_CHECK + DG_SN;
                    // rs485_goal.arg_strings.push_back(CHARGE_OK + DG_SN);
                    // rs485_goal.arg_strings.push_back(CHARGE_NOT_OK + DG_SN);
                    rs485_goal.timeout = 0;
                    serial_operation_rs485_ac.sendGoal(rs485_goal, boost::bind(&ARDockingOperation::matrixSerialDoneCb, this, _1, _2));
                }
                
                // set charging time
                charging_time = ros::Time::now().toSec() + goal->charging_time;
                
                // set schedule
                if(goal->schedule.cmd != "")
                {
                    // ***** available please tranform msg in matrix_schedule_operation.cpp to matrix_msgs pkg
                    // matrix_msgs::ScheduleGoal goal_schedule;
                    // goal_schedule.date = goal->schedule;
                    // schedule_operation_ac.sendGoal(goal_schedule, boost::bind(&ARDockingOperation::matrixScheduleDoneCb, this, _1, _2));
                }
                results_.success = false;
                results_.result = 0;
                results_.text = "";

                feedback_.sequence = ARDockingControlState::CHECK_CONDITION_FINISH;
                break;
            }
            case ARDockingControlState::CHECK_CONDITION_FINISH:
                ROS_INFO("[matrix_docking_multi_methods_operation]: CHECK_CONDITION_FINISH");
                feedback_.text = "CHECK_CONDITION_FINISH";

                //if charging_time set 0 --> ignore charging_time
                if(goal->charging_time != 0)
                {
                    ROS_INFO("[matrix_docking_multi_methods_operation]: Check charging_time");
                    feedback_.timeout = charging_time - ros::Time::now().toSec();
                    ROS_INFO("[matrix_docking_multi_methods_operation]: Check charging_time countdown %d", feedback_.timeout);
                    //check time out
                    if(feedback_.timeout <= 0)
                    {
                        results_.success = true;
                        results_.result = ARDockingOperationMultiMethodsResult::RESULT_TIMMER_FINISH;
                        results_.text = "charge finish (Charging time finish)";
                    }
                }

                if( batt_state.power_supply_status != sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
                {
                    results_.success = true;
                    results_.result = ARDockingOperationMultiMethodsResult::RESULT_BMS_FINISH;
                    results_.text = "charge finish (BMS change state finish)";
                    ROS_INFO("battery state %d", batt_state.power_supply_status);
                }

                if(Matrix_SerialCompleted)
                {
                    results_.success = true;
                    results_.result = ARDockingOperationMultiMethodsResult::RESULT_SERIAL_FINISH;
                    results_.text = "charge finish (Serial change state finish)";
                }

                if(results_.success)
                {
                    // rs485_goal.cmd = SerialOperationGoal::serial_send;
                    // rs485_goal.arg_string = "#DG_FINISH4$";
                    // Matrix_SerialCompleted = false;

                    // serial_operation_rs485_ac.sendGoal(rs485_goal, boost::bind(&ARDockingOperation::matrixSerialDoneCb, this, _1, _2));

                    results_.sequence = feedback_.sequence;
                    feedback_.sequence = ARDockingControlState::FINISH;
                }

                
                break;
            
            case ARDockingControlState::DODCKING_NOT_SYNC:
                ROS_INFO("[matrix_docking_multi_methods_operation]: ERROR");
                feedback_.text = "Docking state not Sync Robot state:[" + feedback_.text + "] docking state:[" +docking_state_str+"]";
                results_.success = false;
                results_.result = 77;
                results_.text = feedback_.text;
                results_.sequence = feedback_.sequence;
                feedback_.sequence = ARDockingControlState::ERROR;
                break;


            case ARDockingControlState::ERROR:
                ROS_INFO("[matrix_docking_multi_methods_operation]: ERROR");
                feedback_.text = "ERROR" + feedback_.text;
                ROS_ERROR("[Matrix_docking_operation]: ERROR is %s", results_.text.c_str());
                
                error_counter++;

                if(error_counter < 3)
                {
                    if(results_.sequence <= ARDockingControlState::SEARCH_AR)
                    {
                        feedback_.sequence = ARDockingControlState::SEND_CG_END;
                    }
                    else if(results_.sequence <= ARDockingControlState::WAIT_FINISH_AR_TRACKING )
                    {
                        Matrix_ARTrack3Completed = false;
                        feedback_.sequence = ARDockingControlState::SEARCH_AR;
                    }
                    else
                    {
                        goto finish_state;
                    }
                    
                }
                else
                {
                    finish_state:
                    LaunchController(ActionControllerRequest::CMD_ACTION_KILL, ActionControllerRequest::AC_AR3_REAR);
                    feedback_.sequence = ARDockingControlState::FINISH;
                }
                
                break;

            //**********************************************************//
            //                                                          //
            //                      Pause Control state                 //
            //                                                          //
            //**********************************************************//
            case ARDockingControlState::PAUSE:
                ROS_INFO("[matrix_docking_multi_methods_operation]: PAUSE");
                feedback_.text = "PAUSE";
                //special for docking_operation
                if(period_state >= ARDockingControlState::SEARCH_AR)
                {
                    fnStop();
                    // artrack3_operation_ac.cancelAllGoals();
                    std_msgs::Bool pause_dock_msg;
                    pause_dock_msg.data = true;
                    pause_dock_pub.publish(pause_dock_msg);
                }

                

                if(current_robotmode != matrix_msgs::RobotMode::PAUSE)
                {
                    //if((period_state == ARDockingControlState::CHARGING) || (period_state == ARDockingControlState::SET_SCHEDULE) || (period_state == ARDockingControlState::SET_TIMMER))
                    if(period_state >= ARDockingControlState::SEARCH_AR)
                    {
                        setDockingMode("on");
                        std_msgs::Bool pause_dock_msg;
                        pause_dock_msg.data = false;
                        pause_dock_pub.publish(pause_dock_msg);
                    }
                    feedback_.sequence = period_state;
                    feedback_.text = period_text;
                    period_text.clear();
                    setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
                }
                break;
            //***********************************************************//

            case ARDockingControlState::FINISH:
                ROS_INFO("[matrix_docking_multi_methods_operation]: FINISH");
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

        if((!_isDockingStateSync) && (feedback_.sequence != ARDockingControlState::ERROR))
        {
            ROS_ERROR("_isDockingStateSync");
            feedback_.sequence = ARDockingControlState::DODCKING_NOT_SYNC;
            _isDockingStateSync = true;
        }

        //**********************************************************//
        //                                                          //
        //                      check pause state                   //
        //                                                          //
        //**********************************************************//
        if((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != ARDockingControlState::PAUSE))
        {
            // store period control state
            period_state = feedback_.sequence;
            period_text = feedback_.text;
            // set new control sate to PAUSE 
            feedback_.sequence = ARDockingControlState::PAUSE;
            //get countdown time out
            period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

            feedback_.text = period_text + " [Pausing_mode]";
            
        }

        as_.publishFeedback(feedback_);
        r.sleep();
    }
    results_.result = results_.result;
    ROS_INFO("%s: %s", action_name_.c_str(), (results_.result) ? "Succeeded" : "Fail");
    as_.setSucceeded(results_);

    ROS_INFO("********************************");
    ROS_INFO(" ");

}


void ARDockingOperation::matrixCVshapeDonceCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::CenterVShapeTrackingResultConstPtr &result)
{
    Matrix_CVShapeCompleted = true;
}

void ARDockingOperation::matrixSerialDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SerialOperationResultConstPtr &result)
{
    Matrix_SerialCompleted = true;
}

void ARDockingOperation::matrixScheduleDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::ScheduleResultConstPtr &result)
{
    Matrix_ScheduleCompleted = true;
}

void ARDockingOperation::matrixARTrack3DonceCb(const actionlib::SimpleClientGoalState &state, const autodock_core::AutoDockingResultConstPtr &result)
{
Matrix_ARTrack3Completed = true;
}
 
void ARDockingOperation::preemptCB()
{
    ROS_WARN("%s got preempted!", action_name_.c_str());
    finished = true;
    success = false;
    clear_params(false, true);
    fnStop();
    as_.setPreempted();
}

void ARDockingOperation::clear_params(bool cancel_action_ar3, bool kill_action_ar3)
{
    //charger check state
    _isDockingStateSync = true;
    docking_state_str = "";
    isConditionMatch = false;
    condition1 = "";
    condition2 = "";
    //moving params
    // lastError = 0.0;
    // moving_params_.linear_dis = 0.0;
    // moving_params_.max_vel = 0.0;
    // moving_params_.control_timeout = 0.0;
    // moving_params_.start_pos_x = 0.0;
    // moving_params_.start_pos_y = 0.0;

    

    setDockingMode("off");
    schedule_operation_ac.cancelAllGoals();
    serial_operation_rs485_ac.cancelAllGoals();
    
    Matrix_ScheduleCompleted = false;
    Matrix_SerialCompleted = false;
    Matrix_ARTrack3Completed = false;
    matrix_msgs::SetIOs set_ios_srv;

    set_ios_srv.request.cmd = "enable_busbar";
    set_ios_srv.request.arg0 = 0;
    set_ios_srv.request.arg1 = 0;
    set_ios_sc.call(set_ios_srv);

    SerialOperationGoal rs485_goal;
    rs485_goal.cmd = SerialOperationGoal::serial_send;
    rs485_goal.arg_string = "#" + DG_FINISH + DG_SN +"$";
    Matrix_SerialCompleted = false;

    serial_operation_rs485_ac.sendGoal(rs485_goal, boost::bind(&ARDockingOperation::matrixSerialDoneCb, this, _1, _2));
    

    setDockingMode("off");

    if(cancel_action_ar3)
    {
        artrack3_operation_ac.cancelAllGoals();
    }
    else;

    if(kill_action_ar3)
    {
        LaunchController(ActionControllerRequest::CMD_ACTION_KILL, ActionControllerRequest::AC_AR3_REAR);

    }
    
    setMotorMode("Unlock_Motor","inactive");
    


    
}

void ARDockingOperation::fnStop(void)
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




void ARDockingOperation::cbOdom(nav_msgs::Odometry msg)
{
    // current_pos_x = msg.pose.pose.position.x;
    // current_pos_y = msg.pose.pose.position.y;
}

void ARDockingOperation::cbBatt(sensor_msgs::BatteryState msg)
{
    batt_state = msg;
}

void ARDockingOperation::cbSerialRead(std_msgs::String::ConstPtr msg)
{
    if((condition1 != "") || (condition2 != ""))
    {
        if((msg->data == condition1) || (msg->data == condition2))
        {
            isConditionMatch = true;
        }
    }
    
}

void ARDockingOperation::cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
{
    current_robotmode = msg->robot_mode;
}

bool ARDockingOperation::setDockingMode(std::string cmd)
{
    std_srvs::SetBool docking_cmd;
    docking_cmd.request.data = (cmd == "on")? true:false;
    docking_mode_sc.call(docking_cmd);

    return docking_cmd.response.success;
}

bool ARDockingOperation::setMotorMode(std::string cmd, std::string mode)
{   
    bool cmd_not_found = false;
    dynamic_reconfigure::Reconfigure srv;
    if(cmd == "Unlock_Motor")
    {
        dynamic_reconfigure::BoolParameter unlock_motor;
        unlock_motor.name = "free_mode";
        unlock_motor.value = (mode == "active")? true:false;
        srv.request.config.bools.push_back(unlock_motor);
    }
    else
    {
        cmd_not_found = true;
    }

    if(!cmd_not_found)
    {
        motor_control_sc.call(srv);
    }
    return ~cmd_not_found;
}

bool ARDockingOperation::LaunchController(std::string cmd, std::string action_name)
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
void ARDockingOperation::SerialActionSend(std::string cmd, std::string send, std::string receive, double timeout, bool repeat)
{
    SerialOperationGoal rs485_goal;
    rs485_goal.cmd = cmd;

    // if(rs485_goal.cmd == SerialOperationGoal::)
    rs485_goal.arg_sr_send = "#" + send + "$";
    rs485_goal.arg_sr_receive = receive;
    rs485_goal.timeout = timeout;
    rs485_goal.arg_sr_send_repeats = repeat;

    Matrix_SerialCompleted = false;

    serial_operation_rs485_ac.sendGoal(rs485_goal, boost::bind(&ARDockingOperation::matrixSerialDoneCb, this, _1, _2));
}

void ARDockingOperation::SerialActionSendMulti(std::string cmd, std::string send, std::string* receive_s, double timeout, bool repeat)
{
    SerialOperationGoal rs485_goal;
    rs485_goal.cmd = cmd;
    if(rs485_goal.cmd == SerialOperationGoal::serial_receive_multi)
    {
        rs485_goal.arg_string = send;
        // rs485_goal.arg_strings = receive_s;
    }
    rs485_goal.timeout = timeout;
    Matrix_SerialCompleted = false;
    serial_operation_rs485_ac.sendGoal(rs485_goal, boost::bind(&ARDockingOperation::matrixSerialDoneCb, this, _1, _2));
}


void ARDockingOperation::cbRS485Receive(std_msgs::String::ConstPtr msg)
{
    ROS_INFO("GetData");
     std::string a;
    a = msg->data;
    ROS_INFO("%s %d", a.c_str(), current_sequence);
    // std::string cg_id_str = std::to_string(goal->charger_id);
    if((current_sequence > ARDockingControlState::SEARCH_AR) && (current_sequence <= ARDockingControlState::WAIT_CG_CONFIRM_STATE))
    {
        ROS_INFO("SSSSS");
        if(_isDockingStateSync)
        {
            // check docking cancel
           ROS_INFO("1111");
            if(a.find("RxRBDCN") != -1)
            {
                _isDockingStateSync = false;
                docking_state_str=msg->data;
            }
            else if(a.find("PLS_PRESS_BUSBAR") != -1)
            {
                _isDockingStateSync = false;
                docking_state_str=msg->data;
            }
            else if(a.find("CHARGE_NOT_OK") != -1)
            {
                _isDockingStateSync = false;
                docking_state_str=msg->data;
            }
            else if(a.find("RxEND") != -1)
            {
                _isDockingStateSync = false;
                docking_state_str=msg->data;
            }
            else
            {
                ROS_INFO("4444");
            }
        }
        
    }

}

void ARDockingOperation::SerialActionWaitFinish(int current_seq, ARDockingControlState pass_seq, ARDockingControlState fail_seq, double goal_timeout)
{
    if(Matrix_SerialCompleted)
    {   
        auto serial_results = serial_operation_rs485_ac.getResult();
        if(serial_results->result == SerialOperationResult::SUCCESS)
        {
            feedback_.sequence = pass_seq;
        }
        else
        {
            results_.result = ARDockingOperationMultiMethodsResult::RESULT_DOCKING_NOT_RESPONSE;
            results_.success = false;
            results_.text = "fail" + feedback_.text;
            results_.sequence = feedback_.sequence;
            feedback_.sequence = fail_seq;
        }
    }
    else
    {
        //if timeout set 0 sec --> ignore timeout
        if(goal_timeout != 0)
        {
            ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout");
            feedback_.timeout = setTimeOut - ros::Time::now().toSec();
            ROS_INFO("[matrix_docking_multi_methods_operation]: Check timeout countdown %d", feedback_.timeout);
            //check time out
            if(ros::Time::now().toSec() > setTimeOut)
            {
                results_.success = false;
                results_.result = ARDockingOperationMultiMethodsResult::RESULT_CONTROL_SEQUENCE_TIMEOUT;
                results_.text = "control sequence timeout -->" + feedback_.text;
                results_.sequence = feedback_.sequence;
                feedback_.sequence = fail_seq;
            }
        }
    }
}

// bool ARDockingOperation::setDockingMode(std::string cmd)
// {
//     std_srvs::SetBool docking_cmd;
//     docking_cmd.request.data = (cmd == "on")? true:false;
//     docking_mode_sc.call(docking_cmd);

//     return docking_cmd.response.success;
// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_docking_multi_methods_operation");

    ARDockingOperation DockingOperation("matrix_docking_multi_methods_operation");
    ros::spin();

    return 0;
}