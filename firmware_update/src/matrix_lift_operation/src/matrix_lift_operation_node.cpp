#include "matrix_lift_operation/matrix_lift_operation_node.h"

bool LiftOperation::executeCB(const matrix_msgs::ROHMLiftOperationGoalConstPtr &goal)
{
    ROS_INFO("[matrix_lift_operation] recieved goal info ------->");
    ROS_INFO("[matrix_lift_operation] lift_req:%s", goal->lift_req.c_str());
    ROS_INFO("[matrix_lift_operation] lift_ready:%s", goal->lift_ready.c_str());
    ROS_INFO("[matrix_lift_operation] timeout_lift_ready:%d", goal->timeout_lift_ready);
    ROS_INFO("[matrix_lift_operation] lift_poi:%s", goal->lift_poi.c_str());
    ROS_INFO("[matrix_lift_operation] lift_finish:%s", goal->lift_finish.c_str());
    ROS_INFO("[matrix_lift_operation] timeout_lift_finish:%d", goal->timeout_lift_finish);
    ROS_INFO("[matrix_lift_operation] left_poi:%s", goal->left_poi.c_str());
    ROS_INFO("[matrix_lift_operation] move_success:%s", goal->move_success.c_str());
    
    ros::Rate r(10);

    feedback_.sequence = LiftControlState::INIT;
    bool finished = false;
    bool success = false;

    std_msgs::String serial_msg;


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
            case LiftControlState::INIT:
                feedback_.sequence = LiftControlState::WAIT_FOR_ROBOT_POSE;
                break;
            
            case LiftControlState::SEND_LIFT_REQ:
            {
                //clear data receive before send lift request 
                serial_data_.clear();

                //send request lift 
                serial_msg.data = goal->lift_req;
                serial_pub.publish(serial_msg);

                //set timeout wait lift ready 
                setTimeOut = ros::Time::now().toSec() + goal->timeout_lift_ready;

                feedback_.sequence = LiftControlState::WAIT_LIFT_READY;
                break;
            }

            case LiftControlState::WAIT_LIFT_READY:
                //wait until data coming equal lift_ready data  --> lift ready to move in 
                if(serial_data_ == goal->lift_ready)
                {
                    feedback_.sequence = LiftControlState::LIFT_READY;
                }
                else
                {
                    //if timeout set 0 sec --> ignore timeout 
                    if(goal->timeout_lift_ready != 0)
                    {
                        //check time out 
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.result = matrix_msgs::ROHMLiftOperationResult::TIMEOUT_LIFT_READY;
                            results_.result_s = "#581{TO,LiftReady}$";
                            feedback_.sequence = LiftControlState::ERROR;
                        }
                    }      
                }
                break;
            
            case LiftControlState::LIFT_READY:
                feedback_.sequence = LiftControlState::MOVE_IN_LIFT;
                break;
            
            case LiftControlState::MOVE_IN_LIFT:
                //call matrix_movement_server with poi_name
                matrix_movementSendGoal(goal->lift_poi);
                feedback_.sequence = LiftControlState::WAIT_MOVE_IN_LIFT_FINISH;
                break;
                
            case LiftControlState::WAIT_MOVE_IN_LIFT_FINISH:
            {
                matrix_msgs::XbeeCommunicationResult move_result = waitMatrixMovementFinish();
                if(move_result.result == matrix_msgs::XbeeCommunicationResult::REACHED_TARGET_POS)
                {
                    feedback_.sequence = LiftControlState::SEND_CMD;
                }
                else if(move_result.result == matrix_msgs::XbeeCommunicationResult::ACTIVE)
                {
                    ;
                }
                else
                {
                    results_.result = move_result.result;
                    results_.result_s = move_result.text;
                    feedback_.sequence = LiftControlState::ERROR;
                }
                break;
            }
            
            case LiftControlState::CALL_ENVI_SWAP:
            {
                matrix_msgs::SwapEnviOperationGoal goal_swapenvi_;
                goal_swapenvi_.envi_des = goal->envi_swap;
                matrix_swapenvi_ac.sendGoal(goal_swapenvi_, boost::bind(&LiftOperation::matrixSwapEnviDoneCb, this, _1, _2));
                //set timeout wait lift ready 
                setTimeOut = ros::Time::now().toSec() + goal->timeout_lift_finish;
                feedback_.sequence = LiftControlState::WAIT_ENVI_SWAP_FINISH;
                break;
            }

            case LiftControlState::WAIT_ENVI_SWAP_FINISH:
                if(Matrix_SeapenviCompleted)
                {
                    auto result_ = matrix_swapenvi_ac.getResult();
                    switch (result_->result)
                    {
                        case matrix_msgs::SwapEnviOperationResult::SUCCESS:
                            feedback_.sequence = LiftControlState::WAIT_LIFT_FINISH;
                            break;
            
                        default:
                            results_.result = result_->result;
                            results_.result_s =result_->text;
                            feedback_.sequence = LiftControlState::ERROR;
                    }
                }
                break;
            
            case LiftControlState::WAIT_LIFT_FINISH:
                 //wait until data coming equal lift_finish data  --> lift_finish level destination 
                if(serial_data_ == goal->lift_finish)
                {
                    feedback_.sequence = LiftControlState::LIFT_FINISH;
                }
                else
                {
                    //if timeout set 0 sec --> ignore timeout 
                    if(goal->timeout_lift_finish != 0)
                    {
                        //check time out 
                        if(ros::Time::now().toSec() > setTimeOut)
                        {
                            results_.result_s = "#581{TO,LiftFinish}$";
                            results_.result = matrix_msgs::ROHMLiftOperationResult::TIMEOUT_LIFT_FINISH;
                            feedback_.sequence = LiftControlState::ERROR;
                        }
                    }
                }
                break;
            
            case LiftControlState::LIFT_FINISH:
                feedback_.sequence = LiftControlState::LEFT_LIFT;
                break;
            
            case LiftControlState::LEFT_LIFT:
                //call matrix_movement_server with poi_name
                matrix_movementSendGoal(goal->left_poi);
                feedback_.sequence = LiftControlState::WAIT_LEFT_LIFT_FINISH;
                break;
            
            case LiftControlState::WAIT_LEFT_LIFT_FINISH:
            {
                matrix_msgs::XbeeCommunicationResult move_result = waitMatrixMovementFinish();
                if(move_result.result == matrix_msgs::XbeeCommunicationResult::REACHED_TARGET_POS)
                {
                    feedback_.sequence = LiftControlState::SEND_CMD;
                }
                else if(move_result.result == matrix_msgs::XbeeCommunicationResult::ACTIVE)
                {
                    ;
                }
                else
                {
                    results_.result = move_result.result;
                    results_.result_s = move_result.text;
                    feedback_.sequence = LiftControlState::ERROR;
                }
                break;
            }
            
            case LiftControlState::SEND_CMD:
                //send request lift 
                serial_msg.data = goal->move_success;
                serial_pub.publish(serial_msg);
                success = true;
                feedback_.sequence = LiftControlState::FINISH;
                break;

            case LiftControlState::ERROR:
                success = false;
                feedback_.sequence = LiftControlState::FINISH;
                break;
            
            case LiftControlState::FINISH:
                finished = true;
                break;

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

void LiftOperation::preemptCB()
{
    ROS_WARN("%s got preempted!", action_name_.c_str());
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


void LiftOperation::cbSerialRead(const std_msgs::String::ConstPtr &msg)
{
    serial_data_ = msg->data;
}

void LiftOperation::moveDoneCb(const actionlib::SimpleClientGoalState &state)
{
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    moveCompleted = true;
}

void LiftOperation::matrixSwapEnviDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::SwapEnviOperationResultConstPtr &result)
{
    Matrix_SeapenviCompleted = true;
}

void LiftOperation::matrixMovementDoneCb(const actionlib::SimpleClientGoalState &state, const matrix_msgs::XbeeCommunicationResultConstPtr &result)
{
    //ROS_INFO("[Matrix_XbeeManagerServer]:matrix_movement DONECB: Finished in state [%s]", state.toString().c_str());
    Matrix_MovingCompleted = true;
    // goal_xbee_ ="";
    //ROS_INFO("result is %d", result->result);
}

bool LiftOperation::matrix_movementSendGoal(std::string poi_name)
{
    Matrix_MovingCompleted = false;
    matrix_msgs::XbeeCommunicationGoal goal_in;
    //send goal to matrix_xbeemovement_server in POI mode
    goal_in.recieve_string = "#601{PO," + poi_name + "}$" ;
    if(!isSimState)
    {
        matrix_movement_ac.sendGoal(goal_in, boost::bind(&LiftOperation::matrixMovementDoneCb, this, _1, _2));
    }
    ROS_INFO("[matrix_lift_operation] Call action matrix_xbeemovement_server --> goal:%s", goal_in.recieve_string.c_str());
}

matrix_msgs::XbeeCommunicationResult LiftOperation::waitMatrixMovementFinish()
{
    matrix_msgs::XbeeCommunicationResult results_return;
    if(Matrix_MovingCompleted)
    {
        ROS_INFO("[matrix_lift_operation] MatrixMovement_FINISH");
        if(matrix_movement_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // ROS_INFO("[Matrix_XbeeMovementServer]:move_results_->result = %s", move_results_->result ? "true" : "false");
            // feedback_.sequence = LiftControlState::REACHED_TARGET_POS;
        }
        else
        {
            // feedback_.sequence = LiftControlState::MOVE_TARGET_FAIL;
        }
        //ROS_INFO("MatrixMovement_NavGoal_WaitFinish ==> move Finish");
        auto results_ = matrix_movement_ac.getResult();

        // if(results_->result != matrix_msgs::XbeeCommunicationResult::REACHED_TARGET_POS)
        // {
        //     feedback_.sequence = LiftControlState::CALL_ENVI_SWAP;
        // }
        // action_response = results_->text + "," + results_->info.poi_name;
        Matrix_MovingCompleted = false;
        // ROS_INFO("MatrixMovement_NavGoal_WaitFinish ==> %s", action_response.c_str());

        results_return = *results_;
    }
    else
    {
        // ROS_INFO("MatrixMovement_NavGoal_WaitFinish ==> isnt Finish");
        results_return.result = matrix_msgs::XbeeCommunicationResult::ACTIVE;
        results_return.text = matrix_msgs::XbeeCommunicationResult::ACTIVE_s;
    }
    
    return results_return;
}

void LiftOperation::showGoal(matrix_msgs::ROHMLiftOperationGoalConstPtr &goal)
{
    // ROS_INFO("[matrix_lift_operation] lift_req:%s", goal->lift_req.c_str());
    // ROS_INFO("[matrix_lift_operation] lift_ready:%s", goal->lift_ready.c_str());
    // ROS_INFO("[matrix_lift_operation] timeout_lift_ready:%d", goal->timeout_lift_ready);
    // ROS_INFO("[matrix_lift_operation] lift_poi:%s", goal->lift_poi.c_str());
    // ROS_INFO("[matrix_lift_operation] lift_finish:%s", goal->lift_finish.c_str());
    // ROS_INFO("[matrix_lift_operation] timeout_lift_finish:%d", goal->timeout_lift_finish);
    // ROS_INFO("[matrix_lift_operation] left_poi:%s", goal->left_poi.c_str());
    // ROS_INFO("[matrix_lift_operation] move_success:%s", goal->move_success.c_str());
    ;


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_lift_operation");

    LiftOperation liftoperation("matrix_lift_operation");
    ros::spin();

    return 0;
}