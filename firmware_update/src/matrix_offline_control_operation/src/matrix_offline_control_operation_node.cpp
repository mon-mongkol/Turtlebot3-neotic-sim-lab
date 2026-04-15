#include "matrix_offline_control_operation/matrix_offline_control_operation_node.h"

using namespace matrix_msgs;

bool OfflineControlOperation::executeCB(const matrix_msgs::OfflineControlOperationGoalConstPtr &goal)
{
    ros::Rate r(30);

    feedback_.sequence = OfflineControlState::INIT;
    finished = false;
    success = false;

    theta_travel_goal = goal->travel_dist;

    while (!finished)
    {
        // Check for ros
        if (!ros::ok())
        {
            // result.final_count = progress;
            // as_.setAborted(result,"I failed !");
            ROS_INFO("%s Shutting down", action_name_.c_str());
            break;
        }

        if (!as_.isActive() || as_.isPreemptRequested())
        {
            ;
        }

        switch (feedback_.sequence)
        {
        case OfflineControlState::INIT:
            ROS_INFO("[matrix_offline_control_operation]: INIT");
            feedback_.text = "OfflineControlState::INIT";
            // clear parameters before execute goal
            clear_params();
            feedback_.sequence = OfflineControlState::CmdSelector;

            break;

        case OfflineControlState::CmdSelector:
            if (goal->cmd == OfflineControlOperationGoal::forward)
            {
                ROS_INFO("[matrix_offline_control_operation]: CMD_SELECTOR --> Forward ");
                feedback_.sequence = OfflineControlState::MOVEMENT_INIT;
            }
            else if (goal->cmd == OfflineControlOperationGoal::backward)
            {
                ROS_INFO("[matrix_offline_control_operation]: CMD_SELECTOR --> Backward");
                feedback_.sequence = OfflineControlState::MOVEMENT_INIT;
            }
            else if (goal->cmd == OfflineControlOperationGoal::rotate_left)
            {
                ROS_INFO("[matrix_offline_control_operation]: CMD_SELECTOR --> Rotate Left");
                dir = -1;
                feedback_.sequence = OfflineControlState::ROTATE_INIT;
            }
            else if (goal->cmd == OfflineControlOperationGoal::rotate_right)
            {
                ROS_INFO("[matrix_offline_control_operation]: CMD_SELECTOR --> Rotate Right");
                dir = 1;
                feedback_.sequence = OfflineControlState::ROTATE_INIT;
            }
            else
            {
                ROS_ERROR("[matrix_offline_control_operation]: CMD_SELECTOR --> ERROR");
                results_.result = matrix_msgs::OfflineControlOperationResult::CMD_NOTFOUND;
                results_.text = "command not found";
                feedback_.sequence = OfflineControlState::ERROR;
            }
            break;

        case OfflineControlState::MOVEMENT_INIT:
            moving_params_.linear_dis = ((goal->travel_dist * offset_linear) < 0.1)? 0.1:goal->travel_dist * offset_linear ;
            moving_params_.max_vel = (goal->vel < 0.1)? 0.1:goal->vel;
            moving_params_.control_timeout = ros::Time::now().toSec() + (moving_params_.linear_dis / moving_params_.max_vel) * 2.0 + 10;
            setTimeOut = moving_params_.control_timeout;
            moving_params_.start_pos_x = current_pos_x;
            moving_params_.start_pos_y = current_pos_y;
            feedback_.sequence = OfflineControlState::MOVEMENT;
            feedback_.timeout = setTimeOut;
            break;

        case OfflineControlState::MOVEMENT:
        {
            double error_control = 0.0;
            if (goal->cmd == OfflineControlOperationGoal::backward)
            {
                error_control = abs(MovingControl(MovingControlMode::BACKWARD, moving_params_));
            }
            else
            {
                error_control = abs(MovingControl(MovingControlMode::FORWARD, moving_params_));
            }

            if (error_control < 0.01)
            {

                fnStop();
                ROS_INFO("[matrix_offline_control_operation]: MOVEMENT_INIT --> FINISH");
                ROS_INFO("[matrix_offline_control_operation]: start x:%f, y%f | end x:%f, y:%f", moving_params_.start_pos_x, moving_params_.start_pos_y, current_pos_x, current_pos_y);
                results_.result = matrix_msgs::OfflineControlOperationResult::SUCCESS;
                results_.text = "success";
                feedback_.sequence = OfflineControlState::FINISH;
                success = true;
            }
            else
            {
                // get timeout counter
                feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);

                if (ros::Time::now().toSec() > setTimeOut)
                {
                    fnStop();
                    ROS_ERROR("[matrix_offline_control_operation]: WAIT_CHARGING_STATE --> Timeout");
                    results_.result = matrix_msgs::OfflineControlOperationResult::TIMEOUT;
                    results_.text = "timeout control backward please increase timeout parameter";
                    feedback_.sequence = OfflineControlState::ERROR;
                }
            }
            feedback_.text = "backward --> current error = " + std::to_string(error_control) + " m";
            break;
        }

        case OfflineControlState::ROTATE_INIT:
            ROS_INFO("[matrix_offline_control_operation]: ROTATE_INIT");
            // goal rotate
            // desired_theta = current_theta + 3.316;
            if (goal->cmd == OfflineControlOperationGoal::rotate_left)
            {
                // desired_theta = (current_theta + (goal->travel_dist / 180) * 3.14) * 1;
                // +10 degree 
                desired_theta = (current_theta + (goal->travel_dist/ 180) * 3.14) * offset_angular;
            }
            else
            {
                // desired_theta = (current_theta - (goal->travel_dist / 180) * 3.14) * 1;
                // +10 degree 
                desired_theta = (current_theta - (goal->travel_dist/ 180) * 3.14) * offset_angular;
            }

            ROS_INFO("current theta %f, desired_theta %f error %f", current_theta, desired_theta, desired_theta - current_theta);
            lastError = 0.0;
            // _isInit_rotate = true;
            setTimeOut = ros::Time::now().toSec() + (goal->travel_dist / (goal->vel * 180 / math_pi)) * 2.0 + 5;
            theta_travel_goal = goal->travel_dist*1.15;
            theta_now = current_theta;
            theta_old = current_theta;
            feedback_.sequence = OfflineControlState::ROTATE;
            feedback_.timeout = setTimeOut;
            break;

        case OfflineControlState::ROTATE:
            if (abs(fnrotate(goal->vel, dir)) < 0.020)
            {
                fnStop();
                ROS_INFO("[matrix_offline_control_operation]: ROTATE --> FINISH");
                results_.result = matrix_msgs::OfflineControlOperationResult::SUCCESS;
                results_.text = "success";
                feedback_.sequence = OfflineControlState::FINISH;
                success = true;
            }
            else
            {
                // get timeout counter
                feedback_.timeout = setTimeOut - ros::Time::now().toSec();
                ROS_INFO("[matrix_serial_operation]: Check timeout countdown %d", feedback_.timeout);

                if (ros::Time::now().toSec() > setTimeOut)
                {
                    fnStop();
                    ROS_ERROR("[matrix_offline_control_operation]: WAIT_CHARGING_STATE --> Timeout");
                    results_.result = matrix_msgs::OfflineControlOperationResult::TIMEOUT;
                    results_.text = "timeout control backward please increase timeout parameter";
                    feedback_.sequence = OfflineControlState::ERROR;
                }
            }
            break;

        case OfflineControlState::ERROR:
            ROS_ERROR("[matrix_offline_control_operation]: ERROR is %s", results_.text.c_str());
            feedback_.sequence = OfflineControlState::FINISH;
            break;

        //**********************************************************//
        //                                                          //
        //                      Pause Control state                 //
        //                                                          //
        //**********************************************************//
        case OfflineControlState::PAUSE:
            ROS_WARN("[matrix_offline_control_operation]: Robot state in Pausing mode");

            // special for docking_operation
            if (period_state == OfflineControlState::MOVEMENT || period_state == OfflineControlState::ROTATE)
            {
                fnStop();
            }

            if (current_robotmode != matrix_msgs::RobotMode::PAUSE)
            {
                if ((period_state == OfflineControlState::MOVEMENT_INIT) || (period_state == OfflineControlState::ROTATE_INIT))
                {
                    // setDockingMode("on");
                    ;
                }
                feedback_.sequence = period_state;
                feedback_.text = period_text;
                period_text.clear();
                setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
            }
            break;
            //***********************************************************//

        case OfflineControlState::FINISH:
            ROS_INFO("[matrix_offline_control_operation]: FINISH");
            fnStop();
            // setDockingMode("off");

            finished = true;
            break;
        }

        //**********************************************************//
        //                                                          //
        //                      check pause state                   //
        //                                                          //
        //**********************************************************//
        if ((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != OfflineControlState::PAUSE))
        {
            // store period control state
            period_state = feedback_.sequence;
            period_text = feedback_.text;
            // set new control sate to PAUSE
            feedback_.sequence = OfflineControlState::PAUSE;
            // get countdown time out
            period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

            feedback_.text = period_text + " [Pausing_mode]";
        }

        as_.publishFeedback(feedback_);
        r.sleep();
    }
    fnStop();
    results_.result = results_.result;
    ROS_INFO("%s: %s", action_name_.c_str(), (success) ? "Succeeded" : "Fail");
    as_.setSucceeded(results_);

    ROS_INFO("********************************");
    ROS_INFO(" ");
}

void OfflineControlOperation::preemptCB()
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

void OfflineControlOperation::clear_params(void)
{
    // charger check state
    //  isConditionMatch = false;
    //  condition1 = "";
    //  condition2 = "";
    // moving params
    current_pos_x = 0.0;
    current_pos_y = 0.0;
    current_theta = 0.0;
    last_current_theta = 0.0;
    desired_theta = 0.0;
    lastError = 0;
    moving_params_.linear_dis = 0.0;
    moving_params_.max_vel = 0.0;
    moving_params_.control_timeout = 0.0;
    moving_params_.start_pos_x = 0.0;
    moving_params_.start_pos_y = 0.0;
    theta_travel_current = 0.0;
    theta_now = 0.0;
    theta_old = 0.0;
    theta_travel_goal = 0.0;

    // setDockingMode("off");
}

void OfflineControlOperation::fnStop(void)
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

// float OfflineControlOperation::fnrotateModern(float vel, int dir)
// {
//     theta_now = current_theta;
//     theta_travel_current += abs(abs(theta_now) - abs(theta_old));
//     theta_old = theta_now;

// }

float OfflineControlOperation::fnrotate(float vel, int dir)
{

    theta_now = current_theta;
    theta_travel_current += abs(abs(theta_now) - abs(theta_old)) * 1.2;
    float err_theta = (theta_travel_goal * math_pi / 180) - theta_travel_current;
    theta_old = theta_now;
    ROS_INFO("err_theta: %f , theta_travel_goal: %f, theta_travel_current: %f", err_theta, theta_travel_goal, theta_travel_current);
    // float err_theta = imu_pose2d.theta - desired_theta;
    // float err_theta = current_theta - desired_theta;
    // ROS_INFO("err_theta: %f , desired_theta: %f, current_theta: %f", err_theta, desired_theta, current_theta);
    // ROS_INFO("err_theta: %f , desired_theta: %f, current_theta: %f", err_theta, desired_theta, imu_pose2d.theta);

    float Kp = 0.3;
    float Ki = 0.05;
    float Kd = 0.03;

    float max_vel = vel;

    float angular_z = (Kp * err_theta) + (Ki * (err_theta + lastError)) + (Kd * (err_theta - lastError));
    lastError = err_theta;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;

    twist.angular.z = (angular_z < 0) ? -std::max(angular_z, -max_vel) : -std::min(angular_z, max_vel);
    ROS_INFO("angular_z %f, twist.angular.z %f", angular_z, twist.angular.z);

    twist.angular.z = std::min(copysign((float)max_vel, (float)twist.angular.z), std::max(copysign((float)0.13, (float)twist.angular.z), (float)twist.angular.z));
    twist.angular.z *= dir;
    // if(abs(angular_z) > max_vel)

    // {
    //     twist.angular.z = std::copysign(max_vel, angular_z);
    // }
    // else
    // {
    //     twist.angular.z = angular_z;
    // }

    cmd_vel_pub.publish(twist);

    return err_theta;
}

float OfflineControlOperation::MovingControl(MovingControlMode mode, MovingParams moving_params)
{
    geometry_msgs::Twist twist;
    float err_pos = 0.0;
    switch (mode)
    {
    case MovingControlMode::BACKWARD:
        // err_pos = sqrt(pow(current_pos_x - moving_params_.start_pos_x, 2) + pow(current_pos_y - moving_params_.start_pos_y, 2)) + moving_params.linear_dis;
        err_pos = -moving_params.linear_dis + abs(sqrt(pow(current_pos_x - moving_params_.start_pos_x, 2) + pow(current_pos_y - moving_params_.start_pos_y, 2)));
        twist.linear.x = -moving_params.max_vel;
        break;

    case MovingControlMode::FORWARD:
        err_pos = moving_params.linear_dis - sqrt(pow(current_pos_x - moving_params_.start_pos_x, 2) + pow(current_pos_y - moving_params_.start_pos_y, 2));
        twist.linear.x = moving_params.max_vel;
        break;

    default:
        break;
    }

    ROS_INFO("[matrix_offline_control_operation]: Moving control :%d, error_pos: %f", mode, err_pos);

    // if(err_pos < 0)
    // {
    //     twist.linear.x = -(moving_params.max_vel);
    // }
    // else
    // {
    //     twist.linear.x = moving_params.max_vel;
    // }

    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);

    return err_pos;
}

void OfflineControlOperation::cbOdom(nav_msgs::Odometry msg)
{
    current_pos_x = msg.pose.pose.position.x;
    current_pos_y = msg.pose.pose.position.y;

    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // pose2d.theta = yaw * (180.0/3.141592653589793238463);
    // if(pose2d.theta < 0) pose2d.theta += 360.0;
    // ROS_INFO("current theta degree %f", pose2d.theta);

    current_theta = yaw;

    // if(current_theta < 0)
    // {
    //     current_theta += 6.2831;
    // }

    // if((current_theta - last_current_theta) < -math_pi)
    // {
    //     current_theta = (2.0 * math_pi) + current_theta;
    //     last_current_theta = math_pi;
    // }
    // else if((current_theta - last_current_theta) > math_pi)
    // {
    //     current_theta = (-2.0 * math_pi) + current_theta;
    //     last_current_theta = -math_pi;
    // }
    // else
    // {
    //     last_current_theta = current_theta;
    // }

    // ROS_INFO("current theta %f", current_theta);
}

void OfflineControlOperation::cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
{
    current_robotmode = msg->robot_mode;
}

// bool OfflineControlOperation::setDockingMode(std::string cmd)
// {
//     std_srvs::SetBool docking_cmd;
//     docking_cmd.request.data = (cmd == "on")? true:false;
//     docking_mode_sc.call(docking_cmd);

//     return docking_cmd.response.success;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_offline_control_operation");

    OfflineControlOperation OfflineControlOperation("matrix_offline_control_operation");
    ros::spin();

    return 0;
}