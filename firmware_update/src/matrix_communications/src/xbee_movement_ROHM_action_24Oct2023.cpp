#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ist_roads_srv/GoRoadAction.h>
#include <web_interface_msgs/Layout.h>
#include <matrix_msgs/XbeeCommuROHM.h>
#include <bits/stdc++.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <web_interface_msgs/RouteFollowAction.h>
#include <ist_roads_srv/GoRoadAction.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <matrix_msgs/XbeeCommunicationAction.h>
#include <std_srvs/Empty.h>
#include <matrix_msgs/ListMap.h>
#include <dynamic_reconfigure/Reconfigure.h>

// for pause state
#include <matrix_msgs/RobotMode.h>
#include <std_msgs/String.h>

using namespace matrix_msgs;
typedef actionlib::SimpleActionClient<ist_roads_srv::GoRoadAction> IstRoadsClient;
typedef actionlib::SimpleActionClient<web_interface_msgs::RouteFollowAction> IstRouteClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class XbeeMovementServer
{
protected:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<matrix_msgs::XbeeCommunicationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    matrix_msgs::XbeeCommunicationFeedback feedback_;
    matrix_msgs::XbeeCommunicationResult result_;

    IstRouteClient ist_route_ac;
    IstRoadsClient ist_roads_ac;
    MoveBaseClient move_base_ac;

    bool IstRoads_MovingCompleted;
    bool IstRoute_MovingCompleted;

    bool moveCompleted = false;
    uint movebase_status;

    ros::Publisher pub_movebase_goal, pub_remote_xbee;

    ros::Subscriber sub_movebase_result;

    ros::ServiceClient layouts_srv_, clearcostmap_srv_, envi_info_sc, costmap_global_cfg, costmap_local_cfg, DWA_cfg, movebase_cfg;

    // for pause state
    ros::Subscriber serial_sub, robotmode_sub;
    int period_state = 0;
    int period_timeout_coutdown = 0;
    int current_robotmode = 0;
    std::string period_text = "";
    double setTimeOut;
    bool pause_flag = false;

    // bool trigger = false;

    uint poi_mode = 0;
    enum Task_Sequence
    {
        Init = 0,
        DecodeString = 1,

        RobotConditionCheck,
        ConditionMatch,
        ConditionMissMatch,

        CmdSelector,

        MoveBase_LayoutsNavGoal,
        MoveBase_LayoutsNavGoal_WaitFinish,

        MoveBase_NavGoal,
        MoveBase_NavGoal_WaitFinish,

        IstRoads_NavGoal,
        IstRoads_NavGoal_WaitFinish,

        REACHED_TARGET_POS,
        MOVE_TARGET_FAIL,

        WaitClearObstacle,

        CancleAllMoveGoals,

        SendRespond,
        Fisnish,
        ERROR,
        PAUSE
    };

    std::string serial_no = "ROHM020020220001A";

    unsigned int period_id;
    bool success = true;
    bool task_completed = false;

    // configuring parameters
    std::string map_frame, base_frame;

    std::string command_roads,
        command_poi,
        command_cancel,
        command_getpoi,
        serialno_split_begin,
        serialno_split_end,
        movementmode_split_begin,
        movementmode_split_end,
        poiname_split_begin,
        poiname_split_end;

    int movefail_repeats = 4;
    int movefail_counter = 0;
    int wait_clear_obstacle = 3;
    // double setTimeOut = 0;
    bool move_base = false;

public:
    XbeeMovementServer(std::string name) : as_(nh_, name, boost::bind(&XbeeMovementServer::executeCB, this, _1), false),
                                           action_name_(name),
                                           ist_roads_ac("ist_roads_action", true),
                                           ist_route_ac("ist_route_follow", true),
                                           move_base_ac("move_base", true)
    {

        // wait for the action server to come up
        while (!ist_roads_ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("[Matrix_XbeeMovementServer]:Waiting for the ist_roads_ac action server to come up");
        }

        // wait for the action server to come up
        while (!move_base_ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("[Matrix_XbeeMovementServer]:Waiting for the move_base_ac action server to come up");
        }

        as_.start();
        as_.registerPreemptCallback(boost::bind(&XbeeMovementServer::preemptCB, this));

        init_pub_();
        init_sub_();
        init_srv_();
        init_param_();
    }

    void init_pub_()
    {
        pub_movebase_goal = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        pub_remote_xbee = nh_.advertise<std_msgs::String>("remote_xbee", 10);
    }

    void init_sub_()
    {
        // for pause state
        robotmode_sub = nh_.subscribe("/matrix_mode_controller/mode", 1, &XbeeMovementServer::cbRobotMode, this);
        serial_sub = nh_.subscribe("/raw_read", 1, &XbeeMovementServer::cbSerialRead, this);

        sub_movebase_result = nh_.subscribe("move_base/result", 1, &XbeeMovementServer::movebaseResultCallback, this);
    }

    void init_srv_()
    {
        layouts_srv_ = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
        clearcostmap_srv_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        envi_info_sc = nh_.serviceClient<matrix_msgs::ListMap>("matrix_map_list");
        costmap_global_cfg = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/global_costmap/rgbd_obstacle_layer/set_parameters");
        costmap_local_cfg = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/local_costmap/rgbd_obstacle_layer/set_parameters");
        DWA_cfg = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/DWAPlannerROS/set_parameters");
        movebase_cfg = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
    }

    void init_param_()
    {
        ros::param::param<std::string>("~serial_no", serial_no, "ROHM020020220001A");
        ros::param::param<std::string>("~map_frame", map_frame, "/map");
        ros::param::param<std::string>("~base_frame", base_frame, "/base_link");

        ros::param::param<std::string>("~command_roads", command_roads, "RO");
        ros::param::param<std::string>("~command_poi", command_poi, "PO");
        ros::param::param<std::string>("~command_cancel", command_cancel, "CANCEL");
        ros::param::param<std::string>("~command_getpoi", command_getpoi, "GETPOI");

        ros::param::param<std::string>("~serialno_split_begin", serialno_split_begin, "#");
        ros::param::param<std::string>("~serialno_split_end", serialno_split_end, "{");
        ros::param::param<std::string>("~movementmode_split_begin", movementmode_split_begin, "{");
        ros::param::param<std::string>("~movementmode_split_end", movementmode_split_end, ",");
        ros::param::param<std::string>("~poiname_split_begin", poiname_split_begin, ",");
        ros::param::param<std::string>("~poiname_split_end", poiname_split_end, "}$");

        ros::param::param<int>("~movefail_repeats", movefail_repeats, 4);
        ros::param::param<int>("~wait_clear_obstacle", wait_clear_obstacle, 3);
    }

    ~XbeeMovementServer(void)
    {
    }

    void preemptCB()
    {
        ROS_WARN("%s got preempted!", action_name_.c_str());
        step_complete();
        response_serial("", "cancel_accept");

        cancelMoveAction();
        // step_complete();
        success = false;
        task_completed = true;
        result_.info = feedback_.info;
        result_.result = XbeeCommunicationResult::GOT_PREEMPTED;
        result_.text = XbeeCommunicationResult::GOT_PREEMPTED_s;
        as_.setSucceeded(result_);
        as_.setPreempted();
        response_serial("", "cancel_success");

        costmap_3d_enabled(false);
    }

    void executeCB(const matrix_msgs::XbeeCommunicationGoalConstPtr &goal)
    {
        pause_flag = false;

        ros::Rate rate(5);

        step_complete();

        success = false;
        task_completed = false;

        feedback_.sequence = Task_Sequence::Init;

        matrix_msgs::XbeeCommuROHM data_info_;

        // if (!as_.isActive() || as_.isPreemptRequested())
        //     return;

        while (!task_completed)
        {
            // Check for ros
            if (!ros::ok())
            {
                // result.final_count = progress;
                // as_.setAborted(result,"I failed !");
                ROS_INFO("[Matrix_XbeeMovementServer]:%s Shutting down", action_name_.c_str());
                break;
            }

            // if (!as_.isActive() || as_.isPreemptRequested())
            // {
            //     return;
            // }

            switch (feedback_.sequence)
            {
            case Task_Sequence::Init:
                ROS_INFO("[Matrix_XbeeMovementServer]:Receive goal --> %s", goal->recieve_string.c_str());
                cancelMoveAction();
                step_complete();
                feedback_.sequence = Task_Sequence::DecodeString;
                break;

            case Task_Sequence::DecodeString:
                data_info_ = getCmdInfo(goal->recieve_string);
                feedback_.info = data_info_;
                result_.info = feedback_.info;
                feedback_.sequence = Task_Sequence::RobotConditionCheck;
                break;

            case Task_Sequence::RobotConditionCheck:
            {
                int robot_no_ = DecString_int(split_string(serial_no, "ROHM02002022", "A"));
                if (data_info_.robot_no == robot_no_)
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:Robot number Match with Serial No. %d!!", robot_no_);
                    feedback_.sequence = Task_Sequence::ConditionMatch;
                }
                else
                {
                    ROS_ERROR("[Matrix_XbeeMovementServer]:Robot number miss match Serial No.!! current robot no. %d", robot_no_);
                    result_.result = XbeeCommunicationResult::ROBOT_MISS_MATCH;
                    result_.text = XbeeCommunicationResult::ROBOT_MISS_MATCH_s;
                    feedback_.sequence = Task_Sequence::SendRespond;
                }
                break;
            }

            case Task_Sequence::ConditionMatch:
                feedback_.sequence = Task_Sequence::CmdSelector;
                break;

            case Task_Sequence::CmdSelector:
            {
                bool error = false;
                ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::CmdSelector");
                // else if(data_info_.cmd_string == matrix_msgs::XbeeCommuROHM::roads)
                if (data_info_.cmd_string == command_roads)
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:Go to State --> IstRoads_NavGoal");
                    data_info_.cmd_uint = matrix_msgs::XbeeCommuROHM::CMD_ROADS;
                    feedback_.sequence = Task_Sequence::IstRoads_NavGoal;
                }
                // else if(data_info_.cmd_string == matrix_msgs::XbeeCommuROHM::poi)
                else if (data_info_.cmd_string == command_poi)
                {
                    if (goal->invert_dir)
                    {
                        ROS_INFO("[Matrix_XbeeMovementServer]:Go to State --> MoveBase_NavGoal(invert_dir set True)");
                        feedback_.sequence = Task_Sequence::MoveBase_NavGoal;
                    }
                    else
                    {
                        ROS_INFO("[Matrix_XbeeMovementServer]:Go to State --> MoveBase_LayoutsNavGoal");
                        data_info_.cmd_uint = matrix_msgs::XbeeCommuROHM::CMD_POI;
                        feedback_.sequence = Task_Sequence::MoveBase_LayoutsNavGoal;
                    }
                }
                // else if(data_info_.cmd_string == matrix_msgs::XbeeCommuROHM::cancel)
                else if (data_info_.cmd_string == command_cancel)
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:Go to State --> CancleAllMoveGoals");
                    data_info_.cmd_uint = matrix_msgs::XbeeCommuROHM::CMD_CANCLE_MOVE;
                    feedback_.sequence = Task_Sequence::CancleAllMoveGoals;
                }
                else
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s Command not found!!", data_info_.cmd_string);
                    result_.result = XbeeCommunicationResult::COMMAND_NOT_FOUND;
                    result_.text = XbeeCommunicationResult::COMMAND_NOT_FOUND_s;
                    feedback_.sequence = Task_Sequence::SendRespond;
                    error = true;
                }

                if (data_info_.cmd_string != command_cancel && !error)
                {
                    // check command for enable RGB obstacle layer global & local costmap
                    costmap_3d_enabled(true);
                    if (goal->allow_backward)
                    {
                        MoveBase_parameters_cfg("DWA");
                        DWA_parameters_cfg(-0.35, 2.5);
                    }
                }
                break;
            }

            case Task_Sequence::CancleAllMoveGoals:

                feedback_.sequence = Task_Sequence::SendRespond;
                break;

            case Task_Sequence::MoveBase_NavGoal:
            {
                ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::MoveBase_NavGoal");
                geometry_msgs::PoseStamped mb_goal;
                // get traget pose from /matrix_list_map (srv)
                matrix_msgs::ListMap get_pose;
                get_pose.request.cmd = matrix_msgs::ListMapRequest::pose_from_poi_invert;
                get_pose.request.arg = data_info_.poi_name;
                if (envi_info_sc.call(get_pose))
                {
                    mb_goal.pose = get_pose.response.pose;

                    if ((movefail_counter == 0) && (!pause_flag))
                    {
                        response_serial(data_info_, "goal_accept");
                        response_serial(data_info_, "move_pending");
                    }
                    else
                    {
                        response_serial(data_info_, "retry");
                    }

                    mb_goal.header.stamp = ros::Time::now();
                    mb_goal.header.frame_id = "map";
                    pub_movebase_goal.publish(mb_goal);
                    feedback_.sequence = Task_Sequence::MoveBase_NavGoal_WaitFinish;
                }
                else
                {
                    ROS_ERROR("[Matrix_XbeeMovementServer]:cannot call /matrix_list_map");
                    feedback_.sequence = Task_Sequence::ERROR;
                    result_.text = matrix_msgs::XbeeCommunicationResult::CANNOT_GET_POSE_SRV_s;
                    result_.result = matrix_msgs::XbeeCommunicationResult::CANNOT_GET_POSE_SRV;
                }
                movebase_status = actionlib_msgs::GoalStatus::PENDING;
                moveCompleted = false;
                move_base = true;
                break;
            }

            case Task_Sequence::MoveBase_NavGoal_WaitFinish:
                ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::MoveBase_NavGoal_WaitFinish");
                if (moveCompleted)
                {
                    if (movebase_status == actionlib_msgs::GoalStatus::SUCCEEDED)
                    {
                        ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::MoveBase_NavGoal_WaitFinish REACHED_TARGET_POS");
                        feedback_.sequence = Task_Sequence::REACHED_TARGET_POS;
                        // ROS_INFO("[Matrix_XbeeMovementServer]:Robot reached target position");
                    }
                    else if (/* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::ABORTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::PENDING || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::REJECTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::LOST)
                    {
                        ROS_ERROR("[Matrix_XbeeMovementServer]:Task_Sequence::MoveBase_NavGoal_WaitFinish MOVE_TARGET_FAIL");
                        feedback_.sequence = Task_Sequence::MOVE_TARGET_FAIL;
                    }
                    else
                        ;
                }
                // feedback_.sequence = Task_Sequence::SendRespond;
                break;

            case Task_Sequence::MoveBase_LayoutsNavGoal:
            {
                if (poi_mode == 0)
                {
                    web_interface_msgs::RouteFollowGoal goal;
                    // goal.uuid = getUUID(getPOIActionForm(data_info_.poi));
                    goal.uuid = getUUID(data_info_.poi_name);

                    if (goal.uuid != "")
                    {
                        if ((movefail_counter == 0) && (!pause_flag))
                        {
                            response_serial(data_info_, "goal_accept");
                            response_serial(data_info_, "move_pending");
                        }
                        else
                        {
                            response_serial(data_info_, "retry");
                        }

                        goal.type = 28;
                        goal.repeat = 0;
                        goal.reverse = false;
                        ROS_INFO("[Matrix_XbeeMovementServer]:%s", goal.uuid.c_str());
                        ist_route_ac.sendGoal(goal, boost::bind(&XbeeMovementServer::istRouteDoneCb, this, _1, _2));
                        feedback_.sequence = Task_Sequence::MoveBase_LayoutsNavGoal_WaitFinish;
                    }
                    else
                    {
                        response_serial("NotFoundPOI", "move");
                        ROS_ERROR("[Matrix_XbeeMovementServer]:RouteFollowGoal %s POI name not found!!", data_info_.poi_name.c_str());
                        result_.result = XbeeCommunicationResult::POI_NAME_NOT_FOUND;
                        result_.text = XbeeCommunicationResult::POI_NAME_NOT_FOUND_s;
                        feedback_.sequence = Task_Sequence::SendRespond;
                    }
                    moveCompleted = false;
                    movebase_status = actionlib_msgs::GoalStatus::PENDING;
                    IstRoute_MovingCompleted = false;
                }
                else
                {
                    ;
                    // ROS_INFO("[Matrix_XbeeMovementServer]:MoveBase_LayoutsNavGoal");
                    // move_base_msgs::MoveBaseGoal goal = getPoseFromLayouts(getPOIActionForm(data_info_.poi));
                    // goal.target_pose.header.frame_id = "map";
                    // goal.target_pose.header.stamp = ros::Time::now();

                    // ROS_INFO("[Matrix_XbeeMovementServer]:x %f, y %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

                    // tf::Quaternion q(
                    //     goal.target_pose.pose.orientation.x,
                    //     goal.target_pose.pose.orientation.y,
                    //     goal.target_pose.pose.orientation.z,
                    //     goal.target_pose.pose.orientation.w);
                    // tf::Matrix3x3 m(q);
                    // double roll, pitch, yaw;
                    // m.getRPY(roll, pitch, yaw);
                    // ROS_INFO("[Matrix_XbeeMovementServer]:R %f, P %F, Y %f", roll, pitch, yaw);

                    // moveCompleted = false;
                    // movebase_status = actionlib_msgs::GoalStatus::PENDING;
                    // pub_movebase_goal.publish(goal.target_pose);
                }
                break;
            }

            case Task_Sequence::MoveBase_LayoutsNavGoal_WaitFinish:
                if (poi_mode == 0)
                {
                    // if (IstRoute_MovingCompleted)
                    // {
                    //     if(ist_route_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    //     {
                    //         // auto ist_result_ = ist_roads_ac.getResult();
                    //         // ROS_INFO("[Matrix_XbeeMovementServer]:move_result_->result = %s", move_result_->result ? "true" : "false");
                    //         feedback_.sequence = Task_Sequence::REACHED_TARGET_POS;
                    //     }
                    //     else
                    //     {

                    //     }
                    // }

                    if (moveCompleted)
                    {
                        if (movebase_status == actionlib_msgs::GoalStatus::SUCCEEDED)
                        {
                            feedback_.sequence = Task_Sequence::REACHED_TARGET_POS;
                            // ROS_INFO("[Matrix_XbeeMovementServer]:Robot reached target position");
                        }
                        else if (/* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::ABORTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::PENDING || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::REJECTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::LOST)
                        {
                            // if(movefail_counter > movefail_repeats)
                            // {
                            feedback_.sequence = Task_Sequence::MOVE_TARGET_FAIL;
                            // }
                            // else
                            // {
                            ist_route_ac.cancelAllGoals();
                            move_base_ac.cancelAllGoals();
                            //     movefail_counter++;
                            //     moveCompleted = false;
                            //     movebase_status = actionlib_msgs::GoalStatus::PENDING;
                            //     IstRoute_MovingCompleted = false;
                            //     feedback_.sequence = Task_Sequence::CmdSelector;

                            // }

                            // ROS_ERROR("[Matrix_XbeeMovementServer]:current movefial_counter: %d", movefail_counter);
                        }
                        else
                        {
                            ROS_WARN("Preempting %d", movebase_status);
                            preemptCB();
                        }
                    }
                    else
                    {
                        ROS_ERROR("Task_Sequence::MoveBase_LayoutsNavGoal_WaitFinish");
                    }
                }
                else
                {
                    if (moveCompleted)
                    {
                        // if (movebase_status == actionlib_msgs::GoalStatus::SUCCEEDED)
                        // {
                        //     feedback_.sequence = Task_Sequence::REACHED_TARGET_POS;
                        //     // ROS_INFO("[Matrix_XbeeMovementServer]:Robot reached target position");
                        // }
                        if (/* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::ABORTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::PENDING || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::REJECTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::LOST)
                        {
                            feedback_.sequence = Task_Sequence::MOVE_TARGET_FAIL;
                        }
                    }
                }

                break;

            case Task_Sequence::IstRoads_NavGoal:
            {
                ist_roads_srv::GoRoadGoal goal;
                goal.id = matrix_msgs::XbeeCommuROHM::IST_ROAD_ROADS;
                goal.cmd = "poi";
                goal.from = "";

                if (checkPOI(data_info_.poi_name))
                {
                    if ((movefail_counter == 0) && (!pause_flag))
                    {
                        response_serial(data_info_, "goal_accept");
                        response_serial(data_info_, "move_pending");
                    }
                    else
                    {
                        response_serial(data_info_, "retry");
                    }

                    goal.target = data_info_.poi_name;
                    ist_roads_ac.sendGoal(goal, boost::bind(&XbeeMovementServer::istRoadsDoneCb, this, _1, _2));
                    feedback_.sequence = Task_Sequence::IstRoads_NavGoal_WaitFinish;
                }
                else
                {
                    response_serial("NotFoundPOI", "move");
                    ROS_ERROR("[Matrix_XbeeMovementServer]:IstRoads_NavGoal %s POI name not found!!", data_info_.poi_name.c_str());
                    result_.result = XbeeCommunicationResult::POI_NAME_NOT_FOUND;
                    result_.text = XbeeCommunicationResult::POI_NAME_NOT_FOUND_s;
                    feedback_.sequence = Task_Sequence::SendRespond;
                }

                break;
            }

            case Task_Sequence::IstRoads_NavGoal_WaitFinish:
                if (IstRoads_MovingCompleted)
                {
                    if (ist_roads_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        // auto ist_result_ = ist_roads_ac.getResult();
                        // ROS_INFO("[Matrix_XbeeMovementServer]:move_result_->result = %s", move_result_->result ? "true" : "false");
                        feedback_.sequence = Task_Sequence::REACHED_TARGET_POS;
                    }
                    else
                    {
                        feedback_.sequence = Task_Sequence::MOVE_TARGET_FAIL;
                    }

                    if (moveCompleted)
                    {
                        // if (movebase_status == actionlib_msgs::GoalStatus::SUCCEEDED)
                        // {
                        //     feedback_.sequence = Task_Sequence::REACHED_TARGET_POS;
                        //     // ROS_INFO("[Matrix_XbeeMovementServer]:Robot reached target position");
                        // }
                        if (/* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::ABORTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::PENDING || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::REJECTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::LOST)
                        {
                            feedback_.sequence = Task_Sequence::MOVE_TARGET_FAIL;
                        }
                    }
                }
                break;

            case Task_Sequence::REACHED_TARGET_POS:
                ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::REACHED_TARGET_POS");
                result_.result = XbeeCommunicationResult::REACHED_TARGET_POS;
                result_.text = XbeeCommunicationResult::REACHED_TARGET_POS_s;
                feedback_.sequence = Task_Sequence::SendRespond;
                break;

            case Task_Sequence::MOVE_TARGET_FAIL:
                ROS_ERROR("[Matrix_XbeeMovementServer]:Task_Sequence::MOVE_TARGET_FAIL");
                if (movefail_counter < movefail_repeats)
                {
                    ROS_ERROR("[Matrix_XbeeMovementServer]:current movefial_counter: %d", movefail_counter);
                    ROS_WARN("[Matrix_XbeeMovementServer]:clear costmap func");
                    clear_costmaps_func();
                    movefail_counter++;
                    setTimeOut = ros::Time::now().toSec() + wait_clear_obstacle;
                    feedback_.sequence = Task_Sequence::WaitClearObstacle;
                }
                else
                {
                    ROS_ERROR("[Matrix_XbeeMovementServer]:cofirm move traget fail!!");
                    result_.result = XbeeCommunicationResult::MOVE_TRAGET_FAIL;
                    result_.text = XbeeCommunicationResult::MOVE_TRAGET_FAIL_s;
                    feedback_.sequence = Task_Sequence::SendRespond;
                }
                break;

            case Task_Sequence::WaitClearObstacle:
                ROS_WARN("[Matrix_XbeeMovementServer]:wait obstacle clear %d", setTimeOut - ros::Time::now().toSec());
                if (ros::Time::now().toSec() > setTimeOut)
                {
                    clear_costmaps_func();
                    // if(period_state == Task_Sequence::PAUSE)
                    // {
                    //     feedback_.sequence = Task_Sequence::Init;
                    //     period_state = 0;
                    // }
                    // else
                    // {
                    feedback_.sequence = Task_Sequence::CmdSelector;
                    // }
                }
                break;

            case Task_Sequence::SendRespond:
            {
                ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::SendRespond");
                std::string action_res;
                action_res = result_.text + "," + result_.info.poi_name;
                response_serial(action_res, "move");
                success = (result_.result == XbeeCommunicationResult::REACHED_TARGET_POS) ? true : false;
                feedback_.sequence = Task_Sequence::Fisnish;
                break;
            }
            case Task_Sequence::Fisnish:
                ROS_INFO("[Matrix_XbeeMovementServer]:Task_Sequence::Fisnish");

                task_completed = true;
                step_complete();
                feedback_.sequence = Task_Sequence::Init;
                break;

            //**********************************************************//
            //                                                          //
            //                      Pause Control state                 //
            //                                                          //
            //**********************************************************//
            case Task_Sequence::PAUSE:
                ROS_WARN("[Matrix_docking_operation]: Robot state in Pausing mode");
                if (current_robotmode != matrix_msgs::RobotMode::PAUSE)
                {
                    period_state = feedback_.sequence;
                    setTimeOut = ros::Time::now().toSec() + 5;
                    feedback_.sequence = Task_Sequence::WaitClearObstacle;
                    // feedback_.sequence = Task_Sequence::Init;
                    // setTimeOut = ros::Time::now().toSec() + period_timeout_coutdown;
                }
                break;
                //***********************************************************//
            }

            //**********************************************************//
            //                                                          //
            //                      check pause state                   //
            //                                                          //
            //**********************************************************//
            if ((current_robotmode == matrix_msgs::RobotMode::PAUSE) && (feedback_.sequence != Task_Sequence::PAUSE))
            {
                // store period control state
                // period_state = feedback_.sequence;
                // period_text = feedback_.text;
                // set new control sate to PAUSE
                feedback_.sequence = Task_Sequence::PAUSE;
                pause_flag = true;
                cancelMoveAction();
                // get countdown time out
                //  period_timeout_coutdown = setTimeOut - ros::Time::now().toSec();

                feedback_.text = period_text + " [Pausing_mode]";
            }

            as_.publishFeedback(feedback_);
            rate.sleep();
        }
        // result_.info = feedback_.info;
        as_.setSucceeded(result_);
        // result_.sequence = feedback_.current_sequence;
        ROS_INFO("[Matrix_XbeeMovementServer]:%s: %s", action_name_.c_str(), (success) ? "Succeeded" : "Fail");
        ROS_INFO(" ");
        ROS_INFO("*************************");
    }

    bool MoveBase_parameters_cfg(std::string planner)
    {
        if(serial_no.find("SMR0100")  != -1)
        {
            dynamic_reconfigure::Reconfigure cfg_params;
            dynamic_reconfigure::BoolParameter bools_;
            dynamic_reconfigure::IntParameter ints_;
            dynamic_reconfigure::StrParameter strs_;
            dynamic_reconfigure::DoubleParameter doubles_;
            dynamic_reconfigure::GroupState gropts_;
            cfg_params.request.config.bools.push_back(bools_);
            cfg_params.request.config.ints.push_back(ints_);
            cfg_params.request.config.strs.push_back(strs_);
            cfg_params.request.config.groups.push_back(gropts_);

            strs_.name = "base_local_planner";
            strs_.value = (planner == "TEB") ? "teb_local_planner/TebLocalPlannerROS" : "dwa_local_planner/DWAPlannerROS";
            ROS_INFO("[Matrix_XbeeMovementServer]: change planner to %s", strs_.value.c_str());
            cfg_params.request.config.strs.push_back(strs_);

            if (movebase_cfg.call(cfg_params))
            {
                ;
            }
        }
        
    }

    bool DWA_parameters_cfg(double backward, double sim_time)
    {

        dynamic_reconfigure::Reconfigure cfg_params;
        dynamic_reconfigure::BoolParameter bools_;
        dynamic_reconfigure::IntParameter ints_;
        dynamic_reconfigure::StrParameter strs_;
        dynamic_reconfigure::DoubleParameter doubles_backward;
        dynamic_reconfigure::DoubleParameter simtime;
        dynamic_reconfigure::GroupState gropts_;

        cfg_params.request.config.bools.push_back(bools_);
        cfg_params.request.config.ints.push_back(ints_);
        cfg_params.request.config.strs.push_back(strs_);
        cfg_params.request.config.groups.push_back(gropts_);

        doubles_backward.name = "min_vel_x";
        doubles_backward.value = backward;
        cfg_params.request.config.doubles.push_back(doubles_backward);

        simtime.name = "sim_time";
        simtime.value = sim_time;
        cfg_params.request.config.doubles.push_back(simtime);

        if (DWA_cfg.call(cfg_params))
        {
            ;
        }
    }

    bool costmap_3d_enabled(bool enabled)
    {
        dynamic_reconfigure::Reconfigure cfg_params;
        dynamic_reconfigure::BoolParameter bool_enable_3d;
        dynamic_reconfigure::IntParameter empty_ints;
        dynamic_reconfigure::StrParameter empty_strs;
        dynamic_reconfigure::DoubleParameter empty_doubles;
        dynamic_reconfigure::GroupState empty_groups;

        bool_enable_3d.name = "enabled";
        bool_enable_3d.value = enabled;
        cfg_params.request.config.bools.push_back(bool_enable_3d);
        cfg_params.request.config.ints.push_back(empty_ints);
        cfg_params.request.config.strs.push_back(empty_strs);
        cfg_params.request.config.doubles.push_back(empty_doubles);
        cfg_params.request.config.groups.push_back(empty_groups);

        if (costmap_global_cfg.call(cfg_params))
        {
            ;
        }
    }

    bool clear_costmaps_func(void)
    {
        bool result;
        std_srvs::Empty empty_;
        result = (clearcostmap_srv_.call(empty_)) ? true : false;
        return result;
    }

    void response_serial(matrix_msgs::XbeeCommuROHM data_info_, std::string type)
    {
        std_msgs::String ss;
        if (type == "goal_accept")
        {
            ss.data = "#581{" + data_info_.cmd_string + "," + data_info_.poi_name + "}$";
        }
        else if (type == "move_pending")
        {
            ss.data = "#581{PENDING," + data_info_.cmd_string + "," + data_info_.poi_name + "}$";
        }
        else if (type == "retry")
        {
            ss.data = "#581{RETRY," + data_info_.cmd_string + "," + data_info_.poi_name + "}$";
        }
        pub_remote_xbee.publish(ss);
    }

    void response_serial(std::string str, std::string type)
    {
        std_msgs::String ss;
        if (type == "cancel_accept")
        {
            ss.data = "#581{" + command_cancel + "}$";
        }
        else if (type == "cancel_success")
        {
            ss.data = "#581{" + command_cancel + "<SC>" + "}$";
        }
        else if (type == "move")
        {
            ss.data = "#581{" + str + "}$";
        }
        else
        {
            ss.data = "#581{" + str + "}$";
        }
        pub_remote_xbee.publish(ss);
    }

    void cancelMoveAction()
    {
        ist_roads_ac.cancelAllGoals();
        ist_route_ac.cancelAllGoals();
        move_base_ac.cancelAllGoals();
    }

    void step_complete()
    {
        // DWA_parameters_cfg(-0.0, 1.0);
        MoveBase_parameters_cfg("TEB");
        costmap_3d_enabled(false);
        moveCompleted = false;
        IstRoads_MovingCompleted = false;
        IstRoute_MovingCompleted = false;
        movefail_counter = 0;
        move_base = false;
    }

    void cbSerialRead(std_msgs::String::ConstPtr msg)
    {
        // if((condition1 != "") || (condition2 != ""))
        // {
        //     if((msg->data == condition1) || (msg->data == condition2))
        //     {
        //         isConditionMatch = true;
        //     }
        // }
    }

    void cbRobotMode(const matrix_msgs::RobotMode::ConstPtr &msg)
    {
        current_robotmode = msg->robot_mode;
    }

    void movebaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
    {
        movebase_status = msg->status.status;
        ROS_INFO("movebase Result(%d):%s", movebase_status, msg->status.text.c_str());
        // if(move_base)
        // {
        //     if(movebase_status != 2)
        //     {
        //         moveCompleted = true;
        //     }
        //     else
        //     {
        //         ROS_WARN("movebase Result(%d): not trigger", movebase_status, msg->status.text.c_str());

        //     }
        // }
        // else
        // {
        moveCompleted = true;
        // }
    }

    // for convert String 8 byte (Hex 4 byte 0x00, 0x00, 0x00, 0x00)
    float HexString_float(std::string value, float divider)
    {
        unsigned int i;
        std::istringstream iss(value);
        iss >> std::hex >> i;
        auto j = static_cast<int>(i);
        float b = j;
        b = b / divider;
        // std::cout << b << std::endl; // 1000
        return b;
    }

    // get string between 2 delimer
    std::string split_string(std::string string_cmd, std::string begin_pos, std::string end_pos)
    {
        size_t pos = 0;
        std::string token;
        while ((pos = string_cmd.find(begin_pos)) != std::string::npos)
        {
            string_cmd.erase(0, pos + begin_pos.length());
            string_cmd.erase(string_cmd.find(end_pos));
        }
        // std::cout << string_cmd << std::endl;
        return string_cmd.c_str();
    }

    int DecString_int(std::string value)
    {
        int int_Ticket = 0;
        std::stringstream ssTicket(value);
        ssTicket >> int_Ticket;

        return int_Ticket;
    }

    matrix_msgs::XbeeCommuROHM getCmdInfo(std::string string_receive)
    {
        // xbee_receive_.data = "#0x601{ROADSgoPOI15}$";

        matrix_msgs::XbeeCommuROHM info_;
        info_.robot_no = HexString_float(split_string(string_receive, serialno_split_begin, serialno_split_end), 1) - 0x600;
        info_.cmd_string = split_string(string_receive, movementmode_split_begin, movementmode_split_end);
        info_.poi_name = split_string(string_receive, poiname_split_begin, poiname_split_end);
        return info_;
    }

    // bool Check_RobotCondition(matrix_msgs::XbeeCommu info_)
    // {
    //     unsigned int robot_no_ = HexString_float(split_string(string_receive, "$R", "CMD"), 1);
    // }

    void istRoadsDoneCb(const actionlib::SimpleClientGoalState &state, const ist_roads_srv::GoRoadResultConstPtr &result)
    {
        ROS_INFO("[Matrix_XbeeMovementServer]:ist_roads DONECB: Finished in state [%s]", state.toString().c_str());
        IstRoads_MovingCompleted = true;
    }

    void istRouteDoneCb(const actionlib::SimpleClientGoalState &state, const web_interface_msgs::RouteFollowResultConstPtr &result)
    {
        ROS_INFO("[Matrix_XbeeMovementServer]:ist_route DONECB: Finished in state [%s]", state.toString().c_str());
        IstRoute_MovingCompleted = true;
        // step_complete();
    }

    void task_manager()
    {

        // xbee_receive_.data = "#0x601{ROADSgoPOI15}$";

        // ROS_INFO("%s\n", xbee_receive_.data.c_str());
    }

    bool checkPOI(std::string poi_name)
    {
        web_interface_msgs::Layout layouts_srv;
        layouts_srv.request.cmd = "type";
        web_interface_msgs::UILayout layouts_;
        layouts_.type = 28;
        layouts_srv.request.layouts.push_back(layouts_);

        if (layouts_srv_.call(layouts_srv))
        {

            for (double i = 0; i < layouts_srv.response.layouts.size(); i++)
            {
                std::string POI_no_ = layouts_srv.response.layouts[i].text;
                ROS_INFO("[Matrix_XbeeMovementServer]:find poi name : %s", POI_no_.c_str());
                if (POI_no_ == poi_name)
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s match with %s UILayout!!!", poi_name.c_str(), POI_no_.c_str());
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s ", layouts_srv.response.layouts[i].uuid.c_str());
                    return true;
                    break;
                }
                else
                {
                    if ((layouts_srv.response.layouts.size() - i) <= 1)
                    {
                        return false;
                        break;
                    }
                }
            }
        }
        else
        {
            ROS_ERROR("[Matrix_XbeeMovementServer]:call srv Failll!!!");
        }
    }

    std::string getPOIActionForm(int poi_no)
    {
        std::string string_form = "POI";
        std::stringstream ss;
        ss << poi_no;
        std::string s;
        ss >> s;
        return string_form + s;
    }

    std::string getUUID(std::string poi)
    {
        web_interface_msgs::Layout layouts_srv;
        layouts_srv.request.cmd = "type";
        web_interface_msgs::UILayout layouts_;
        layouts_.type = 28;
        layouts_srv.request.layouts.push_back(layouts_);

        if (layouts_srv_.call(layouts_srv))
        {

            for (double i = 0; i < layouts_srv.response.layouts.size(); i++)
            {
                std::string POI_no_ = layouts_srv.response.layouts[i].text;
                ROS_INFO("[Matrix_XbeeMovementServer]:find poi name : %s", POI_no_.c_str());
                if (POI_no_ == poi)
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s match with %s UILayout!!!", poi.c_str(), POI_no_.c_str());
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s ", layouts_srv.response.layouts[i].uuid.c_str());
                    return layouts_srv.response.layouts[i].uuid;
                    break;
                }
                else
                {
                    if ((layouts_srv.response.layouts.size() - i) <= 1)
                    {
                        return "";
                        break;
                    }
                }
            }
        }
        else
        {
            ROS_ERROR("[Matrix_XbeeMovementServer]:call srv Failll!!!");
        }
    }

    move_base_msgs::MoveBaseGoal getPoseFromLayouts(std::string poi)
    {
        ROS_INFO("[Matrix_XbeeMovementServer]:get pose from poi no: %s", poi.c_str());
        move_base_msgs::MoveBaseGoal move_info_;

        web_interface_msgs::Layout layouts_srv;
        layouts_srv.request.cmd = "type";

        web_interface_msgs::UILayout layouts_;
        layouts_.type = 28;
        layouts_srv.request.layouts.push_back(layouts_);
        if (layouts_srv_.call(layouts_srv))
        {

            for (double i = 0; i < layouts_srv.response.layouts.size(); i++)
            {
                std::string POI_no_ = layouts_srv.response.layouts[i].text;
                ROS_INFO("[Matrix_XbeeMovementServer]:find poi name : %s", POI_no_.c_str());
                if (POI_no_ == poi)
                {
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s match with %s UILayout!!!", poi.c_str(), POI_no_.c_str());
                    ROS_INFO("[Matrix_XbeeMovementServer]:%s ", layouts_srv.response.layouts[i].uuid.c_str());
                    move_info_.target_pose.pose.position = layouts_srv.response.layouts[i].pos;
                    move_info_.target_pose.pose.orientation = getQuat(layouts_srv.response.layouts[i].pos);
                    break;
                }
            }
        }
        else
        {
            ROS_ERROR("[Matrix_XbeeMovementServer]:call srv Failll!!!");
        }
        return move_info_;
    }

    geometry_msgs::Quaternion getQuat(geometry_msgs::Point traget_pos_)
    {

        geometry_msgs::Quaternion orientation_;

        double last_x = 0;
        double last_y = 0;
        int last_number = 0;
        tf::StampedTransform transform;

        // create the listener
        tf::TransformListener listener;
        listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(2.0));

        // ROS_INFO("[Matrix_XbeeMovementServer]:Get current robot position");
        try
        {
            listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
            // pose_stamped.pose.orientation.x = transform.getRotation().getX();
            // pose_stamped.pose.orientation.y = transform.getRotation().getY();
            // pose_stamped.pose.orientation.z = transform.getRotation().getZ();
            // pose_stamped.pose.orientation.w = transform.getRotation().getW();
            // pose_stamped.pose.position.x = transform.getOrigin().getX();
            // pose_stamped.pose.position.y = transform.getOrigin().getY();
            // pose_stamped.pose.position.z = transform.getOrigin().getZ();
            last_x = transform.getOrigin().getX();
            last_y = transform.getOrigin().getY();
        }
        catch (tf::TransformException &ex)
        {
            // just continue on
        }

        if (last_x != traget_pos_.x && last_y != traget_pos_.y)
        {
            double dx = traget_pos_.x - last_x;
            double dy = traget_pos_.y - last_y;

            double ang = atan2(dy, dx);
            double qz = sin(ang / 2.0);
            double qw = cos(ang / 2.0);
            orientation_.x = 0.0;
            orientation_.y = 0.0;
            orientation_.z = qz;
            orientation_.w = qw;
        }

        last_x = traget_pos_.x;
        last_y = traget_pos_.y;

        return orientation_;
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_xbeemovement_server");
    ROS_INFO("[Matrix_XbeeMovementServer]:Start matrix_xbeemovement_server");

    XbeeMovementServer Xbee_Cmd_Server("matrix_xbeemovement_server");
    ros::spin();

    return 0;
}
