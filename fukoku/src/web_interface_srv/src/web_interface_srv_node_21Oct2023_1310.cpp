#include <iostream>
#include <list>
#include <iterator>
#include <string>
#include <map>
#include <vector>
#include <unistd.h>
#include <ostream>

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// #include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include "web_interface_msgs/WebCommandAction.h"
#include "web_interface_msgs/WebCommand.h"
#include "web_interface_msgs/Layout.h"
#include "web_interface_msgs/UILayout.h"
#include "web_interface_msgs/GuiMap.h"
#include "web_interface_msgs/UIMap.h"



typedef actionlib::SimpleActionClient<web_interface_msgs::WebCommandAction> WebLaunchClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


enum StateEnum
{
  ACCEPT_COMMAND = 0,
  STOP_PROCESS,
  STOP_PROCESS1,
  STOP_PROCESS2,
  START_PROCESS,
  INIT_PROCESS,
  GET_MAP_PROCESS,
  SET_MAP_PROCESS,
  FINISH_PROCESS
};

enum DockingState
{
  INIT = 0,
  WAIT_FOR_ROBOT_POSE,
  START_SYNC,
  GET_DOCKING_GUI,
  NOTFOUND_DOCKING_GUI,
  DOCKING_GUI_POSE,
  GOTO_PRE_DOCKING,
  CHECK_PRE_DOCKING_REACHED,
  REACHED_PRE_DOCKING,

  GOTO_DOCKING,
  CHECK_DOCKING_REACHED,
  REACHED_DOCKING_STATION,
  CALL_CHARGING_PROCESS,
  WAIT_CHARGING_PROCESS,
  CHARGING_PROCESS_COMPLETED
};

class WebAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<web_interface_msgs::WebCommandAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  web_interface_msgs::WebCommandFeedback feedback_;
  web_interface_msgs::WebCommandResult result_;

  WebLaunchClient launchClient;
  bool launchCompleted;
  std::string map_frame, base_frame; //,"/map","/base_link"
  // tf::TransformListener listener;

  ros::Subscriber robotPos_sub;
  geometry_msgs::PoseStamped pose_stamped;
  bool robot_pose_ready = false;

  MoveBaseClient ac;
  bool moveCompleted;

  bool autoDockingCompleted;
  std::string docking_sequence;
  int current_sequence = -1;

public:
  WebAction(std::string name) : as_(nh_, name, boost::bind(&WebAction::executeCB, this, _1), false),
                                action_name_(name),
                                map_frame("/map"),
                                base_frame("/base_link"),
                                launchClient("web_launch_server", true) //tell the action client that we want to spin a thread by default
                                ,
                                ac("move_base", true)

  {
    as_.start();
    ROS_INFO("web_interface_server is ready");

    // ROS_INFO("Waiting for robot transform");
    // listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(2.0));

    robotPos_sub = nh_.subscribe("robot_pose", 1, &WebAction::robotPoseCallback, this);

    //wait for the action server to come up
    while (!launchClient.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("WebAction Waiting for the web_launch_server action server to come up");
    }
  }

private:
  // bool getRobotTranform()
  // {
  // tf::StampedTransform transform;
  // try
  // {
  //   listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

  //   // construct a pose message

  //   pose_stamped.header.frame_id = map_frame;
  //   pose_stamped.header.stamp = ros::Time::now();

  //   pose_stamped.pose.orientation.x = transform.getRotation().getX();
  //   pose_stamped.pose.orientation.y = transform.getRotation().getY();
  //   pose_stamped.pose.orientation.z = transform.getRotation().getZ();
  //   pose_stamped.pose.orientation.w = transform.getRotation().getW();

  //   pose_stamped.pose.position.x = transform.getOrigin().getX();
  //   pose_stamped.pose.position.y = transform.getOrigin().getY();
  //   pose_stamped.pose.position.z = transform.getOrigin().getZ();

  //   return true;
  // }
  // catch (tf::TransformException &ex)
  // {
  //   // just continue on
  // }

  //return false;
  //   return true;
  // }
  // void RestoreRobotTF()
  // {
  //     ros::NodeHandle nh_;
  //     ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);

  //     std::string fixed_frame = "map";
  //     geometry_msgs::PoseWithCovarianceStamped pose;
  //     pose.header.frame_id = fixed_frame;
  //     pose.header.stamp = ros::Time::now();

  //     // set x,y coord
  //     pose.pose.pose.orientation.x = pose_stamped.pose.orientation.x;
  //     pose.pose.pose.orientation.y = pose_stamped.pose.orientation.y;
  //     pose.pose.pose.orientation.z = pose_stamped.pose.orientation.z;
  //     pose.pose.pose.orientation.w = pose_stamped.pose.orientation.w;

  //     pose.pose.pose.position.x = pose_stamped.pose.position.x;
  //     pose.pose.pose.position.y = pose_stamped.pose.position.y;
  //     pose.pose.pose.position.z = pose_stamped.pose.position.z;

  //     // // set theta
  //     // tf::Quaternion quat;
  //     // quat.setRPY(0.0, 0.0, theta);
  //     // tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
  //     pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  //     pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  //     pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

  //     // publish
  //     // ROS_INFO("x: %f, y: %f, z: 0.0, theta: %f",x,y,theta);
  //     pub_.publish(pose);
  // }

  void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
  {

    pose_stamped.header.frame_id = map_frame;
    pose_stamped.header.stamp = ros::Time::now();

    pose_stamped.pose.orientation.x = msg->orientation.x;
    pose_stamped.pose.orientation.y = msg->orientation.y;
    pose_stamped.pose.orientation.z = msg->orientation.z;
    pose_stamped.pose.orientation.w = msg->orientation.w;

    pose_stamped.pose.position.x = msg->position.x;
    pose_stamped.pose.position.y = msg->position.y;
    pose_stamped.pose.position.z = msg->position.z;

    // ROS_INFO("Position: x: %f, y: %f, z: %f", msg->position.x, msg->position.y, msg->position.z);
    // ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    robot_pose_ready = true;
  }






  void moveDoneCb(const actionlib::SimpleClientGoalState &state)
  {
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    moveCompleted = true;
  }

  void sendResult(bool success)
  {
    result_.command = feedback_.command;
    result_.sequence = feedback_.sequence;
    ROS_INFO("%s: %s", action_name_.c_str(), (success) ? "Succeeded" : "Fail");
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }

  void executeCB(const web_interface_msgs::WebCommandGoalConstPtr &goal)
  {
    ROS_INFO("web_interface_server recieved:%s", goal->command.c_str());

    // helper variables
    ros::Rate r(1);
    bool success = true;
    int i = 0;
    feedback_.command = goal->command;
    feedback_.sequence.clear();
    for (std::string arg : goal->args)
    {
      ROS_INFO("Arg:%s", arg.c_str());
    }

    if (std::strcmp(goal->command.c_str(), "newmap") == 0)
    {
      feedback_.sequence.push_back(goal->command);
      as_.publishFeedback(feedback_);
      success = newMap(goal->args);
    }

    if (std::strcmp(goal->command.c_str(), "loadmap") == 0)
    {
      feedback_.sequence.push_back(goal->command);
      as_.publishFeedback(feedback_);
      success = loadMap(goal->args);
    }

    if (std::strcmp(goal->command.c_str(), "savemap") == 0)
    {
      feedback_.sequence.push_back(goal->command);
      as_.publishFeedback(feedback_);
      success = saveMap(goal->args);
    }

    // if (std::strcmp(goal->command.c_str(), "docking") == 0)
    // {
    //   feedback_.sequence.push_back(goal->command);
    //   as_.publishFeedback(feedback_);
    //   success = Docking(goal->args);
    // }

    if (std::strcmp(goal->command.c_str(), "guilayout") == 0)
    {
      feedback_.sequence.push_back(goal->command);
      as_.publishFeedback(feedback_);
      success = loadGuiLayout(goal->args);
    }

    if (std::strcmp(goal->command.c_str(), "defaultmap") == 0)
    {
      feedback_.sequence.push_back(goal->command);
      as_.publishFeedback(feedback_);
      success = defaultMap(goal->args);
    }

    sendResult(success);
  }

  bool loadGuiLayout(std::vector<std::string> args)
  {
    ROS_INFO("Refresh GUI Layout");
    if (args.size() > 0)
    {
      ROS_INFO("args is:%s", args[0].c_str());
    }

    ros::Rate r(10);
    StateEnum state = StateEnum::ACCEPT_COMMAND;
    web_interface_msgs::WebCommandGoal goal;

    feedback_.sequence.push_back("GUI Layout ACCEPT_COMMAND");
    as_.publishFeedback(feedback_);
    goal.command = "run";
    goal.args.clear();
    goal.args.push_back("ist_gui_map_server");

    launchCompleted = false;
    launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());
    while (!launchCompleted)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        return false;
      }
      r.sleep();
    }

    return true;
  }

  ///move_base/global_costmap/costmap_prohibition_layer/prohibition_areas

  void launchDoneCb(const actionlib::SimpleClientGoalState &state)
  {
    ROS_INFO("LAUNCH CB: Finished in state [%s]", state.toString().c_str());
    launchCompleted = true;
  }

  // bool Docking(std::vector<std::string> args)
  // {

  //   ROS_INFO("Docking");
  //   if (args.size() > 0)
  //   {
  //     ROS_INFO("args is:%s", args[0].c_str());
  //   }

  //   ros::Rate r(10);

  //   move_base_msgs::MoveBaseGoal goalPreDock;
  //   move_base_msgs::MoveBaseGoal goalDocking;

  //   ir_auto_docking::AutodockOperateGoal dockingGoal;
  //   // actionlib::ActionClient::GoalHandle moveHandle;
  //   // actionlib::ActionClient::GoalHandledockingHandle;
  //   DockingState state = DockingState::INIT;
  //   feedback_.sequence.push_back("INIT");
  //   bool finished = false;
  //   while (!finished)
  //   {
  //     switch (state)
  //     {
  //     case DockingState::INIT:

  //       state = DockingState::WAIT_FOR_ROBOT_POSE;
  //       feedback_.sequence.push_back("WAIT_FOR_ROBOT_POSE");
  //       break;
  //     case DockingState::WAIT_FOR_ROBOT_POSE:
  //       if (robot_pose_ready)
  //       {

  //         state = DockingState::START_SYNC;
  //       }

  //       break;
  //     case DockingState::START_SYNC:
  //       ROS_INFO("GET_DOCKING_GUI");
  //       state = DockingState::GET_DOCKING_GUI;
  //       feedback_.sequence.push_back("START_SYNC");
  //       break;
  //     case DockingState::GET_DOCKING_GUI:
  //     {
  //       feedback_.sequence.push_back("GET_DOCKING_GUI");
  //       ros::ServiceClient client = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
  //       web_interface_msgs::Layout srv;
  //       web_interface_msgs::UILayout layout_docking;
  //       srv.request.cmd = "type";
  //       layout_docking.type = 6; //Docking: 6,
  //       srv.request.layouts.push_back(layout_docking);

  //       if (client.call(srv))
  //       {
  //         ROS_INFO("Layout result:%s", srv.response.result.c_str());
  //         if (srv.response.layouts.size() > 0)
  //         {
  //           auto &mp = srv.response.layouts[0]; //now support only 1 docking station
  //           ROS_INFO("Layout: uuid=%s, type=%d", mp.uuid.c_str(), (int)mp.type);
  //           ROS_INFO("startHandle=%d, rotateHandle=%d", mp.startHandle, mp.rotateHandle);

  //           int i = 0;
  //           for (const geometry_msgs::Point &p : mp.points)
  //           {
  //             ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
  //           }

  //           //back side
  //           const geometry_msgs::Point &target = mp.points[2];
  //           const geometry_msgs::Point &start = mp.points[6];
  //           const geometry_msgs::Point &dockingPose = mp.points[0];

  //           double dx = target.x - start.x;
  //           double dy = target.y - start.y;
  //           double ang = atan2(dy, dx);
  //           double qz = sin(ang / 2.0);
  //           double qw = cos(ang / 2.0);
  //           goalPreDock.target_pose.pose.position.x = target.x;
  //           goalPreDock.target_pose.pose.position.y = target.y;
  //           goalPreDock.target_pose.pose.position.z = target.z;

  //           goalPreDock.target_pose.pose.orientation.x = 0;
  //           goalPreDock.target_pose.pose.orientation.y = 0;
  //           goalPreDock.target_pose.pose.orientation.z = qz;
  //           goalPreDock.target_pose.pose.orientation.w = qw;

  //           goalDocking.target_pose.pose.position.x = dockingPose.x;
  //           goalDocking.target_pose.pose.position.y = dockingPose.y;
  //           goalDocking.target_pose.pose.position.z = dockingPose.z;

  //           goalDocking.target_pose.pose.orientation.x = 0;
  //           goalDocking.target_pose.pose.orientation.y = 0;
  //           goalDocking.target_pose.pose.orientation.z = qz;
  //           goalDocking.target_pose.pose.orientation.w = qw;

  //           state = DockingState::DOCKING_GUI_POSE;
  //         }
  //         else
  //         {
  //           state = DockingState::NOTFOUND_DOCKING_GUI;
  //         }
  //       }
  //       else
  //       {
  //         state = DockingState::NOTFOUND_DOCKING_GUI;
  //       }
  //     }

  //     break;
  //     case DockingState::NOTFOUND_DOCKING_GUI:
  //       feedback_.sequence.push_back("NOTFOUND_DOCKING_GUI");
  //       ROS_ERROR("Not found Docking GUI");
  //       return false;
  //       break;
  //     case DockingState::DOCKING_GUI_POSE:
  //       feedback_.sequence.push_back("DOCKING_GUI_POSE");
  //       state = DockingState::GOTO_PRE_DOCKING;
  //       break;

  //     case DockingState::GOTO_PRE_DOCKING:
  //     {
  //       feedback_.sequence.push_back("GOTO_PRE_DOCKING");
  //       goalPreDock.target_pose.header.frame_id = "map";
  //       goalPreDock.target_pose.header.stamp = ros::Time::now();

  //       moveCompleted = false;
  //       ac.sendGoal(goalPreDock, boost::bind(&WebAction::moveDoneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
  //       state = DockingState::CHECK_PRE_DOCKING_REACHED;
  //     }
  //     break;
  //     case DockingState::CHECK_PRE_DOCKING_REACHED:
  //       if (moveCompleted)
  //       {
  //         feedback_.sequence.push_back("CHECK_PRE_DOCKING_REACHED");

  //         if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //         {
  //           state = DockingState::REACHED_PRE_DOCKING;
  //         }
  //         if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
  //         {
  //           feedback_.sequence.push_back("ABORTED");
  //           return false;
  //         }
  //       }
  //       break;
  //     case DockingState::REACHED_PRE_DOCKING:
  //       feedback_.sequence.push_back("REACHED_PRE_DOCKING");
  //       state = DockingState::GOTO_DOCKING;

  //       break;

  //     case DockingState::GOTO_DOCKING:
  //     {
  //       feedback_.sequence.push_back("GOTO_DOCKING");

  //       goalDocking.target_pose.header.frame_id = "map";
  //       goalDocking.target_pose.header.stamp = ros::Time::now();

  //       moveCompleted = false;
  //       ac.sendGoal(goalDocking, boost::bind(&WebAction::moveDoneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
  //       state = DockingState::CHECK_DOCKING_REACHED;
  //     }
  //     break;
  //     case DockingState::CHECK_DOCKING_REACHED:
  //       if (moveCompleted)
  //       {
  //         feedback_.sequence.push_back("CHECK_DOCKING_REACHED");

  //         if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //         {
  //           state = DockingState::REACHED_DOCKING_STATION;
  //         }
  //         if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
  //         {
  //           feedback_.sequence.push_back("ABORTED");
  //           return false;
  //         }
  //       }
  //       break;
  //     case DockingState::REACHED_DOCKING_STATION:
  //       feedback_.sequence.push_back("CHECK_DOCKING_REACHED");
  //       if (args.size() > 0)
  //       {
  //         if (args[0] == "IST")
  //         {
  //           state = DockingState::CALL_CHARGING_PROCESS;
  //         }
  //       }
  //       else
  //       {
  //         return true;
  //       }

  //       break;
  //     case DockingState::CALL_CHARGING_PROCESS:
  //       feedback_.sequence.push_back("CALL_CHARGING_PROCESS");
  //       ROS_INFO("CALL_CHARGING_PROCESS");
  //       dockingGoal.order = 1;
  //       current_sequence = -1;
  //       autoDockingCompleted = false;
  //       docking.sendGoal(dockingGoal, boost::bind(&WebAction::autoDockDoneCb, this, _1, _2), boost::bind(&WebAction::autoDockActive, this), boost::bind(&WebAction::autoDockFeedBack, this, _1));
  //       feedback_.sequence.push_back("WAIT_CHARGING_PROCESS");

  //       state = DockingState::WAIT_CHARGING_PROCESS;
  //       break;
  //     case DockingState::WAIT_CHARGING_PROCESS:

  //       if (autoDockingCompleted)
  //       {

  //         if (docking.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //         {
  //           state = DockingState::CHARGING_PROCESS_COMPLETED;
  //         }

  //         if (docking.getState() == actionlib::SimpleClientGoalState::ABORTED)
  //         {
  //           feedback_.sequence.push_back("AUTO DOCKING ABORTED");
  //           ROS_ERROR("AUTO DOCKING ABORTED");
  //           return false;
  //         }
  //       }

  //       break;
  //     case DockingState::CHARGING_PROCESS_COMPLETED:
  //       feedback_.sequence.push_back("CHARGING_PROCESS_COMPLETED");
  //       ROS_INFO("CHARGING_PROCESS_COMPLETED");
  //       return true;
  //     }

  //     // check that preempt has not been requested by the client
  //     if (as_.isPreemptRequested() || !ros::ok())
  //     {
  //       ROS_INFO("%s: Preempted", action_name_.c_str());
  //       // set the action state to preempted
  //       as_.setPreempted();

  //       if (!autoDockingCompleted)
  //       {
  //         docking.cancelAllGoals();
  //       }

  //       if (!moveCompleted)
  //       {
  //         ac.cancelAllGoals();
  //       }

  //       return false;
  //     }
  //     // publish the feedback
  //     as_.publishFeedback(feedback_);

  //     r.sleep();
  //   }

  //   return true;
  // }

  bool defaultMap(std::vector<std::string> args)
  {
    ROS_INFO("defaultMap");
    if (args.size() > 0)
    {
      ROS_INFO("file name:%s", args[0].c_str());
    }

    return true;
  }

  bool saveMap(std::vector<std::string> args)
  {
    ROS_INFO("saveMap");
    if (args.size() <= 0)
      return false;

    ROS_INFO("file name:%s", args[0].c_str());
    ros::Rate r(10);
    StateEnum state = StateEnum::ACCEPT_COMMAND;
    web_interface_msgs::WebCommandGoal goal;

    bool finished = false;
    while (!finished)
    {
      switch (state)
      {
      case StateEnum::ACCEPT_COMMAND:
        ROS_INFO("ACCEPT_COMMAND");
        feedback_.sequence.push_back("saveMap ACCEPT_COMMAND");
        as_.publishFeedback(feedback_);
        goal.command = "run";
        goal.args.clear();
        goal.args.push_back("ist_map_saver");
        for (std::string arg : args)
        {
          goal.args.push_back(arg);
        }
        // goal.args.push_back("map:=" + args[0]);
        // if (args.size() > 1)
        //   goal.args.push_back("desc:=" + args[1]);

        //goal.args.push_back(args[0]);
        launchCompleted = false;
        launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());

        state = StateEnum::SET_MAP_PROCESS;
        break;
      case StateEnum::SET_MAP_PROCESS:
        if (launchCompleted)
        {
          if (launchClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("saveMap SUCCEEDED");
            feedback_.sequence.push_back("saveMap SUCCEEDED");
            as_.publishFeedback(feedback_);

            state = StateEnum::FINISH_PROCESS;
          }
          if (launchClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
            return false;
          }
        }
        break;
      case StateEnum::FINISH_PROCESS:
        ROS_INFO("newMap FINISH_PROCESS");
        feedback_.sequence.push_back("saveMap FINISH_PROCESS");
        as_.publishFeedback(feedback_);
        finished = true;
        break;
      }
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        return false;
      }
      r.sleep();
    }

    return true;
  }

  bool newMap(std::vector<std::string> args)
  {

    ROS_INFO("newMap");
    if (args.size() > 0)
    {
      ROS_INFO("file name:%s", args[0].c_str());
    }

    ros::Rate r(10);
    StateEnum state = StateEnum::ACCEPT_COMMAND;
    web_interface_msgs::WebCommandGoal goal;
    bool finished = false;

    bool ck_tr = robot_pose_ready;
    ROS_INFO("Backup Robot Tranform: %s", (ck_tr) ? "Succeeded" : "Fail");

    ros::ServiceClient ui_srv = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
    web_interface_msgs::Layout ui;
    ui.request.cmd = "clear";

    if (!ui_srv.call(ui))
    {
      //throw std::runtime_error("call ist_layouts_srv clear");
      ROS_ERROR("call ist_layouts_srv clear");
      return false;
    }

    ros::ServiceClient map = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    if (map.exists())
    {
      ros::ServiceClient movebase_srv = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/global_costmap/ist_costmap_prohibition_layer/set_parameters");

      dynamic_reconfigure::Reconfigure dy;
      dynamic_reconfigure::BoolParameter en;
      dynamic_reconfigure::BoolParameter polygon;
      en.name = "enabled";
      en.value = true;
      polygon.name = "dynamic_polygons";
      polygon.value = true;

      dy.request.config.bools.clear();
      dy.request.config.bools.push_back(en);
      dy.request.config.bools.push_back(polygon);
      if (!movebase_srv.call(dy))
      {
        //throw std::runtime_error("call ist_gui_map_server ist_costmap_prohibition_layer/set_parameters");
        ROS_ERROR("call ist_gui_map_server ist_costmap_prohibition_layer/set_parameters");
        return false;
      }
    }

    while (!finished)
    {
      switch (state)
      {
      case StateEnum::ACCEPT_COMMAND:
        ROS_INFO("ACCEPT_COMMAND");
        goal.command = "stop";
        goal.args.clear();
        goal.args.push_back("ist_map_server");

        launchCompleted = false;
        launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());
        feedback_.sequence.push_back("stop ist_map_server");
        as_.publishFeedback(feedback_);
        state = StateEnum::STOP_PROCESS;
        ROS_INFO("sendGoal stop map_server");
        break;
      case StateEnum::STOP_PROCESS:
        if (launchCompleted)
        {
          if (launchClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("stop map_server SUCCEEDED");
            feedback_.sequence.push_back("stop map_server SUCCEEDED");
            as_.publishFeedback(feedback_);

            goal.command = "start";
            goal.args.clear();
            goal.args.push_back("ist_slam");

            launchCompleted = false;
            launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());
            feedback_.sequence.push_back("start ist_slam");
            as_.publishFeedback(feedback_);

            state = StateEnum::START_PROCESS;
          }
          if (launchClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
            return false;
          }
        }
        break;
      case StateEnum::START_PROCESS:
        if (launchCompleted)
        {
          if (launchClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("start ist_slam SUCCEEDED");
            feedback_.sequence.push_back("start ist_slam SUCCEEDED");
            as_.publishFeedback(feedback_);

            state = StateEnum::FINISH_PROCESS;
          }
          if (launchClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
            return false;
          }
        }
        break;
      case StateEnum::FINISH_PROCESS:
        ROS_INFO("newMap FINISH_PROCESS");
        feedback_.sequence.push_back("newMap FINISH_PROCESS");
        as_.publishFeedback(feedback_);
        finished = true;
        break;
      }
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        return false;
      }
      r.sleep();
    }

    ROS_INFO("exit newMap");
    return true;
  }

  bool loadMap(std::vector<std::string> args)
  {

    ROS_INFO("loadMap");
    if (args.size() > 0)
    {
      ROS_INFO("file name:%s", args[0].c_str());
    }

    ros::NodeHandle n;
    ros::ServiceClient getMapclient = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap getMapSrv;
    ros::ServiceClient setMapclient = n.serviceClient<nav_msgs::SetMap>("set_map");
    nav_msgs::SetMap setMapSrv;

    ros::Rate r(10);
    StateEnum state = StateEnum::ACCEPT_COMMAND;
    web_interface_msgs::WebCommandGoal goal;
    bool finished = false;
    bool ck_tr = robot_pose_ready;
    ROS_INFO("Backup Robot Tranform: %s", (ck_tr) ? "Succeeded" : "Fail");

    while (!finished)
    {
      switch (state)
      {
      case StateEnum::ACCEPT_COMMAND:
        ROS_INFO("ACCEPT_COMMAND");
        goal.command = "stop";
        goal.args.clear();
        goal.args.push_back("ist_slam");

        launchCompleted = false;
        launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());
        feedback_.sequence.push_back("stop ist_slam");
        as_.publishFeedback(feedback_);
        state = StateEnum::STOP_PROCESS;
        ROS_INFO("sendGoal stop ist_slam");
        break;
      case StateEnum::STOP_PROCESS:
        if (launchCompleted)
        {
          if (launchClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("stop slam SUCCEEDED");
            feedback_.sequence.push_back("stop ist_slam SUCCEEDED");
            as_.publishFeedback(feedback_);

            goal.command = "stop";
            goal.args.clear();
            goal.args.push_back("ist_map_server");

            launchCompleted = false;
            launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());
            feedback_.sequence.push_back("stop ist_map_server");
            as_.publishFeedback(feedback_);
            ROS_INFO("stop ist_map_server");
            state = StateEnum::STOP_PROCESS1;
          }
          if (launchClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
            return false;
          }
        }
        break;
      case StateEnum::STOP_PROCESS1:
        if (launchCompleted)
        {
          if (launchClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("stop ist_map_server SUCCEEDED");
            feedback_.sequence.push_back("stop ist_slam SUCCEEDED");
            as_.publishFeedback(feedback_);

            goal.command = "start";
            goal.args.clear();
            goal.args.push_back("ist_map_server");
            for (std::string arg : args)
            {
              goal.args.push_back(arg);
            }

            launchCompleted = false;
            launchClient.sendGoal(goal, boost::bind(&WebAction::launchDoneCb, this, _1), WebLaunchClient::SimpleActiveCallback());
            feedback_.sequence.push_back("start ist_map_server");
            as_.publishFeedback(feedback_);
            ROS_INFO("start ist_map_server");
            state = StateEnum::START_PROCESS;
          }
          if (launchClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
            return false;
          }
        }
        break;
      case StateEnum::START_PROCESS:
        if (launchCompleted)
        {
          if (launchClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("start ist_map_server SUCCEEDED");
            feedback_.sequence.push_back("start ist_map_server SUCCEEDED");
            as_.publishFeedback(feedback_);
            if (ck_tr)
            {

              //set robot position
              ROS_INFO("Init Robot TF process");
              //nothing to do with getMapSrv
              state = StateEnum::GET_MAP_PROCESS;
            }
            else
            {
              state = StateEnum::FINISH_PROCESS;
            }
          }
          if (launchClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
          {
            return false;
          }
        }
        break;

      case StateEnum::GET_MAP_PROCESS:
        if (getMapclient.call(getMapSrv))
        {
          ROS_INFO("Get static map completed");

          setMapSrv.request.map = getMapSrv.response.map;
          setMapSrv.request.initial_pose.header.frame_id = map_frame;
          setMapSrv.request.initial_pose.header.stamp = ros::Time::now();
          // set x,y coord
          setMapSrv.request.initial_pose.pose.pose.orientation.x = pose_stamped.pose.orientation.x;
          setMapSrv.request.initial_pose.pose.pose.orientation.y = pose_stamped.pose.orientation.y;
          setMapSrv.request.initial_pose.pose.pose.orientation.z = pose_stamped.pose.orientation.z;
          setMapSrv.request.initial_pose.pose.pose.orientation.w = pose_stamped.pose.orientation.w;

          setMapSrv.request.initial_pose.pose.pose.position.x = pose_stamped.pose.position.x;
          setMapSrv.request.initial_pose.pose.pose.position.y = pose_stamped.pose.position.y;
          setMapSrv.request.initial_pose.pose.pose.position.z = pose_stamped.pose.position.z;

          setMapSrv.request.initial_pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
          setMapSrv.request.initial_pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
          setMapSrv.request.initial_pose.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;

          ROS_INFO("Set new map");
          state = StateEnum::SET_MAP_PROCESS;
        }
        break;
      case StateEnum::SET_MAP_PROCESS:
        if (setMapclient.call(setMapSrv))
        {
          ROS_INFO("Set new map completed");
          //re publish
          //ros::Publisher metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, false);
          //metadata_pub.publish(getMapSrv.response.map.info);

          state = StateEnum::FINISH_PROCESS;
        }
        break;

      case StateEnum::FINISH_PROCESS:

        ROS_INFO("loadMap FINISH_PROCESS");
        feedback_.sequence.push_back("loadMap FINISH_PROCESS");
        as_.publishFeedback(feedback_);
        finished = true;
        break;
      }
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        return false;
      }
      r.sleep();
    }

    ROS_INFO("exit loadMap");

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "web_interface_server");
  WebAction web("web_interface_server");
  ros::spin();

  return 0;
}
