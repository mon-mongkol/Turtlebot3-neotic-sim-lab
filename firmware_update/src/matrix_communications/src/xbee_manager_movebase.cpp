#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <ist_roads_srv/GoRoadAction.h>
#include <web_interface_msgs/Layout.h>

#include <matrix_msgs/XbeeCommu.h>
#include <bits/stdc++.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <web_interface_msgs/RouteFollowAction.h>
#include <ist_roads_srv/GoRoadAction.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>

using namespace matrix_msgs;
typedef actionlib::SimpleActionClient<ist_roads_srv::GoRoadAction> IstRoadsClient;
typedef actionlib::SimpleActionClient<web_interface_msgs::RouteFollowAction> IstRouteClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class XbeeCmdServer
{
    protected:
        ros::NodeHandle nh_;

        IstRouteClient ist_route_ac;
        IstRoadsClient ist_roads_ac;
        MoveBaseClient move_base_ac;
        
        bool IstRoadsCompleted;
        bool IstRouteCompleted;

       

        ros::Publisher pub_movebase_goal;
        ros::Subscriber sub_movebase_result;

        ros::Subscriber sub_xbee_;

        ros::ServiceClient layouts_srv_;

        std_msgs::String xbee_receive_;

        bool trigger = false;
        bool moveCompleted = false;
        uint movebase_status;
        uint poi_mode = 0;
        enum class Task_Sequence
        {
            Init            =   0,
            DecodeString    =   1,

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

            CancleAllMoveGoals,

            SendRespond,
            Fisnish,
        };
        Task_Sequence task_seq_ = Task_Sequence::Init;

        matrix_msgs::XbeeCommu data_info_;
        std::string serial_no = "ROHM020020220001A";

        bool IstRoads_MovingCompleted ;
        bool IstRoute_MovingCompleted ;
        unsigned int period_id;

        // configuring parameters
        std::string map_frame, base_frame; 

    
    public:
        XbeeCmdServer(std::string name) : ist_roads_ac("ist_roads_action", true),
                                            ist_route_ac("ist_route_follow", true),
                                            move_base_ac("move_base", true) {
            
            //wait for the action server to come up
            while(!ist_roads_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the ist_roads_ac action server to come up");
            }

            //wait for the action server to come up
            while(!move_base_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base_ac action server to come up");
            }

            init_pub_();
            init_sub_();
            init_srv_();
            init_param_();
            
        }

        void init_pub_()
        {
            pub_movebase_goal = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        }

        void init_sub_()
        {
            sub_xbee_ = nh_.subscribe("/xbee_receive", 10, &XbeeCmdServer::cbXbeeReceive, this);
            sub_movebase_result = nh_.subscribe("move_base/result", 1, &XbeeCmdServer::movebaseResultCallback, this);
        }

        void init_srv_()
        {
            layouts_srv_ = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
        }

        void init_param_()
        {
            ros::param::param<std::string>("~serial_no", serial_no, "ROHM020020220001A");
            ros::param::param<std::string>("~map_frame", map_frame, "/map");
            ros::param::param<std::string>("~base_frame", base_frame, "/base_link");
        }

        void spin()
        {
            ros::Rate r(10);

            while (ros::ok())
            {
                if(trigger)
                {
                    task_manager();
                }
                
                ros::spinOnce();
                r.sleep();
            }
        }


        void cbXbeeReceive(const std_msgs::String &msg)
        {
            xbee_receive_ = msg;
            ROS_INFO("%s", xbee_receive_.data.c_str());
            trigger = true;
        }

        void movebaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
        {
            movebase_status = msg->status.status;
            ROS_INFO("missions_operate : movebase Result(%d):%s", movebase_status, msg->status.text.c_str());
            moveCompleted = true;
        }

        

        // for convert String 8 byte (Hex 4 byte 0x00, 0x00, 0x00, 0x00)
        float HexString_float(std::string value, float divider)
        {
            unsigned int i;
            std::istringstream iss(value);
            iss >> std::hex >> i;
            auto j = static_cast<int>(i);
            float b = j;
            b = b/divider;
            std::cout << b << std::endl; // 1000
            return b;
        }


         // get string between 2 delimer 
        std::string split_string(std::string string_cmd, std::string begin_pos, std::string end_pos)
        {
            size_t pos = 0;
            std::string token;
            while ((pos = string_cmd.find(begin_pos)) != std::string::npos) {
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
            ssTicket>>int_Ticket;

            return int_Ticket;
        }

        matrix_msgs::XbeeCommu getCmdInfo(std::string string_receive)
        {
            matrix_msgs::XbeeCommu info_;
            info_.robot_no = HexString_float(split_string(string_receive, "#R", "CMD"), 1);
            info_.cmd = HexString_float(split_string(string_receive, "CMD", "POI"), 1);
            info_.poi = HexString_float(split_string(string_receive, "POI", "PX"), 1);
            info_.px = HexString_float(split_string(string_receive, "PX", "PY"), 100);
            info_.py = HexString_float(split_string(string_receive, "PY", "OR"), 100);
            info_.theta = HexString_float(split_string(string_receive, "OR", "ID"), 1);
            info_.id = HexString_float(split_string(string_receive, "ID", "$"), 1);

            return info_;
        }

        // bool Check_RobotCondition(matrix_msgs::XbeeCommu info_)
        // {
        //     unsigned int robot_no_ = HexString_float(split_string(string_receive, "$R", "CMD"), 1);
        // }
    
        void istRoadsDoneCb(const actionlib::SimpleClientGoalState &state, const ist_roads_srv::GoRoadResultConstPtr &result)
        {
            ROS_INFO("ist_roads DONECB: Finished in state [%s]", state.toString().c_str());
            IstRoads_MovingCompleted = true;
        }

        void istRouteDoneCb(const actionlib::SimpleClientGoalState &state, const web_interface_msgs::RouteFollowResultConstPtr &result)
        {
            ROS_INFO("ist_route DONECB: Finished in state [%s]", state.toString().c_str());
            IstRoute_MovingCompleted = true;
        }

        void task_manager()
        {   
            
            // xbee_receive_.data = "#R0001CMD0001POI0001PXFFFFD8BCPYFFFFEF65OR000082ID00000001$";

            // ROS_INFO("%s\n", xbee_receive_.data.c_str());

            switch(task_seq_)
            {
                case Task_Sequence::Init:
                    task_seq_ = Task_Sequence::DecodeString;
                    break;
                
                case Task_Sequence::DecodeString:
                    data_info_ = getCmdInfo(xbee_receive_.data);
                    task_seq_ = Task_Sequence::RobotConditionCheck;
                    break;
                
                case Task_Sequence::RobotConditionCheck:
                    if(data_info_.robot_no == DecString_int(split_string(serial_no, "ROHM02002022", "A")))
                    {
                        task_seq_ = Task_Sequence::ConditionMatch;
                    }
                    else
                    {
                        ROS_ERROR("Robot number miss match");
                        task_seq_ = Task_Sequence::SendRespond;
                    }
                    break;
                
                case Task_Sequence::ConditionMatch:
                    task_seq_ = Task_Sequence::CmdSelector;
                    break;
                
                case Task_Sequence::CmdSelector:
                    switch(data_info_.cmd)
                    {
                        case XbeeCommu::CMD_CANCLE_MOVE:
                            task_seq_ = Task_Sequence::CancleAllMoveGoals;
                            break;

                        case XbeeCommu::CMD_XY:
                            task_seq_ = Task_Sequence::MoveBase_NavGoal;
                            break;
                        
                        case XbeeCommu::CMD_POI:
                            ROS_INFO("Select POI cmd");
                            task_seq_ = Task_Sequence::MoveBase_LayoutsNavGoal;
                            break;
                        
                        case XbeeCommu::CMD_ROAD:
                            task_seq_ = Task_Sequence::IstRoads_NavGoal;
                            break;
                    }
                    break;
                

                case Task_Sequence::CancleAllMoveGoals:
                    ist_roads_ac.cancelAllGoals();
                    ist_route_ac.cancelAllGoals();
                    move_base_ac.cancelAllGoals();
                    task_seq_ = Task_Sequence::SendRespond;
                    break;
                
                case Task_Sequence::MoveBase_NavGoal:
                    task_seq_ = Task_Sequence::MoveBase_NavGoal_WaitFinish;
                    break;
                
                case Task_Sequence::MoveBase_NavGoal_WaitFinish:
                    task_seq_ = Task_Sequence::SendRespond;
                    break;
            


                case Task_Sequence::MoveBase_LayoutsNavGoal:
                {
                    if(poi_mode == 0)
                    {
                        web_interface_msgs::RouteFollowGoal goal;
                        goal.uuid = getUUID(getPOIActionForm(data_info_.poi));
                        goal.type = 28;
                        goal.repeat = 0;
                        goal.reverse = false;
                        ROS_INFO("%s", goal.uuid.c_str());
                        ist_route_ac.sendGoal(goal, boost::bind(&XbeeCmdServer::istRouteDoneCb, this, _1, _2));
                    }
                    else
                    {
                        ROS_INFO("MoveBase_LayoutsNavGoal");
                        move_base_msgs::MoveBaseGoal goal = getPoseFromLayouts(getPOIActionForm(data_info_.poi));
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();

                        ROS_INFO("x %f, y %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

                        tf::Quaternion q(
                            goal.target_pose.pose.orientation.x,
                            goal.target_pose.pose.orientation.y,
                            goal.target_pose.pose.orientation.z,
                            goal.target_pose.pose.orientation.w);
                        tf::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        ROS_INFO("R %f, P %F, Y %f", roll, pitch, yaw);

                        moveCompleted = false;
                        movebase_status = actionlib_msgs::GoalStatus::PENDING;
                        // pub_movebase_goal.publish(goal.target_pose);
                    }
                    

                    task_seq_ = Task_Sequence::MoveBase_LayoutsNavGoal_WaitFinish;
                    break;
                }
                
                case Task_Sequence::MoveBase_LayoutsNavGoal_WaitFinish:
                    if(poi_mode == 0)
                    {
                        if (IstRoute_MovingCompleted)
                        {
                            if(ist_route_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                            {
                                // auto ist_result_ = ist_roads_ac.getResult();
                                // ROS_INFO("move_result_->result = %s", move_result_->result ? "true" : "false");
                                task_seq_ = Task_Sequence::REACHED_TARGET_POS;
                            }
                            else
                            {
                                task_seq_ = Task_Sequence::MOVE_TARGET_FAIL;
                            }
                        }
                    }
                    else
                    {
                        if (moveCompleted)
                        {
                            if (movebase_status == actionlib_msgs::GoalStatus::SUCCEEDED)
                            {
                                task_seq_ = Task_Sequence::REACHED_TARGET_POS;
                                // ROS_INFO("Robot reached target position");
                            }
                            if (/* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::ABORTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::PENDING || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::REJECTED || /* ac.getState() */ movebase_status == actionlib_msgs::GoalStatus::LOST)
                            {
                                task_seq_ = Task_Sequence::MOVE_TARGET_FAIL;
                            }
                        }
                    }
                    
                    break;
                



                case Task_Sequence::IstRoads_NavGoal:
                {   
                    ist_roads_srv::GoRoadGoal goal;
                    goal.id = matrix_msgs::XbeeCommu::IST_ROAD_POI;
                    goal.cmd = "poi";
                    goal.from= "";
                    goal.target = getPOIActionForm(data_info_.poi);
                    ist_roads_ac.sendGoal(goal, boost::bind(&XbeeCmdServer::istRoadsDoneCb, this, _1, _2));

                    task_seq_ = Task_Sequence::IstRoads_NavGoal_WaitFinish;
                    break;
                }
                
                case Task_Sequence::IstRoads_NavGoal_WaitFinish:
                    if(IstRoads_MovingCompleted)
                    {
                        if(ist_roads_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            // auto ist_result_ = ist_roads_ac.getResult();
                            // ROS_INFO("move_result_->result = %s", move_result_->result ? "true" : "false");
                            task_seq_ = Task_Sequence::REACHED_TARGET_POS;
                        }
                        else
                        {
                            task_seq_ = Task_Sequence::MOVE_TARGET_FAIL;
                        }
                    }
                    break;




                case Task_Sequence::REACHED_TARGET_POS:
                    ROS_INFO("Task_Sequence::REACHED_TARGET_POS");
                    task_seq_ = Task_Sequence::SendRespond;
                    break;
                
                case Task_Sequence::MOVE_TARGET_FAIL:
                    ROS_INFO("Task_Sequence::MOVE_TARGET_FAIL");
                    task_seq_ = Task_Sequence::SendRespond;
                    break;



                case Task_Sequence::SendRespond:
                    ROS_INFO("Task_Sequence::SendRespond\n");
                    task_seq_ = Task_Sequence::Fisnish;
                    break;
                
                case Task_Sequence::Fisnish:
                    ROS_INFO("Task_Sequence::Fisnish\n");
                    trigger = false;
                    task_seq_ = Task_Sequence::Init;
                    break;
            }
        }

        std::string getPOIActionForm(int poi_no)
        {
            std::string string_form = "POI";
            std::stringstream ss;
            ss<<poi_no;
            std::string s;
            ss>>s;
            return string_form + s;
        }

        std::string getUUID(std::string poi)
        {
            web_interface_msgs::Layout layouts_srv;
            layouts_srv.request.cmd="type";
            web_interface_msgs::UILayout layouts_;
            layouts_.type = 28;
            layouts_srv.request.layouts.push_back(layouts_);

            if(layouts_srv_.call(layouts_srv))
            {
                
                for(double i = 0; i < layouts_srv.response.layouts.size(); i++)
                {   
                    std::string POI_no_ = layouts_srv.response.layouts[i].text;
                    ROS_INFO("find poi name : %s", POI_no_.c_str() );
                    if(POI_no_ == poi)
                    {
                        ROS_INFO("%s match with %s UILayout!!!", poi.c_str(), POI_no_.c_str() );
                        ROS_INFO("%s ", layouts_srv.response.layouts[i].uuid.c_str() );
                        return layouts_srv.response.layouts[i].uuid;
                        break;
                        
                    }
                }
            }else
            {
                 ROS_ERROR("call srv Failll!!!");
            }
        }

        move_base_msgs::MoveBaseGoal getPoseFromLayouts(std::string poi)
        {
            ROS_INFO("get pose from poi no: %s", poi.c_str() );
            move_base_msgs::MoveBaseGoal move_info_;

            web_interface_msgs::Layout layouts_srv;
            layouts_srv.request.cmd="type";
            
            web_interface_msgs::UILayout layouts_;
            layouts_.type = 28;
            layouts_srv.request.layouts.push_back(layouts_);
            if(layouts_srv_.call(layouts_srv))
            {
                
                for(double i = 0; i < layouts_srv.response.layouts.size(); i++)
                {   
                    std::string POI_no_ = layouts_srv.response.layouts[i].text;
                    ROS_INFO("find poi name : %s", POI_no_.c_str() );
                    if(POI_no_ == poi)
                    {
                        ROS_INFO("%s match with %s UILayout!!!", poi.c_str(), POI_no_.c_str() );
                        ROS_INFO("%s ", layouts_srv.response.layouts[i].uuid.c_str() );
                        move_info_.target_pose.pose.position =  layouts_srv.response.layouts[i].pos;
                        move_info_.target_pose.pose.orientation = getQuat(layouts_srv.response.layouts[i].pos);
                        break;
                    }
                }
            }else
            {
                 ROS_ERROR("call srv Failll!!!");
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

            // ROS_INFO("Get current robot position");
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

            if( last_x != traget_pos_.x && last_y != traget_pos_.y)
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
    ros::init(argc, argv, "xbee_management");

    XbeeCmdServer Xbee_Cmd_Server("xbee_management");
    Xbee_Cmd_Server.spin();

    return 0;
}
