#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <ist_roads_srv/GoRoadAction.h>
#include <web_interface_msgs/Layout.h>
#include <matrix_msgs/XbeeCommu.h>
#include <bits/stdc++.h>
#include <move_base_msgs/MoveBaseAction.h>

// using namespace ist_roads_srv;
typedef actionlib::SimpleActionClient<ist_roads_srv::GoRoadAction> IstRoadsSrvClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class XbeeCmdServer
{
    protected:
        ros::NodeHandle nh_;
        IstRoadsSrvClient ist_roads_ac;
        MoveBaseClient move_base_ac;

        bool IstRoadsCompleted;

        double last_x = 0;
        double last_y = 0;

        ros::Publisher pub_movebase_goal;
        ros::Subscriber sub_xbee_;

        ros::ServiceClient layouts_srv_;

        std_msgs::String xbee_receive_;

        bool trigger = false;

        enum class Task_Sequence
        {
            Init            =   0,
            DecodeString    =   1,

            RobotConditionCheck,
            ConditionMatch,
            ConditionMissMatch,

            CmdSelector,

            NavigationGoal,
            NavigationGoal_WaitFinish,

            SendRespond,
            Fisnish,
        };
        Task_Sequence task_seq_ = Task_Sequence::Init;

        matrix_msgs::XbeeCommu data_info_;
        std::string serial_no = "ROHM020020220001A";

        bool MovingCompleted ;
        unsigned int period_id; 

    
    public:
        XbeeCmdServer(std::string name) : ist_roads_ac("ist_roads_action", true),
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
        }

        void init_srv_()
        {
            layouts_srv_ = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
        }

        void init_param_()
        {
            ros::param::param<std::string>("~serial_no", serial_no, "ROHM020020220001A");
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
            MovingCompleted = true;
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
                    task_seq_ = Task_Sequence::ConditionMatch;
                    break;
                
                case Task_Sequence::ConditionMatch:
                    task_seq_ = Task_Sequence::CmdSelector;
                    break;
                
                case Task_Sequence::CmdSelector:
                {   

                    // if(data_info_.id != period_id)
                    // {
                        printf("id match!!\n");
                        using namespace matrix_msgs;

                        ist_roads_ac.cancelAllGoals();

                        ist_roads_srv::GoRoadGoal goal;

                        //classification id in ist_roads_srv/GoRoadGoal (mode of operation) 0 = poi, 1 = road
                        if(data_info_.cmd != XbeeCommu::CMD_CANCLE_MOVE)
                        {
                            if(data_info_.cmd != XbeeCommu::CMD_XY)
                            {
                                goal.id = (data_info_.cmd == XbeeCommu::CMD_POI)? 0 : 1;
                                goal.cmd = "poi";
                                goal.from= "";
                                goal.target = getPOIActionForm(data_info_.poi);
                                ist_roads_ac.sendGoal(goal, boost::bind(&XbeeCmdServer::istRoadsDoneCb, this, _1, _2));
                                // printf("POI no. %s\n", goal.goal.target.c_str());
                                task_seq_ = Task_Sequence::NavigationGoal_WaitFinish;
                                break;
                            }
                            else
                            {
                                task_seq_ = Task_Sequence::Fisnish;
                                break;
                            }
                        }
                        else
                        {
                            ist_roads_ac.cancelAllGoals();
                            task_seq_ = Task_Sequence::Fisnish;
                            break;
                        }

                    //     period_id = data_info_.id;
                    // }
                    // else
                    // {
                    //     task_seq_ = Task_Sequence::Fisnish;
                    // }
                    
                    break;
                }

                case Task_Sequence::NavigationGoal_WaitFinish:
                {
                    printf("Task_Sequence::NavigationGoal_WaitFinish\n");
                    if(MovingCompleted)
                    {
                        if(ist_roads_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            printf("ist_roads_ac.getState\n");
                            // auto move_result_ = ist_roads_ac.getResult();
                            // ROS_INFO("move_result_->result = %s", move_result_->result ? "true" : "false");
                            task_seq_ = Task_Sequence::Fisnish;
                        }
                        task_seq_ = Task_Sequence::Fisnish;
                    }
                    printf("%d\n", MovingCompleted);
                    break;
                }
                case Task_Sequence::SendRespond:
                    printf("Task_Sequence::SendRespond\n");
                    task_seq_ = Task_Sequence::Fisnish;
                    
                    break;
                
                case Task_Sequence::Fisnish:
                    printf("Task_Sequence::Fisnish\n");
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
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "xbee_management");

    XbeeCmdServer Xbee_Cmd_Server("xbee_management");
    Xbee_Cmd_Server.spin();

    return 0;
}
