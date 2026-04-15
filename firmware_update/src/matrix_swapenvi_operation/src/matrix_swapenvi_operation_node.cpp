#include "matrix_swapenvi_operation/matrix_swapenvi_operation_node.h"

bool SwapEnviOperation::executeCB(const matrix_msgs::SwapEnviOperationGoalConstPtr &goal)
{
    // ROS_INFO("[matrix_lift_operation] recieved Goal info ------->");
    // ROS_INFO("[matrix_lift_operation] layout_name:%s", goal->envi_des.layout_name.c_str());
    // ROS_INFO("[matrix_lift_operation] map_name:%s", goal->envi_des.map_name.c_str());
    // ROS_INFO("[matrix_lift_operation] use estimate pose(AMCL)?:%s", (goal->envi_des.amcl_estimate)? "true":"false");
    // ROS_INFO("[matrix_lift_operation] estimate_func:%s", (goal->envi_des.estimate_func == )));
    // ROS_INFO("[matrix_lift_operation] lift_finish:%s", goal->envi_des.poi.c_str());
    // ROS_INFO("[matrix_lift_operation] timeout_lift_finish:%d", goal->envi_des.pose.x, goal->envi_des.pose.y);

    enum EnviSwapControlState
    {
        INIT = 0,
        CHECK_GOAL,
        CHANGE_MAP,
        WAIT_CHANGE_MAP_FINISH,
        GET_LAYOUT_INFO,
        CHANGE_LAYOUT,
        ESTIMATE_AMCL,
        FINISH,
        ERROR
    };

    // using namespace EnviSwapControlState;

    ros::Rate r(10);
    feedback_.sequence = INIT;

    finished = false;
    success = false;
    

    

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
            case EnviSwapControlState::INIT:
                ROS_INFO("[matrix_swapenvi_operation]: INIT");
                search_error_count = 0;
                feedback_.sequence = EnviSwapControlState::CHECK_GOAL;
                break;
            case EnviSwapControlState::CHECK_GOAL:
                ROS_INFO("[matrix_swapenvi_operation]: CHECK_GOAL");
                if(goalEnviChecker(goal))
                {
                    feedback_.sequence = EnviSwapControlState::CHANGE_MAP;
                }     
                else
                {
                    
                    // if(search_error_count > 1)
                    // {
                        results_.text = "environment miss match";
                        results_.result = matrix_msgs::SwapEnviOperationResult::NOT_FOUND_ENVIRONMENT;
                        feedback_.sequence = EnviSwapControlState::ERROR;
                    // }
                    //F else
                    // {
                    //     search_error_count++;
                    // }
                    
                }           
                break;
                
            case EnviSwapControlState::CHANGE_MAP:
            {
                ROS_INFO("[matrix_swapenvi_operation]: CHANGE_MAP");
                web_interface_msgs::WebCommandGoal goalMap;
                goalMap.command = "loadmap";
                auto map_name = "map_file:=" + goal->envi_des.map_name;

                ROS_INFO("[matrix_swapenvi_operation]:command: %s, goal: %s", goalMap.command, map_name.c_str());

                goalMap.args.push_back(map_name);
                map_change_ac.sendGoal(goalMap);


                // set default map
                // goalMap.command = "defaultmap";
                // map_change_ac.sendGoal(goalMap);
                // ROS_INFO("[matrix_swapenvi_operation]:command: %s, goal: %s", goalMap.command, map_name.c_str());

                json j;
                j["name"] = goal->envi_des.map_name;
                // j["id"] = 0;
                std::string map_info_json = j.dump();

                matrix_msgs::ListMap map_data;
                map_data.request.cmd = "default_map";
                map_data.request.arg = map_info_json;
                if(map_default_sc.call(map_data))
                {
                    if(map_data.response.success)
                    {
                        ROS_INFO("[matrix_swapenvi_operation]: set default map success!");
                        feedback_.sequence = EnviSwapControlState::WAIT_CHANGE_MAP_FINISH;
                    }else;
                }else;


                
                break;
            }

            case EnviSwapControlState::WAIT_CHANGE_MAP_FINISH:
            {
                ROS_INFO("[matrix_swapenvi_operation]: WAIT_CHANGE_MAP_FINISH");
                //wait for the action to return
                bool finished_before_timeout = map_change_ac.waitForResult(ros::Duration(30.0));

                if (finished_before_timeout)
                {
                    actionlib::SimpleClientGoalState state = map_change_ac.getState();
                    ROS_INFO("[matrix_swapenvi_operation]: loadmap Action finished: %s", state.toString().c_str());
                    feedback_.sequence = EnviSwapControlState::GET_LAYOUT_INFO;
                }
                else
                    ROS_INFO("[matrix_swapenvi_operation]: load map Action did not finish before the time out.");
                break;
            }

            case EnviSwapControlState::GET_LAYOUT_INFO:
                ROS_INFO("[matrix_swapenvi_operation]: GET_LAYOUT_INFO");
                layout_data = getGUIMapLayout(goal->envi_des.layout_name);
                ROS_INFO("[matrix_swapenvi_operation]: getGUIMapLayout layout_data");
                // layout_data.id = NULL;
                if(layout_data.name == goal->envi_des.layout_name)
                {
                    feedback_.sequence =EnviSwapControlState::CHANGE_LAYOUT;
                }
                else
                {
                    results_.result = matrix_msgs::SwapEnviOperationResult::NOT_FOUND_ENVIRONMENT;
                    feedback_.sequence =EnviSwapControlState::ERROR;
                }
                break;
            
            case EnviSwapControlState::CHANGE_LAYOUT:
                ROS_INFO("[matrix_swapenvi_operation]: CHANGE_LAYOUT");
                try
                {
                    layout_change_pub.publish(layout_data);

                    json j;
                    j["name"] = goal->envi_des.layout_name;
                    std::string layout_info_data = j.dump();

                    matrix_msgs::ListMap layout_data;
                    layout_data.request.cmd = "default_layout";
                    layout_data.request.arg = layout_info_data;
                    if(map_default_sc.call(layout_data))
                    {
                        if(layout_data.response.success)
                        {
                            ROS_INFO("[matrix_swapenvi_operation]: set default layout success!");
                            // feedback_.sequence = EnviSwapControlState::WAIT_CHANGE_MAP_FINISH;
                            feedback_.sequence =EnviSwapControlState::ESTIMATE_AMCL;
                        }else;
                    }else;


                    feedback_.sequence =EnviSwapControlState::ESTIMATE_AMCL;
                }
                catch (const std::runtime_error &e)
                {
                    
                    ROS_ERROR("[matrix_swapenvi_operation]: Error:%s", e.what());
                    feedback_.sequence =EnviSwapControlState::ERROR;
                }
                break;
            
            case EnviSwapControlState::ESTIMATE_AMCL:
            {
                bool error = false;
                ROS_INFO("[matrix_swapenvi_operation]: ESTIMATE_AMCL");
                if(goal->envi_des.amcl_estimate)
                {
                    geometry_msgs::PoseWithCovarianceStamped es_pose;
                    switch(goal->envi_des.estimate_func)
                    {
                        case matrix_msgs::MapLayoutSwap::func_poi:
                            es_pose = getPosePOI(goal->envi_des.poi_es, false);
                            break;
                        
                        case matrix_msgs::MapLayoutSwap::func_car:
                            es_pose.header.frame_id = "map";
                            es_pose.pose.pose = goal->envi_des.pose_es;
                            break;
                            
                        case matrix_msgs::MapLayoutSwap::func_poi_invert:
                            es_pose = getPosePOI(goal->envi_des.poi_es, true);
                            break; 
                        
                        default:
                            error = true;
                            break;
                    }
                    amcl_initose_pub.publish(es_pose);
                }
                else
                {
                    ;
                }

                if(!error)
                {
                    results_.text = "change success";
                    success = true;
                    results_.result = matrix_msgs::SwapEnviOperationResult::SUCCESS;
                    feedback_.sequence =EnviSwapControlState::FINISH;
                }
                else
                {
                    results_.text = "estimate function not found";
                    success = false;
                    results_.result = matrix_msgs::SwapEnviOperationResult::ESTIMATE_FUNC_NOT_FOUND;
                    feedback_.sequence =EnviSwapControlState::ERROR;
                }
                
                break;
            }
                

            case EnviSwapControlState::ERROR:
                ROS_INFO("[matrix_swapenvi_operation]: ERROR");
                success = false;
                // results_.text = "environment destination missmatch";
                feedback_.sequence =EnviSwapControlState::FINISH;
                break;

            case EnviSwapControlState::FINISH:
                ROS_INFO("[matrix_swapenvi_operation]: FINISH");
                // MoveBase_parameters_cfg("global_planner/GlobalPlanner");
                // ros::Duration(0.1).sleep();
                // MoveBase_parameters_cfg("SBPLLatticePlanner");
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

bool SwapEnviOperation::goalEnviChecker(const matrix_msgs::SwapEnviOperationGoalConstPtr &goal)
{
    matrix_msgs::ListMap get_poi_info_, get_mapDB_info_;
    bool map_match = false;
    bool layout_match = false;
    bool poi_match = false;

    get_mapDB_info_.request.cmd =matrix_msgs::ListMap::Request::map_list;
    get_mapDB_info_.request.arg="";
    if(map_info_sc.call(get_mapDB_info_))
    {
        for(auto map: get_mapDB_info_.response.maps)
        {
            // ROS_INFO("[matrix_swapenvi_operation]: found map_name %s", map.name.c_str());
            if(map.name == goal->envi_des.map_name)
            {
                ROS_INFO("[matrix_swapenvi_operation]:goalEnviChecker %s map name match!!", map.name.c_str());
                map_match = true;
                break;
            }
            else
            {
                map_match = false;
            }
        }
    }

    get_poi_info_.request.cmd=matrix_msgs::ListMap::Request::map_list;
    get_poi_info_.request.arg="";
    if(poi_info_sc.call(get_poi_info_))
    {
        // for(auto map: get_poi_info_.response.maps)
        // {
        //     // ROS_INFO("[matrix_swapenvi_operation]: found map_name %s", map.name.c_str());
        //     if(map.name == goal->envi_des.map_name)
        //     {
        //         ROS_INFO("[matrix_swapenvi_operation]:goalEnviChecker %s map name match!!", map.name.c_str());
        //         map_match = true;
        //         break;
        //     }
        //     else
        //     {
        //         map_match = false;
        //     }
        // }

        if(map_match)
        {
            auto layouts = get_poi_info_.response.layouts;
            for(int i = 0; i<layouts.size(); i++)
            {   
                if(layouts[i].name == goal->envi_des.layout_name)
                {
                    if(goal->envi_des.amcl_estimate)
                    {
                        if(goal->envi_des.estimate_func == matrix_msgs::MapLayoutSwap::func_poi && goal->envi_des.estimate_func == matrix_msgs::MapLayoutSwap::func_poi_invert)
                        {
                            ROS_INFO("[matrix_swapenvi_operation]:goalEnviChecker %s layouts name match!!", layouts[i].name.c_str());
                            auto pois = layouts[i].poi;
                            for(int i=0; i<pois.size(); i++)
                            {
                                if(pois[i] == goal->envi_des.poi_es)
                                {
                                    ROS_INFO("[matrix_swapenvi_operation]:goalEnviChecker %s poi name for estimate match!!", pois[i].c_str());
                                    poi_match = true;
                                    return poi_match;
                                    break;
                                }
                                else
                                {
                                    if(i == (pois.size() - 1))
                                    {
                                        ROS_ERROR("[matrix_swapenvi_operation]: Goal state POI MissMatch");
                                        poi_match = false;
                                        return poi_match;
                                        break;
                                    }
                                }
                            }
                        }else
                        {
                            poi_match = true;
                            return poi_match;
                            break;
                        }
                    }else
                    {
                        poi_match = true;
                        return poi_match;
                        break;
                    }
                    layout_match = true;
                }
                else
                {
                    if(!layout_match)
                    {
                        if(i == (layouts.size() - 1))
                        {
                            ROS_ERROR("[matrix_swapenvi_operation]: Goal state layout MissMatch");
                            map_match = false;
                            return poi_match;
                            break;
                        }
                    }
                    
                }
            }

            // for(auto layout: get_envi_info_.response.layouts)
            // {
            //     // ROS_INFO("[matrix_swapenvi_operation]: found layout_name %s", layout.name.c_str());
            //     if(layout.name == goal->envi_des.layout_name)
            //     {
            //         // ROS_INFO("[matrix_swapenvi_operation]: %s layout name match!!", layout.name.c_str());
            //         for(auto poi: layout.poi)
            //         {
            //             // ROS_INFO("[matrix_swapenvi_operation]: found poi_name %s", poi.c_str());
            //             if(poi == goal->envi_des.poi_es)
            //             {
            //                 ROS_INFO("[matrix_swapenvi_operation]: %s poi name for estimate match!!", poi.c_str());
            //                 map_match = true;
            //                 break;
            //             }
            //             else
            //             {
            //                 map_match = false;
            //             }
            //         }
            //     }
            // }
        }
        else
        {
            ROS_ERROR("[matrix_swapenvi_operation]: Goal state Map MissMatch");
            map_match = false;
            return map_match;
        }
        
    }
    else
    {
        ROS_ERROR("[matrix_swapenvi_operation]: Fail to call srv envi_info_sc(/matrix_map_list)!!");
        map_match = false;
        return map_match;
    }

    // if(map_match)
    // {
    //     ROS_INFO("[matrix_swapenvi_operation]: goal state True");
    // }
    // else
    // {
    //     ROS_ERROR("[matrix_swapenvi_operation]: Goal state MissMatch");
    // }
    return map_match;
}

geometry_msgs::PoseWithCovarianceStamped SwapEnviOperation::getPosePOI(std::string poi_name, bool invert)
{
    geometry_msgs::PoseWithCovarianceStamped posewcos;
    matrix_msgs::ListMap get_info_;
    if(invert)
    {
        get_info_.request.cmd = matrix_msgs::ListMap::Request::pose_from_poi_invert;
    }
    else
    {
        get_info_.request.cmd = matrix_msgs::ListMap::Request::pose_from_poi;
    }
    get_info_.request.arg = poi_name;
    if(poi_info_sc.call(get_info_))
    {
        posewcos.header.frame_id="map";
        posewcos.pose.pose = get_info_.response.pose;
        return posewcos;
    }
}

void SwapEnviOperation::preemptCB()
{
    ROS_WARN("[matrix_swapenvi_operation]: %s got preempted!", action_name_.c_str());
    // cancelMoveAction();
    // step_complete();
    // success = false;
    // task_completed = true;
    // results_.info = feedback_.info;
    // results_.result = XbeeCommunicationResult::GOT_PREEMPTED;
    // results_.text = XbeeCommunicationResult::GOT_PREEMPTED_s;
    // as_.setSucceeded(results_);
    finished = true;
    success = false; 
    as_.setPreempted();
}

web_interface_msgs::UIMap SwapEnviOperation::getGUIMapLayout(std::string layout_name)
{
    web_interface_msgs::UIMap return_info;
    web_interface_msgs::GuiMap layout_info;
    layout_info.request.cmd = "list";

    int32_t map_id = 0;

    // add empty arrey member for match case ist_gui_map_server/ist_gui_map_server_node.cpp line 78
    web_interface_msgs::UIMap empty_member; 
    layout_info.request.gui_maps.push_back(empty_member);
    if(layout_change_sc.call(layout_info))
    {
        ROS_INFO("[matrix_swapenvi_operation]: total layout of robot %d", layout_info.response.gui_maps.size());
        for(auto getGUI : layout_info.response.gui_maps)
        {
            if(getGUI.name == layout_name)
            {
                map_id = getGUI.id;
                ROS_INFO("[matrix_swapenvi_operation]: %s layout name match!", getGUI.name.c_str());
                return return_info = getGUI;
                break;
                
            }
            else
            {
                ROS_WARN("[matrix_swapenvi_operation]: %s layout miss match layout %s!", getGUI.name.c_str(), layout_name.c_str());
            }
        }
    }

    // if(map_id != 0)
    // {
    //     web_interface_msgs::UIMap default_layout_info;
    //     web_interface_msgs::GuiMap set_default_layout;
    //     set_default_layout.request.cmd="set_default";
    //     default_layout_info.id = map_id;
    //     set_default_layout.request.gui_maps.push_back(default_layout_info);
    //     layout_change_sc.call(set_default_layout);

    //     set_default_layout.request.cmd="get_default";
    //     set_default_layout.request.gui_maps.clear();
    //     if(layout_change_sc.call(set_default_layout))
    //     {
    //         for(auto getGUI : set_default_layout.response.gui_maps)
    //         {
    //             if((getGUI.name == layout_name) && (getGUI.is_default))
    //             {
    //                 // map_id = getGUI.id;
    //                 ROS_INFO("[matrix_swapenvi_operation]: %s set default layout success!!", getGUI.name.c_str());
    //                 return return_info = getGUI;
    //                 break;
                    
    //             }
    //             else
    //             {
    //                 // ROS_WARN("[matrix_swapenvi_operation]: %s layout miss match layout %s!", getGUI.name.c_str(), layout_name.c_str());
    //             }
    //         }
    //     }

    // }
    
    
}

bool SwapEnviOperation::MoveBase_parameters_cfg(std::string planner)
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

    strs_.name = "base_global_planner";
    strs_.value = planner;
    ROS_INFO("[Matrix_SwapEnvironment]: change planner to %s", strs_.value.c_str());
    cfg_params.request.config.strs.push_back(strs_);

    if (movebase_cfg.call(cfg_params))
    {
        ;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "matrix_swapenvi_operation");

    SwapEnviOperation swapenvioperation("matrix_swapenvi_operation");
    ros::spin();

    return 0;
}