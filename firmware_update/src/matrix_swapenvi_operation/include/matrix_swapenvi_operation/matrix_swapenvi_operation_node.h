#ifndef _MATRIX_SWAP_ENVI_NODE_H
#define _MATRIX_SWAP_ENVI_NODE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <matrix_msgs/SwapEnviOperationAction.h>
#include "web_interface_msgs/WebCommandAction.h"     // map server 
#include <geometry_msgs/PoseWithCovarianceStamped.h> // amcl initail pose
#include <web_interface_msgs/GuiMap.h>
#include <matrix_msgs/ListMap.h>
#include <tf/tf.h>
#include <cmath>
#include <dynamic_reconfigure/Reconfigure.h>

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

typedef actionlib::SimpleActionClient<web_interface_msgs::WebCommandAction> WebCommandClient;

class SwapEnviOperation
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<matrix_msgs::SwapEnviOperationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs
        std::string action_name_;
        // create messages that are used to published feedback/result
        matrix_msgs::SwapEnviOperationFeedback feedback_;
        matrix_msgs::SwapEnviOperationResult   results_;

        WebCommandClient map_change_ac; 
        bool MapChangeCompleted = false;

        ros::Publisher amcl_initose_pub, layout_change_pub;
        ros::ServiceClient layout_change_sc, map_info_sc, poi_info_sc, movebase_cfg, map_default_sc;

        web_interface_msgs::UIMap layout_data;
        int search_error_count;
        bool finished = false;
        bool success = false; 

    public:
        SwapEnviOperation(std::string name)
            :   as_(nh_, name, boost::bind(&SwapEnviOperation::executeCB, this, _1), false),
            action_name_(name),
            map_change_ac("web_interface_server", true)
        {
            //wait for the action server to come up
            while(!map_change_ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the map_change_ac action server to come up");
            }
            
            

            layout_change_sc = nh_.serviceClient<web_interface_msgs::GuiMap>("ist_gui_map_server");
            movebase_cfg = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
            map_info_sc = nh_.serviceClient<matrix_msgs::ListMap>("matrix_listmaps"); //matrix_map_list
            map_default_sc = nh_.serviceClient<matrix_msgs::ListMap>("/matrix_listmaps/default_map"); //matrix_map_list
            poi_info_sc = nh_.serviceClient<matrix_msgs::ListMap>("matrix_map_list");
            
            layout_change_pub = nh_.advertise<web_interface_msgs::UIMap>("/gui_layout_map", 1);
            amcl_initose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);



            layout_change_sc.waitForExistence();
            map_info_sc.waitForExistence();
            map_default_sc.waitForExistence();
            poi_info_sc.waitForExistence();

            as_.start();
            as_.registerPreemptCallback(boost::bind(&SwapEnviOperation::preemptCB, this));
            ROS_INFO("matrix_swapenvi_operation action server is ready!");
        }
    
    private:
        void preemptCB();
        bool executeCB(const matrix_msgs::SwapEnviOperationGoalConstPtr &goal);

        web_interface_msgs::UIMap getGUIMapLayout(std::string layout_name);
        geometry_msgs::PoseWithCovarianceStamped getPosePOI(std::string poi_name, bool invert);
        bool goalEnviChecker(const matrix_msgs::SwapEnviOperationGoalConstPtr &goal);
        bool MoveBase_parameters_cfg(std::string planner);
        // void MapChnageDoneCb(const actionlib::SimpleClientGoalState &state, const web_interface_msgs::WebCommandResultConstPtr &result);

};

#endif // _MATRIX_SWAP_ENVI_NODE_H