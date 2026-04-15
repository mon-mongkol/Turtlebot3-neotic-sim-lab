#ifndef _IST_LAYOUT_AUTO_LOAD_H
#define _IST_LAYOUT_AUTO_LOAD_H

#include <ros/ros.h>
#include <iostream>
#include <list>
#include <iterator>
#include <string>
#include <map>
#include <vector>


#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <std_msgs/Bool.h>

#include "web_interface_msgs/GuiMap.h"
#include "web_interface_msgs/UIMap.h"
#include "web_interface_msgs/UILayout.h"
#include "web_interface_msgs/Layout.h"

#include "ist_gui.h"
#include "ist_layout_interface.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;


namespace ist_gui
{



    class GuiLayout
    {

    protected:
        ros::NodeHandle nh_;
        ros::Subscriber gui_layout_map_sub;
        ros::Subscriber map_meta_data_sub;
        nav_msgs::MapMetaData meta_data_message_;
        bool map_info_ready = false;
        std::vector<web_interface_msgs::UIMap> uiMapList;
        ros::ServiceClient layout_srv;
        ros::ServiceClient movebase_srv;
        ros::ServiceClient movebase_local_srv;

        ros::Publisher pub_layout_ready;

        GuiInterface gui_;

    public:
        GuiLayout(ros::NodeHandle &nh, ros::NodeHandle &private_nh);


    private:
        void guiLayoutMapCallback(const web_interface_msgs::UIMap::ConstPtr &msg);
        void mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr &msg);
        void convertToRos();

    };
}; // namespace ist_gui

#endif // _IST_LAYOUT_AUTO_LOAD_H