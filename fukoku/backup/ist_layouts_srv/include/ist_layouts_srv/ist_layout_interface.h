#ifndef _IST_LAYOUT_INTERFACE_H
#define _IST_LAYOUT_INTERFACE_H

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

#include "web_interface_msgs/GuiMap.h"
#include "web_interface_msgs/UIMap.h"
#include "web_interface_msgs/UILayout.h"
#include "web_interface_msgs/Layout.h"

#include "mission_msgs/MissionRunAction.h"
#include "mission_msgs/Mission.h"
#include "mission_msgs/MissionRoom.h"
#include "mission_msgs/MissionOperateAction.h"
#include "mission_msgs/Rooms.h"
#include "mission_msgs/MissionRunning.h"
#include "mission_msgs/MoveTarget.h"

#include "ist_gui.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace ist_gui
{
    class GuiInterface
    {
    public:
        GuiInterface(const std::string &name);
        std::vector<web_interface_msgs::UILayout> toUILayout(const std::string &objects, nav_msgs::MapMetaData meta_data_message, bool debug = false);
        web_interface_msgs::Layout convertToRos(std::vector<web_interface_msgs::UIMap> uiMapList, nav_msgs::MapMetaData meta_data_message);
        void showLayout(const web_interface_msgs::UILayout &mp);
        void getMissionRoom(mission_msgs::MissionRoom *mission, mission_msgs::MoveTarget *manual_in, mission_msgs::MoveTarget *manual_out, const std::vector<web_interface_msgs::UILayout> &guiList, web_interface_msgs::UILayout *layout_radiation = nullptr);
        void showMission(const mission_msgs::MissionRoom &mp);
        std::vector<web_interface_msgs::UILayout> getDoors(std::vector<web_interface_msgs::UILayout> doorList, web_interface_msgs::UILayout room);
        void getMissionToDoors(mission_msgs::MissionRoom *mission, std::vector<web_interface_msgs::UILayout> doorList, web_interface_msgs::UILayout room);

        std::vector<int> getRadiationTimes(web_interface_msgs::UILayout layout_radiation);
        double polygonArea(std::vector<geometry_msgs::Point> pts);

    private:
        std::vector<geometry_msgs::Point> toWorldPoints(geometry_msgs::Point p, geometry_msgs::Point rotate, std::vector<geometry_msgs::Point> points);
        geometry_msgs::Point toROS(geometry_msgs::Point p, nav_msgs::MapMetaData meta_data_message);

        std::string package_;
    };

}; // namespace ist_gui

#endif // _IST_LAYOUT_INTERFACE_H