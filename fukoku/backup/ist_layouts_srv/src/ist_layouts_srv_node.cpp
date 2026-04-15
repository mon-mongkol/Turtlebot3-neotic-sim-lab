#include <iostream>
#include <list>
#include <iterator>
#include <string>
#include <map>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "web_interface_msgs/Layout.h"
#include "web_interface_msgs/UILayout.h"
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>

//std::map<std::string, web_interface_msgs::UILayout> dicLayouts;
std::vector<web_interface_msgs::UILayout> dicLayouts;

void showLayout(const web_interface_msgs::UILayout &mp)
{
    ROS_INFO("Layout: uuid=%s, name=%s, type=%d", mp.uuid.c_str(), mp.name.c_str(), (int)mp.type);
    ROS_INFO("startHandle=%d, rotateHandle=%d", mp.startHandle, mp.rotateHandle);
    int i = 0;
    ROS_INFO("points:");
    for (const geometry_msgs::Point &p : mp.points)
    {
        ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
    }
    ROS_INFO("handles:");
    for (const geometry_msgs::Point &p : mp.handles)
    {
        ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
    }
}

web_interface_msgs::UILayout *FindBy(std::string uuid, int type)
{
    // for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
    // {
    //     auto &mp = it->second;
    //     if (mp.uuid == uuid && (int)mp.type == type)
    //     {
    //         return &mp;
    //     }
    // }
    for (auto &mp : dicLayouts)
    {
        if (mp.uuid == uuid && (int)mp.type == type)
        {
            return &mp;
        }
    }

    return NULL;
}

bool interaction(web_interface_msgs::Layout::Request &req,
                 web_interface_msgs::Layout::Response &res)
{
    //ROS_INFO("Layout command:%s", req.cmd.c_str());
    res.result = req.cmd;
    res.layouts.clear();

    // for (const web_interface_msgs::UILayout &mp : req.layouts)
    // {
    //     showLayout(mp);
    // }

    if (std::strcmp(req.cmd.c_str(), "add") == 0)
    {
        //ROS_INFO("Layout command:%s", req.cmd.c_str());

        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            if (FindBy(mp.uuid, mp.type) == NULL)
            {
                //dicLayouts.insert({mp.uuid, mp});
                dicLayouts.push_back(mp);
                res.layouts.push_back(mp);
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "edit") == 0)
    {
        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {

                if ((*it).uuid == mp.uuid && ((*it).type == mp.type))
                {
                    (*it) = mp;
                    ROS_INFO("edit uuid:%s", (*it).uuid.c_str());
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "list") == 0)
    {

        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {
            if (mp.uuid == "" && mp.type == 0)
            {
                for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
                {
                    res.layouts.push_back(*it);
                }
            }
            else
            {
                for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
                {
                    if ((*it).uuid == mp.uuid && (*it).type == mp.type)
                    {
                        res.layouts.push_back(*it);
                    }
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "type") == 0)
    {
        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {
                if ((*it).type == mp.type)
                {
                    res.layouts.push_back(*it);
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "type_uuid") == 0)
    {
        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {
                if (it->type == mp.type && it->uuid == mp.uuid)
                {
                    res.layouts.push_back(*it);
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "type_name") == 0)
    {
        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {
                if (it->type == mp.type && it->name == mp.name)
                {
                    res.layouts.push_back(*it);
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "type_text") == 0)
    {
        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {
                if (it->type == mp.type && it->text == mp.text)
                {
                    res.layouts.push_back(*it);
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "uuid") == 0)
    {

        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {
                if ((*it).uuid == mp.uuid && (*it).type == mp.type)
                {
                    res.layouts.push_back(*it);
                    break;
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "erase") == 0)
    {
        //ROS_INFO("Layout command:%s", req.cmd.c_str());
        for (const web_interface_msgs::UILayout &mp : req.layouts)
        {

            for (auto it = dicLayouts.begin(); it != dicLayouts.end(); ++it)
            {
                if ((*it).uuid == mp.uuid && (*it).type == mp.type)
                {
                    res.layouts.push_back(*it);
                    dicLayouts.erase(it);
                    break;
                }
            }
        }

        return true;
    }

    if (std::strcmp(req.cmd.c_str(), "clear") == 0)
    {
        ROS_INFO("Layout command:%s", req.cmd.c_str());
        dicLayouts.clear();
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ist_layouts_srv");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("ist_layouts_srv", interaction);
    ROS_INFO("Ready to recieve layouts.");
    ros::spin();

    return 0;
}
