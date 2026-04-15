#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ist_layouts_srv/ist_layout_auto_load.h"

using namespace Eigen;
namespace ist_gui
{

    GuiLayout::GuiLayout(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
        : nh_(nh), gui_("ist_layout_auto_load")
    {

        ros::NodeHandle nh_priv("~");

        std::string global_costmap_ist;
        std::string local_costmap_ist;

        nh_priv.param<std::string>("global_costmap_ist_check", global_costmap_ist, "move_base/global_costmap/ist_costmap_prohibition_layer/set_parameters");
        nh_priv.param<std::string>("local_costmap_ist_check", local_costmap_ist, "move_base/local_costmap/ist_costmap_prohibition_layer/set_parameters");

        ros::ServiceClient map = nh_.serviceClient<nav_msgs::GetMap>("static_map");
        ros::ServiceClient gui_map = nh_.serviceClient<web_interface_msgs::GuiMap>("ist_gui_map_server");
        layout_srv = nh_.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
        movebase_srv = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(global_costmap_ist);
        movebase_local_srv = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(local_costmap_ist);

        pub_layout_ready = nh_.advertise<std_msgs::Bool>("ist_layouts_srv/is_ready", 1, true);

        while (ros::ok() && !map.waitForExistence(ros::Duration(5.0)))
        {
            ROS_INFO("ist_gui_map_server_auto_load wait for static_map come to ready.");
        }

        while (ros::ok() && !gui_map.waitForExistence(ros::Duration(5.0)))
        {
            ROS_INFO("ist_gui_map_server_auto_load wait for ist_gui_map_server come to ready.");
        }

        while (ros::ok() && !layout_srv.waitForExistence(ros::Duration(5.0)))
        {
            ROS_INFO("ist_gui_map_server_auto_load wait for ist_layouts_srv come to ready.");
        }

        while (ros::ok() && !movebase_srv.waitForExistence(ros::Duration(5.0)))
        {
            ROS_INFO_STREAM("ist_gui_map_server_auto_load wait for " << global_costmap_ist << " come to ready.");
        }
        while (ros::ok() && !movebase_local_srv.waitForExistence(ros::Duration(5.0)))
        {
            ROS_INFO_STREAM("ist_gui_map_server_auto_load wait for " << local_costmap_ist << " come to ready.");
        }

        gui_layout_map_sub = nh_.subscribe("gui_layout_map", 1, &GuiLayout::guiLayoutMapCallback, this);
        map_meta_data_sub = nh_.subscribe("map_metadata", 1, &GuiLayout::mapMetaDataCallback, this);
    };

    void GuiLayout::mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr &msg)
    {
        ROS_INFO("Map MataData Callback");
        meta_data_message_ = (*msg);

        ROS_INFO("Map  %d X %d map @ %.3lf m/cell",
                 meta_data_message_.width,
                 meta_data_message_.height,
                 meta_data_message_.resolution);
        map_info_ready = true;
        convertToRos();
    }

    // string uuid
    // string name
    // string text
    // int32 type
    // int32 startHandle
    // int32 rotateHandle
    // float32 rot
    // float32 scale
    // geometry_msgs/Point pos
    // geometry_msgs/Point[] points
    // web_interface_msgs::UILayout::ConstPtr GuiLayout::toROSLayout(json obj)
    // {

    //     return null;
    // }

    void GuiLayout::guiLayoutMapCallback(const web_interface_msgs::UIMap::ConstPtr &msg)
    {
        ROS_INFO("GUI Layout Callback");
        ROS_INFO("[%s] id:%d name=%s, description=%s", msg->is_default ? "*" : " ", msg->id, msg->name.c_str(), msg->description.c_str());
        uiMapList.clear();
        uiMapList.push_back(*msg);

        convertToRos();
    }

    void GuiLayout::convertToRos()
    {
        if (!map_info_ready)
        {
            ROS_ERROR("ist_layout_auto_load: map infor not ready!");
            return;
        }

        try
        {

            // pub_layout_ready
            std_msgs::Bool is_ready;
            is_ready.data = false;
            pub_layout_ready.publish(is_ready);

            web_interface_msgs::Layout layout;
            layout.request.cmd = "clear";
            layout.request.layouts.clear();

            if (!layout_srv.call(layout))
            {
                throw std::runtime_error("call ist_gui_map_server clear");
            }

            auto guiList = gui_.convertToRos(uiMapList, meta_data_message_);

            guiList.request.cmd = "add";
            if (layout_srv.call(guiList))
            {
                ros::ServiceClient map = nh_.serviceClient<nav_msgs::GetMap>("static_map");
                if (map.exists())
                {
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
                    if (movebase_srv.call(dy))
                    {

                        if (!movebase_local_srv.call(dy))
                        {
                            throw std::runtime_error("call ist_gui_map_server move_base/local_costmap/ist_costmap_prohibition_layer/set_parameters");
                        }
                        is_ready.data = true;
                        pub_layout_ready.publish(is_ready);
                    }
                    else
                    {
                        throw std::runtime_error("call ist_gui_map_server move_base/global_costmap/ist_costmap_prohibition_layer/set_parameters");
                    }
                }
            }
            else
            {
                throw std::runtime_error("call ist_gui_map_server add");
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Layout Callback exception: %s", e.what());
        }
    }

}; // namespace ist_gui

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ist_layout_auto_load");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try
    {
        ROS_INFO("START ist_layout_auto_load\n");
        ist_gui::GuiLayout gui(nh, private_nh);
        ros::spin();
    }
    catch (std::runtime_error &e)
    {
        ROS_ERROR("ist_layout_auto_load exception: %s", e.what());
        return -1;
    }

    return 0;
}