#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <web_interface_msgs/Layout.h>
#include <web_interface_msgs/UILayout.h>
#include <web_interface_msgs/Zones.h>
#include <web_interface_msgs/IntersectPOI.h>
#include "clipper.hpp"
#include <algorithm> // std::max
using namespace ClipperLib;

const int point_scale = 10000;
geometry_msgs::PoseStamped pose_stamped;
bool robot_footprint_ready = false;

Paths robot_footprint(1);

//https://stackoverflow.com/questions/38382777/how-do-i-determine-if-two-polygons-intersect-using-clipper

bool Intersects(const Paths &subj, const Paths &clip)
{
    ClipperLib::Clipper c;

    c.AddPaths(subj, ClipperLib::ptSubject, true);
    c.AddPaths(clip, ClipperLib::ptClip, true);

    ClipperLib::Paths solution;
    c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    return solution.size() != 0;
}

// void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
// {

//     pose_stamped.header.frame_id = "/map";
//     pose_stamped.header.stamp = ros::Time::now();

//     pose_stamped.pose.orientation.x = msg->orientation.x;
//     pose_stamped.pose.orientation.y = msg->orientation.y;
//     pose_stamped.pose.orientation.z = msg->orientation.z;
//     pose_stamped.pose.orientation.w = msg->orientation.w;

//     pose_stamped.pose.position.x = msg->position.x;
//     pose_stamped.pose.position.y = msg->position.y;
//     pose_stamped.pose.position.z = msg->position.z;

//     ROS_INFO("Position: x: %f, y: %f, z: %f", msg->position.x, msg->position.y, msg->position.z);
//     // ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
//     robot_pose_ready = true;
// }

void robotFootprintCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg)
{
    int i = 0;
    //ROS_INFO("robot footprint points:");
    robot_footprint_ready = false;
    robot_footprint[0].clear();
    for (auto &p : msg->polygon.points)
    {
       // ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
        robot_footprint[0].push_back(IntPoint(p.x * point_scale, p.y * point_scale));
    }
    robot_footprint_ready = true;
}

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

int main(int argc, char **argv)
{
    // initialize ROS and the node
    ros::init(argc, argv, "ist_layout_poi");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // configuring parameters
    std::string map_frame, base_frame, footprint_topic;
    bool is_check_footprint;
    double publish_frequency;

    ros::Publisher p_pub, p_poi;
    ros::Subscriber robot_footprint_sub;

    std::string current_poi, last_poi;

    nh_priv.param<bool>("is_check_footprint", is_check_footprint, true);
    nh_priv.param<std::string>("map_frame", map_frame, "/map");
    nh_priv.param<std::string>("base_frame", base_frame, "/base_footprint");
    nh_priv.param<std::string>("footprint_topic", footprint_topic, "/move_base/global_costmap/footprint");

    nh_priv.param<double>("publish_frequency", publish_frequency, 1);

    p_pub = nh.advertise<web_interface_msgs::Zones>("robot_zone", 1);
    p_poi = nh.advertise<web_interface_msgs::IntersectPOI>("robot_on_poi", 1);

    robot_footprint_sub = nh.subscribe(footprint_topic, 1, &robotFootprintCallback);

    // create the listener
    tf::TransformListener listener;
    if (!is_check_footprint)
    {
        ROS_INFO("ist_layout_zones Waiting for robot_pose");
        listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(2.0));
        ROS_INFO("robot transform is ready");
    }
    else
    {
        ROS_INFO("ist_layout_zones Waiting for %s", footprint_topic.c_str());
    }

    ros::Rate rate(publish_frequency);

    // getLayouts();
    ros::ServiceClient client = nh.serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");
    // web_interface_msgs::Layout srv;
    // web_interface_msgs::UILayout layout_green;
    // web_interface_msgs::UILayout layout_yellow;
    // web_interface_msgs::UILayout layout_orange;
    // web_interface_msgs::UILayout layout_room;
    // web_interface_msgs::UILayout layout_door;

    // srv.request.cmd = "type";
    // layout_room.type = 7;    //room
    // layout_door.type = 10;   //door
    // layout_green.type = 17;  // ZoneGreen : 17,
    // layout_yellow.type = 18; // ZoneYellow : 18,
    // layout_orange.type = 19; // ZoneOrange : 19,

    // srv.request.layouts.push_back(layout_door);
    // srv.request.layouts.push_back(layout_room);
    // srv.request.layouts.push_back(layout_green);
    // srv.request.layouts.push_back(layout_yellow);
    // srv.request.layouts.push_back(layout_orange);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    while (nh.ok())
    {
        int zone = 0;
        web_interface_msgs::Zones result;
        IntPoint pt;
        bool ready = false;
        bool check_result = false;
        try
        {
            if (is_check_footprint)
            {
                if (robot_footprint_ready)
                {
                    ready = true;
                }
            }
            else
            {
                tf::StampedTransform transform;
                double x, y;
                listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
                //ROS_INFO("robot_pose_ready");
                x = transform.getOrigin().getX();
                y = transform.getOrigin().getY();
                pt = IntPoint(x * point_scale, y * point_scale);
                ready = true;
            }

            if (ready)
            {
                //ROS_INFO("Get layouts list");
                // if (client.call(srv))
                // {
                //     //ROS_INFO("Layout result:%s", srv.response.result.c_str());
                //     for (const web_interface_msgs::UILayout &mp : srv.response.layouts)
                //     {
                //         //showLayout(mp);
                //         //ROS_INFO("Layout: rot=%f uuid=%s, type=%d", mp.rot, mp.uuid.c_str(), (int)mp.type);
                //         int i = 0;
                //         Path poly;
                //         // for (int j = mp.startHandle; j < mp.rotateHandle /* mp.points.size() */; ++j)
                //         // {
                //         //     //ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, mp.points[j].x, mp.points[j].y, mp.points[j].z);
                //         //     poly.push_back(IntPoint(mp.points[j].x * point_scale, mp.points[j].y * point_scale));
                //         // }
                //         for (const geometry_msgs::Point &p : mp.points)
                //         {
                //             //ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
                //             poly.push_back(IntPoint(p.x * point_scale, p.y * point_scale));
                //         }

                //         if (is_check_footprint)
                //         {
                //             Paths zone(1);
                //             zone.push_back(poly);
                //             check_result = Intersects(robot_footprint, zone);
                //         }
                //         else
                //         {
                //             //See "The Point in Polygon Problem for Arbitrary Polygons" by Hormann & Agathos
                //             //http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.88.5498&rep=rep1&type=pdf
                //             int ret = PointInPolygon(pt, poly);
                //             check_result = (ret != 0);
                //             //returns 0 if false, +1 if true, -1 if pt ON polygon boundary
                //         }

                //         if (check_result)
                //         {
                //             zone = std::max((int)mp.type, zone);
                //             if (!result.is_room)
                //                 result.is_room = mp.type == layout_room.type;
                //             result.zones.push_back((int)mp.type);
                //             result.zone_id = zone;
                //         }
                //     }
                // }
                // //ROS_INFO("robot pos:x=%f, y=%f, zone=%d", x, y, zone);
                // // std_msgs::Int32 p;
                // // p.data = zone;
                // p_pub.publish(result);


                // addition by Jaruwat D. date: Wed Sep 14 2022, 1.30PM
                /********************************************/
                /*          Check Footprint on POI          */
                /********************************************/      
                web_interface_msgs::Layout srv_;
                web_interface_msgs::UILayout layout_poi;
                srv_.request.cmd = "type";
                layout_poi.type = 28;    //poi
                srv_.request.layouts.push_back(layout_poi);

                //ROS_INFO("Get layouts POI list");
                if (client.call(srv_))
                {   
                    current_poi = "";
                    //ROS_INFO("Layout result:%s", srv.response.result.c_str());
                    for (const web_interface_msgs::UILayout &mp : srv_.response.layouts)
                    {
                        //showLayout(mp);
                        //ROS_INFO("Layout: rot=%f uuid=%s, type=%d", mp.rot, mp.uuid.c_str(), (int)mp.type);
                        int i = 0;
                        Path poly;
                        // for (int j = mp.startHandle; j < mp.rotateHandle /* mp.points.size() */; ++j)
                        // {
                        //     //ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, mp.points[j].x, mp.points[j].y, mp.points[j].z);
                        //     poly.push_back(IntPoint(mp.points[j].x * point_scale, mp.points[j].y * point_scale));
                        // }
                        for (const geometry_msgs::Point &p : mp.points)
                        {
                            //ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
                            poly.push_back(IntPoint(p.x * point_scale, p.y * point_scale));
                        }

                        if (is_check_footprint)
                        {
                            Paths poi(1);
                            poi.push_back(poly);
                            check_result = Intersects(robot_footprint, poi);
                        }
                        else
                        {
                            //See "The Point in Polygon Problem for Arbitrary Polygons" by Hormann & Agathos
                            //http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.88.5498&rep=rep1&type=pdf
                            int ret = PointInPolygon(pt, poly);
                            check_result = (ret != 0);
                            //returns 0 if false, +1 if true, -1 if pt ON polygon boundary
                        }

                        if (check_result)
                        {
                            if((current_poi != last_poi))
                            {
                                last_poi = current_poi;
                            }
                            
                            current_poi = mp.text;
                            break;
                        }
                        // else
                        // {
                        //     current_poi ="";
                        // }
                       
                        
                    }

                    

                    web_interface_msgs::IntersectPOI msg;
                    msg.current_poi = current_poi;
                    msg.last_poi = last_poi;
                    p_poi.publish(msg);

                    check_result = false;
                }

            }
        }
        catch (const std::runtime_error &e)
        {
            // just continue on
            ROS_ERROR("ist_layout_zones exception: %s", e.what());
        }
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}