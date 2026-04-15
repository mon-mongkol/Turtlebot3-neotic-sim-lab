#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <matrix_msgs/ListMap.h>
#include <iostream>
#include <vector>
#include <dirent.h>
#include <web_interface_msgs/GuiMap.h>
#include <web_interface_msgs/Layout.h>
#include <nlohmann/json.hpp>
#include <tf/tf.h>
#include <cmath>

using json = nlohmann::json;

using std::cout; using std::cin;
using std::endl; using std::vector;

class MapList {

    private:
    // int counter;
    // ros::Publisher pub;
    // ros::Subscriber number_subscriber;
    ros::ServiceServer map_list_service;
    ros::ServiceClient ist_gui_layout_client, ist_layout_info_client;

    std::string user;
    std::string path_maps_;
    

    public:
    MapList(ros::NodeHandle *nh) {
        // counter = 0;

        // pub = nh->advertise<std_msgs::Int64>("/number_count", 10);    
        // number_subscriber = nh->subscribe("/number", 1000, 
        //     &MapList::callback_number, this);
        map_list_service = nh->advertiseService("/matrix_map_list", 
            &MapList::callback_map_list, this);

        ist_gui_layout_client = nh->serviceClient<web_interface_msgs::GuiMap>("ist_gui_map_server");
        ist_layout_info_client = nh->serviceClient<web_interface_msgs::Layout>("ist_layouts_srv");

        ros::Rate loop_rate(1);

        user = getenv("USER");
        path_maps_ = "/home/" + user + "/maps";
        ROS_INFO("maps file path is --> %s", path_maps_.c_str());

        while(ros::ok())
        {
            // getLayoutList();
            // cout<<getenv("USER")<<endl;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    // void callback_number(const std_msgs::Int64& msg) {
    //     counter += msg.data;
    //     std_msgs::Int64 new_msg;
    //     new_msg.data = counter;
    //     pub.publish(new_msg);
    // }

    bool callback_map_list(matrix_msgs::ListMap::Request &req, 
                           matrix_msgs::ListMap::Response &res)
    {
        ROS_INFO("[Matrix_map_list]: service call cmd:%s w/ arg:%s", req.cmd.c_str(), req.arg.c_str());
        res.cmd = req.cmd;
        res.arg = req.arg;
        if (req.cmd == matrix_msgs::ListMap::Request::map_list) {
            

            for(int i=0; i<2; i++)
            {
                // res.maps = getMapList();

                matrix_msgs::MapsInfo get_maps_;
                DIR *dir; struct dirent *diread;
                vector<char *> files;

                

                const char * path_ = path_maps_.c_str();


                if ((dir = opendir(path_)) != nullptr) {
                    while ((diread = readdir(dir)) != nullptr) {
                        files.push_back(diread->d_name);
                    }
                    closedir (dir);
                } else {
                    perror ("opendir");
                    // return EXIT_FAILURE;
                }

                for (auto file : files)
                {
                    std::string a;
                    a = file;
                    
                    if(a.find(".yaml") != -1)
                    {
                        a.erase(a.find(".yaml"));
                        ROS_INFO("%s, \n", a.c_str());
                        // res.data.data.push_back(a);
                        matrix_msgs::MapsInfo map_name_;
                        map_name_.name = a;
                        res.maps.push_back(map_name_);
                    }

                
                    
                }


                // matrix_msgs::LayoutInfo get_layouts_;
                web_interface_msgs::GuiMap getLayout;
                getLayout.request.cmd="list";
                web_interface_msgs::UIMap empty_member;
                getLayout.request.gui_maps.push_back(empty_member);
                std::string begin_track;
                if(ist_gui_layout_client.call(getLayout))
                {
                    for(auto layout : getLayout.response.gui_maps)
                    {
                        // if(layout.name == layout_name)
                        // {
                            ROS_INFO("[Matrix_map_list]: poi_list in layout_name: %s ----> ", layout.name.c_str());
                            // ROS_INFO("Show objects --> %s", layout.objects.c_str());

                            matrix_msgs::LayoutInfo get_layout_info_;

                            get_layout_info_.name = layout.name;

                            auto valid_text = layout.objects;

                            // std::cout << std::boolalpha
                            //           << json::accept(valid_text) << '\n';
                            
                            json layout_json = json::parse(valid_text);
                            // std::cout << std::setw(4) << second << "\n\n";

                            for(auto found_poi : layout_json)
                            {
                                if(found_poi["type"] == 28)
                                {
                                    std::string poi_name_;
                                    found_poi["text"].get_to(poi_name_);
                                    ROS_INFO("[Matrix_map_list]: poi_name found %s", poi_name_.c_str());
                                    get_layout_info_.poi.push_back(poi_name_);
                                }

                                // if(found_poi["type"] == 6)
                                // {
                                //     ROS_INFO("Found Dock");
                                //     std::string dock_name_;
                                //     found_poi["text"].get_to(dock_name_);
                                //     ROS_INFO("[Matrix_map_list]: dock_name found %s", dock_name_.c_str());
                                //     // get_layout_info_.dock.push_back(dock_name_);
                                // }
                                
                            }

                            // for(auto found_poi : layout_json)
                            // {
                            //     if(found_poi["type"] == 28)
                            //     {
                            //         std::string poi_name_;
                            //         found_poi["text"].get_to(poi_name_);
                            //         ROS_INFO("[Matrix_map_list]: poi_name found %s", poi_name_.c_str());
                            //         get_layout_info_.poi.push_back(poi_name_);
                            //     }
                                
                            // }

                        res.layouts.push_back(get_layout_info_);
                            
                            
                        // }
                    }
                }

                if(i == 0)
                {
                    res.maps.clear();
                    res.layouts.clear();
                }
            }
            

            



            // web_interface_msgs::GuiMap ist_layout;
            // ist_layout.request.cmd="get_default";
            // if(ist_gui_layout_client.call(ist_layout))
            // {
            //     ROS_INFO("size %d", ist_layout.response.gui_maps.size());
            //     // res.default_layout = ist_layout.response.gui_maps[0].name;
            //     // ROS_INFO("%s",ist_layout.response.gui_maps[0].name);
            // }
            
        }
        else if(req.cmd == matrix_msgs::ListMap::Request::pose_from_poi)
        {
            ROS_INFO("pose_from_poi!!");
            res.pose = getPosePOI(req.arg);
        }
        else if(req.cmd == matrix_msgs::ListMap::Request::pose_from_poi_invert)
        {
            ROS_INFO("pose_from_poi!!");
            res.pose = getPosePOI(req.arg);
            tf::Quaternion q(
                res.pose.orientation.x,
                res.pose.orientation.y,
                res.pose.orientation.z,
                res.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            float invert_yaw;
            if(yaw >= 0.00000)
            {
                invert_yaw = M_PI - yaw;
                invert_yaw *= -1; //opposite 
            }else
            {
                invert_yaw = M_PI + yaw;
                invert_yaw = abs(invert_yaw);
            }

            ROS_INFO("[Matrix_MapList]: yaw = %f, invert_yaw = %f", yaw, invert_yaw);

            q.setRPY(0,0,invert_yaw);
            q = q.normalize();

            res.pose.orientation.x = 0;
            res.pose.orientation.y = 0;
            res.pose.orientation.z = q[2];
            res.pose.orientation.w = q[3];
        }

        else if(req.cmd == matrix_msgs::ListMap::Request::poi_list)
        {
            // res.data = getPOIfromLyout(req.arg);
        }
        else if(req.cmd == matrix_msgs::ListMap::Request::layout_info)
        {
            // // matrix_msgs::LayoutInfo get_layouts_;
            // web_interface_msgs::GuiMap getLayout;
            // getLayout.request.cmd="list";
            // web_interface_msgs::UIMap empty_member;
            // getLayout.request.gui_maps.push_back(empty_member);
            // std::string begin_track;
            // if(ist_gui_layout_client.call(getLayout))
            // {
            //     for(auto layout : getLayout.response.gui_maps)
            //     {
            //         // if(layout.name == layout_name)
            //         // {
            //             ROS_INFO("[Matrix_map_list]: poi_list in layout_name: %s ----> ", layout.name.c_str());
            //             // ROS_INFO("Show objects --> %s", layout.objects.c_str());

            //             matrix_msgs::LayoutInfo get_layout_info_;

            //             get_layout_info_.name = layout.name;

            //             auto valid_text = layout.objects;

            //             // std::cout << std::boolalpha
            //             //           << json::accept(valid_text) << '\n';
                        
            //             json layout_json = json::parse(valid_text);
            //             // std::cout << std::setw(4) << second << "\n\n";

            //             for(auto found_poi : layout_json)
            //             {
            //                 if(found_poi["type"] == 28)
            //                 {
            //                     std::string poi_name_;
            //                     found_poi["text"].get_to(poi_name_);
            //                     ROS_INFO("[Matrix_map_list]: poi_name found %s", poi_name_.c_str());
            //                     get_layout_info_.poi.push_back(poi_name_);
            //                 }
                            
            //             }

            //            res.layouts.push_back(get_layout_info_);
                        
                        
            //         // }
            //     }
            // }
        }
        else {
            ;
        }

        ROS_INFO("[Matrix_map_list]: **************************** ");
        ROS_INFO(" ");
        return true;
    }

    geometry_msgs::Pose getPosePOI(std::string poi_name)
    {
        geometry_msgs::Pose target;
        // req.arg poi name request
        web_interface_msgs::Layout get_;
        get_.request.cmd="type";
        web_interface_msgs::UILayout layouts_;
        layouts_.type=28;
        get_.request.layouts.push_back(layouts_);
        if(ist_layout_info_client.call(get_))
        {
            for(auto poi_data : get_.response.layouts)
            {
                // ROS_INFO("poi list %s find %s", poi_data.name.c_str(), poi_name.c_str());
                if(poi_data.text == poi_name)
                {
                    // ROS_INFO("POI match!!");
                    target.position = poi_data.handles[0];
                    double dx = poi_data.handles[2].x - target.position.x;
                    double dy = poi_data.handles[2].y - target.position.y;

                    double ang = atan2(dy, dx);
                    double qz = sin(ang / 2.0);
                    double qw = cos(ang / 2.0);
                    target.orientation.z = qz;
                    target.orientation.w = qw; 
                    break;
                }
            }
        }
        return target; 
    }

    // matrix_msgs::StringArray getMapList()
    // {
    //     ;
    // }

    // matrix_msgs::StringArray getPOIfromLyout(std::string layout_name)
    // {
    //     matrix_msgs::StringArray return_data;
    //     web_interface_msgs::GuiMap getLayout;
    //     getLayout.request.cmd="list";
    //     web_interface_msgs::UIMap empty_member;
    //     getLayout.request.gui_maps.push_back(empty_member);
    //     std::string begin_track;
    //     if(ist_gui_layout_client.call(getLayout))
    //     {
    //         for(auto layout : getLayout.response.gui_maps)
    //         {
    //             if(layout.name == layout_name)
    //             {
    //                 ROS_INFO("[Matrix_map_list]: poi_list in layout_name: %s ----> ", layout.name.c_str());
    //                 // ROS_INFO("Show objects --> %s", layout.objects.c_str());

    //                 auto valid_text = layout.objects;

    //                 // std::cout << std::boolalpha
    //                 //           << json::accept(valid_text) << '\n';
                    
    //                 json second = json::parse(valid_text);
    //                 // std::cout << std::setw(4) << second << "\n\n";
                    
                    

    //                 for(auto found_poi : second)
    //                 {
    //                     if(found_poi["type"] == 28)
    //                     {
    //                         std::string v6;
    //                         found_poi["text"].get_to(v6);
    //                         ROS_INFO("[Matrix_map_list]: poi_name found %s", v6.c_str());
    //                         return_data.data.push_back(v6);
    //                     }
                        
    //                 }
                    
                    
    //             }
    //         }
    //     }

    //     return return_data;
    // }

    // void getLayoutList()
    // {
    //     web_interface_msgs::GuiMap ist_layout;
    //     ist_layout.request.cmd="list";
    //     if(ist_gui_layout_client.call(ist_layout))
    //     {
    //         ROS_INFO("no. of all layout %d", ist_layout.response.gui_maps.size());
    //         // res.default_layout = ist_layout.response.gui_maps[0].name;
    //         // ROS_INFO("%s",ist_layout.response.gui_maps[0].name);
    //     }
    // }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "matrix_map_list");
    ros::NodeHandle nh;
    MapList ml = MapList(&nh);
    ros::spin();
}