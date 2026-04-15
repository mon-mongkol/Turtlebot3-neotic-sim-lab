#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ist_layouts_srv/ist_layout_interface.h"
#include "clipper.hpp"

using namespace ClipperLib;
using namespace Eigen;
const int point_scale = 10000;

namespace ist_gui
{
    GuiInterface::GuiInterface(const std::string &name)
        : package_(name)
    {
    }

    Paths getUIPaths(const web_interface_msgs::UILayout &ly)
    {
        Path poly;
        for (const geometry_msgs::Point &p : ly.points)
        {
            poly.push_back(IntPoint(p.x * point_scale, p.y * point_scale));
        }
        Paths paths_(1);
        paths_.push_back(poly);

        return paths_;
    }

    // https://stackoverflow.com/questions/38382777/how-do-i-determine-if-two-polygons-intersect-using-clipper
    int Intersects(const Paths &subj, const Paths &clip)
    {
        ClipperLib::Clipper c;

        c.AddPaths(subj, ClipperLib::ptSubject, true);
        c.AddPaths(clip, ClipperLib::ptClip, true);

        ClipperLib::Paths solution;
        c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        return solution.size();
    }

    std::vector<web_interface_msgs::UILayout> GuiInterface::getDoors(std::vector<web_interface_msgs::UILayout> doorList, web_interface_msgs::UILayout room)
    {
        std::vector<web_interface_msgs::UILayout> doors;
        auto room_path = getUIPaths(room);
        std::copy_if(
            doorList.begin(),
            doorList.end(),
            std::back_inserter(doors),
            [this, room_path](const auto &item)
            {
                auto door_path = getUIPaths(item);
                return Intersects(room_path, door_path);
            });
        return doors;
    }

    void GuiInterface::getMissionToDoors(mission_msgs::MissionRoom *mission, std::vector<web_interface_msgs::UILayout> doorList, web_interface_msgs::UILayout room)
    {

        auto layout_in = std::find_if(
            doorList.begin(), doorList.end(),
            [&](const auto &obj)
            { return obj.type == ist_gui::ObjectType::Door && obj.uuid == mission->ref_in_uuid; });
        auto layout_out = std::find_if(
            doorList.begin(), doorList.end(),
            [&](const auto &obj)
            { return obj.type == ist_gui::ObjectType::Door && obj.uuid == mission->ref_out_uuid; });

        if (layout_in != doorList.end() && layout_out != doorList.end())
        {

            // ROS_INFO("ref_in_handle:%d", mr.ref_in_handle);
            {
                mission->inPose.position = layout_in->handles[mission->ref_in_handle];
                auto ct = layout_in->handles[0];
                double dx = ct.x - mission->inPose.position.x;
                double dy = ct.y - mission->inPose.position.y;

                double ang = atan2(dy, dx);
                double qz = sin(ang / 2.0);
                double qw = cos(ang / 2.0);
                mission->inPose.orientation.z = qz;
                mission->inPose.orientation.w = qw;

                // if (mission->ref_in_handle == 4)
                // {
                //     manual_in.target.position = layout_in->handles[8];
                // }
                // else
                // {
                //     manual_in.target.position = layout_in->handles[4];
                // }
                // manual_in.target.orientation.z = qz;
                // manual_in.target.orientation.w = qw;
            }

            // ROS_INFO("ref_out_handle:%d", mr.ref_out_handle);
            {
                mission->outPose.position = layout_in->handles[mission->ref_out_handle];
                auto ct = layout_in->handles[0];

                double dx = ct.x - mission->outPose.position.x;
                double dy = ct.y - mission->outPose.position.y;

                double ang = atan2(dy, dx);
                double qz = sin(ang / 2.0);
                double qw = cos(ang / 2.0);
                mission->outPose.orientation.z = qz;
                mission->outPose.orientation.w = qw;

                // if (mission->ref_out_handle == 4)
                // {
                //     manual_out.target.position = layout_in->handles[8];
                // }
                // else
                // {
                //     manual_out.target.position = layout_in->handles[4];
                // }
                // manual_out.target.orientation.z = qz;
                // manual_out.target.orientation.w = qw;
            }

            mission->inPoseInv = mission->outPose.orientation;
            mission->outPoseInv = mission->inPose.orientation;
        }
        else
        {
            std::vector<web_interface_msgs::UILayout> doors;
            auto room_path = getUIPaths(room);
            std::copy_if(
                doorList.begin(),
                doorList.end(),
                std::back_inserter(doors),
                [this, room_path](const auto &item)
                {
                    auto door_path = getUIPaths(item);
                    return Intersects(room_path, door_path);
                });

            for (const auto &door : doors)
            {
                auto ct = door.handles[0];
                IntPoint pt = ClipperLib::IntPoint(door.handles[8].x * point_scale, door.handles[8].y * point_scale);
                int ret = ClipperLib::PointInPolygon(pt, room_path[0]);
                mission->ref_in_handle = (ret == 0) ? 8 : 4; // outside room
                mission->ref_in_uuid = door.uuid;
                mission->ref_out_uuid = door.uuid;
                mission->ref_out_handle = (mission->ref_in_handle == 8) ? 4 : 8;

                mission->inPose.position = door.handles[mission->ref_in_handle];

                double dx = ct.x - mission->inPose.position.x;
                double dy = ct.y - mission->inPose.position.y;

                double ang = atan2(dy, dx);
                double qz = sin(ang / 2.0);
                double qw = cos(ang / 2.0);
                mission->inPose.orientation.z = qz;
                mission->inPose.orientation.w = qw;

                mission->outPose.position = door.handles[mission->ref_out_handle];

                dx = ct.x - mission->outPose.position.x;
                dy = ct.y - mission->outPose.position.y;

                ang = atan2(dy, dx);
                qz = sin(ang / 2.0);
                qw = cos(ang / 2.0);
                mission->outPose.orientation.z = qz;
                mission->outPose.orientation.w = qw;

                mission->inPoseInv = mission->outPose.orientation;
                mission->outPoseInv = mission->inPose.orientation;
            }
        }

        mission->name = room.text;
        mission->rect[0] = room.points[0];
        mission->rect[1] = room.points[1];
        mission->rect[2] = room.points[2];
        mission->rect[3] = room.points[3];
    }

    std::vector<int> GuiInterface::getRadiationTimes(web_interface_msgs::UILayout layout_radiation)
    {
        std::vector<int> times;
        auto gui_layouts = json::parse(layout_radiation.objects);
        if (gui_layouts.contains("times"))
        {
            times = gui_layouts["times"].get<std::vector<int>>();
        }
        return times;
    }

    // C++ program to evaluate area of a polygon using
    // shoelace formula
    // https://en.wikipedia.org/wiki/Shoelace_formula
    // (X[i], Y[i]) are coordinates of i'th point.
    double GuiInterface::polygonArea(std::vector<geometry_msgs::Point> pts)
    {
        // Initialize area std::vector<geometry_msgs::Point

        double area = 0.0;

        // Calculate value of shoelace formula
        int n = pts.size();
        int j = n - 1;
        for (int i = 0; i < n; i++)
        {
            area += (pts[j].x + pts[i].x) * (pts[j].y - pts[i].y);
            j = i; // j is previous vertex to i
        }

        // Return absolute value
        return abs(area / 2.0);
    }

    void GuiInterface::showLayout(const web_interface_msgs::UILayout &mp)
    {
        ROS_INFO("Layout: uuid=%s, text=%s, name=%s, type=%d", mp.uuid.c_str(), mp.text.c_str(), mp.name.c_str(), (int)mp.type);
        ROS_INFO("startHandle=%d, rotateHandle=%d", mp.startHandle, mp.rotateHandle);
        int i = 0;
        for (const geometry_msgs::Point &p : mp.points)
        {
            ROS_INFO("[%d]:x=%f, y=%f, z=%f", i++, p.x, p.y, p.z);
        }
    }

    void GuiInterface::showMission(const mission_msgs::MissionRoom &mp)
    {
        ROS_INFO_STREAM(package_ << ":");
        ROS_INFO("mission: uuid=%s, number=%d, name=%s", mp.uuid.c_str(), (int)mp.number, mp.name.c_str());
        ROS_INFO_STREAM("ref_in_uuid:" << mp.ref_in_uuid);
        ROS_INFO_STREAM("ref_in_handle:" << std::to_string(mp.ref_in_handle));
        ROS_INFO_STREAM("ref_out_uuid:" << mp.ref_out_uuid);
        ROS_INFO_STREAM("ref_out_handle:" << mp.ref_out_handle);

        ROS_INFO("inPose:x=%f, y=%f, rot z=%f, w=%f", mp.inPose.position.x, mp.inPose.position.y, mp.inPose.orientation.z, mp.inPose.orientation.w);
        ROS_INFO("outPose:x=%f, y=%f,rot z=%f, w=%f", mp.outPose.position.x, mp.outPose.position.y, mp.outPose.orientation.z, mp.outPose.orientation.w);
        ROS_INFO("status:%ld", mp.status);
        ROS_INFO("doors:%ld", mp.doors);
        ROS_INFO("checkList:%ld", mp.checkList);
        ROS_INFO("interlock:%d", mp.interlock);
        ROS_INFO("result:%d", mp.result);
        ROS_INFO("Rect:");
        ROS_INFO("[0]:x=%f, y=%f, z=%f", mp.rect[0].x, mp.rect[0].y, mp.rect[0].z);
        ROS_INFO("[1]:x=%f, y=%f, z=%f", mp.rect[1].x, mp.rect[1].y, mp.rect[1].z);
        ROS_INFO("[2]:x=%f, y=%f, z=%f", mp.rect[2].x, mp.rect[2].y, mp.rect[2].z);
        ROS_INFO("[3]:x=%f, y=%f, z=%f", mp.rect[3].x, mp.rect[3].y, mp.rect[3].z);
    }

    std::vector<geometry_msgs::Point> GuiInterface::toWorldPoints(geometry_msgs::Point p, geometry_msgs::Point rotate_point, std::vector<geometry_msgs::Point> points)
    {
        std::vector<geometry_msgs::Point> world_points;

        // if (ui->rot != 0)
        // {
        //     Eigen::Affine3d
        // }

        // geometry_msgs::Point ros_pos = toROS(wp);

        return world_points;
    }

    geometry_msgs::Point GuiInterface::toROS(geometry_msgs::Point p, nav_msgs::MapMetaData meta_data_message)
    {
        geometry_msgs::Point rp;
        rp.x = p.x * meta_data_message.resolution + meta_data_message.origin.position.x;
        rp.y = -(p.y - meta_data_message.height) * meta_data_message.resolution + meta_data_message.origin.position.y;
        return rp;
    }

    std::vector<web_interface_msgs::UILayout> GuiInterface::toUILayout(const std::string &objects, nav_msgs::MapMetaData meta_data_message, bool debug)
    {
        std::vector<web_interface_msgs::UILayout> layouts;
        auto gui_layouts = json::parse(objects);
        if (debug)
            ROS_INFO_STREAM(package_ << ":" << gui_layouts.dump(4));

        for (auto &el : gui_layouts)
        {
            // std::cout << el << "\n";
            //  std::cout << "\n";
            //  for (auto &it : el.items())
            //  {
            //      std::cout << it.key() << " : " << it.value() << "\n";
            //  }
            web_interface_msgs::UILayout ui;
            geometry_msgs::Point objPos;
            Eigen::Transform<float, 3, Eigen::Affine> t;
            ui.objects = el.dump();

            if (el.contains("uuid"))
                ui.uuid = el["uuid"].get<std::string>();

            if (el.contains("name"))
                ui.name = el["name"].get<std::string>();

            if (el.contains("text"))
                ui.text = el["text"].get<std::string>();

            if (el.contains("type"))
                ui.type = el["type"].get<int>();

            ist_gui::ObjectType ui_type = (ist_gui::ObjectType)ui.type;
            if (ist_gui::gui_maps.count(ui_type))
            {
                auto ck_ui = ist_gui::gui_maps[ui_type];
                ui.startHandle = ck_ui.start_handle;
                ui.rotateHandle = ck_ui.rotate_handle;
            }

            if (el.contains("rot"))
                ui.rot = el["rot"].get<float>();

            if (el.contains("scale"))
                ui.scale = el["scale"].get<float>();

            if (el.contains("pos"))
            {
                objPos.x = el["pos"]["x"].get<float>();
                objPos.y = el["pos"]["y"].get<float>();
            }

            ui.pos = toROS(objPos, meta_data_message);

            if (el.contains("points"))
            {

                std::vector<geometry_msgs::Point> points;

                float sx = 0;
                float sy = 0;
                for (auto &it : el["points"])
                {
                    geometry_msgs::Point p;
                    p.x = it["x"].get<float>();
                    p.y = it["y"].get<float>();
                    sx += p.x;
                    sy += p.y;
                    points.push_back(p);
                }

                Eigen::Vector3f tran = Eigen::Vector3f(objPos.x, objPos.y, 0);
                float total_points = points.size();
                Eigen::Vector3f rotate_point = Eigen::Vector3f(sx / total_points, sy / total_points, 0);

                if (ui.rot != 0.0)
                {
                    t = Eigen::Translation3f(tran) * Eigen::Translation3f(rotate_point) * Eigen::AngleAxisf(ui.rot, Eigen::Vector3f::UnitZ()) * Eigen::Translation3f(-rotate_point) * Eigen::Scaling(ui.scale);
                }
                else
                {
                    t = Eigen::Translation3f(tran) * Eigen::Scaling(ui.scale);
                }

                for (auto &p : points)
                {
                    Eigen::Vector3f pos = Eigen::Vector3f(p.x, p.y, 0);
                    Eigen::Vector3f res = t * pos;
                    geometry_msgs::Point wp;
                    wp.x = res[0];
                    wp.y = res[1];

                    ui.points.push_back(toROS(wp, meta_data_message));
                }
            }

            if (el.contains("handles"))
            {
                if (ui.rotateHandle < 0)
                {
                    ui.rotateHandle = el["handles"].size() - ui.startHandle;
                }

                for (auto &it : el["handles"])
                {

                    geometry_msgs::Point p;
                    p.x = it["x"].get<float>();
                    p.y = it["y"].get<float>();

                    Eigen::Vector3f pos = Eigen::Vector3f(p.x, p.y, 0);
                    Eigen::Vector3f res = t * pos;
                    geometry_msgs::Point wp;
                    wp.x = res[0];
                    wp.y = res[1];

                    ui.handles.push_back(toROS(wp, meta_data_message));
                }
            }
            if (debug)
            {
                for (int i = 0; i < ui.points.size(); i++)
                {
                    ROS_INFO_STREAM("p[" << std::to_string(i) << "] x=" << std::to_string(ui.points[i].x) << ",y=" << std::to_string(ui.points[i].y));
                }

                for (int i = 0; i < ui.handles.size(); i++)
                {
                    ROS_INFO_STREAM("h[" << std::to_string(i) << "] x=" << std::to_string(ui.handles[i].x) << ",y=" << std::to_string(ui.handles[i].y));
                }
            }

            layouts.push_back(ui);
            // std::cout << '\n';
        }

        return layouts;
    }

    web_interface_msgs::Layout GuiInterface::convertToRos(std::vector<web_interface_msgs::UIMap> uiMapList, nav_msgs::MapMetaData meta_data_message)
    {

        web_interface_msgs::Layout layout;
        try
        {

            layout.request.layouts.clear();

            for (auto &msg : uiMapList)
            {
                if (msg.objects != "")
                {
                    layout.request.layouts.clear();
                    ROS_INFO_STREAM(package_
                                    << ":Start convert GUI Layout to ROS");

                    auto uiList = toUILayout(msg.objects, meta_data_message);
                    layout.request.layouts.insert(layout.request.layouts.end(), uiList.begin(), uiList.end());
                }
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM(package_
                             << ":Layout Callback exception:"
                             << e.what());
        }

        return layout;
    }

    void GuiInterface::getMissionRoom(mission_msgs::MissionRoom *mission, mission_msgs::MoveTarget *manual_in, mission_msgs::MoveTarget *manual_out, const std::vector<web_interface_msgs::UILayout> &guiList, web_interface_msgs::UILayout *layout_radiation)
    {

        // for (auto it = guiList.begin(); it != guiList.end(); ++it)
        // {
        //     if ((*it).type == ist_gui::ObjectType::Room || (*it).type == ist_gui::ObjectType::Door || (*it).type == ist_gui::ObjectType::RadiationPoints)
        //     {
        //         showLayout(*it);
        //     }
        // }

        auto it_room = std::find_if(
            guiList.begin(), guiList.end(),
            [&](const auto &obj)
            { return obj.type == ist_gui::ObjectType::Room && obj.uuid == mission->refUUID; });

        if (it_room != guiList.end())
        {
            ROS_INFO_STREAM(package_ << ":room ->" << it_room->text);

            mission->name = it_room->text;
            if (it_room->points.size() < 4)
            {
                throw std::runtime_error(package_ + ":room " + it_room->text + " layouts error! , please create new room layout and try again.");
            }
            mission->rect[0] = it_room->points[0];
            mission->rect[1] = it_room->points[1];
            mission->rect[2] = it_room->points[2];
            mission->rect[3] = it_room->points[3];

            auto layout_in = std::find_if(
                guiList.begin(), guiList.end(),
                [&](const auto &obj)
                { return obj.type == ist_gui::ObjectType::Door && obj.uuid == mission->ref_in_uuid; });

            auto layout_out = std::find_if(
                guiList.begin(), guiList.end(),
                [&](const auto &obj)
                { return obj.type == ist_gui::ObjectType::Door && obj.uuid == mission->ref_out_uuid; });

            if (layout_in != guiList.end())
            {
                ROS_INFO_STREAM(package_ << ":layout_in ->" << layout_in->uuid);
                mission->inPose.position = layout_in->handles[mission->ref_in_handle];
                auto ct = layout_in->handles[0];
                double dx = ct.x - mission->inPose.position.x;
                double dy = ct.y - mission->inPose.position.y;

                double ang = atan2(dy, dx);
                double qz = sin(ang / 2.0);
                double qw = cos(ang / 2.0);
                mission->inPose.orientation.z = qz;
                mission->inPose.orientation.w = qw;

                if (mission->ref_in_handle == 4)
                {
                    manual_in->target.position = layout_in->handles[8];
                }
                else
                {
                    manual_in->target.position = layout_in->handles[4];
                }
                manual_in->target.orientation.z = qz;
                manual_in->target.orientation.w = qw;
            }
            else
            {
                ROS_ERROR_STREAM(package_ << ":Not found layout_in ->" << mission->ref_in_uuid);
            }

            if (layout_out != guiList.end())
            {
                ROS_INFO_STREAM(package_ << ":layout_out ->" << layout_out->uuid);
                mission->outPose.position = layout_out->handles[mission->ref_out_handle];
                auto ct = layout_out->handles[0];

                double dx = ct.x - mission->outPose.position.x;
                double dy = ct.y - mission->outPose.position.y;

                double ang = atan2(dy, dx);
                double qz = sin(ang / 2.0);
                double qw = cos(ang / 2.0);
                mission->outPose.orientation.z = qz;
                mission->outPose.orientation.w = qw;

                if (mission->ref_out_handle == 4)
                {
                    manual_out->target.position = layout_out->handles[8];
                }
                else
                {
                    manual_out->target.position = layout_out->handles[4];
                }
                manual_out->target.orientation.z = qz;
                manual_out->target.orientation.w = qw;
            }
            else
            {
                ROS_ERROR_STREAM(package_ << ":Not found layout_out ->" << mission->ref_out_uuid);
            }

            if (layout_radiation)
            {
                auto it_ref_point = std::find_if(
                    guiList.begin(), guiList.end(),
                    [&](const auto &obj)
                    { return obj.type == ist_gui::ObjectType::RadiationPoints && obj.uuid == mission->refUUID; });

                if (it_ref_point != guiList.end())
                {
                    *layout_radiation = *it_ref_point;
                }
                else
                {
                    ROS_ERROR_STREAM(package_ << ":Not found layout_radiation ->" << mission->refUUID);
                }
            }
        }
        else
        {
            ROS_ERROR_STREAM(package_ << ":Not found room ->" << mission->refUUID);
        }
    }

}