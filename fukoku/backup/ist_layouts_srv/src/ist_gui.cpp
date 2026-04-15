#include "ist_layouts_srv/ist_gui.h"
namespace ist_gui
{


   std::map<ObjectType, GUI_ST> gui_maps = {
        {ObjectType::CanvasObject, {1, 0}},
        {ObjectType::ImageMapClient, {1, 0}},
        {ObjectType::Line, {1, 0}},
        {ObjectType::PolyLine, {1, -1}}, //points.length + start_handle
        {ObjectType::Rect, {1, 9}},
        {ObjectType::Ellipse, {1, 9}},
        {ObjectType::Docking, {1, 9}},
        {ObjectType::Door, {1, 9}},
        {ObjectType::Text, {1, 9}},
        {ObjectType::NoEntry, {1, 9}},
        {ObjectType::Room, {1, 9}},
        {ObjectType::RouteLine, {1, 0}},
        {ObjectType::RoutePolyLine, {1, -1}}, //points.length + start_handle
        {ObjectType::Wall, {1, 0}},
        {ObjectType::Ruler, {1, 0}},

        {ObjectType::Mission, {1, 9}},
        {ObjectType::Robot, {1, 9}},
        {ObjectType::ZoneGreen, {1, 9}},
        {ObjectType::ZoneYellow, {1, 9}},
        {ObjectType::ZoneOrange, {1, 9}},

        {ObjectType::RadiationPoints, {1, -1}}, //points.length + start_handle
        {ObjectType::ImageList, {1, 9}},
        {ObjectType::Triangle, {1, 9}},
        {ObjectType::Range, {1, 9}},
        {ObjectType::MotionSensor, {1, 9}},
        {ObjectType::SensorOnOff, {1, 9}},
        {ObjectType::CliffSensor, {1, 9}},
        {ObjectType::LineArrow, {1, 9}},
        {ObjectType::PointOfInterest, {1, 9}},
        {ObjectType::RoadPolyLine, {1, -1}}, //points.length + start_handle

        {ObjectType::ScreenObject, {1, 0}},
        {ObjectType::ScreenRect, {1, 9}},
        {ObjectType::ScreenText, {1, 9}},
        {ObjectType::ScreenObjetProperties, {1, 9}},

    };

    
}