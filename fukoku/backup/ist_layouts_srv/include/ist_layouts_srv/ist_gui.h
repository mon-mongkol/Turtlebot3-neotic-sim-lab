#ifndef _IST_LAYOUTS_SRV__IST_GUI__H
#define _IST_LAYOUTS_SRV__IST_GUI__H
#include <map>

namespace ist_gui
{

    enum ObjectType
    {
        CanvasObject = 0,
        ImageMapClient = 1,
        Line = 2,
        PolyLine = 3,
        Rect = 4,
        Ellipse = 5,
        Docking = 6,
        Room = 7,
        Text = 8,
        NoEntry = 9,
        Door = 10,
        RouteLine = 11,
        RoutePolyLine = 12,
        Wall = 13,
        Ruler = 14,
        Mission = 15,
        Robot = 16,
        ZoneGreen = 17,
        ZoneYellow = 18,
        ZoneOrange = 19,
        RadiationPoints = 20,
        ImageList = 21,
        Triangle = 22,
        Range = 23,
        MotionSensor = 24,
        SensorOnOff = 25,
        CliffSensor = 26,
        LineArrow = 27,
        PointOfInterest = 28,
        RoadPolyLine = 29,

        ScreenObject = 30,
        ScreenRect = 31,
        ScreenText = 32,
        ScreenObjetProperties = 33,

    };

    typedef struct st_gui
    {
        int start_handle;
        int rotate_handle;

    } GUI_ST;

    extern std::map<ObjectType, GUI_ST> gui_maps;

    // = {
    //     {ObjectType::CanvasObject, {1, 0}},
    //     {ObjectType::ImageMapClient, {1, 0}},
    //     {ObjectType::Line, {1, 0}},
    //     {ObjectType::PolyLine, {1, -1}}, //points.length + start_handle
    //     {ObjectType::Rect, {1, 9}},
    //     {ObjectType::Ellipse, {1, 9}},
    //     {ObjectType::Docking, {1, 9}},
    //     {ObjectType::Door, {1, 9}},
    //     {ObjectType::Text, {1, 9}},
    //     {ObjectType::NoEntry, {1, 9}},
    //     {ObjectType::Room, {1, 9}},
    //     {ObjectType::RouteLine, {1, 0}},
    //     {ObjectType::RoutePolyLine, {1, -1}}, //points.length + start_handle
    //     {ObjectType::Wall, {1, 0}},
    //     {ObjectType::Ruler, {1, 0}},

    //     {ObjectType::Mission, {1, 9}},
    //     {ObjectType::Robot, {1, 9}},
    //     {ObjectType::ZoneGreen, {1, 9}},
    //     {ObjectType::ZoneYellow, {1, 9}},
    //     {ObjectType::ZoneOrange, {1, 9}},

    //     {ObjectType::RadiationPoints, {1, -1}}, //points.length + start_handle
    //     {ObjectType::ImageList, {1, 9}},
    //     {ObjectType::Triangle, {1, 9}},
    //     {ObjectType::Range, {1, 9}},
    //     {ObjectType::MotionSensor, {1, 9}},
    //     {ObjectType::SensorOnOff, {1, 9}},
    //     {ObjectType::CliffSensor, {1, 9}},
    //     {ObjectType::LineArrow, {1, 9}},
    //     {ObjectType::PointOfInterest, {1, 9}},

    //     {ObjectType::ScreenObject, {1, 0}},
    //     {ObjectType::ScreenRect, {1, 9}},
    //     {ObjectType::ScreenText, {1, 9}},
    //     {ObjectType::ScreenObjetProperties, {1, 9}},

    // };

}

#endif // _IST_LAYOUTS_SRV__IST_GUI__H