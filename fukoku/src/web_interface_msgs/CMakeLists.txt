cmake_minimum_required(VERSION 3.0.2)
project(web_interface_msgs)


find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    actionlib_msgs
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
)

# msgs
add_message_files(
  FILES
    UILayout.msg
    UIMap.msg
    RobotOperate.msg
    RobotOperateList.msg
    Zones.msg
    MapData.msg
    IntersectPOI.msg
)

## Generate services in the 'srv' folder
 add_service_files(
  FILES
    WebCommand.srv
    GuiMap.srv
    #SetMap.srv
    Layout.srv
)


# actions
add_action_files(
  DIRECTORY
    action
  FILES
    WebCommand.action
    Docking.action
    RouteFollow.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs std_msgs message_runtime
)
