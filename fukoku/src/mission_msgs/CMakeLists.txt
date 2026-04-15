cmake_minimum_required(VERSION 3.0.2)
project(mission_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    actionlib_msgs
    geometry_msgs
    roscpp
    std_msgs
)

add_message_files(
  FILES
    MissionRunning.msg
    MissionRoom.msg
    MissionInfo.msg
    RequestInfo.msg
    MovePoint.msg
    Rooms.msg
    OperatePoint.msg
    DisinfectionInfo.msg
    UVCLocationOptions.msg
    UVCMovementOptions.msg
    UVCOperateOptions.msg
    RobotOptions.msg
    MoveTarget.msg
    BatteryStamp.msg
    RobotOperation.msg
    RobotTask.msg
    UserConfirm.msg
    UVCTime.msg
    TrayInfo.msg
    DeliveryInfo.msg
    DeliveryRequest.msg
    SMROpInfo.msg
    SMRTrayInfo.msg
    SMRDeliveryRequest.msg
    SMRDeliveryInfo.msg
)

add_service_files(
  FILES
    Mission.srv
    Operate.srv
    Query.srv
)

add_action_files(
  DIRECTORY action
  FILES
  MissionRequest.action
  MissionRun.action
  MissionOperate.action
  MissionAutoRun.action
  MissionDelivery.action
  MissionSMRDelivery.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs 
    geometry_msgs 
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    geometry_msgs
    std_msgs
)


