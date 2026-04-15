# cmake_minimum_required(VERSION 3.0.2)
# project(smr_msgs)

# find_package(catkin REQUIRED COMPONENTS
#   genmsg
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#   message_generation 
#   actionlib_msgs
# )

# # Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   IOs_buzzer.msg
#   IOs_prox.msg
#   IOs_lift_signal.msg
#   IOs_lift_state.msg
#   IOs_lift.msg
#   IOs_controller.msg
#   IOs.msg
#   IOs_btton_state.msg
#   IOs_mcu01_input.msg
#   IOs_mcu01_output.msg
#   IOs_mcu01.msg
#   BoxPose.msg
#   BoxCtrl.msg
#   RobotState.msg
#   Devices.msg
#   Error.msg
#   Diagnostics.msg
# )

# add_action_files(FILES
#   BoxOperation.action
# )

# # Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
#   CATKIN_DEPENDS
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#   message_runtime
# )

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(matrix_msgs)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  actionlib_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  BatteryStatus.msg
  State.msg
  IOs_buzzer.msg
  IOs_prox.msg
  IOs_lift_signal.msg
  IOs_lift_state.msg
  IOs_lift.msg
  IOs_controller.msg
  IOs.msg
  IOs_button_state.msg
  IOs_mcu_state.msg
  IOs_mcu01_input.msg
  IOs_mcu01_output.msg
  IOs_mcu01.msg
  Exchange_pose.msg
  BoxPose.msg
  BoxCtrl.msg
  RobotState.msg
  Devices.msg
  Error.msg
  Diagnostics.msg
  TM5_FeedbackState.msg
  TrayState.msg
  MatState.msg
  MatError.msg
  MovmentStatus.msg
  DeliveryDes.msg
  RelayLED.msg
  RobotMode.msg
  XbeeCommu.msg
  XbeeCommuROHM.msg
  MapLayoutSwap.msg
  StringArray.msg
  LayoutInfo.msg
  MapsInfo.msg
  ARTrack.msg
  DistanceAngle.msg
  Docking.msg
  IO_state.msg
  IOs_state.msg
  Date.msg
  ModernBoxPose.msg
  Diagnostics_modern.msg
  LEDState.msg
  LEDStateArray.msg
  Serial.msg
  PinStateDigital.msg
  DeviceStateCanopen.msg
  NodeRedStatus.msg
  IODescription.msg
  IO.msg
  DeviceStateCanopen_info.msg
  DeviceStateCanopens.msg
)

add_service_files(FILES
  TM5_SetEvent.srv
  TM5_SetIO.srv
  SetMatState.srv
  SetAudioLevel.srv
  ListMap.srv
  SerialCommu.srv
  SetIOs.srv
  ActionController.srv
  SetLEDs.srv
  SetIO.srv
  NodeRedStatus_srv.srv
)

add_action_files(FILES
  BoxOperation.action
  DeliveryOperation.action
  XbeeCommunication.action
  ROHMLiftOperation.action
  SwapEnviOperation.action
  SerialOperation.action
  ROHMChargerOperation.action
  Schedule.action
  ARDockingOperation.action
  ModernBoxOperation.action
  WallFollowing.action
  OfflineControlOperation.action
  CenterVShapeTracking.action
  IOOperation.action
  ARTrackOperation.action 
  ARDockingOperationMultiMethods.action
  SHLOperation.action
  CommonCommand.action
)

generate_messages(
  DEPENDENCIES
  diagnostic_msgs
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)


################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs diagnostic_msgs geometry_msgs sensor_msgs message_runtime actionlib_msgs
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################