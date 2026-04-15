cmake_minimum_required(VERSION 3.0.2)
project(aragorn_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  diagnostic_msgs
  sensor_msgs
  nav_msgs
  # deligreencs_bms
  aragorn_msg
  roscpp
  rospy
)

catkin_python_setup()


add_message_files(
  FILES
  Cautions.msg
  DiagPort.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)




catkin_package(
#   INCLUDE_DIRS include
   # CATKIN_DEPENDS aragorn_msg deligreencs_bms message_runtime std_msgs sensor_msgs diagnostic_msgs roscpp rospy
   CATKIN_DEPENDS aragorn_msg message_runtime std_msgs sensor_msgs diagnostic_msgs roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aragorn_diagnostics src/aragorn_diagnostics.cpp)
add_dependencies(aragorn_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aragorn_diagnostics ${catkin_LIBRARIES})

# add_executable(aragorn_diagnosticsMo src/aragorn_diagnosticsMo.cpp)
# add_dependencies(aragorn_diagnosticsMo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(aragorn_diagnosticsMo ${catkin_LIBRARIES})

install(TARGETS aragorn_diagnostics 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# add_executable(aragorn_diagnostics_all src/aragorn_diagnostics_all.cpp)
# add_dependencies(aragorn_diagnostics_all ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(aragorn_diagnostics_all ${catkin_LIBRARIES})

# install(TARGETS aragorn_diagnostics_all
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



catkin_install_python(PROGRAMS
  script/startup_robot
  script/battery_logger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)






install(DIRECTORY launch param  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
