cmake_minimum_required(VERSION 2.8.3)

project(sound_play)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp actionlib_msgs dynamic_reconfigure)

add_action_files(DIRECTORY action FILES SoundRequest.action)
add_message_files(DIRECTORY msg FILES SoundRequest.msg)
add_service_files(DIRECTORY srv FILES SoundMode.srv)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/SoundPlay.cfg
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs dynamic_reconfigure
               INCLUDE_DIRS include)

catkin_install_python(PROGRAMS
  scripts/playbuiltin.py
  scripts/play.py
  scripts/say.py
  scripts/shutup.py
  scripts/soundplay_node.py
  scripts/test.py
  scripts/test_actionlib_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



install(FILES
  soundplay_node.launch
  test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch sounds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# if(CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(scripts/test)
#   add_subdirectory(test)
# endif()
