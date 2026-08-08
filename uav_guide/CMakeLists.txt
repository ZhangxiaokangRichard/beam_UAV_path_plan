cmake_minimum_required(VERSION 3.0.2)
project(uav_guide)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  beam_dubins
)

add_message_files(
  FILES
  UavPlannedPath.msg
  UavCmd.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  beam_dubins
)

add_dependencies(${PROJECT_NAME}_generate_messages beam_dubins_generate_messages)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs beam_dubins
)

catkin_install_python(PROGRAMS
  scripts/uav_guide_loop_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
