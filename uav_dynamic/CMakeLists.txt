cmake_minimum_required(VERSION 3.0.2)
project(uav_dynamic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  beam_dubins
)

add_service_files(
  FILES
  UavDynamicsSolve.srv
)

generate_messages(
  DEPENDENCIES
  beam_dubins
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_dynamic_core
  CATKIN_DEPENDS roscpp std_msgs message_runtime beam_dubins
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 核心算法库：DWA 水平跟踪 + 协调转弯 + 纵向 PI
add_library(uav_dynamic_core
  src/dwa_tracker.cpp
)
target_link_libraries(uav_dynamic_core ${catkin_LIBRARIES})

# 服务节点
add_executable(uav_dynamics_server src/uav_dynamics_server.cpp)
target_link_libraries(uav_dynamics_server uav_dynamic_core ${catkin_LIBRARIES})
add_dependencies(uav_dynamics_server
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
