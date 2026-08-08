cmake_minimum_required(VERSION 3.0.2)
project(ros_udp_bridge)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_udp_bridge_core
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 核心库（自包含：geodesy / udp_socket / udp_codec）
add_library(ros_udp_bridge_core
  src/geodesy.cpp
  src/udp_socket.cpp
  src/udp_codec.cpp
)
target_link_libraries(ros_udp_bridge_core ${catkin_LIBRARIES})

# udp_receiver 节点（阶段 1-3：收包 / uav 初始化 / target 卡尔曼）
add_executable(udp_receiver src/udp_receiver.cpp)
target_link_libraries(udp_receiver ros_udp_bridge_core ${catkin_LIBRARIES})
add_dependencies(udp_receiver ${catkin_EXPORTED_TARGETS})

# udp_sender 节点（阶段 6：uav/state → red_interceptor → UDP 发送）
add_executable(udp_sender src/udp_sender.cpp)
target_link_libraries(udp_sender ros_udp_bridge_core ${catkin_LIBRARIES})
add_dependencies(udp_sender ${catkin_EXPORTED_TARGETS})

# udp_mssn_bridge 节点（服务入口：本地 11302 指令 → 启停 launch / 切换拦截模式）
add_executable(udp_mssn_bridge src/udp_mssn_bridge.cpp)
target_link_libraries(udp_mssn_bridge ros_udp_bridge_core ${catkin_LIBRARIES})
add_dependencies(udp_mssn_bridge ${catkin_EXPORTED_TARGETS})
