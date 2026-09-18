cmake_minimum_required(VERSION 3.8)
project(uav_guide)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(beam_dubins REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── 接口 ──────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UavPlannedPath.msg"
  "msg/Setpoint.msg"
  "msg/Guidance.msg"
  DEPENDENCIES std_msgs beam_dubins
)

# ── 算法层：**零 ROS 依赖**（geo/path/setpoint/guidance/orchestrator）──
add_library(uav_guide_algo STATIC
  src/geo.cpp
  src/path.cpp
  src/setpoint.cpp
  src/guidance.cpp
  src/orchestrator.cpp
)
target_include_directories(uav_guide_algo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── 节点复用层：ROS ↔ 算法结构体适配（三节点共用）──
add_library(uav_guide_ros_utils STATIC src/ros_utils.cpp)
target_link_libraries(uav_guide_ros_utils uav_guide_algo)
ament_target_dependencies(uav_guide_ros_utils rclcpp std_msgs nav_msgs beam_dubins)

rosidl_get_typesupport_target(ug_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(uav_guide_ros_utils "${ug_typesupport}")

# ── 三个节点（全权分布式，由一个 launch 拉起）──
set(UAV_GUIDE_NODES
  uav_guide_loop_node
  uav_guide_point_node
  uav_guidance_node
)
foreach(node IN LISTS UAV_GUIDE_NODES)
  add_executable(${node} src/nodes/${node}.cpp)
  target_link_libraries(${node} uav_guide_algo uav_guide_ros_utils)
  ament_target_dependencies(${node} rclcpp std_msgs nav_msgs tf2 tf2_ros beam_dubins)
  target_link_libraries(${node} "${ug_typesupport}")
endforeach()

# ── 安装 ─────────────────────────────────────────────────────
install(TARGETS uav_guide_algo uav_guide_ros_utils ${UAV_GUIDE_NODES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# ── 测试（算法层零 ROS 单测）──────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uav_guide test/test_uav_guide.cpp)
  if(TARGET test_uav_guide)
    target_link_libraries(test_uav_guide uav_guide_algo)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(uav_guide_algo uav_guide_ros_utils)
ament_export_dependencies(rclcpp std_msgs nav_msgs tf2 tf2_ros beam_dubins)
ament_package()
