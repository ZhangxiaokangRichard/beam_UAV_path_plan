cmake_minimum_required(VERSION 3.8)
project(ros_mqtt_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uav_guide REQUIRED)

# libmosquitto（系统库）
find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h REQUIRED)
find_library(MOSQUITTO_LIBRARY mosquitto REQUIRED)

# ── 库层：**零 ROS 依赖**（与 1.0.1 保持一致）──
add_library(ros_mqtt_bridge_core STATIC
  src/mqtt_client.cpp
  src/geodesy.cpp
  src/json_codec.cpp
  src/mode_gate.cpp
)
target_include_directories(ros_mqtt_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOSQUITTO_INCLUDE_DIR}
)
target_link_libraries(ros_mqtt_bridge_core ${MOSQUITTO_LIBRARY})

# ── 节点 ─────────────────────────────────────────────────────

# MQTT → ROS 位姿桥（P2）
add_executable(mqtt_target_bridge src/mqtt_target_bridge.cpp)
target_link_libraries(mqtt_target_bridge ros_mqtt_bridge_core)
ament_target_dependencies(mqtt_target_bridge rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

# ROS 制导 → MQTT 出站（P6，按模式截断）
add_executable(mqtt_control_bridge src/mqtt_control_bridge.cpp)
target_link_libraries(mqtt_control_bridge ros_mqtt_bridge_core)
ament_target_dependencies(mqtt_control_bridge rclcpp std_msgs nav_msgs uav_guide)

# aoa/ai_guide/* 入站：模式 + 启停（P6）
add_executable(mqtt_mssn_bridge src/mqtt_mssn_bridge.cpp)
target_link_libraries(mqtt_mssn_bridge ros_mqtt_bridge_core)
ament_target_dependencies(mqtt_mssn_bridge rclcpp std_msgs)

install(TARGETS ros_mqtt_bridge_core mqtt_target_bridge mqtt_control_bridge mqtt_mssn_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_json_codec test/test_json_codec.cpp)
  if(TARGET test_json_codec)
    target_link_libraries(test_json_codec ros_mqtt_bridge_core)
  endif()

  ament_add_gtest(test_mode_gate test/test_mode_gate.cpp)
  if(TARGET test_mode_gate)
    target_link_libraries(test_mode_gate ros_mqtt_bridge_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(ros_mqtt_bridge_core)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros)
ament_package()
