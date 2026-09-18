cmake_minimum_required(VERSION 3.8)
project(beam_dubins)

# ── C++17（与 1.0.1 一致）──────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# ── 接口（字段与 1.0.1 完全一致）──────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/PathPoint.msg"
  "srv/PlanPath.srv"
  DEPENDENCIES std_msgs
)

# ── 核心算法库：零 ROS 依赖（R1：文件内容与 1.0.1 逐字节一致）──
add_library(beam_dubins_core STATIC
  src/geometry.cpp
  src/collision.cpp
  src/dubins_path.cpp
  src/primitive_gen.cpp
  src/beam_scorer.cpp
  src/beam_search.cpp
)
target_include_directories(beam_dubins_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
# 注意：核心库**不**链接 rclcpp（保持零 ROS 依赖）

# ── planner_server：唯一 ROS 包装层 ───────────────────────────
add_executable(planner_server src/planner_server.cpp)
target_link_libraries(planner_server beam_dubins_core)
ament_target_dependencies(planner_server rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(planner_server "${cpp_typesupport_target}")

# ── 安装 ─────────────────────────────────────────────────────
install(TARGETS beam_dubins_core planner_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# ── 测试（断言与用例与 1.0.1 一致）────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 测试文件自带 main()（1.0.1 原样保留），故使用不带 gtest_main 的可执行目标
  ament_add_gtest_executable(test_beam_search test/test_beam_search.cpp)
  if(TARGET test_beam_search)
    target_link_libraries(test_beam_search beam_dubins_core gtest)
    add_test(NAME test_beam_search COMMAND test_beam_search)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(beam_dubins_core)
ament_export_dependencies(rclcpp std_msgs Eigen3)
ament_package()
