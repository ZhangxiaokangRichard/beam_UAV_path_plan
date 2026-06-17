cmake_minimum_required(VERSION 3.0.2)
project(beam_dubins)

# C++17 标准，用于 std::optional, std::clamp 等
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# 查找 Eigen3（线性代数库，无 ROS 包装）
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# ── 自定义消息 ──────────────────────────────────────────────
add_message_files(
  FILES
  Obstacle.msg
  PathPoint.msg
)

# ── 自定义服务 ──────────────────────────────────────────────
add_service_files(
  FILES
  PlanPath.srv
)

# 生成消息和服务代码
generate_messages(
  DEPENDENCIES
  std_msgs
)

# ── 包配置 ──────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beam_dubins_core
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
  DEPENDS
    Eigen3
)

# ── 包含目录 ────────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 核心算法库（纯 C++，无 ROS 依赖）──────────────────────
add_library(beam_dubins_core STATIC
  src/geometry.cpp
  src/collision.cpp
  src/dubins_path.cpp
  src/primitive_gen.cpp
  src/beam_scorer.cpp
  src/beam_search.cpp
)

target_link_libraries(beam_dubins_core
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# ── 单元测试（依赖 gtest + 核心库）─────────────────────────
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # 添加 gtest 支持
  catkin_add_gtest(test_beam_search
    test/test_beam_search.cpp
  )

  target_link_libraries(test_beam_search
    beam_dubins_core
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
endif()

# ── planner_server 节点（ROS Service 封装，Phase 2a）───────
add_executable(planner_server
  src/planner_server.cpp
)

target_link_libraries(planner_server
  beam_dubins_core
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 确保消息头文件在 planner_server 编译前已生成
add_dependencies(planner_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
