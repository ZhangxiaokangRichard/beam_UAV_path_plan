cmake_minimum_required(VERSION 3.0.2)
project(uav_guide_env)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  visualization_msgs
  tf
  beam_dubins          # 依赖 Phase 1-2a 的算法包
)

# 包配置
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    visualization_msgs
    tf
    beam_dubins
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 环境模型库（静态库）───────────────────────────────────
add_library(uav_guide_env_core STATIC
  src/uav_model.cpp
  src/target_trajectory.cpp
  src/obstacle_manager.cpp
)

target_link_libraries(uav_guide_env_core
  ${catkin_LIBRARIES}
)

# 确保依赖的消息已生成
add_dependencies(uav_guide_env_core
  beam_dubins_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# ── simulation_loop 节点（Phase 3 v0.1）─────────────────────
add_executable(simulation_loop
  src/simulation_loop.cpp
)
target_link_libraries(simulation_loop
  uav_guide_env_core
  ${catkin_LIBRARIES}
)
add_dependencies(simulation_loop
  beam_dubins_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# ── target_publisher 节点（Phase 3 v0.1 新增）───────────────
add_executable(target_publisher
  src/target_publisher.cpp
)
target_link_libraries(target_publisher
  uav_guide_env_core
  ${catkin_LIBRARIES}
)

# ── rviz_publisher 节点（Phase 3）───────────────────────────
add_executable(rviz_publisher
  src/rviz_publisher.cpp
)
target_link_libraries(rviz_publisher
  uav_guide_env_core
  ${catkin_LIBRARIES}
)
