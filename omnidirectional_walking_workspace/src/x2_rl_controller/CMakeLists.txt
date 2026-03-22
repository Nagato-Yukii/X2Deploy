cmake_minimum_required(VERSION 3.8)
project(x2_rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 基础ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# ONNXRUNTIME
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
  set(ONNXRUNTIME_DIR "/agibot/data/home/agi/custom_workspace/onnxruntime-linux-aarch64-1.24.4")
else()
  set(ONNXRUNTIME_DIR "/home/suzumiyaharuhi/Desktop/onnxruntime-linux-x64-1.16.3")
endif()
message(STATUS "Selected ONNX Runtime path: ${ONNXRUNTIME_DIR}")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)


# 声明可执行文件
add_executable(rl_node 
  src/rl_node.cpp 
  src/mlp_policy.cpp
)

# 暴露头文件目录
target_include_directories(rl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接 ROS 2 依赖
ament_target_dependencies(rl_node
  rclcpp
  aimdk_msgs
  sensor_msgs
  ament_index_cpp
  tf2
  std_msgs
)

# 链接 ONNX Runtime 动态库
target_link_libraries(rl_node onnxruntime)

# ================= 安装规则 =================

# 将二进制可执行文件安装到 lib 目录
install(TARGETS rl_node
  DESTINATION lib/${PROJECT_NAME}
)

# 【极度致命的细节】
# 将 launch 脚本、yaml 参数、以及存放 policy.onnx 的 models 目录一并安装到 share 目录下！
# 如果你不把 models 安装过去，你的程序在 ros2 launch 时根本找不到神经网络权重！
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()