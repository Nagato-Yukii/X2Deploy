cmake_minimum_required(VERSION 3.8)
project(x2_rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找基础 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 2. 寻找 ONNX Runtime (极其关键！)
# 如果你在华硕笔记本或 RK3588 上遇到找不到的问题，
# 也可以用 include_directories() 和 link_directories() 强行指定下载好的 ONNX 路径
#find_package(onnxruntime REQUIRED)
set(ONNXRUNTIME_DIR "/home/suzumiyaharuhi/Desktop/onnxruntime-linux-x64-1.16.3")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)
# 3. 声明可执行文件 (注意：这里必须把两个 .cpp 文件都加进去编译！)
add_executable(rl_node 
  src/rl_node.cpp 
  src/mlp_policy.cpp
)

# 4. 暴露头文件目录
target_include_directories(rl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 5. 链接 ROS 2 依赖
ament_target_dependencies(rl_node
  rclcpp
  aimdk_msgs
  sensor_msgs
  ament_index_cpp
)

# 6. 链接 ONNX Runtime 动态库
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