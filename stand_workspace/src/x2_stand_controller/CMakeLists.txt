cmake_minimum_required(VERSION 3.8)
project(x2_stand_controller)

# 开启现代 C++ 编译警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找必要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)

# 1. 声明可执行文件
add_executable(stand_node src/stand_node.cpp)

# 2. 【极其关键】告诉编译器去哪里找你的 stand_node.hpp
# 注意：这一步必须放在 add_executable 之后！
target_include_directories(stand_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. 链接 ROS 2 和自定义消息的依赖库
ament_target_dependencies(stand_node
  rclcpp
  aimdk_msgs
)

# 安装规则：可执行文件
install(TARGETS stand_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装规则：启动脚本和 YAML 参数配置
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()