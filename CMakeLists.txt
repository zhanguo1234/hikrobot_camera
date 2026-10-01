cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ========== 关键：提前链接海康库 ==========
# 告诉编译器去哪找海康的头文件
include_directories(SYSTEM /opt/MVS/include)

# ---------- 假图节点（无相机时的冒烟测试，保留）----------
add_executable(fake_camera_publisher src/fake_camera_publisher.cpp)
ament_target_dependencies(fake_camera_publisher rclcpp sensor_msgs)
target_link_libraries(fake_camera_publisher /opt/MVS/lib/64/libMvCameraControl.so)

# ---------- 真实相机节点 ----------
add_executable(hik_camera_node src/hik_camera_node.cpp)
ament_target_dependencies(hik_camera_node rclcpp sensor_msgs)
target_link_libraries(hik_camera_node /opt/MVS/lib/64/libMvCameraControl.so)

# 关键：安装后的可执行文件也要找得到海康的 .so
set_target_properties(hik_camera_node PROPERTIES INSTALL_RPATH "/opt/MVS/lib/64")

# 安装可执行文件
install(TARGETS
  fake_camera_publisher
  hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（不装的话 ros2 launch 找不到）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
