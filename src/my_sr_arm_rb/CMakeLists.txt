cmake_minimum_required(VERSION 3.8) # ROS 2 khuyến khích dùng 3.8+
project(my_sr_arm_rb)

# ============================================================
# 1. CẤU HÌNH CHUẨN C++ (SỬA LỖI std::is_same_v)
# ============================================================
# Thay thế -std=c++14 bằng C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Thêm tối ưu hóa hiệu năng
add_compile_options(-O3 -Wall -Wextra -Wpedantic)

# ============================================================
# 2. TÌM PACKAGE PHỤ THUỘC
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # Thêm vì dùng TransformStamped
find_package(std_msgs REQUIRED)      # Thêm vì dùng Float64MultiArray
find_package(Eigen3 REQUIRED)

# ============================================================
# 3. BUILD NODE C++
# ============================================================
add_executable(virtual_constraint_controller_node
  src/virtual_constraint_controller.cpp
)

# Khai báo thư mục include
target_include_directories(virtual_constraint_controller_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Liên kết các thư viện ROS 2
ament_target_dependencies(virtual_constraint_controller_node
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  trajectory_msgs
  geometry_msgs
  std_msgs
)

# Link trực tiếp với Eigen
target_link_libraries(virtual_constraint_controller_node
  Eigen3::Eigen
)

# ============================================================
# 4. CÀI ĐẶT (INSTALL)
# ============================================================
# Cài đặt file thực thi
install(
  TARGETS virtual_constraint_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Cài đặt các thư mục share
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 5. ĐÓNG GÓI
# ============================================================
ament_package()