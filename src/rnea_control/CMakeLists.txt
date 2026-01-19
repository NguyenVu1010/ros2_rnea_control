cmake_minimum_required(VERSION 3.8)
project(rnea_control)

# 1. Cấu hình Trình biên dịch (Tối ưu hóa tốc độ)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# 2. Tìm kiếm Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include thư mục header
include_directories(include)

# ==============================================================================
# 3. BUILD CÁC MODULE LÕI (STATIC LIBRARIES)
# ==============================================================================

# --- A. CORE (Dữ liệu & Toán cơ bản) ---
add_library(rnea_core STATIC
  src/core/math_utils.cpp
)
target_link_libraries(rnea_core Eigen3::Eigen)
target_include_directories(rnea_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- B. KINEMATICS (Động học thuận & Jacobian) ---
add_library(rnea_kinematics STATIC
  src/kinematics/forward_kinematics.cpp
  src/kinematics/jacobian.cpp
)
# Link với Core để dùng Struct LinkParams
target_link_libraries(rnea_kinematics rnea_core Eigen3::Eigen)

# --- C. DYNAMICS (RNEA Solver) ---
add_library(rnea_dynamics STATIC
  src/dynamics/rnea_solver.cpp
)
target_link_libraries(rnea_dynamics rnea_core Eigen3::Eigen)

# --- D. TRAJECTORY (Sinh quỹ đạo) ---
add_library(rnea_trajectory STATIC
  src/trajectory/cartesian_interpolator.cpp
  src/trajectory/trajectory_generator.cpp
)
target_link_libraries(rnea_trajectory rnea_core Eigen3::Eigen)

# --- E. CONTROL LOGIC (Tracking / CLIK) ---
add_library(rnea_tracking STATIC
  src/control/motion_control.cpp
)
target_link_libraries(rnea_tracking rnea_core Eigen3::Eigen)

# ==============================================================================
# 4. BUILD ROS 2 CONTROLLER (SHARED LIBRARY - PLUGIN)
# ==============================================================================

add_library(rnea_controller SHARED
  src/rnea_controller.cpp
)

# Link tất cả các module tĩnh vào module chính
# 1. Link với các module tĩnh nội bộ (Core, Kinematics...)
target_link_libraries(rnea_controller
  rnea_core
  rnea_kinematics
  rnea_dynamics
  rnea_trajectory
  rnea_tracking
)

# 2. Link với các gói ROS 2 (Tự động thêm cả Libs và Includes)
ament_target_dependencies(rnea_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface  
  hardware_interface
  pluginlib
  std_msgs
  Eigen3
)

# ==============================================================================
# 5. INSTALL & EXPORT
# ==============================================================================

# Export Plugin Description
pluginlib_export_plugin_description_file(controller_interface rnea_controller_plugins.xml)

# Install Libraries
install(
  TARGETS 
    rnea_controller 
    rnea_core 
    rnea_kinematics 
    rnea_dynamics 
    rnea_trajectory 
    rnea_tracking
  EXPORT export_rnea_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install Config  & Plugin XML
install(
  FILES rnea_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

# Export Dependencies
ament_export_include_directories(include)
ament_export_libraries(
  rnea_controller 
  rnea_core 
  rnea_kinematics 
  rnea_dynamics 
  rnea_trajectory 
  rnea_tracking
)
ament_export_dependencies(
  rclcpp 
  controller_interface 
  hardware_interface 
  std_msgs 
  Eigen3
)

ament_package()