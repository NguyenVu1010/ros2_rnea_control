cmake_minimum_required(VERSION 3.8)
project(rnea_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(robot_common REQUIRED)
find_package(robot_shm REQUIRED)
find_package(robot_dynamics REQUIRED)
find_package(robot_trajectory REQUIRED)
find_package(robot_parser REQUIRED)

add_library(rnea_controller src/rnea_controller.cpp)
target_compile_features(rnea_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rnea_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  rnea_controller
  "rclcpp"
  "controller_interface"
  "hardware_interface"
  "pluginlib"
  "realtime_tools"
  "robot_common"
  "robot_shm"
  "robot_dynamics"
  "robot_trajectory"
  "robot_parser"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rnea_controller PRIVATE "RNEA_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rnea_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  rnea_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
