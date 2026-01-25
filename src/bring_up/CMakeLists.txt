cmake_minimum_required(VERSION 3.8)
project(bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install các thư mục tài nguyên để ROS tìm thấy
install(
  DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()