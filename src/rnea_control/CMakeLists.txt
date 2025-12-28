cmake_minimum_required(VERSION 3.8)
project(rnea_control)

set(CMAKE_CXX_STANDARD 17)
# Tối ưu hóa hiệu năng
add_compile_options(-O3 -march=native -ffast-math)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

# Tạo thư viện Shared
add_library(${PROJECT_NAME} SHARED src/rnea_controller.cpp)

pluginlib_export_plugin_description_file(controller_interface rnea_controller_plugins.xml)

target_include_directories(${PROJECT_NAME} PRIVATE include)

# Liên kết các phụ thuộc
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  controller_interface 
  hardware_interface 
  pluginlib
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)



# Cài đặt thư viện vào thư mục lib
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Cài đặt file XML của plugin vào thư mục share của package
install(
  FILES rnea_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Cài đặt thư mục include
install(DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# ----------------------

ament_package()