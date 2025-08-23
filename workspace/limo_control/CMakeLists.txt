cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Imported Compiled Libraries
add_library(unicycle_model src/unicycle_model.cpp)
target_include_directories(unicycle_model PUBLIC include)

# Executables
add_executable(limo_controller_node src/limo_controller_node.cpp)
ament_target_dependencies(limo_controller_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(limo_controller_node unicycle_model)

install(TARGETS
  unicycle_model
  limo_controller_node
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()