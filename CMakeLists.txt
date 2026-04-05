cmake_minimum_required(VERSION 3.10)

project(ros2_console_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(occupancy_grid_node src/occupancy_grid_node.cpp)
ament_target_dependencies(occupancy_grid_node
  nav_msgs
  rclcpp
)

install(TARGETS occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
