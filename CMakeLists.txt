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
find_package(Curses REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_executable(occupancy_grid_node src/occupancy_grid_node.cpp)
ament_target_dependencies(occupancy_grid_node
  nav_msgs
  rclcpp
)

install(TARGETS occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(parameter_commander src/parameter_console_node.cpp)
ament_target_dependencies(parameter_commander
  rclcpp
)
target_include_directories(parameter_commander PRIVATE ${CURSES_INCLUDE_DIR})
target_link_libraries(parameter_commander ${CURSES_LIBRARIES})

install(TARGETS parameter_commander
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(topic_monitor src/topic_monitor.cpp)
ament_target_dependencies(topic_monitor
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(topic_monitor PRIVATE ${CURSES_INCLUDE_DIR})
target_link_libraries(topic_monitor ${CURSES_LIBRARIES})

install(TARGETS topic_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
