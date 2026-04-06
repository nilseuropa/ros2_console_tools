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
set(CURSES_NEED_WIDE TRUE)
find_package(Curses REQUIRED)
find_library(NCURSESW_LIBRARY NAMES ncursesw)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(occupancy_grid_node src/occupancy_grid_node.cpp)
ament_target_dependencies(occupancy_grid_node
  nav_msgs
  rclcpp
)

install(TARGETS occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(parameter_commander_lib
  src/parameter_commander_backend.cpp
  src/parameter_commander_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(parameter_commander_lib
  rclcpp
)
target_include_directories(parameter_commander_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(parameter_commander_lib ${NCURSESW_LIBRARY})

add_executable(parameter_commander src/parameter_commander.cpp)
ament_target_dependencies(parameter_commander
  rclcpp
)
target_include_directories(parameter_commander PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(parameter_commander parameter_commander_lib)

install(TARGETS parameter_commander_lib parameter_commander
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(topic_monitor src/topic_monitor.cpp)
target_sources(topic_monitor PRIVATE src/tui.cpp)
ament_target_dependencies(topic_monitor
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(topic_monitor PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(topic_monitor ${NCURSESW_LIBRARY})

install(TARGETS topic_monitor
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(log_viewer src/log_viewer.cpp)
target_sources(log_viewer PRIVATE src/tui.cpp)
ament_target_dependencies(log_viewer
  rcl_interfaces
  rclcpp
)
target_include_directories(log_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(log_viewer ${NCURSESW_LIBRARY})

install(TARGETS log_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(service_commander src/service_commander.cpp)
target_sources(service_commander PRIVATE src/tui.cpp)
ament_target_dependencies(service_commander
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(service_commander PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(service_commander ${NCURSESW_LIBRARY})

install(TARGETS service_commander
  DESTINATION lib/${PROJECT_NAME}
)

add_library(tf_monitor_lib
  src/tf_monitor_backend.cpp
  src/tf_monitor_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(tf_monitor_lib
  geometry_msgs
  rclcpp
  tf2_msgs
)
target_include_directories(tf_monitor_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(tf_monitor_lib ${NCURSESW_LIBRARY})

add_executable(tf_monitor src/tf_monitor.cpp)
ament_target_dependencies(tf_monitor
  geometry_msgs
  rclcpp
  tf2_msgs
)
target_include_directories(tf_monitor PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(tf_monitor tf_monitor_lib)

install(TARGETS tf_monitor_lib tf_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
