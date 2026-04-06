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
find_package(ament_index_cpp REQUIRED)
set(CURSES_NEED_WIDE TRUE)
find_package(Curses REQUIRED)
find_library(NCURSESW_LIBRARY NAMES ncursesw)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(map_viewer_lib
  src/map_viewer_backend.cpp
  src/map_viewer_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(map_viewer_lib
  ament_index_cpp
  nav_msgs
  rclcpp
)
target_include_directories(map_viewer_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(map_viewer_lib ${NCURSESW_LIBRARY})

add_executable(map_viewer src/map_viewer.cpp)
ament_target_dependencies(map_viewer
  rclcpp
)
target_include_directories(map_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(map_viewer map_viewer_lib)

install(TARGETS map_viewer_lib map_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_library(parameter_commander_lib
  src/parameter_commander_backend.cpp
  src/parameter_commander_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(parameter_commander_lib
  ament_index_cpp
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

add_library(topic_monitor_lib
  src/topic_monitor_backend.cpp
  src/topic_monitor_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(topic_monitor_lib
  ament_index_cpp
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(topic_monitor_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(topic_monitor_lib ${NCURSESW_LIBRARY})

add_executable(topic_monitor src/topic_monitor.cpp)
ament_target_dependencies(topic_monitor
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(topic_monitor PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(topic_monitor topic_monitor_lib)

install(TARGETS topic_monitor_lib topic_monitor
  DESTINATION lib/${PROJECT_NAME}
)

add_library(log_viewer_lib
  src/log_viewer_backend.cpp
  src/log_viewer_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(log_viewer_lib
  ament_index_cpp
  rcl_interfaces
  rclcpp
)
target_include_directories(log_viewer_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(log_viewer_lib ${NCURSESW_LIBRARY})

add_executable(log_viewer src/log_viewer.cpp)
ament_target_dependencies(log_viewer
  rcl_interfaces
  rclcpp
)
target_include_directories(log_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(log_viewer log_viewer_lib)

install(TARGETS log_viewer_lib log_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_library(node_commander_lib
  src/node_commander_backend.cpp
  src/node_commander_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(node_commander_lib
  ament_index_cpp
  rclcpp
)
target_include_directories(node_commander_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(
  node_commander_lib
  ${NCURSESW_LIBRARY}
  parameter_commander_lib
  topic_monitor_lib
  log_viewer_lib
)

add_executable(node_commander src/node_commander.cpp)
ament_target_dependencies(node_commander
  ament_index_cpp
  rclcpp
)
target_include_directories(node_commander PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(node_commander node_commander_lib)

install(TARGETS node_commander_lib node_commander
  DESTINATION lib/${PROJECT_NAME}
)

add_library(service_commander_lib
  src/service_commander_backend.cpp
  src/service_commander_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(service_commander_lib
  ament_index_cpp
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(service_commander_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(service_commander_lib ${NCURSESW_LIBRARY})

add_executable(service_commander src/service_commander.cpp)
ament_target_dependencies(service_commander
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(service_commander PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(service_commander service_commander_lib)

install(TARGETS service_commander_lib service_commander
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(node_commander_lib service_commander_lib)

add_library(tf_monitor_lib
  src/tf_monitor_backend.cpp
  src/tf_monitor_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(tf_monitor_lib
  ament_index_cpp
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

target_link_libraries(node_commander_lib tf_monitor_lib)

add_library(urdf_inspector_lib
  src/urdf_inspector_backend.cpp
  src/urdf_inspector_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(urdf_inspector_lib
  ament_index_cpp
  rclcpp
  urdf
)
target_include_directories(urdf_inspector_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(urdf_inspector_lib ${NCURSESW_LIBRARY})

add_executable(urdf_inspector src/urdf_inspector.cpp)
ament_target_dependencies(urdf_inspector
  rclcpp
  urdf
)
target_include_directories(urdf_inspector PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(urdf_inspector urdf_inspector_lib)

install(TARGETS urdf_inspector_lib urdf_inspector
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(node_commander_lib urdf_inspector_lib)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES config/tui_theme.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
