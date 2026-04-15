cmake_minimum_required(VERSION 3.10)

project(ros2_console_tools)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" ROS2_CONSOLE_TOOLS_PACKAGE_XML)
string(
  REGEX MATCH
  "<version>([^<]+)</version>"
  ROS2_CONSOLE_TOOLS_PACKAGE_VERSION_MATCH
  "${ROS2_CONSOLE_TOOLS_PACKAGE_XML}")
if(NOT ROS2_CONSOLE_TOOLS_PACKAGE_VERSION_MATCH)
  message(FATAL_ERROR "Failed to extract ros2_console_tools version from package.xml")
endif()
string(STRIP "${CMAKE_MATCH_1}" ROS2_CONSOLE_TOOLS_PACKAGE_VERSION)

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
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
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

add_library(image_viewer_lib
  src/image_viewer_backend.cpp
  src/image_viewer_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(image_viewer_lib
  ament_index_cpp
  rclcpp
  sensor_msgs
)
target_include_directories(image_viewer_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(image_viewer_lib ${NCURSESW_LIBRARY})

add_executable(image_viewer src/image_viewer.cpp)
ament_target_dependencies(image_viewer
  rclcpp
  sensor_msgs
)
target_include_directories(image_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(image_viewer image_viewer_lib)

install(TARGETS image_viewer_lib image_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_library(joy_viewer_lib
  src/joy_viewer_backend.cpp
  src/joy_viewer_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(joy_viewer_lib
  ament_index_cpp
  rclcpp
  sensor_msgs
)
target_include_directories(joy_viewer_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(joy_viewer_lib ${NCURSESW_LIBRARY})

add_executable(joy_viewer src/joy_viewer.cpp)
ament_target_dependencies(joy_viewer
  rclcpp
  sensor_msgs
)
target_include_directories(joy_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(joy_viewer joy_viewer_lib)

install(TARGETS joy_viewer_lib joy_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_library(imu_viewer_lib
  src/imu_viewer_backend.cpp
  src/imu_viewer_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(imu_viewer_lib
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_include_directories(imu_viewer_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(imu_viewer_lib ${NCURSESW_LIBRARY})

add_executable(imu_viewer src/imu_viewer.cpp)
ament_target_dependencies(imu_viewer
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_include_directories(imu_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(imu_viewer imu_viewer_lib)

install(TARGETS imu_viewer_lib imu_viewer
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
  sensor_msgs
  rosidl_typesupport_introspection_cpp
)
target_include_directories(topic_monitor_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(topic_monitor_lib ${NCURSESW_LIBRARY} map_viewer_lib image_viewer_lib joy_viewer_lib imu_viewer_lib)

add_executable(topic_monitor src/topic_monitor.cpp)
ament_target_dependencies(topic_monitor
  rclcpp
  sensor_msgs
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

add_library(diagnostics_viewer_lib
  src/diagnostics_viewer_backend.cpp
  src/diagnostics_viewer_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(diagnostics_viewer_lib
  ament_index_cpp
  diagnostic_msgs
  rclcpp
)
target_include_directories(diagnostics_viewer_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(diagnostics_viewer_lib ${NCURSESW_LIBRARY})

add_executable(diagnostics_viewer src/diagnostics_viewer.cpp)
ament_target_dependencies(diagnostics_viewer
  diagnostic_msgs
  rclcpp
)
target_include_directories(diagnostics_viewer PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(diagnostics_viewer diagnostics_viewer_lib)

## Manual diagnostics publisher kept on disk for ad hoc local testing, but disabled by default.
# add_executable(diagnostics_mock_node test/diagnostics_mock_node.cpp)
# ament_target_dependencies(diagnostics_mock_node
#   diagnostic_msgs
#   rclcpp
# )

install(TARGETS diagnostics_viewer_lib diagnostics_viewer
  DESTINATION lib/${PROJECT_NAME}
)

add_library(node_commander_lib
  src/node_commander_backend.cpp
  src/node_commander_screen.cpp
  src/tui.cpp
)
target_compile_definitions(node_commander_lib
  PUBLIC ROS2_CONSOLE_TOOLS_PACKAGE_VERSION=\"${ROS2_CONSOLE_TOOLS_PACKAGE_VERSION}\"
)
ament_target_dependencies(node_commander_lib
  ament_index_cpp
  rclcpp
)
target_include_directories(node_commander_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(
  node_commander_lib
  ${NCURSESW_LIBRARY}
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

add_library(action_commander_lib
  src/action_commander_backend.cpp
  src/action_commander_screen.cpp
  src/tui.cpp
)
ament_target_dependencies(action_commander_lib
  ament_index_cpp
  rclcpp
)
target_include_directories(action_commander_lib PUBLIC ${CURSES_INCLUDE_DIR} include)
target_link_libraries(action_commander_lib ${NCURSESW_LIBRARY})

add_executable(action_commander src/action_commander.cpp)
ament_target_dependencies(action_commander
  rclcpp
)
target_include_directories(action_commander PRIVATE ${CURSES_INCLUDE_DIR} include)
target_link_libraries(action_commander action_commander_lib)

install(TARGETS action_commander_lib action_commander
  DESTINATION lib/${PROJECT_NAME}
)

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

target_link_libraries(
  node_commander_lib
  parameter_commander_lib
  topic_monitor_lib
  joy_viewer_lib
  imu_viewer_lib
  log_viewer_lib
  diagnostics_viewer_lib
  service_commander_lib
  action_commander_lib
  tf_monitor_lib
  urdf_inspector_lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES config/tui_theme.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(service_commander_backend_test
    test/service_commander_backend_test.cpp
  )
  if(TARGET service_commander_backend_test)
    ament_target_dependencies(service_commander_backend_test
      ament_index_cpp
      rclcpp
      rosidl_typesupport_introspection_cpp
    )
    target_include_directories(service_commander_backend_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(service_commander_backend_test
      node_commander_lib
      service_commander_lib
    )
  endif()

  ament_add_gtest(node_commander_screen_test
    test/node_commander_screen_test.cpp
  )
  if(TARGET node_commander_screen_test)
    ament_target_dependencies(node_commander_screen_test
      ament_index_cpp
      geometry_msgs
      rclcpp
    )
    target_include_directories(node_commander_screen_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(node_commander_screen_test
      node_commander_lib
    )
  endif()

  ament_add_gtest(topic_monitor_backend_test
    test/topic_monitor_backend_test.cpp
  )
  if(TARGET topic_monitor_backend_test)
    ament_target_dependencies(topic_monitor_backend_test
      ament_index_cpp
      geometry_msgs
      rclcpp
      rosidl_typesupport_introspection_cpp
    )
    target_include_directories(topic_monitor_backend_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(topic_monitor_backend_test
      topic_monitor_lib
    )
  endif()

  ament_add_gtest(parameter_commander_backend_test
    test/parameter_commander_backend_test.cpp
  )
  if(TARGET parameter_commander_backend_test)
    ament_target_dependencies(parameter_commander_backend_test
      ament_index_cpp
      rclcpp
    )
    target_include_directories(parameter_commander_backend_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(parameter_commander_backend_test
      parameter_commander_lib
    )
  endif()

  ament_add_gtest(image_viewer_backend_test
    test/image_viewer_backend_test.cpp
  )
  if(TARGET image_viewer_backend_test)
    ament_target_dependencies(image_viewer_backend_test
      ament_index_cpp
      rclcpp
      sensor_msgs
    )
    target_include_directories(image_viewer_backend_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(image_viewer_backend_test
      image_viewer_lib
    )
  endif()

  ament_add_gtest(tf_monitor_backend_test
    test/tf_monitor_backend_test.cpp
  )
  if(TARGET tf_monitor_backend_test)
    ament_target_dependencies(tf_monitor_backend_test
      geometry_msgs
      rclcpp
      tf2_msgs
    )
    target_include_directories(tf_monitor_backend_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(tf_monitor_backend_test
      tf_monitor_lib
    )
  endif()

  ament_add_gtest(tui_test
    test/tui_test.cpp
    src/tui.cpp
  )
  if(TARGET tui_test)
    ament_target_dependencies(tui_test
      ament_index_cpp
    )
    target_include_directories(tui_test PRIVATE ${CURSES_INCLUDE_DIR} include)
    target_link_libraries(tui_test
      ${NCURSESW_LIBRARY}
    )
  endif()

endif()

ament_package()
