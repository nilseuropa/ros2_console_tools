#ifndef ROS2_CONSOLE_TOOLS__MAP_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__MAP_VIEWER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_map_viewer_tool(const std::string & topic = "", bool embedded_mode = false);

class MapViewerScreen;

class MapViewerBackend : public rclcpp::Node {
public:
  explicit MapViewerBackend(const std::string & topic = "");

private:
  friend class MapViewerScreen;

  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  void grid_callback(OccupancyGrid::ConstSharedPtr message);
  OccupancyGrid::ConstSharedPtr latest_grid_snapshot() const;

  std::string topic_;
  double render_hz_{5.0};
  int max_width_{0};
  int max_height_{0};
  int rotation_degrees_{90};
  bool show_free_{true};
  bool show_legend_{true};
  mutable std::mutex latest_grid_mutex_;
  OccupancyGrid::ConstSharedPtr latest_grid_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr subscription_;
};

class MapViewerScreen {
public:
  explicit MapViewerScreen(std::shared_ptr<MapViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  void draw();
  void draw_waiting_message(int top, int left, int bottom, int right) const;
  void draw_grid_view(
    int top, int left, int bottom, int right,
    const nav_msgs::msg::OccupancyGrid & message) const;
  void draw_legend_line(int row, int left, int width) const;
  void draw_status_line(int row, int columns, const std::string & text) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<MapViewerBackend> backend_;
  bool embedded_mode_{false};
  std::chrono::steady_clock::time_point startup_time_{};
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__MAP_VIEWER_HPP_
