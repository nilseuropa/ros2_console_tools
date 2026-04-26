#ifndef ROS2_CONSOLE_TOOLS__LASER_SCAN_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__LASER_SCAN_VIEWER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_laser_scan_viewer_tool(const std::string & topic = "", bool embedded_mode = false);

enum class LaserScanViewerRenderMode {
  Auto = 0,
  Braille = 1,
  Points = 2,
};

using LaserScanViewerClock = std::chrono::steady_clock;

struct LaserScanFrame {
  std::string topic;
  std::string frame_id;
  builtin_interfaces::msg::Time stamp;
  float angle_min{0.0f};
  float angle_max{0.0f};
  float angle_increment{0.0f};
  float scan_time{0.0f};
  float time_increment{0.0f};
  float range_min{0.0f};
  float range_max{0.0f};
  std::vector<float> ranges;
  std::vector<float> intensities;
  LaserScanViewerClock::time_point received_time{};
};

struct LaserScanStats {
  std::size_t total_count{0};
  std::size_t valid_count{0};
  std::size_t invalid_count{0};
  float min_range{0.0f};
  float max_range{0.0f};
  float average_range{0.0f};
};

class LaserScanViewerScreen;

class LaserScanViewerBackend : public rclcpp::Node {
public:
  explicit LaserScanViewerBackend(const std::string & topic = "");

private:
  friend class LaserScanViewerScreen;

  using LaserScan = sensor_msgs::msg::LaserScan;

  void scan_callback(LaserScan::ConstSharedPtr message);
  std::shared_ptr<const LaserScanFrame> latest_frame_snapshot() const;

  std::string topic_;
  double render_hz_{20.0};
  mutable std::mutex latest_frame_mutex_;
  std::shared_ptr<LaserScanFrame> latest_frame_;
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
};

class LaserScanViewerScreen {
public:
  explicit LaserScanViewerScreen(
    std::shared_ptr<LaserScanViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  void draw();
  void draw_waiting_message(int top, int left, int bottom, int right) const;
  void draw_scan_view(int top, int left, int bottom, int right, const LaserScanFrame & frame) const;
  void draw_stats_panel(
    int top, int left, int bottom, int right, const LaserScanFrame & frame,
    const LaserScanStats & stats) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;
  void reset_view();

  std::shared_ptr<LaserScanViewerBackend> backend_;
  bool embedded_mode_{false};
  std::chrono::steady_clock::time_point startup_time_{};
  double zoom_factor_{1.0};
  bool frozen_{false};
  bool show_invalid_{false};
  LaserScanViewerRenderMode render_mode_{LaserScanViewerRenderMode::Auto};
  std::shared_ptr<const LaserScanFrame> frozen_frame_;
  std::string status_line_{"Waiting for LaserScan messages..."};
};

LaserScanStats compute_laser_scan_stats(const LaserScanFrame & frame);

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__LASER_SCAN_VIEWER_HPP_
