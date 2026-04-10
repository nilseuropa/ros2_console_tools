#ifndef ROS2_CONSOLE_TOOLS__JOY_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__JOY_VIEWER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_joy_viewer_tool(const std::string & topic = "", bool embedded_mode = false);

using JoyViewerClock = std::chrono::steady_clock;

struct JoyFrame {
  std::string topic;
  builtin_interfaces::msg::Time stamp;
  std::vector<float> axes;
  std::vector<int32_t> buttons;
  JoyViewerClock::time_point received_time{};
};

class JoyViewerScreen;

class JoyViewerBackend : public rclcpp::Node {
public:
  explicit JoyViewerBackend(const std::string & topic = "");

private:
  friend class JoyViewerScreen;

  using Joy = sensor_msgs::msg::Joy;

  void joy_callback(Joy::ConstSharedPtr message);
  std::shared_ptr<const JoyFrame> latest_frame_snapshot() const;

  std::string topic_;
  double render_hz_{20.0};
  mutable std::mutex latest_frame_mutex_;
  std::shared_ptr<JoyFrame> latest_frame_;
  rclcpp::Subscription<Joy>::SharedPtr subscription_;
};

class JoyViewerScreen {
public:
  explicit JoyViewerScreen(std::shared_ptr<JoyViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  void draw();
  void draw_waiting_message(int top, int left, int bottom, int right) const;
  void draw_joy_view(int top, int left, int bottom, int right, const JoyFrame & frame) const;
  void draw_axes(int top, int left, int bottom, int right, const JoyFrame & frame) const;
  void draw_buttons(int top, int left, int bottom, int right, const JoyFrame & frame) const;
  void draw_stick_pad(
    int top, int left, int bottom, int right, const JoyFrame & frame,
    int x_axis, int y_axis, const std::string & title) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<JoyViewerBackend> backend_;
  bool embedded_mode_{false};
  std::chrono::steady_clock::time_point startup_time_{};
  bool frozen_{false};
  std::shared_ptr<const JoyFrame> frozen_frame_;
  std::string status_line_{"Waiting for joystick messages..."};
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__JOY_VIEWER_HPP_
