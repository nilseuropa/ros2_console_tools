#ifndef ROS2_CONSOLE_TOOLS__IMU_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__IMU_VIEWER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_imu_viewer_tool(const std::string & topic = "", bool embedded_mode = false);

using ImuViewerClock = std::chrono::steady_clock;

struct ImuFrame {
  std::string topic;
  std::string frame_id;
  builtin_interfaces::msg::Time stamp;
  sensor_msgs::msg::Imu message;
  ImuViewerClock::time_point received_time{};
};

struct ImuOrientation {
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

enum class ImuCovarianceState {
  Unknown,
  Zero,
  Valid,
};

class ImuViewerScreen;

class ImuViewerBackend : public rclcpp::Node {
public:
  explicit ImuViewerBackend(const std::string & topic = "");

private:
  friend class ImuViewerScreen;

  using Imu = sensor_msgs::msg::Imu;

  void imu_callback(Imu::ConstSharedPtr message);
  std::shared_ptr<const ImuFrame> latest_frame_snapshot() const;

  std::string topic_;
  double render_hz_{20.0};
  mutable std::mutex latest_frame_mutex_;
  std::shared_ptr<ImuFrame> latest_frame_;
  rclcpp::Subscription<Imu>::SharedPtr subscription_;
};

class ImuViewerScreen {
public:
  explicit ImuViewerScreen(std::shared_ptr<ImuViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  void draw();
  void draw_waiting_message(int top, int left, int bottom, int right) const;
  void draw_imu_view(int top, int left, int bottom, int right, const ImuFrame & frame) const;
  void draw_vector_panel(
    int top, int left, int bottom, int right, const std::string & title,
    const std::array<double, 3> & values, double scale) const;
  void draw_orientation_panel(
    int top, int left, int bottom, int right, const ImuFrame & frame,
    ImuCovarianceState covariance_state) const;
  void draw_tilt_panel(
    int top, int left, int bottom, int right, const ImuFrame & frame,
    ImuCovarianceState covariance_state) const;
  void draw_covariance_line(int row, int left, int width, const ImuFrame & frame) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<ImuViewerBackend> backend_;
  bool embedded_mode_{false};
  std::chrono::steady_clock::time_point startup_time_{};
  bool frozen_{false};
  std::shared_ptr<const ImuFrame> frozen_frame_;
  std::string status_line_{"Waiting for IMU messages..."};
};

ImuOrientation orientation_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion);
ImuCovarianceState covariance_state(const std::array<double, 9> & covariance);

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__IMU_VIEWER_HPP_
