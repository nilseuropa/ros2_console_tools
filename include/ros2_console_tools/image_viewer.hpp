#ifndef ROS2_CONSOLE_TOOLS__IMAGE_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__IMAGE_VIEWER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_image_viewer_tool(const std::string & topic = "", bool embedded_mode = false);

struct ImageFrame {
  std::string topic;
  std::string source_encoding;
  uint32_t width{0};
  uint32_t height{0};
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
  std::vector<uint8_t> gray8;
};

enum class ImageViewerRenderMode {
  Auto = 0,
  Ascii = 1,
  Shade = 2,
};

class ImageViewerScreen;

class ImageViewerBackend : public rclcpp::Node {
public:
  explicit ImageViewerBackend(const std::string & topic = "");

private:
  friend class ImageViewerScreen;

  using Image = sensor_msgs::msg::Image;

  void image_callback(Image::ConstSharedPtr message);
  std::shared_ptr<const ImageFrame> latest_frame_snapshot() const;
  std::string latest_error_snapshot() const;
  bool decode_to_frame(const Image & message, ImageFrame & frame, std::string & error) const;

  std::string topic_;
  double render_hz_{10.0};
  mutable std::mutex latest_frame_mutex_;
  std::shared_ptr<ImageFrame> latest_frame_;
  std::string latest_error_;
  rclcpp::Subscription<Image>::SharedPtr subscription_;
};

class ImageViewerScreen {
public:
  explicit ImageViewerScreen(std::shared_ptr<ImageViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  void draw();
  void draw_waiting_message(int top, int left, int bottom, int right) const;
  void draw_error_message(int top, int left, int bottom, int right, const std::string & error) const;
  void draw_image_view(int top, int left, int bottom, int right, const ImageFrame & frame) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;
  void reset_view();

  std::shared_ptr<ImageViewerBackend> backend_;
  bool embedded_mode_{false};
  std::chrono::steady_clock::time_point startup_time_{};
  double zoom_factor_{1.0};
  int pan_x_{0};
  int pan_y_{0};
  bool invert_grayscale_{false};
  bool frozen_{false};
  ImageViewerRenderMode render_mode_{ImageViewerRenderMode::Auto};
  std::shared_ptr<const ImageFrame> frozen_frame_;
  std::string status_line_{"Waiting for images..."};
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__IMAGE_VIEWER_HPP_
