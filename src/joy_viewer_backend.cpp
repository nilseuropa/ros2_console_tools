#include "ros2_console_tools/joy_viewer.hpp"

namespace ros2_console_tools {

JoyViewerBackend::JoyViewerBackend(const std::string & topic)
: Node("joy_viewer"),
  topic_(this->declare_parameter<std::string>("topic", topic.empty() ? "/joy" : topic))
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }

  render_hz_ = std::max(1.0, this->declare_parameter<double>("render_hz", 20.0));

  subscription_ = this->create_subscription<Joy>(
    topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&JoyViewerBackend::joy_callback, this, std::placeholders::_1));
}

void JoyViewerBackend::joy_callback(Joy::ConstSharedPtr message) {
  auto frame = std::make_shared<JoyFrame>();
  frame->topic = topic_;
  frame->stamp = message->header.stamp;
  frame->axes = message->axes;
  frame->buttons = message->buttons;
  frame->received_time = JoyViewerClock::now();

  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  latest_frame_ = std::move(frame);
}

std::shared_ptr<const JoyFrame> JoyViewerBackend::latest_frame_snapshot() const {
  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  return latest_frame_;
}

}  // namespace ros2_console_tools
