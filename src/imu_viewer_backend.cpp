#include "ros2_console_tools/imu_viewer.hpp"

namespace ros2_console_tools {

ImuViewerBackend::ImuViewerBackend(const std::string & topic)
: Node("imu_viewer"),
  topic_(this->declare_parameter<std::string>("topic", topic.empty() ? "/imu/data" : topic))
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

  subscription_ = this->create_subscription<Imu>(
    topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ImuViewerBackend::imu_callback, this, std::placeholders::_1));
}

void ImuViewerBackend::imu_callback(Imu::ConstSharedPtr message) {
  auto frame = std::make_shared<ImuFrame>();
  frame->topic = topic_;
  frame->frame_id = message->header.frame_id;
  frame->stamp = message->header.stamp;
  frame->message = *message;
  frame->received_time = ImuViewerClock::now();

  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  latest_frame_ = std::move(frame);
}

std::shared_ptr<const ImuFrame> ImuViewerBackend::latest_frame_snapshot() const {
  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  return latest_frame_;
}

}  // namespace ros2_console_tools
