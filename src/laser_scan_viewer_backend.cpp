#include "ros2_console_tools/laser_scan_viewer.hpp"

namespace ros2_console_tools {

LaserScanViewerBackend::LaserScanViewerBackend(const std::string & topic)
: Node("laser_scan_viewer"),
  topic_(this->declare_parameter<std::string>("topic", topic.empty() ? "/scan" : topic))
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

  subscription_ = this->create_subscription<LaserScan>(
    topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LaserScanViewerBackend::scan_callback, this, std::placeholders::_1));
}

void LaserScanViewerBackend::scan_callback(LaserScan::ConstSharedPtr message) {
  auto frame = std::make_shared<LaserScanFrame>();
  frame->topic = topic_;
  frame->frame_id = message->header.frame_id;
  frame->stamp = message->header.stamp;
  frame->angle_min = message->angle_min;
  frame->angle_max = message->angle_max;
  frame->angle_increment = message->angle_increment;
  frame->scan_time = message->scan_time;
  frame->time_increment = message->time_increment;
  frame->range_min = message->range_min;
  frame->range_max = message->range_max;
  frame->ranges = message->ranges;
  frame->intensities = message->intensities;
  frame->received_time = LaserScanViewerClock::now();

  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  latest_frame_ = std::move(frame);
}

std::shared_ptr<const LaserScanFrame> LaserScanViewerBackend::latest_frame_snapshot() const {
  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  return latest_frame_;
}

}  // namespace ros2_console_tools
