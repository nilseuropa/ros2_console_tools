#include "ros2_console_tools/map_viewer.hpp"

#include <algorithm>

namespace ros2_console_tools {

MapViewerBackend::MapViewerBackend(const std::string & topic)
: Node("map_viewer"),
  topic_(this->declare_parameter<std::string>("topic", topic.empty() ? "/map" : topic))
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }

  render_hz_ = std::max(1.0, this->declare_parameter<double>("render_hz", 5.0));
  max_width_ = std::max(0, static_cast<int>(this->declare_parameter("max_width", 0)));
  max_height_ = std::max(0, static_cast<int>(this->declare_parameter("max_height", 0)));
  show_free_ = this->declare_parameter<bool>("show_free", true);
  show_legend_ = this->declare_parameter<bool>("show_legend", true);
  rotation_degrees_ = this->declare_parameter<int>("rotation", 90);
  if (rotation_degrees_ != 90 && rotation_degrees_ != 180 && rotation_degrees_ != 270) {
    RCLCPP_WARN(
      this->get_logger(),
      "Unsupported rotation '%d'; falling back to 90 degrees.",
      rotation_degrees_);
    rotation_degrees_ = 90;
  }

  subscription_ = this->create_subscription<OccupancyGrid>(
    topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&MapViewerBackend::grid_callback, this, std::placeholders::_1));
}

void MapViewerBackend::grid_callback(OccupancyGrid::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(latest_grid_mutex_);
  latest_grid_ = std::move(message);
}

MapViewerBackend::OccupancyGrid::ConstSharedPtr MapViewerBackend::latest_grid_snapshot() const {
  std::lock_guard<std::mutex> lock(latest_grid_mutex_);
  return latest_grid_;
}

}  // namespace ros2_console_tools
