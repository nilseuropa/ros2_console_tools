#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace {

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticKeyValue = diagnostic_msgs::msg::KeyValue;

DiagnosticKeyValue make_value(const std::string & key, const std::string & value) {
  DiagnosticKeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

class DiagnosticsMockNode : public rclcpp::Node {
public:
  DiagnosticsMockNode()
  : Node("diagnostics_mock_node")
  {
    diagnostics_publisher_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    diagnostics_agg_publisher_ = create_publisher<DiagnosticArray>("/diagnostics_agg", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publish_cycle(); });
  }

private:
  void publish_cycle() {
    DiagnosticArray message;
    message.header.stamp = now();

    const int phase = tick_ % 12;
    const double battery = std::max(5.0, 100.0 - static_cast<double>(tick_) * 0.7);
    const double cpu_temperature = 48.0 + std::sin(static_cast<double>(tick_) * 0.35) * 16.0;

    DiagnosticStatus battery_status;
    battery_status.name = "/mock/power/battery";
    battery_status.hardware_id = "mock_robot";
    battery_status.level =
      battery < 15.0 ? DiagnosticStatus::ERROR : (battery < 35.0 ? DiagnosticStatus::WARN : 0u);
    battery_status.message =
      battery_status.level == DiagnosticStatus::ERROR ? "Battery critically low"
      : battery_status.level == DiagnosticStatus::WARN ? "Battery getting low"
      : "Battery nominal";
    battery_status.values.push_back(make_value("percentage", std::to_string(static_cast<int>(battery))));
    battery_status.values.push_back(make_value("voltage", std::to_string(21.5 + battery * 0.01)));
    battery_status.values.push_back(make_value("charging", phase >= 9 ? "true" : "false"));
    message.status.push_back(battery_status);

    DiagnosticStatus cpu_status;
    cpu_status.name = "/mock/system/cpu_temperature";
    cpu_status.hardware_id = "mock_compute";
    cpu_status.level =
      cpu_temperature > 72.0 ? DiagnosticStatus::ERROR
      : cpu_temperature > 62.0 ? DiagnosticStatus::WARN
      : 0u;
    cpu_status.message =
      cpu_status.level == DiagnosticStatus::ERROR ? "CPU too hot"
      : cpu_status.level == DiagnosticStatus::WARN ? "CPU warming up"
      : "CPU temperature nominal";
    cpu_status.values.push_back(make_value("temperature_c", std::to_string(cpu_temperature)));
    cpu_status.values.push_back(make_value("fan_rpm", std::to_string(1200 + phase * 180)));
    cpu_status.values.push_back(make_value("load_pct", std::to_string(28 + (phase * 5) % 70)));
    message.status.push_back(cpu_status);

    DiagnosticStatus lidar_status;
    lidar_status.name = "/mock/sensors/lidar_front";
    lidar_status.hardware_id = "lidar_front";
    lidar_status.level =
      phase == 5 ? DiagnosticStatus::ERROR
      : phase == 4 || phase == 6 ? DiagnosticStatus::WARN
      : 0u;
    lidar_status.message =
      lidar_status.level == DiagnosticStatus::ERROR ? "No scan data"
      : lidar_status.level == DiagnosticStatus::WARN ? "Scan rate degraded"
      : "Sensor online";
    lidar_status.values.push_back(make_value("scan_rate_hz", phase == 5 ? "0.0" : std::to_string(9.8 - phase * 0.3)));
    lidar_status.values.push_back(make_value("dropped_packets", std::to_string(phase >= 4 ? phase * 3 : 0)));
    lidar_status.values.push_back(make_value("frame_id", "laser_front"));
    message.status.push_back(lidar_status);

    DiagnosticStatus planner_status;
    planner_status.name = "/mock/navigation/planner";
    planner_status.hardware_id = "nav_stack";
    planner_status.level = phase == 8 ? DiagnosticStatus::WARN : 0u;
    planner_status.message = phase == 8 ? "Replanning repeatedly" : "Planner healthy";
    planner_status.values.push_back(make_value("active_goal", phase < 6 ? "dock" : "patrol"));
    planner_status.values.push_back(make_value("path_points", std::to_string(42 + phase)));
    planner_status.values.push_back(make_value("replans", std::to_string(phase == 8 ? 7 : phase / 2)));
    message.status.push_back(planner_status);

    diagnostics_publisher_->publish(message);
    diagnostics_agg_publisher_->publish(message);
    ++tick_;
  }

  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_agg_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int tick_{0};
};

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagnosticsMockNode>());
  rclcpp::shutdown();
  return 0;
}
