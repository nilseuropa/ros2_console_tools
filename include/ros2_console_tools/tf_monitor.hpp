#ifndef ROS2_CONSOLE_TOOLS__TF_MONITOR_HPP_
#define ROS2_CONSOLE_TOOLS__TF_MONITOR_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_tf_monitor_tool(bool embedded_mode = false);

using TfClock = std::chrono::steady_clock;
using TFMessage = tf2_msgs::msg::TFMessage;
using TransformStamped = geometry_msgs::msg::TransformStamped;

struct TfLink {
  std::string parent_frame;
  std::string child_frame;
  bool is_static{false};
  builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::Transform transform;
  TfClock::time_point received_time{};
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose {
  Vec3 translation;
  Quat rotation;
};

struct TfRow {
  std::string parent_frame;
  std::string child_frame;
  bool is_static{false};
  bool is_root{false};
  int depth{0};
  std::string freshness;
  bool stale{false};
};

struct FramePose {
  std::string root_frame;
  Pose pose_from_root;
};

struct InspectResult {
  std::string from_frame;
  std::string to_frame;
  Pose transform;
};

inline std::string normalize_frame(std::string name) {
  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  return name.empty() ? "<unnamed>" : name;
}

inline std::string format_freshness(const TfLink & link, const TfClock::time_point now, bool & stale) {
  if (link.is_static) {
    stale = false;
    return "static";
  }

  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - link.received_time);
  const double seconds = static_cast<double>(age.count()) / 1000.0;
  stale = seconds > 1.5;

  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2fs", seconds);
  return buffer;
}

inline Quat normalize_quat(Quat quat) {
  const double norm = std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
  if (norm <= 1e-9) {
    return Quat{};
  }
  quat.x /= norm;
  quat.y /= norm;
  quat.z /= norm;
  quat.w /= norm;
  return quat;
}

inline Quat quat_multiply_raw(const Quat & lhs, const Quat & rhs) {
  return Quat{
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z};
}

inline Quat quat_multiply(const Quat & lhs, const Quat & rhs) {
  return normalize_quat(quat_multiply_raw(lhs, rhs));
}

inline Quat quat_conjugate(const Quat & quat) {
  return Quat{-quat.x, -quat.y, -quat.z, quat.w};
}

inline Vec3 rotate_vec(const Quat & quat, const Vec3 & vec) {
  const Quat vector_quat{vec.x, vec.y, vec.z, 0.0};
  const Quat rotated = quat_multiply_raw(quat_multiply_raw(quat, vector_quat), quat_conjugate(quat));
  return Vec3{rotated.x, rotated.y, rotated.z};
}

inline Pose compose_pose(const Pose & lhs, const Pose & rhs) {
  const Vec3 rotated = rotate_vec(lhs.rotation, rhs.translation);
  return Pose{
    Vec3{
      lhs.translation.x + rotated.x,
      lhs.translation.y + rotated.y,
      lhs.translation.z + rotated.z},
    quat_multiply(lhs.rotation, rhs.rotation)};
}

inline Pose invert_pose(const Pose & pose) {
  const Quat inverse_rotation = quat_conjugate(normalize_quat(pose.rotation));
  const Vec3 inverse_translation = rotate_vec(
    inverse_rotation,
    Vec3{-pose.translation.x, -pose.translation.y, -pose.translation.z});
  return Pose{inverse_translation, inverse_rotation};
}

inline Pose pose_from_msg(const geometry_msgs::msg::Transform & transform) {
  return Pose{
    Vec3{transform.translation.x, transform.translation.y, transform.translation.z},
    normalize_quat(Quat{
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z,
      transform.rotation.w})};
}

inline Vec3 rpy_from_quat(const Quat & quat_in) {
  const Quat quat = normalize_quat(quat_in);
  const double sinr_cosp = 2.0 * (quat.w * quat.x + quat.y * quat.z);
  const double cosr_cosp = 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Vec3{roll, pitch, yaw};
}

class TfMonitorScreen;

class TfMonitorBackend : public rclcpp::Node {
public:
  TfMonitorBackend();
  void initialize_subscriptions();

private:
  friend class TfMonitorScreen;

  void handle_tf_message(const TFMessage & message, bool is_static);
  void refresh_rows();
  void clamp_selection();
  void set_status(const std::string & text);
  std::optional<InspectResult> compute_selected_transform() const;
  void toggle_selected_row();
  void open_inspect_popup();

  mutable std::mutex mutex_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_static_subscription_;
  std::map<std::string, TfLink> links_by_child_;
  std::map<std::string, FramePose> frame_poses_;
  std::vector<TfRow> rows_;
  std::vector<std::string> selected_frames_;
  int selected_index_{0};
  int scroll_{0};
  bool inspect_popup_open_{false};
  InspectResult inspect_result_;
  std::string status_line_{"Waiting for TF messages..."};
};

class TfMonitorScreen {
public:
  explicit TfMonitorScreen(std::shared_ptr<TfMonitorBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_popup_key(int key);
  bool handle_search_key(int key);
  int page_step() const;
  void draw();
  void draw_tree_pane(int top, int left, int bottom, int right);
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;
  void draw_inspect_popup(int rows, int columns) const;

  std::shared_ptr<TfMonitorBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
};

}  // namespace ros2_console_tools

#endif
