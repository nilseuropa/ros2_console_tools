#include "ros2_console_tools/tf_monitor.hpp"

namespace ros2_console_tools {

TfMonitorBackend::TfMonitorBackend()
: Node("tf_monitor")
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }
}

void TfMonitorBackend::initialize_subscriptions() {
  tf_subscription_ = this->create_subscription<TFMessage>(
    "/tf", rclcpp::QoS(rclcpp::KeepLast(200)),
    [this](const TFMessage & message) { handle_tf_message(message, false); });

  tf_static_subscription_ = this->create_subscription<TFMessage>(
    "/tf_static", rclcpp::QoS(rclcpp::KeepLast(200)).reliable().transient_local(),
    [this](const TFMessage & message) { handle_tf_message(message, true); });
}

void TfMonitorBackend::handle_tf_message(const TFMessage & message, bool is_static) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = TfClock::now();
  for (const TransformStamped & transform : message.transforms) {
    TfLink link;
    link.parent_frame = normalize_frame(transform.header.frame_id);
    link.child_frame = normalize_frame(transform.child_frame_id);
    link.is_static = is_static;
    link.stamp = transform.header.stamp;
    link.transform = transform.transform;
    link.received_time = now;
    links_by_child_[link.child_frame] = std::move(link);
  }
}

void TfMonitorBackend::toggle_selected_row() {
  clamp_selection();
  if (rows_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(rows_.size())) {
    set_status("No TF link selected.");
    return;
  }

  const std::string frame = rows_[static_cast<std::size_t>(selected_index_)].child_frame;
  auto found = std::find(selected_frames_.begin(), selected_frames_.end(), frame);
  if (found == selected_frames_.end()) {
    if (selected_frames_.size() >= 2) {
      set_status("TF inspect supports exactly two selected frames.");
      return;
    }
    selected_frames_.push_back(frame);
    set_status("Selected " + frame + ".");
  } else {
    selected_frames_.erase(found);
    set_status("Deselected " + frame + ".");
  }
}

std::optional<InspectResult> TfMonitorBackend::compute_selected_transform() const {
  if (selected_frames_.size() < 2) {
    return std::nullopt;
  }

  const std::string & from = selected_frames_[0];
  const std::string & to = selected_frames_[1];
  const auto from_it = frame_poses_.find(from);
  const auto to_it = frame_poses_.find(to);
  if (from_it == frame_poses_.end() || to_it == frame_poses_.end()) {
    return std::nullopt;
  }
  if (from_it->second.root_frame != to_it->second.root_frame) {
    return std::nullopt;
  }

  const Pose relative = compose_pose(
    invert_pose(from_it->second.pose_from_root),
    to_it->second.pose_from_root);
  return InspectResult{from, to, relative};
}

void TfMonitorBackend::open_inspect_popup() {
  if (selected_frames_.size() < 2) {
    set_status("Select at least two links with Space.");
    return;
  }

  const auto result = compute_selected_transform();
  if (!result.has_value()) {
    set_status("Selected frames are not connected.");
    return;
  }

  inspect_result_ = *result;
  inspect_popup_open_ = true;
  set_status("Inspecting " + inspect_result_.from_frame + " -> " + inspect_result_.to_frame + ".");
}

void TfMonitorBackend::refresh_rows() {
  std::lock_guard<std::mutex> lock(mutex_);

  std::map<std::string, std::vector<std::string>> children_by_parent;
  std::set<std::string> all_frames;
  std::set<std::string> child_frames;
  for (const auto & [child, link] : links_by_child_) {
    children_by_parent[link.parent_frame].push_back(child);
    all_frames.insert(link.parent_frame);
    all_frames.insert(child);
    child_frames.insert(child);
  }
  for (auto & [parent, children] : children_by_parent) {
    std::sort(children.begin(), children.end());
  }

  std::vector<std::string> roots;
  for (const auto & frame : all_frames) {
    if (child_frames.find(frame) == child_frames.end()) {
      roots.push_back(frame);
    }
  }
  if (roots.empty() && !all_frames.empty()) {
    roots.push_back(*all_frames.begin());
  }
  std::sort(roots.begin(), roots.end());

  rows_.clear();
  frame_poses_.clear();
  const auto now = TfClock::now();
  std::set<std::string> visited;
  const auto append_root_row = [this](const std::string & root_frame) {
      rows_.push_back(TfRow{
        "",
        root_frame,
        false,
        true,
        0,
        "root",
        false});
    };

  std::function<void(const std::string &, int, const std::string &, const Pose &)> append_children =
    [&](const std::string & parent, int depth, const std::string & root_frame, const Pose & parent_pose) {
      auto found = children_by_parent.find(parent);
      if (found == children_by_parent.end()) {
        return;
      }
      for (const auto & child : found->second) {
        if (!visited.insert(child).second) {
          continue;
        }
        const auto link_it = links_by_child_.find(child);
        if (link_it == links_by_child_.end()) {
          continue;
        }

        bool stale = false;
        const Pose child_pose = compose_pose(parent_pose, pose_from_msg(link_it->second.transform));
        frame_poses_[child] = FramePose{root_frame, child_pose};
        rows_.push_back(TfRow{
          link_it->second.parent_frame,
          link_it->second.child_frame,
          link_it->second.is_static,
          false,
          depth,
          format_freshness(link_it->second, now, stale),
          stale});
        append_children(child, depth + 1, root_frame, child_pose);
      }
    };

  for (const auto & root : roots) {
    frame_poses_[root] = FramePose{root, Pose{}};
    visited.insert(root);
    append_root_row(root);
    append_children(root, 1, root, Pose{});
  }
  for (const auto & [child, link] : links_by_child_) {
    if (visited.find(child) != visited.end()) {
      continue;
    }
    visited.insert(link.parent_frame);
    bool stale = false;
    frame_poses_[link.parent_frame] = FramePose{link.parent_frame, Pose{}};
    append_root_row(link.parent_frame);
    const Pose child_pose = pose_from_msg(link.transform);
    frame_poses_[child] = FramePose{link.parent_frame, child_pose};
    rows_.push_back(TfRow{
      link.parent_frame,
      link.child_frame,
      link.is_static,
      false,
      1,
      format_freshness(link, now, stale),
      stale});
    visited.insert(child);
    append_children(child, 2, link.parent_frame, child_pose);
  }

  selected_frames_.erase(
    std::remove_if(
      selected_frames_.begin(), selected_frames_.end(),
      [this](const std::string & frame) { return frame_poses_.find(frame) == frame_poses_.end(); }),
    selected_frames_.end());

  clamp_selection();
  status_line_ =
    "Loaded " + std::to_string(links_by_child_.size()) + " TF links across "
    + std::to_string(all_frames.size()) + " frames.";
}

void TfMonitorBackend::clamp_selection() {
  if (rows_.empty()) {
    selected_index_ = 0;
    scroll_ = 0;
    return;
  }
  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(rows_.size()) - 1);
}

void TfMonitorBackend::set_status(const std::string & text) {
  std::lock_guard<std::mutex> lock(mutex_);
  status_line_ = text;
}

}  // namespace ros2_console_tools
