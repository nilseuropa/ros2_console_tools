#include <ncursesw/ncurses.h>

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
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

namespace {

using Clock = std::chrono::steady_clock;
using TFMessage = tf2_msgs::msg::TFMessage;
using TransformStamped = geometry_msgs::msg::TransformStamped;

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorStatus = tui::kColorStatus,
  kColorHelp = tui::kColorHelp,
  kColorDynamic = tui::kColorAccent,
  kColorStale = tui::kColorError,
};

using tui::Session;
using tui::draw_box;
using tui::draw_box_char;
using tui::draw_help_bar;
using tui::draw_help_bar_region;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::SearchState;
using tui::start_search;
using tui::truncate_text;

struct TfLink {
  std::string parent_frame;
  std::string child_frame;
  bool is_static{false};
  builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::Transform transform;
  Clock::time_point received_time{};
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

std::string normalize_frame(std::string name) {
  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  return name.empty() ? "<unnamed>" : name;
}

std::string format_freshness(const TfLink & link, const Clock::time_point now, bool & stale) {
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

Quat normalize_quat(Quat quat) {
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

Quat quat_multiply_raw(const Quat & lhs, const Quat & rhs) {
  return Quat{
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z};
}

Quat quat_multiply(const Quat & lhs, const Quat & rhs) {
  return normalize_quat(quat_multiply_raw(lhs, rhs));
}

Quat quat_conjugate(const Quat & quat) {
  return Quat{-quat.x, -quat.y, -quat.z, quat.w};
}

Vec3 rotate_vec(const Quat & quat, const Vec3 & vec) {
  const Quat vector_quat{vec.x, vec.y, vec.z, 0.0};
  const Quat rotated = quat_multiply_raw(
    quat_multiply_raw(quat, vector_quat),
    quat_conjugate(quat));
  return Vec3{rotated.x, rotated.y, rotated.z};
}

Pose compose_pose(const Pose & lhs, const Pose & rhs) {
  return Pose{
    Vec3{
      lhs.translation.x + rotate_vec(lhs.rotation, rhs.translation).x,
      lhs.translation.y + rotate_vec(lhs.rotation, rhs.translation).y,
      lhs.translation.z + rotate_vec(lhs.rotation, rhs.translation).z},
    quat_multiply(lhs.rotation, rhs.rotation)};
}

Pose invert_pose(const Pose & pose) {
  const Quat inverse_rotation = quat_conjugate(normalize_quat(pose.rotation));
  const Vec3 inverse_translation = rotate_vec(
    inverse_rotation,
    Vec3{-pose.translation.x, -pose.translation.y, -pose.translation.z});
  return Pose{inverse_translation, inverse_rotation};
}

Pose pose_from_msg(const geometry_msgs::msg::Transform & transform) {
  return Pose{
    Vec3{transform.translation.x, transform.translation.y, transform.translation.z},
    normalize_quat(Quat{
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z,
      transform.rotation.w})};
}

Vec3 rpy_from_quat(const Quat & quat_in) {
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

class TfMonitorNode : public rclcpp::Node {
public:
  TfMonitorNode()
  : Node("tf_monitor") {}

  void initialize_subscriptions() {
    tf_subscription_ = this->create_subscription<TFMessage>(
      "/tf", rclcpp::QoS(rclcpp::KeepLast(200)),
      [this](const TFMessage & message) { handle_tf_message(message, false); });

    tf_static_subscription_ = this->create_subscription<TFMessage>(
      "/tf_static", rclcpp::QoS(rclcpp::KeepLast(200)).reliable().transient_local(),
      [this](const TFMessage & message) { handle_tf_message(message, true); });
  }

  int run() {
    Session ncurses_session;
    refresh_rows();

    bool running = true;
    while (running && rclcpp::ok()) {
      refresh_rows();
      draw();
      const int key = getch();
      if (key == ERR) {
        continue;
      }
      running = handle_key(key);
    }

    return 0;
  }

private:
  bool handle_key(int key) {
    if (inspect_popup_open_) {
      return handle_popup_key(key);
    }
    if (search_state_.active) {
      return handle_search_key(key);
    }

    switch (key) {
      case KEY_F(10):
        return false;
      case KEY_F(4):
        refresh_rows();
        return true;
      case 27:
        if (is_alt_binding(key, 's')) {
          start_search(search_state_);
          set_status("Search.");
          return true;
        }
        return true;
      case ' ':
      case KEY_IC:
        toggle_selected_row();
        return true;
      case '\n':
      case KEY_ENTER:
        open_inspect_popup();
        return true;
      case KEY_UP:
      case 'k':
        if (selected_index_ > 0) {
          --selected_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_index_ + 1 < static_cast<int>(rows_.size())) {
          ++selected_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_index_ = std::max(0, selected_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!rows_.empty()) {
          selected_index_ = std::min(static_cast<int>(rows_.size()) - 1, selected_index_ + page_step());
        }
        return true;
      default:
        return true;
    }
  }

  bool handle_popup_key(int key) {
    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
      case '\n':
      case KEY_ENTER:
        inspect_popup_open_ = false;
        return true;
      default:
        return true;
    }
  }

  bool handle_search_key(int key) {
    const SearchInputResult result = handle_search_input(search_state_, key);
    if (result == SearchInputResult::Cancelled) {
      set_status("Search cancelled.");
      return true;
    }
    if (result == SearchInputResult::Accepted) {
      set_status(search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query);
      return true;
    }
    if (result != SearchInputResult::Changed) {
      return true;
    }

    std::vector<std::string> labels;
    labels.reserve(rows_.size());
    for (const auto & row : rows_) {
      labels.push_back(row.parent_frame + " " + row.child_frame);
    }
    const int match = find_best_match(labels, search_state_.query, selected_index_);
    if (match >= 0) {
      selected_index_ = match;
    }
    set_status("Search: " + search_state_.query);
    return true;
  }

  int page_step() const {
    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    (void)columns;
    return std::max(5, rows - 8);
  }

  void handle_tf_message(const TFMessage & message, bool is_static) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
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

  void toggle_selected_row() {
    clamp_selection();
    if (rows_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(rows_.size())) {
      set_status("No TF link selected.");
      return;
    }

    const std::string frame = rows_[static_cast<std::size_t>(selected_index_)].child_frame;
    auto found = std::find(selected_frames_.begin(), selected_frames_.end(), frame);
    if (found == selected_frames_.end()) {
      selected_frames_.push_back(frame);
      set_status("Selected " + frame + ".");
    } else {
      selected_frames_.erase(found);
      set_status("Deselected " + frame + ".");
    }
  }

  std::optional<InspectResult> compute_selected_transform() const {
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

  void open_inspect_popup() {
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

  void refresh_rows() {
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
    const auto now = Clock::now();
    std::set<std::string> visited;

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
            depth,
            format_freshness(link_it->second, now, stale),
            stale});
          append_children(child, depth + 1, root_frame, child_pose);
        }
      };

    for (const auto & root : roots) {
      frame_poses_[root] = FramePose{root, Pose{}};
      append_children(root, 0, root, Pose{});
    }
    for (const auto & [child, link] : links_by_child_) {
      if (visited.find(child) != visited.end()) {
        continue;
      }
      bool stale = false;
      frame_poses_[link.parent_frame] = FramePose{link.parent_frame, Pose{}};
      const Pose child_pose = pose_from_msg(link.transform);
      frame_poses_[child] = FramePose{link.parent_frame, child_pose};
      rows_.push_back(TfRow{
        link.parent_frame,
        link.child_frame,
        link.is_static,
        0,
        format_freshness(link, now, stale),
        stale});
    }

    selected_frames_.erase(
      std::remove_if(
        selected_frames_.begin(), selected_frames_.end(),
        [this](const std::string & frame) { return frame_poses_.find(frame) == frame_poses_.end(); }),
      selected_frames_.end());

    clamp_selection();
    status_line_ =
      "Loaded " + std::to_string(rows_.size()) + " TF links from "
      + std::to_string(links_by_child_.size()) + " child frames.";
  }

  void clamp_selection() {
    if (rows_.empty()) {
      selected_index_ = 0;
      scroll_ = 0;
      return;
    }
    selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(rows_.size()) - 1);
  }

  void draw() {
    erase();

    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    const int help_row = rows - 1;
    const int status_row = rows - 2;
    const int content_bottom = std::max(1, status_row - 1);

    draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
    mvprintw(0, 1, "TF Monitor ");
    draw_tree_pane(1, 1, content_bottom - 1, columns - 2);
    draw_status_line(status_row, columns);
    draw_help_line(help_row, columns);
    draw_search_box(rows, columns, search_state_);
    if (inspect_popup_open_) {
      draw_inspect_popup(rows, columns);
    }
    refresh();
  }

  void draw_tree_pane(int top, int left, int bottom, int right) {
    clamp_selection();

    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    const int tree_width = std::max(24, width - 14);
    const int freshness_width = std::max(10, width - tree_width - 1);
    const int sep_one_x = left + tree_width;

    if (selected_index_ < scroll_) {
      scroll_ = selected_index_;
    }
    if (selected_index_ >= scroll_ + visible_rows - 1) {
      scroll_ = std::max(0, selected_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", tree_width, "Transform");
    draw_box_char(top, sep_one_x, WACS_VLINE, '|');
    mvprintw(top, sep_one_x + 1, "%-*s", freshness_width, "Freshness");
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = scroll_;
    const int last_row = std::min(static_cast<int>(rows_.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_index_);

      mvhline(row, left, ' ', width);
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }
      draw_box_char(row, sep_one_x, WACS_VLINE, '|');

      if (!has_item) {
        continue;
      }

      const auto & entry = rows_[static_cast<std::size_t>(first_row + (row - top - 1))];
      const bool marked = std::find(selected_frames_.begin(), selected_frames_.end(), entry.child_frame) != selected_frames_.end();
      const std::string tree_text =
        std::string(marked ? "* " : "  ") +
        std::string(static_cast<std::size_t>(entry.depth * 2), ' ') +
        entry.child_frame;
      const int text_color = entry.stale ? kColorStale : (entry.is_static ? 0 : kColorDynamic);

      if (text_color != 0 && !selected) {
        attron(COLOR_PAIR(text_color));
      }
      mvprintw(row, left, "%-*s", tree_width, truncate_text(tree_text, tree_width).c_str());
      mvprintw(row, sep_one_x + 1, "%-*s", freshness_width, truncate_text(entry.freshness, freshness_width).c_str());
      if (text_color != 0 && !selected) {
        attroff(COLOR_PAIR(text_color));
      }

      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        mvaddch(row, sep_one_x, '|');
      }
    }
  }

  void draw_status_line(int row, int columns) const {
    std::string line = status_line_;
    if (!rows_.empty() && selected_index_ >= 0 && selected_index_ < static_cast<int>(rows_.size())) {
      const auto & selected = rows_[static_cast<std::size_t>(selected_index_)];
      line = selected.parent_frame + " -> " + selected.child_frame + "  " + status_line_;
    }
    if (!selected_frames_.empty()) {
      line += "  selected=" + std::to_string(selected_frames_.size());
    }
    draw_status_bar(row, columns, line);
  }

  void draw_help_line(int row, int columns) const {
    draw_help_bar(row, columns, "Space Select  Enter Inspect  Alt+S Search  F4 Refresh  F10 Exit");
  }

  void draw_inspect_popup(int rows, int columns) const {
    const int popup_width = std::min(columns - 6, 72);
    const int popup_height = 11;
    const int left = std::max(2, (columns - popup_width) / 2);
    const int top = std::max(2, (rows - popup_height) / 2);
    const int right = left + popup_width - 1;
    const int bottom = top + popup_height - 1;
    const int inner_width = popup_width - 2;

    for (int row = top + 1; row < bottom; ++row) {
      attron(COLOR_PAIR(tui::kColorPopup));
      mvhline(row, left + 1, ' ', inner_width);
      attroff(COLOR_PAIR(tui::kColorPopup));
    }
    draw_box(top, left, bottom, right, kColorFrame);

    attron(COLOR_PAIR(kColorHeader) | A_BOLD);
    mvprintw(top, left + 2, " TF Inspect ");
    attroff(COLOR_PAIR(kColorHeader) | A_BOLD);

    const Vec3 rpy = rpy_from_quat(inspect_result_.transform.rotation);
    const auto print_line = [&](int row, const std::string & text) {
      mvaddnstr(row, left + 2, truncate_text(text, popup_width - 4).c_str(), popup_width - 4);
    };

    char buffer[128];
    print_line(top + 1, inspect_result_.from_frame + " -> " + inspect_result_.to_frame);
    std::snprintf(
      buffer, sizeof(buffer), "XYZ:  %.3f  %.3f  %.3f",
      inspect_result_.transform.translation.x,
      inspect_result_.transform.translation.y,
      inspect_result_.transform.translation.z);
    print_line(top + 3, buffer);
    std::snprintf(
      buffer, sizeof(buffer), "RPY:  %.3f  %.3f  %.3f",
      rpy.x, rpy.y, rpy.z);
    print_line(top + 4, buffer);
    std::snprintf(
      buffer, sizeof(buffer), "Quat: %.4f  %.4f  %.4f  %.4f",
      inspect_result_.transform.rotation.x,
      inspect_result_.transform.rotation.y,
      inspect_result_.transform.rotation.z,
      inspect_result_.transform.rotation.w);
    print_line(top + 6, buffer);
    draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "Enter Close  Esc Close  F10 Exit");
  }

  void set_status(const std::string & text) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_line_ = text;
  }

  mutable std::mutex mutex_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_static_subscription_;
  std::map<std::string, TfLink> links_by_child_;
  std::map<std::string, FramePose> frame_poses_;
  std::vector<TfRow> rows_;
  std::vector<std::string> selected_frames_;
  int selected_index_{0};
  int scroll_{0};
  SearchState search_state_;
  bool inspect_popup_open_{false};
  InspectResult inspect_result_;
  std::string status_line_{"Waiting for TF messages..."};
};

}  // namespace

}  // namespace ros2_console_tools

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_console_tools::TfMonitorNode>();
  node->initialize_subscriptions();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = node->run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
