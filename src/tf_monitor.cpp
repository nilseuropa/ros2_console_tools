#include <ncursesw/ncurses.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
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
  Clock::time_point received_time{};
};

struct TfRow {
  std::string parent_frame;
  std::string child_frame;
  bool is_static{false};
  int depth{0};
  std::string freshness;
  bool stale{false};
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
      link.received_time = now;
      links_by_child_[link.child_frame] = std::move(link);
    }
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
    const auto now = Clock::now();
    std::set<std::string> visited;

    std::function<void(const std::string &, int)> append_children =
      [&](const std::string & parent, int depth) {
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
          rows_.push_back(TfRow{
            link_it->second.parent_frame,
            link_it->second.child_frame,
            link_it->second.is_static,
            depth,
            format_freshness(link_it->second, now, stale),
            stale});
          append_children(child, depth + 1);
        }
      };

    for (const auto & root : roots) {
      append_children(root, 0);
    }
    for (const auto & [child, link] : links_by_child_) {
      if (visited.find(child) != visited.end()) {
        continue;
      }
      bool stale = false;
      rows_.push_back(TfRow{
        link.parent_frame,
        link.child_frame,
        link.is_static,
        0,
        format_freshness(link, now, stale),
        stale});
    }

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
      const std::string tree_text =
        std::string(static_cast<std::size_t>(entry.depth * 2), ' ') + entry.parent_frame + " -> " + entry.child_frame;
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
    draw_status_bar(row, columns, line);
  }

  void draw_help_line(int row, int columns) const {
    draw_help_bar(row, columns, "Alt+S Search  F4 Refresh  F10 Exit");
  }

  void set_status(const std::string & text) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_line_ = text;
  }

  mutable std::mutex mutex_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_static_subscription_;
  std::map<std::string, TfLink> links_by_child_;
  std::vector<TfRow> rows_;
  int selected_index_{0};
  int scroll_{0};
  SearchState search_state_;
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
