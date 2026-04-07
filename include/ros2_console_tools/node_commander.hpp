#ifndef ROS2_CONSOLE_TOOLS__NODE_COMMANDER_HPP_
#define ROS2_CONSOLE_TOOLS__NODE_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

enum class NodeDetailAction {
  None,
  OpenParameters,
  OpenTopicMonitor,
  OpenServiceCommander,
};

struct GraphEndpoint {
  std::string name;
  std::string type;
};

struct NodeDetails {
  std::string node_name;
  std::vector<GraphEndpoint> publishers;
  std::vector<GraphEndpoint> subscribers;
  std::vector<GraphEndpoint> services;
  bool parameter_services_ready{false};
};

struct DetailLine {
  std::string text;
  bool is_header{false};
  int depth{0};
  NodeDetailAction action{NodeDetailAction::None};
  std::string target;
  std::string section_key;
};

enum class NodeCommanderFocusPane {
  NodeList,
  DetailPane,
};

class NodeCommanderScreen;

class NodeCommanderBackend : public rclcpp::Node {
public:
  NodeCommanderBackend();

private:
  friend class NodeCommanderScreen;

  void refresh_nodes();
  void warm_up_node_list();
  void maybe_refresh_nodes();
  void clamp_selection();
  std::string fully_qualified_node_name(const std::string & ns, const std::string & name) const;
  std::vector<DetailLine> selected_node_details() const;
  NodeDetails build_node_details(const std::string & full_name) const;

  std::vector<std::string> node_entries_;
  int selected_index_{0};
  int node_scroll_{0};
  mutable std::mutex mutex_;
  std::string status_line_{"Loading nodes..."};
  std::chrono::steady_clock::time_point last_refresh_time_{std::chrono::steady_clock::time_point::min()};
};

class NodeCommanderScreen {
public:
  explicit NodeCommanderScreen(std::shared_ptr<NodeCommanderBackend> backend);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  int page_step() const;
  std::string selected_node_name() const;
  void refresh_detail_lines_cache(bool force = false);
  void focus_detail_pane();
  int preferred_detail_index() const;
  bool is_detail_section_collapsed(const std::string & section_key) const;
  bool expand_selected_detail_section();
  bool collapse_selected_detail_section();
  bool launch_parameter_commander();
  bool launch_log_viewer();
  bool launch_action_commander();
  bool launch_service_commander();
  bool launch_topic_monitor();
  bool launch_tf_monitor();
  bool launch_urdf_inspector();
  bool launch_selected_node_parameters();
  bool launch_selected_detail_action();
  const DetailLine * selected_detail_line() const;
  void draw();
  void draw_node_list(int top, int left, int bottom, int right);
  void draw_detail_pane(int top, int left, int bottom, int right);
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<NodeCommanderBackend> backend_;
  tui::SearchState search_state_;
  NodeCommanderFocusPane focus_pane_{NodeCommanderFocusPane::NodeList};
  int detail_selected_index_{0};
  int detail_scroll_{0};
  std::string detail_cache_node_name_;
  std::chrono::steady_clock::time_point detail_cache_refresh_time_{
    std::chrono::steady_clock::time_point::min()};
  std::vector<DetailLine> raw_detail_lines_cache_;
  std::vector<DetailLine> detail_lines_cache_;
  std::map<std::string, bool> collapsed_detail_sections_;
  bool help_popup_open_{false};
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__NODE_COMMANDER_HPP_
