#ifndef ROS2_CONSOLE_TOOLS__ACTION_COMMANDER_HPP_
#define ROS2_CONSOLE_TOOLS__ACTION_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_action_commander_tool(const std::string & initial_action = "", bool embedded_mode = false);

struct ActionEntry {
  std::string name;
  std::string type;
};

struct ActionDetailLine {
  std::string text;
  bool is_header{false};
};

class ActionCommanderScreen;

class ActionCommanderBackend : public rclcpp::Node {
public:
  explicit ActionCommanderBackend(const std::string & initial_action = "");

private:
  friend class ActionCommanderScreen;

  struct ActionDetails {
    std::string name;
    std::string type;
    std::string send_goal_service;
    std::string get_result_service;
    std::string cancel_goal_service;
    std::string feedback_topic;
    std::string status_topic;
    std::size_t send_goal_servers{0};
    std::size_t send_goal_clients{0};
    std::size_t get_result_servers{0};
    std::size_t get_result_clients{0};
    std::size_t cancel_servers{0};
    std::size_t cancel_clients{0};
    std::size_t feedback_publishers{0};
    std::size_t feedback_subscribers{0};
    std::size_t status_publishers{0};
    std::size_t status_subscribers{0};
    std::vector<std::string> server_nodes;
    std::vector<std::string> client_nodes;
  };

  void refresh_actions();
  void clamp_selection();
  void select_initial_action_if_present();
  void open_selected_action();
  void close_action_detail();
  std::vector<ActionDetailLine> selected_action_details() const;
  ActionDetails build_action_details(const ActionEntry & entry) const;
  static std::string action_name_from_send_goal_service(const std::string & service_name);
  static std::string action_type_from_send_goal_type(const std::string & service_type);
  static std::string fully_qualified_node_name(const std::string & ns, const std::string & name);

  std::vector<ActionEntry> actions_;
  std::string initial_action_name_;
  bool auto_open_initial_action_{false};
  bool detail_open_{false};
  int selected_index_{0};
  int list_scroll_{0};
  std::string status_line_{"Loading actions..."};
  ActionEntry selected_action_;
};

class ActionCommanderScreen {
public:
  explicit ActionCommanderScreen(
    std::shared_ptr<ActionCommanderBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_list_key(int key);
  bool handle_detail_key(int key);
  int page_step() const;
  void draw();
  void draw_action_list(int top, int left, int bottom, int right);
  void draw_action_detail(int top, int left, int bottom, int right) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<ActionCommanderBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__ACTION_COMMANDER_HPP_
