#ifndef ROS2_CONSOLE_TOOLS__SYSTEMD_COMMANDER_HPP_
#define ROS2_CONSOLE_TOOLS__SYSTEMD_COMMANDER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_console_tools/systemd_client.hpp"
#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_systemd_commander_tool(const std::string & initial_unit = "", bool embedded_mode = false);

struct SystemdDetailRow {
  std::string text;
  bool is_header{false};
};

class SystemdCommanderScreen;

class SystemdCommanderBackend {
public:
  explicit SystemdCommanderBackend(const std::string & initial_unit = "");

private:
  friend class SystemdCommanderScreen;

  void refresh_units();
  void refresh_selected_unit_details();
  void clamp_selection();
  std::string selected_unit_name() const;
  std::vector<SystemdDetailRow> detail_rows_snapshot() const;
  bool perform_action(const std::string & action);

  mutable std::mutex mutex_;
  SystemdClient client_;
  std::vector<SystemdUnitSummary> units_;
  SystemdUnitDetails selected_unit_details_;
  std::string selected_details_unit_;
  std::string initial_unit_;
  int selected_index_{0};
  int unit_scroll_{0};
  std::string status_line_{"Loading services..."};
};

class SystemdCommanderScreen {
public:
  explicit SystemdCommanderScreen(
    std::shared_ptr<SystemdCommanderBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_unit_list_key(int key);
  bool handle_detail_popup_key(int key);
  bool launch_selected_logs();
  int page_step() const;
  void draw();
  void draw_unit_list(int top, int left, int bottom, int right);
  void draw_detail_popup(int rows, int columns);
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<SystemdCommanderBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
  int detail_scroll_{0};
  bool detail_popup_open_{false};
  tui::TerminalPane terminal_pane_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__SYSTEMD_COMMANDER_HPP_
