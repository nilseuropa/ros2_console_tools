#ifndef ROS2_CONSOLE_TOOLS__JOURNAL_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__JOURNAL_VIEWER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_console_tools/journal_client.hpp"
#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_journal_viewer_tool(const std::string & initial_unit = "", bool embedded_mode = false);

struct JournalDetailRow {
  std::string text;
  bool is_header{false};
};

enum class JournalViewerFocusPane {
  EntryList,
  DetailPane,
};

class JournalViewerScreen;

class JournalViewerBackend {
public:
  explicit JournalViewerBackend(const std::string & initial_unit = "");

private:
  friend class JournalViewerScreen;

  void refresh_entries();
  void clamp_selection();
  void cycle_priority_filter();
  void set_text_filter(const std::string & filter_text);
  std::string priority_filter_label() const;
  std::vector<JournalDetailRow> detail_rows_snapshot() const;

  mutable std::mutex mutex_;
  JournalClient client_;
  std::vector<JournalEntry> entries_;
  std::string unit_filter_;
  std::string text_filter_;
  int max_priority_{6};
  int line_count_{200};
  int selected_index_{0};
  int entry_scroll_{0};
  std::string status_line_{"Loading journal entries..."};
};

class JournalViewerScreen {
public:
  explicit JournalViewerScreen(
    std::shared_ptr<JournalViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_filter_prompt_key(int key);
  bool handle_entry_list_key(int key);
  bool handle_detail_key(int key);
  int page_step() const;
  void draw();
  void draw_entry_list(int top, int left, int bottom, int right);
  void draw_detail_pane(int top, int left, int bottom, int right);
  void draw_filter_popup(int rows, int columns) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<JournalViewerBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
  JournalViewerFocusPane focus_pane_{JournalViewerFocusPane::EntryList};
  int detail_scroll_{0};
  bool filter_prompt_open_{false};
  std::string filter_buffer_;
  tui::TerminalPane terminal_pane_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__JOURNAL_VIEWER_HPP_
