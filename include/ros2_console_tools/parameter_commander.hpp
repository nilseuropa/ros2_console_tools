#ifndef ROS2_CONSOLE_TOOLS__PARAMETER_COMMANDER_HPP_
#define ROS2_CONSOLE_TOOLS__PARAMETER_COMMANDER_HPP_

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_parameter_commander_tool(const std::string & target_node = "", bool embedded_mode = false);

using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using ParameterType = rcl_interfaces::msg::ParameterType;

inline std::string trim(std::string value) {
  auto not_space = [](unsigned char character) { return !std::isspace(character); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

inline std::string single_line(std::string value) {
  for (char & character : value) {
    if (character == '\n' || character == '\r' || character == '\t') {
      character = ' ';
    }
  }
  value = trim(value);

  std::string collapsed;
  collapsed.reserve(value.size());
  bool previous_space = false;
  for (unsigned char character : value) {
    if (std::isspace(character)) {
      if (!previous_space) {
        collapsed.push_back(' ');
      }
      previous_space = true;
    } else {
      collapsed.push_back(static_cast<char>(character));
      previous_space = false;
    }
  }
  return collapsed;
}

inline std::string parameter_type_name(uint8_t type) {
  switch (type) {
    case ParameterType::PARAMETER_BOOL:
      return "bool";
    case ParameterType::PARAMETER_INTEGER:
      return "int";
    case ParameterType::PARAMETER_DOUBLE:
      return "double";
    case ParameterType::PARAMETER_STRING:
      return "string";
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool[]";
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return "int[]";
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double[]";
    case ParameterType::PARAMETER_STRING_ARRAY:
      return "string[]";
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte[]";
    case ParameterType::PARAMETER_NOT_SET:
    default:
      return "unset";
  }
}

inline std::string parameter_value_to_string(const rclcpp::ParameterValue & value) {
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return value.get<bool>() ? "true" : "false";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(value.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      std::ostringstream stream;
      stream << value.get<double>();
      return stream.str();
    }
    case rclcpp::ParameterType::PARAMETER_STRING:
      return value.get<std::string>();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return "<array>";
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return "<unset>";
  }
}

inline int visible_width(const std::string & line) {
  int width = 0;
  for (std::size_t index = 0; index < line.size(); ++index) {
    if (line[index] == '\x1b' && index + 1 < line.size() && line[index + 1] == '[') {
      index += 2;
      while (index < line.size() && line[index] != 'm' && line[index] != 'K') {
        ++index;
      }
      continue;
    }
    ++width;
  }
  return width;
}

inline std::string truncate_parameter_line(const std::string & line, int max_columns) {
  if (max_columns <= 0) {
    return "";
  }
  if (visible_width(line) <= max_columns) {
    return line;
  }

  const int reserved_columns = max_columns > 3 ? 3 : 0;
  const int target_width = std::max(0, max_columns - reserved_columns);
  std::string output;
  int width = 0;

  for (std::size_t index = 0; index < line.size() && width < target_width; ++index) {
    if (line[index] == '\x1b' && index + 1 < line.size() && line[index + 1] == '[') {
      const std::size_t escape_start = index;
      index += 2;
      while (index < line.size() && line[index] != 'm' && line[index] != 'K') {
        ++index;
      }
      if (index < line.size()) {
        output.append(line, escape_start, index - escape_start + 1);
      }
      continue;
    }

    output.push_back(line[index]);
    ++width;
  }

  if (reserved_columns > 0) {
    output += "...";
  }
  if (output.find("\x1b[") != std::string::npos) {
    output += "\x1b[0m";
  }
  return output;
}

inline std::string tail_fit(const std::string & value, int max_columns) {
  if (max_columns <= 0) {
    return "";
  }
  if (static_cast<int>(value.size()) <= max_columns) {
    return value;
  }
  return value.substr(value.size() - static_cast<std::size_t>(max_columns));
}

inline bool is_scalar_type(uint8_t type) {
  return type == ParameterType::PARAMETER_BOOL
    || type == ParameterType::PARAMETER_INTEGER
    || type == ParameterType::PARAMETER_DOUBLE
    || type == ParameterType::PARAMETER_STRING;
}

inline bool software_caret_visible() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto phase = std::chrono::duration_cast<std::chrono::milliseconds>(now).count() / 500;
  return (phase % 2) == 0;
}

struct ParameterEntry {
  std::string name;
  ParameterDescriptor descriptor;
  rclcpp::ParameterValue value;
  bool has_value{false};
  bool dirty{false};
  std::string edit_buffer;
};

struct ParameterViewItem {
  bool is_namespace{false};
  std::string label;
  std::string namespace_path;
  int depth{0};
  ParameterEntry * entry{nullptr};
};

enum class ParameterCommanderViewMode {
  NodeList,
  ParameterList,
};

class ParameterCommanderScreen;

class ParameterCommanderBackend : public rclcpp::Node {
public:
  explicit ParameterCommanderBackend(const std::string & target_node);

private:
  friend class ParameterCommanderScreen;

  void refresh_all();
  void refresh_selected();
  void save_selected();
  std::optional<rclcpp::Parameter> parse_edit_buffer(const ParameterEntry & entry);
  void refresh_node_list();
  void select_current_node();
  void switch_to_node_list();
  void reset_parameter_client();
  void maybe_refresh_node_list();
  void warm_up_node_list();
  std::string lowercase(std::string value) const;
  std::string normalize_target_name(const std::string & name) const;
  std::string fully_qualified_node_name(const std::string & ns, const std::string & name) const;
  void toggle_bool_buffer(std::string & buffer);
  bool wait_for_parameter_service();

  template<typename FutureT>
  bool wait_for_future(FutureT & future, const std::string & action) {
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      set_status("Timed out trying to " + action + " on " + target_node_ + ".");
      return false;
    }
    return true;
  }

  void sync_edit_buffer_from_selected();
  ParameterEntry * selected_entry();
  const ParameterEntry * selected_entry() const;
  void activate_selected_parameter_item();
  void expand_selected_namespace();
  void collapse_selected_namespace();
  std::vector<ParameterViewItem> visible_parameter_items() const;
  void append_namespace_children(
    const std::string & parent_namespace, int depth, std::vector<ParameterViewItem> & items) const;
  std::string parameter_namespace(const std::string & parameter_name) const;
  std::string parameter_leaf_name(const std::string & parameter_name) const;
  std::string namespace_label(const std::string & namespace_path) const;
  bool is_in_namespace(const std::string & entry_namespace, const std::string & parent_namespace) const;
  bool is_namespace_expanded(const std::string & namespace_path) const;
  std::string summary_value(const ParameterEntry & entry) const;
  std::string descriptor_summary(const ParameterDescriptor & descriptor) const;
  std::string parameter_min(const ParameterEntry & entry) const;
  std::string parameter_max(const ParameterEntry & entry) const;
  bool popup_is_editable(const ParameterEntry & entry) const;
  void set_status(const std::string & message);

  std::string target_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  ParameterCommanderViewMode current_view_{ParameterCommanderViewMode::NodeList};
  std::vector<std::string> node_entries_;
  std::vector<ParameterEntry> entries_;
  std::map<std::string, bool> collapsed_namespaces_;
  std::chrono::steady_clock::time_point last_node_refresh_time_{
    std::chrono::steady_clock::time_point::min()};
  int selected_node_index_{0};
  int node_scroll_{0};
  int selected_parameter_item_index_{0};
  int list_scroll_{0};
  std::string status_message_{"F4 to load parameters."};
};

class ParameterCommanderScreen {
public:
  explicit ParameterCommanderScreen(
    std::shared_ptr<ParameterCommanderBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_popup_key(int key);
  void handle_list_key(int key);
  void handle_node_list_key(int key);
  int page_step() const;
  void open_popup();
  void close_popup();
  void draw();
  void draw_parameter_list(int top, int left, int bottom, int right);
  void draw_parameter_name_cell(int row, int left, int width, const ParameterViewItem & item) const;
  void draw_node_list(int top, int left, int bottom, int right);
  void draw_popup(int rows, int columns);
  std::string pad_column(const std::string & value, int width) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<ParameterCommanderBackend> backend_;
  bool embedded_mode_{false};
  bool popup_open_{false};
  bool popup_dirty_{false};
  tui::SearchState search_state_;
  std::string popup_buffer_;
  tui::TerminalPane terminal_pane_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__PARAMETER_COMMANDER_HPP_
