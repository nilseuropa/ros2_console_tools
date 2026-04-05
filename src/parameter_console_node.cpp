#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_console_tools {

namespace {

using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using ParameterType = rcl_interfaces::msg::ParameterType;

std::string trim(std::string value) {
  auto not_space = [](unsigned char character) { return !std::isspace(character); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string parameter_type_name(uint8_t type) {
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

std::string parameter_value_to_string(const rclcpp::ParameterValue & value) {
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

int visible_width(const std::string & line) {
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

std::string truncate_line(const std::string & line, int max_columns) {
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

std::string tail_fit(const std::string & value, int max_columns) {
  if (max_columns <= 0) {
    return "";
  }
  if (static_cast<int>(value.size()) <= max_columns) {
    return value;
  }
  return value.substr(value.size() - static_cast<std::size_t>(max_columns));
}

bool is_scalar_type(uint8_t type) {
  return type == ParameterType::PARAMETER_BOOL
    || type == ParameterType::PARAMETER_INTEGER
    || type == ParameterType::PARAMETER_DOUBLE
    || type == ParameterType::PARAMETER_STRING;
}

enum ColorPairId {
  kColorFrame = 1,
  kColorTitle = 2,
  kColorHeader = 3,
  kColorSelection = 4,
  kColorStatus = 5,
  kColorHelp = 6,
  kColorPopup = 7,
  kColorInput = 8,
  kColorDirty = 9,
  kColorCursor = 10,
};

bool software_caret_visible() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto phase = std::chrono::duration_cast<std::chrono::milliseconds>(now).count() / 500;
  return (phase % 2) == 0;
}

class NcursesSession {
public:
  NcursesSession() {
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(100);
    if (has_colors()) {
      start_color();
      use_default_colors();
      init_pair(kColorFrame, COLOR_CYAN, -1);
      init_pair(kColorTitle, COLOR_WHITE, -1);
      init_pair(kColorHeader, COLOR_YELLOW, -1);
      init_pair(kColorSelection, COLOR_BLACK, COLOR_YELLOW);
      init_pair(kColorStatus, COLOR_GREEN, -1);
      init_pair(kColorHelp, COLOR_BLACK, COLOR_CYAN);
      init_pair(kColorPopup, COLOR_WHITE, -1);
      init_pair(kColorInput, COLOR_YELLOW, -1);
      init_pair(kColorDirty, COLOR_RED, -1);
      init_pair(kColorCursor, COLOR_BLACK, COLOR_YELLOW);
    }
  }

  ~NcursesSession() {
    curs_set(1);
    endwin();
  }
};

struct ParameterEntry {
  std::string name;
  ParameterDescriptor descriptor;
  rclcpp::ParameterValue value;
  bool has_value{false};
  bool dirty{false};
  std::string edit_buffer;
};

enum class ViewMode {
  NodeList,
  ParameterList,
};

class ParameterConsoleNode : public rclcpp::Node {
public:
  explicit ParameterConsoleNode(const std::string & target_node)
  : Node("parameter_console_node"),
    target_node_(normalize_target_name(this->declare_parameter<std::string>("target_node", target_node))) {
    current_view_ = target_node_.empty() ? ViewMode::NodeList : ViewMode::ParameterList;
    if (!target_node_.empty()) {
      reset_parameter_client();
    }
  }

  int run() {
    NcursesSession ncurses_session;
    refresh_node_list();
    if (current_view_ == ViewMode::NodeList) {
      warm_up_node_list();
      set_status("Choose a node and press Enter.");
    } else {
      set_status("Connecting to " + target_node_ + "...");
      if (!client_ || !client_->wait_for_service(std::chrono::seconds(2))) {
        set_status("Parameter services for " + target_node_ + " are not available.");
      } else {
        refresh_all();
      }
    }

    bool running = true;
    while (running && rclcpp::ok()) {
      maybe_refresh_node_list();
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
    if (popup_open_) {
      return handle_popup_key(key);
    }

    switch (key) {
      case KEY_F(10):
        curs_set(1);
        return false;
      case KEY_F(4):
        refresh_all();
        return true;
      case KEY_F(3):
        if (current_view_ == ViewMode::NodeList) {
          refresh_node_list();
        } else {
          refresh_selected();
        }
        return true;
      case 27:
        if (current_view_ == ViewMode::ParameterList) {
          switch_to_node_list();
          return true;
        }
        break;
      case '\n':
      case KEY_ENTER:
        if (current_view_ == ViewMode::NodeList) {
          select_current_node();
        } else {
          open_popup();
        }
        return true;
      default:
        break;
    }

    if (current_view_ == ViewMode::NodeList) {
      handle_node_list_key(key);
    } else {
      handle_list_key(key);
    }
    return true;
  }

  bool handle_popup_key(int key) {
    auto * entry = selected_entry();
    if (entry == nullptr) {
      popup_open_ = false;
      return true;
    }

    switch (key) {
      case KEY_F(10):
        close_popup();
        curs_set(1);
        return false;
      case 27:
        close_popup();
        return true;
      case KEY_F(3):
        refresh_selected();
        popup_buffer_ = entry->edit_buffer;
        popup_dirty_ = false;
        return true;
      case KEY_F(2):
        entry->edit_buffer = popup_buffer_;
        entry->dirty = popup_dirty_;
        save_selected();
        popup_buffer_ = entry->edit_buffer;
        popup_dirty_ = entry->dirty;
        return true;
      case '\n':
      case KEY_ENTER:
        entry->edit_buffer = popup_buffer_;
        entry->dirty = popup_dirty_;
        save_selected();
        if (!entry->dirty) {
          close_popup();
        } else {
          popup_buffer_ = entry->edit_buffer;
          popup_dirty_ = entry->dirty;
        }
        return true;
      case KEY_BACKSPACE:
      case 127:
      case '\b':
        if (!popup_buffer_.empty() && popup_is_editable(*entry)) {
          popup_buffer_.pop_back();
          popup_dirty_ = true;
        }
        return true;
      case KEY_DC:
        if (popup_is_editable(*entry)) {
          popup_buffer_.clear();
          popup_dirty_ = true;
        }
        return true;
      case ' ':
        if (!popup_is_editable(*entry)) {
          return true;
        }
        if (entry->descriptor.type == ParameterType::PARAMETER_BOOL) {
          toggle_bool_buffer(popup_buffer_);
        } else {
          popup_buffer_.push_back(' ');
        }
        popup_dirty_ = true;
        return true;
      default:
        if (key >= 32 && key <= 126 && popup_is_editable(*entry)) {
          popup_buffer_.push_back(static_cast<char>(key));
          popup_dirty_ = true;
        }
        return true;
    }
  }

  void handle_list_key(int key) {
    if (entries_.empty()) {
      return;
    }

    switch (key) {
      case KEY_UP:
      case 'k':
        if (selected_index_ > 0) {
          --selected_index_;
        }
        sync_edit_buffer_from_selected();
        break;
      case KEY_DOWN:
      case 'j':
        if (selected_index_ + 1 < static_cast<int>(entries_.size())) {
          ++selected_index_;
        }
        sync_edit_buffer_from_selected();
        break;
      case KEY_PPAGE:
        selected_index_ = std::max(0, selected_index_ - page_step());
        sync_edit_buffer_from_selected();
        break;
      case KEY_NPAGE:
        selected_index_ = std::min(static_cast<int>(entries_.size()) - 1, selected_index_ + page_step());
        sync_edit_buffer_from_selected();
        break;
      default:
        break;
    }
  }

  void handle_node_list_key(int key) {
    if (node_entries_.empty()) {
      return;
    }

    switch (key) {
      case KEY_UP:
      case 'k':
        if (selected_node_index_ > 0) {
          --selected_node_index_;
        }
        break;
      case KEY_DOWN:
      case 'j':
        if (selected_node_index_ + 1 < static_cast<int>(node_entries_.size())) {
          ++selected_node_index_;
        }
        break;
      case KEY_PPAGE:
        selected_node_index_ = std::max(0, selected_node_index_ - page_step());
        break;
      case KEY_NPAGE:
        selected_node_index_ = std::min(
          static_cast<int>(node_entries_.size()) - 1,
          selected_node_index_ + page_step());
        break;
      default:
        break;
    }
  }

  int page_step() const {
    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    return std::max(5, rows - 8);
  }

  void open_popup() {
    auto * entry = selected_entry();
    if (entry == nullptr) {
      set_status("No parameter selected.");
      return;
    }
    popup_open_ = true;
    popup_buffer_ = entry->edit_buffer;
    popup_dirty_ = entry->dirty;
    curs_set(0);
  }

  void close_popup() {
    popup_open_ = false;
    popup_buffer_.clear();
    popup_dirty_ = false;
    curs_set(0);
  }

  void refresh_all() {
    if (current_view_ == ViewMode::NodeList) {
      refresh_node_list();
      return;
    }
    if (!wait_for_parameter_service()) {
      return;
    }

    try {
      auto listed_future = client_->list_parameters({}, std::numeric_limits<uint64_t>::max());
      if (!wait_for_future(listed_future, "list parameters")) {
        return;
      }
      const auto listed = listed_future.get();
      std::vector<std::string> names = listed.names;
      std::sort(names.begin(), names.end());

      auto descriptors_future = client_->describe_parameters(names);
      auto values_future = client_->get_parameters(names);
      if (!wait_for_future(descriptors_future, "describe parameters")
        || !wait_for_future(values_future, "get parameters"))
      {
        return;
      }
      const auto descriptors = descriptors_future.get();
      const auto values = values_future.get();

      entries_.clear();
      entries_.reserve(names.size());
      for (std::size_t index = 0; index < names.size(); ++index) {
        ParameterEntry entry;
        entry.name = names[index];
        if (index < descriptors.size()) {
          entry.descriptor = descriptors[index];
        }
        if (index < values.size()) {
          entry.value = values[index].get_parameter_value();
          entry.has_value = true;
          entry.edit_buffer = parameter_value_to_string(entry.value);
        }
        entries_.push_back(std::move(entry));
      }

      if (entries_.empty()) {
        selected_index_ = 0;
        set_status("No parameters found on " + target_node_ + ".");
        return;
      }

      selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(entries_.size()) - 1);
      sync_edit_buffer_from_selected();
      set_status("Loaded " + std::to_string(entries_.size()) + " parameters from " + target_node_ + ".");
    } catch (const std::exception & exception) {
      set_status(std::string("Refresh all failed: ") + exception.what());
    }
  }

  void refresh_selected() {
    if (current_view_ != ViewMode::ParameterList) {
      return;
    }
    auto * entry = selected_entry();
    if (entry == nullptr) {
      set_status("No parameter selected.");
      return;
    }

    if (!wait_for_parameter_service()) {
      return;
    }

    try {
      auto descriptors_future = client_->describe_parameters({entry->name});
      auto values_future = client_->get_parameters({entry->name});
      if (!wait_for_future(descriptors_future, "describe parameter")
        || !wait_for_future(values_future, "get parameter"))
      {
        return;
      }
      const auto descriptors = descriptors_future.get();
      const auto values = values_future.get();
      if (!descriptors.empty()) {
        entry->descriptor = descriptors.front();
      }
      if (!values.empty()) {
        entry->value = values.front().get_parameter_value();
        entry->has_value = true;
      }
      entry->edit_buffer = parameter_value_to_string(entry->value);
      entry->dirty = false;
      if (popup_open_) {
        popup_buffer_ = entry->edit_buffer;
        popup_dirty_ = false;
      }
      set_status("Refreshed " + entry->name + ".");
    } catch (const std::exception & exception) {
      set_status(std::string("Refresh failed: ") + exception.what());
    }
  }

  void save_selected() {
    if (current_view_ != ViewMode::ParameterList) {
      return;
    }
    auto * entry = selected_entry();
    if (entry == nullptr) {
      set_status("No parameter selected.");
      return;
    }
    if (entry->descriptor.read_only) {
      set_status("Parameter is read-only.");
      return;
    }
    if (!is_scalar_type(entry->descriptor.type)) {
      set_status("Only scalar parameters are editable in this version.");
      return;
    }
    if (!wait_for_parameter_service()) {
      return;
    }

    auto parameter = parse_edit_buffer(*entry);
    if (!parameter.has_value()) {
      return;
    }

    try {
      auto results_future = client_->set_parameters({*parameter});
      if (!wait_for_future(results_future, "set parameter")) {
        return;
      }
      const auto results = results_future.get();
      if (results.empty()) {
        set_status("Set parameters returned no result.");
        return;
      }
      if (!results.front().successful) {
        set_status("Save failed: " + results.front().reason);
        return;
      }
      entry->value = parameter->get_parameter_value();
      entry->has_value = true;
      entry->edit_buffer = parameter_value_to_string(entry->value);
      entry->dirty = false;
      if (popup_open_) {
        popup_buffer_ = entry->edit_buffer;
        popup_dirty_ = false;
      }
      set_status("Saved " + entry->name + ".");
    } catch (const std::exception & exception) {
      set_status(std::string("Save failed: ") + exception.what());
    }
  }

  std::optional<rclcpp::Parameter> parse_edit_buffer(const ParameterEntry & entry) {
    const std::string trimmed = trim(entry.edit_buffer);

    try {
      switch (entry.descriptor.type) {
        case ParameterType::PARAMETER_BOOL: {
          const std::string lowered = lowercase(trimmed);
          if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
            return rclcpp::Parameter(entry.name, true);
          }
          if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
            return rclcpp::Parameter(entry.name, false);
          }
          set_status("Invalid bool. Use true/false.");
          return std::nullopt;
        }
        case ParameterType::PARAMETER_INTEGER:
          return rclcpp::Parameter(entry.name, static_cast<int64_t>(std::stoll(trimmed)));
        case ParameterType::PARAMETER_DOUBLE:
          return rclcpp::Parameter(entry.name, std::stod(trimmed));
        case ParameterType::PARAMETER_STRING:
          return rclcpp::Parameter(entry.name, entry.edit_buffer);
        default:
          set_status("Unsupported parameter type for editing.");
          return std::nullopt;
      }
    } catch (const std::exception &) {
      set_status("Invalid value for type " + parameter_type_name(entry.descriptor.type) + ".");
      return std::nullopt;
    }
  }

  void refresh_node_list() {
    auto nodes = this->get_node_graph_interface()->get_node_names_and_namespaces();
    node_entries_.clear();
    node_entries_.reserve(nodes.size());

    for (const auto & node : nodes) {
      const std::string full_name = fully_qualified_node_name(node.second, node.first);
      if (full_name == this->get_fully_qualified_name()) {
        continue;
      }
      node_entries_.push_back(full_name);
    }

    std::sort(node_entries_.begin(), node_entries_.end());
    node_entries_.erase(std::unique(node_entries_.begin(), node_entries_.end()), node_entries_.end());

    if (node_entries_.empty()) {
      selected_node_index_ = 0;
      set_status("No ROS 2 nodes discovered.");
      return;
    }

    selected_node_index_ = std::clamp(selected_node_index_, 0, static_cast<int>(node_entries_.size()) - 1);
    last_node_refresh_time_ = std::chrono::steady_clock::now();
    set_status("Loaded " + std::to_string(node_entries_.size()) + " nodes. Press Enter to inspect parameters.");
  }

  void select_current_node() {
    if (node_entries_.empty() || selected_node_index_ < 0 || selected_node_index_ >= static_cast<int>(node_entries_.size())) {
      set_status("No node selected.");
      return;
    }

    const std::string candidate_node = node_entries_[static_cast<std::size_t>(selected_node_index_)];
    auto candidate_client = std::make_shared<rclcpp::AsyncParametersClient>(this, candidate_node);
    set_status("Connecting to " + candidate_node + "...");
    if (!candidate_client->wait_for_service(std::chrono::milliseconds(800))) {
      set_status("Node " + candidate_node + " does not answer parameter services.");
      return;
    }

    target_node_ = candidate_node;
    client_ = std::move(candidate_client);
    entries_.clear();
    selected_index_ = 0;
    list_scroll_ = 0;
    current_view_ = ViewMode::ParameterList;
    refresh_all();
  }

  void switch_to_node_list() {
    close_popup();
    current_view_ = ViewMode::NodeList;
    entries_.clear();
    selected_index_ = 0;
    list_scroll_ = 0;
    refresh_node_list();
  }

  void reset_parameter_client() {
    client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, target_node_);
  }

  void maybe_refresh_node_list() {
    if (current_view_ != ViewMode::NodeList || popup_open_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_node_refresh_time_ >= std::chrono::seconds(1)) {
      refresh_node_list();
    }
  }

  void warm_up_node_list() {
    if (!node_entries_.empty()) {
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (node_entries_.empty() && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      refresh_node_list();
    }
  }

  std::string lowercase(std::string value) const {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
    return value;
  }

  std::string normalize_target_name(const std::string & name) const {
    if (name.empty() || name.front() == '/') {
      return name;
    }
    return "/" + name;
  }

  std::string fully_qualified_node_name(const std::string & ns, const std::string & name) const {
    if (ns.empty() || ns == "/") {
      return "/" + name;
    }
    if (ns.back() == '/') {
      return ns + name;
    }
    return ns + "/" + name;
  }

  void toggle_bool_buffer(std::string & buffer) {
    const std::string lowered = lowercase(trim(buffer));
    buffer = (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on")
      ? "false"
      : "true";
  }

  bool wait_for_parameter_service() {
    if (!client_) {
      set_status("No target node selected.");
      return false;
    }
    if (client_->service_is_ready()) {
      return true;
    }
    if (!client_->wait_for_service(std::chrono::milliseconds(200))) {
      set_status("Parameter services for " + target_node_ + " are unavailable.");
      return false;
    }
    return true;
  }

  template<typename FutureT>
  bool wait_for_future(FutureT & future, const std::string & action) {
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      set_status("Timed out trying to " + action + " on " + target_node_ + ".");
      return false;
    }
    return true;
  }

  void sync_edit_buffer_from_selected() {
    auto * entry = selected_entry();
    if (entry == nullptr || entry->dirty) {
      return;
    }
    entry->edit_buffer = parameter_value_to_string(entry->value);
  }

  ParameterEntry * selected_entry() {
    if (entries_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(entries_.size())) {
      return nullptr;
    }
    return &entries_[static_cast<std::size_t>(selected_index_)];
  }

  const ParameterEntry * selected_entry() const {
    if (entries_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(entries_.size())) {
      return nullptr;
    }
    return &entries_[static_cast<std::size_t>(selected_index_)];
  }

  void draw() {
    erase();

    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    const int help_row = rows - 1;
    const int status_row = rows - 2;
    const int content_top = 0;
    const int content_bottom = std::max(content_top + 1, status_row - 1);
    draw_box(0, 0, content_bottom, columns - 1);
    mvprintw(0, 2, " Parameter Commander ");
    if (current_view_ == ViewMode::NodeList) {
      draw_node_list(1, 1, content_bottom - 1, columns - 2);
    } else {
      draw_parameter_list(1, 1, content_bottom - 1, columns - 2);
    }
    draw_status_line(status_row, columns);
    draw_help_line(help_row, columns);
    if (popup_open_) {
      draw_popup(rows, columns);
    }

    refresh();
  }

  void draw_box(int top, int left, int bottom, int right) const {
    attron(COLOR_PAIR(kColorFrame));
    mvaddch(top, left, ACS_ULCORNER);
    mvaddch(top, right, ACS_URCORNER);
    mvaddch(bottom, left, ACS_LLCORNER);
    mvaddch(bottom, right, ACS_LRCORNER);
    mvhline(top, left + 1, ACS_HLINE, right - left - 1);
    mvhline(bottom, left + 1, ACS_HLINE, right - left - 1);
    mvvline(top + 1, left, ACS_VLINE, bottom - top - 1);
    mvvline(top + 1, right, ACS_VLINE, bottom - top - 1);
    attroff(COLOR_PAIR(kColorFrame));
  }

  void draw_parameter_list(int top, int left, int bottom, int right) {
    const int visible_rows = std::max(1, bottom - top + 1);
    const int width = right - left + 1;
    const int name_width = std::max(22, width / 3);
    const int value_width = std::max(12, width / 6);
    const int desc_width = std::max(10, width - name_width - value_width - 2);
    if (selected_index_ < list_scroll_) {
      list_scroll_ = selected_index_;
    }
    if (selected_index_ >= list_scroll_ + visible_rows) {
      list_scroll_ = selected_index_ - visible_rows + 1;
    }

    const std::string header = pad_column("Name", name_width) + " "
      + pad_column("Current", value_width) + " "
      + pad_column("Descriptor", desc_width);
    attron(COLOR_PAIR(kColorHeader) | A_BOLD);
    mvaddnstr(top, left, header.c_str(), width);
    attroff(COLOR_PAIR(kColorHeader) | A_BOLD);

    for (int row = 1; row < visible_rows; ++row) {
      const int entry_index = list_scroll_ + row - 1;
      mvhline(top + row, left, ' ', width);
      if (entry_index >= static_cast<int>(entries_.size())) {
        continue;
      }

      const auto & entry = entries_[static_cast<std::size_t>(entry_index)];
      const std::string rendered = pad_column(entry.dirty ? "*" + entry.name : entry.name, name_width) + " "
        + pad_column(summary_value(entry), value_width) + " "
        + pad_column(descriptor_summary(entry.descriptor), desc_width);

      if (entry_index == selected_index_) {
        attron(COLOR_PAIR(kColorSelection) | A_BOLD);
      }
      mvaddnstr(top + row, left, rendered.c_str(), width);
      if (entry_index == selected_index_) {
        attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
      }
    }
  }

  void draw_node_list(int top, int left, int bottom, int right) {
    const int visible_rows = std::max(1, bottom - top + 1);
    const int width = right - left + 1;
    if (selected_node_index_ < node_scroll_) {
      node_scroll_ = selected_node_index_;
    }
    if (selected_node_index_ >= node_scroll_ + visible_rows - 1) {
      node_scroll_ = selected_node_index_ - visible_rows + 2;
    }

    attron(COLOR_PAIR(kColorHeader) | A_BOLD);
    mvaddnstr(top, left, pad_column("Discovered Nodes", width).c_str(), width);
    attroff(COLOR_PAIR(kColorHeader) | A_BOLD);

    for (int row = 1; row < visible_rows; ++row) {
      const int entry_index = node_scroll_ + row - 1;
      mvhline(top + row, left, ' ', width);
      if (entry_index >= static_cast<int>(node_entries_.size())) {
        continue;
      }
      const std::string rendered = pad_column(node_entries_[static_cast<std::size_t>(entry_index)], width);
      if (entry_index == selected_node_index_) {
        attron(COLOR_PAIR(kColorSelection) | A_BOLD);
      }
      mvaddnstr(top + row, left, rendered.c_str(), width);
      if (entry_index == selected_node_index_) {
        attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
      }
    }
  }

  std::string summary_value(const ParameterEntry & entry) const {
    if (!entry.has_value) {
      return "<no value>";
    }
    return parameter_value_to_string(entry.value);
  }

  void draw_popup(int rows, int columns) {
    const ParameterEntry * entry = selected_entry();
    if (entry == nullptr) {
      return;
    }

    const int popup_width = std::min(columns - 4, 80);
    const int popup_height = std::min(rows - 4, 13);
    const int top = std::max(1, (rows - popup_height) / 2);
    const int left = std::max(2, (columns - popup_width) / 2);
    const int bottom = top + popup_height - 1;
    const int right = left + popup_width - 1;
    const int inner_width = popup_width - 2;
    const int field_top = top + 7;
    const int field_left = left + 2;
    const int field_right = right - 2;
    const int field_inner_width = std::max(1, field_right - field_left - 1);
    const int edit_text_width = std::max(1, field_inner_width - 1);

    for (int row = top + 1; row < bottom; ++row) {
      attron(COLOR_PAIR(kColorPopup));
      mvhline(row, left + 1, ' ', inner_width);
      attroff(COLOR_PAIR(kColorPopup));
    }

    draw_box(top, left, bottom, right);
    attron(COLOR_PAIR(kColorTitle) | A_BOLD);
    mvprintw(top, left + 2, " Edit Parameter ");
    attroff(COLOR_PAIR(kColorTitle) | A_BOLD);

    attron(COLOR_PAIR(kColorPopup));
    mvaddnstr(top + 1, left + 1, truncate_line("name: " + entry->name, inner_width).c_str(), inner_width);
    mvaddnstr(top + 2, left + 1, truncate_line("descriptor: " + descriptor_summary(entry->descriptor), inner_width).c_str(), inner_width);
    mvaddnstr(top + 3, left + 1, truncate_line(
      "type: " + parameter_type_name(entry->descriptor.type)
      + "  min: " + parameter_min(*entry)
      + "  max: " + parameter_max(*entry), inner_width).c_str(), inner_width);
    mvaddnstr(top + 4, left + 1, truncate_line(
      std::string("current: ") + summary_value(*entry), inner_width).c_str(), inner_width);
    mvhline(top + 5, left + 1, ACS_HLINE, inner_width);
    attroff(COLOR_PAIR(kColorPopup));
    draw_box(field_top, field_left, field_top + 2, field_right);
    attron(COLOR_PAIR(kColorInput));
    mvhline(field_top + 1, field_left + 1, ' ', field_inner_width);
    const std::string visible_buffer = tail_fit(popup_buffer_, edit_text_width);
    mvaddnstr(
      field_top + 1,
      field_left + 1,
      visible_buffer.c_str(),
      edit_text_width);
    attroff(COLOR_PAIR(kColorInput));
    if (popup_dirty_) {
      attron(COLOR_PAIR(kColorDirty) | A_BOLD);
      mvaddnstr(top + 6, right - 4, "[*]", 3);
      attroff(COLOR_PAIR(kColorDirty) | A_BOLD);
    }
    if (popup_is_editable(*entry) && software_caret_visible()) {
      const int visible_buffer_width = std::min(edit_text_width, static_cast<int>(visible_buffer.size()));
      const int cursor_x = std::min(
        field_left + field_inner_width - 1,
        field_left + 1 + visible_buffer_width);
      attron(COLOR_PAIR(kColorCursor) | A_BOLD);
      mvaddch(field_top + 1, cursor_x, ' ');
      attroff(COLOR_PAIR(kColorCursor) | A_BOLD);
    }

    attron(COLOR_PAIR(kColorPopup));
    mvhline(bottom - 2, left + 1, ACS_HLINE, inner_width);
    mvaddnstr(bottom - 1, left + 1, "F3 Load  F2 Save  Esc Close  F10 Exit", inner_width);
    attroff(COLOR_PAIR(kColorPopup));
  }

  std::string descriptor_summary(const ParameterDescriptor & descriptor) const {
    if (!descriptor.description.empty()) {
      return descriptor.description;
    }
    if (!descriptor.additional_constraints.empty()) {
      return descriptor.additional_constraints;
    }
    if (descriptor.read_only) {
      return "read-only";
    }
    return "-";
  }

  std::string parameter_min(const ParameterEntry & entry) const {
    if (!entry.descriptor.integer_range.empty()) {
      return std::to_string(entry.descriptor.integer_range.front().from_value);
    }
    if (!entry.descriptor.floating_point_range.empty()) {
      std::ostringstream stream;
      stream << entry.descriptor.floating_point_range.front().from_value;
      return stream.str();
    }
    return "-";
  }

  std::string parameter_max(const ParameterEntry & entry) const {
    if (!entry.descriptor.integer_range.empty()) {
      return std::to_string(entry.descriptor.integer_range.front().to_value);
    }
    if (!entry.descriptor.floating_point_range.empty()) {
      std::ostringstream stream;
      stream << entry.descriptor.floating_point_range.front().to_value;
      return stream.str();
    }
    return "-";
  }

  bool popup_is_editable(const ParameterEntry & entry) const {
    return is_scalar_type(entry.descriptor.type) && !entry.descriptor.read_only;
  }

  std::string pad_column(const std::string & value, int width) const {
    const std::string truncated = truncate_line(value, width);
    if (visible_width(truncated) >= width) {
      return truncated;
    }
    return truncated + std::string(static_cast<std::size_t>(width - visible_width(truncated)), ' ');
  }

  void draw_status_line(int row, int columns) const {
    attron(COLOR_PAIR(kColorStatus) | A_BOLD);
    mvhline(row, 0, ' ', columns);
    mvaddnstr(row, 1, truncate_line(status_message_, columns - 2).c_str(), columns - 2);
    attroff(COLOR_PAIR(kColorStatus) | A_BOLD);
  }

  void draw_help_line(int row, int columns) const {
    std::string help;
    if (popup_open_) {
      help = "F2 Save  F3 Load  Enter Save+Close  Esc Close  F10 Exit";
    } else if (current_view_ == ViewMode::NodeList) {
      help = "Enter Select Node  F3 Refresh Nodes  F4 Refresh Nodes  F10 Exit";
    } else {
      help = "Enter Edit  F3 Refresh Param  F4 Refresh All  Esc Nodes  F10 Exit";
    }
    attron(COLOR_PAIR(kColorHelp));
    mvhline(row, 0, ' ', columns);
    mvaddnstr(row, 0, truncate_line(help, columns).c_str(), columns);
    attroff(COLOR_PAIR(kColorHelp));
  }

  void set_status(const std::string & message) {
    status_message_ = message;
  }

  std::string target_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  ViewMode current_view_{ViewMode::NodeList};
  std::vector<std::string> node_entries_;
  std::vector<ParameterEntry> entries_;
  std::chrono::steady_clock::time_point last_node_refresh_time_{std::chrono::steady_clock::time_point::min()};
  int selected_node_index_{0};
  int node_scroll_{0};
  int selected_index_{0};
  int list_scroll_{0};
  bool popup_open_{false};
  bool popup_dirty_{false};
  std::string popup_buffer_;
  std::string status_message_{"F4 to load parameters."};
};

}  // namespace

}  // namespace ros2_console_tools

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::string target_node;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (!argument.empty() && argument.front() != '-') {
      target_node = argument;
      break;
    }
  }

  auto node = std::make_shared<ros2_console_tools::ParameterConsoleNode>(target_node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int exit_code = node->run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
