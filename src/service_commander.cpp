#include <ncursesw/ncurses.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/create_generic_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

namespace {

using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;
using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;
using ServiceMembers = rosidl_typesupport_introspection_cpp::ServiceMembers;

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorStatus = tui::kColorStatus,
  kColorHelp = tui::kColorHelp,
};

enum class ViewMode {
  ServiceList,
  ServiceDetail,
};

using tui::Session;
using tui::draw_box;
using tui::draw_box_char;
using tui::draw_help_bar;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::draw_text_vline;
using tui::truncate_text;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::SearchState;
using tui::start_search;

template<typename T>
std::string scalar_to_string(const T & value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

template<>
std::string scalar_to_string<bool>(const bool & value) {
  return value ? "true" : "false";
}

template<>
std::string scalar_to_string<uint8_t>(const uint8_t & value) {
  return std::to_string(static_cast<unsigned int>(value));
}

template<>
std::string scalar_to_string<int8_t>(const int8_t & value) {
  return std::to_string(static_cast<int>(value));
}

struct DetailRow {
  int depth{0};
  std::string field;
  std::string value;
};

struct ServiceEntry {
  std::string name;
  std::string type;
};

struct ServiceIntrospection {
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const ServiceMembers * service_members{nullptr};
  const MessageMembers * request_members{nullptr};
  const MessageMembers * response_members{nullptr};
};

class ServiceCommanderNode : public rclcpp::Node {
public:
  ServiceCommanderNode()
  : Node("service_commander") {}

  int run() {
    Session ncurses_session;
    refresh_services();

    bool running = true;
    while (running && rclcpp::ok()) {
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
    switch (view_mode_) {
      case ViewMode::ServiceList:
        return handle_service_list_key(key);
      case ViewMode::ServiceDetail:
        return handle_service_detail_key(key);
    }
    return true;
  }

  bool handle_service_list_key(int key) {
    clamp_service_selection();
    switch (key) {
      case KEY_F(10):
        return false;
      case KEY_F(4):
        refresh_services();
        return true;
      case 27:
        if (is_alt_binding(key, 's')) {
          start_search(search_state_);
          status_line_ = "Search.";
          return true;
        }
        return true;
      case KEY_UP:
      case 'k':
        if (selected_service_index_ > 0) {
          --selected_service_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_service_index_ + 1 < static_cast<int>(services_.size())) {
          ++selected_service_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_service_index_ = std::max(0, selected_service_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!services_.empty()) {
          selected_service_index_ = std::min(
            static_cast<int>(services_.size()) - 1,
            selected_service_index_ + page_step());
        }
        return true;
      case '\n':
      case KEY_ENTER:
        open_selected_service();
        return true;
      default:
        return true;
    }
  }

  bool handle_search_key(int key) {
    const SearchInputResult result = handle_search_input(search_state_, key);
    if (result == SearchInputResult::Cancelled) {
      status_line_ = "Search cancelled.";
      return true;
    }
    if (result == SearchInputResult::Accepted) {
      status_line_ = search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query;
      return true;
    }
    if (result != SearchInputResult::Changed || view_mode_ != ViewMode::ServiceList) {
      return true;
    }

    std::vector<std::string> labels;
    labels.reserve(services_.size());
    for (const auto & entry : services_) {
      labels.push_back(entry.name);
    }
    const int match = find_best_match(labels, search_state_.query, selected_service_index_);
    if (match >= 0) {
      selected_service_index_ = match;
    }
    status_line_ = "Search: " + search_state_.query;
    return true;
  }

  bool handle_service_detail_key(int key) {
    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
        close_service_detail();
        return true;
      case KEY_F(4):
        refresh_services();
        reload_selected_service_request();
        return true;
      case KEY_F(3):
        reload_selected_service_request();
        return true;
      case KEY_F(2):
        call_selected_service();
        return true;
      default:
        return true;
    }
  }

  int page_step() const {
    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    (void)columns;
    return std::max(5, rows - 8);
  }

  void refresh_services() {
    const auto discovered = this->get_service_names_and_types();
    std::vector<ServiceEntry> refreshed;
    refreshed.reserve(discovered.size());
    for (const auto & [name, types] : discovered) {
      if (name.empty()) {
        continue;
      }
      ServiceEntry entry;
      entry.name = name;
      entry.type = types.empty() ? "<unknown>" : types.front();
      refreshed.push_back(std::move(entry));
    }

    std::sort(
      refreshed.begin(), refreshed.end(),
      [](const ServiceEntry & lhs, const ServiceEntry & rhs) { return lhs.name < rhs.name; });

    services_ = std::move(refreshed);
    clamp_service_selection();
    status_line_ = "Loaded " + std::to_string(services_.size()) + " services. Enter inspects a service.";
  }

  void clamp_service_selection() {
    if (services_.empty()) {
      selected_service_index_ = 0;
      service_scroll_ = 0;
      return;
    }
    selected_service_index_ = std::clamp(selected_service_index_, 0, static_cast<int>(services_.size()) - 1);
  }

  void open_selected_service() {
    clamp_service_selection();
    if (services_.empty()) {
      status_line_ = "No service selected.";
      return;
    }

    selected_service_ = services_[static_cast<std::size_t>(selected_service_index_)];
    view_mode_ = ViewMode::ServiceDetail;
    reload_selected_service_request();
    response_rows_.clear();
    response_error_.clear();
    status_line_ = "Inspecting " + selected_service_.name + ". F2 calls with the default request.";
  }

  void close_service_detail() {
    view_mode_ = ViewMode::ServiceList;
    selected_service_ = {};
    request_rows_.clear();
    response_rows_.clear();
    response_error_.clear();
    status_line_ = "Returned to service list.";
  }

  ServiceIntrospection & get_or_create_introspection(const std::string & type) {
    auto found = introspection_cache_.find(type);
    if (found != introspection_cache_.end()) {
      return found->second;
    }

    ServiceIntrospection introspection;
    introspection.cpp_library = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    introspection.introspection_library = rclcpp::get_typesupport_library(
      type, "rosidl_typesupport_introspection_cpp");
    const auto * service_handle = rclcpp::get_service_typesupport_handle(
      type, "rosidl_typesupport_introspection_cpp", *introspection.introspection_library);
    introspection.service_members = static_cast<const ServiceMembers *>(service_handle->data);
    introspection.request_members = introspection.service_members->request_members_;
    introspection.response_members = introspection.service_members->response_members_;

    auto inserted = introspection_cache_.emplace(type, std::move(introspection));
    return inserted.first->second;
  }

  void reload_selected_service_request() {
    if (selected_service_.name.empty()) {
      return;
    }

    try {
      auto & introspection = get_or_create_introspection(selected_service_.type);
      std::vector<uint8_t> storage(introspection.request_members->size_of_);
      introspection.request_members->init_function(
        storage.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
      request_rows_ = flatten_message(introspection.request_members, storage.data());
      introspection.request_members->fini_function(storage.data());
      status_line_ = "Loaded default request for " + selected_service_.name + ".";
    } catch (const std::exception & exception) {
      request_rows_.clear();
      response_rows_.clear();
      response_error_ = exception.what();
      status_line_ = "Request load failed: " + response_error_;
    }
  }

  void call_selected_service() {
    if (selected_service_.name.empty()) {
      status_line_ = "No service selected.";
      return;
    }

    try {
      auto & introspection = get_or_create_introspection(selected_service_.type);
      auto client = rclcpp::create_generic_client(this->shared_from_this(), selected_service_.name, selected_service_.type);
      if (!client->wait_for_service(std::chrono::milliseconds(800))) {
        status_line_ = "Service " + selected_service_.name + " is unavailable.";
        return;
      }

      std::vector<uint8_t> request_storage(introspection.request_members->size_of_);
      introspection.request_members->init_function(
        request_storage.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
      auto future = client->async_send_request(request_storage.data());
      if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        client->remove_pending_request(future.request_id);
        introspection.request_members->fini_function(request_storage.data());
        status_line_ = "Timed out calling " + selected_service_.name + ".";
        return;
      }

      auto response = future.get();
      response_rows_ = flatten_message(introspection.response_members, response.get());
      response_error_.clear();
      introspection.request_members->fini_function(request_storage.data());
      status_line_ = "Called " + selected_service_.name + ".";
    } catch (const std::exception & exception) {
      response_rows_.clear();
      response_error_ = exception.what();
      status_line_ = "Call failed: " + response_error_;
    }
  }

  std::vector<DetailRow> flatten_message(const MessageMembers * members, const void * message_memory) const {
    std::vector<DetailRow> rows;
    append_message_members(rows, 0, members, message_memory);
    return rows;
  }

  void append_message_members(
    std::vector<DetailRow> & rows,
    int depth,
    const MessageMembers * members,
    const void * message_memory) const
  {
    for (uint32_t index = 0; index < members->member_count_; ++index) {
      const auto & member = members->members_[index];
      const auto * field_memory = static_cast<const uint8_t *>(message_memory) + member.offset_;
      append_member(rows, depth, normalize_member_label(*members, member), member, field_memory);
    }
  }

  std::string normalize_member_label(const MessageMembers & parent_members, const MessageMember & member) const {
    if (std::strcmp(member.name_, "structure_needs_at_least_one_member") != 0) {
      return member.name_;
    }
    if (std::strcmp(parent_members.message_name_, "Empty") == 0) {
      return "empty";
    }
    return "empty";
  }

  void append_member(
    std::vector<DetailRow> & rows,
    int depth,
    const std::string & label,
    const MessageMember & member,
    const void * field_memory) const
  {
    if (member.is_array_) {
      const std::size_t element_count =
        member.size_function != nullptr ? member.size_function(field_memory) : member.array_size_;
      rows.push_back({depth, label, "[" + std::to_string(element_count) + "]"});

      for (std::size_t index = 0; index < element_count; ++index) {
        const std::string child_label = "[" + std::to_string(index) + "]";
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
          const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
          const void * element_memory =
            member.get_const_function != nullptr ? member.get_const_function(field_memory, index) : nullptr;
          if (element_memory != nullptr) {
            rows.push_back({depth + 1, child_label, "<message>"});
            append_message_members(rows, depth + 2, sub_members, element_memory);
          }
          continue;
        }

        rows.push_back({depth + 1, child_label, array_element_to_string(member, field_memory, index)});
      }
      return;
    }

    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      rows.push_back({depth, label, "<message>"});
      const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
      append_message_members(rows, depth + 1, sub_members, field_memory);
      return;
    }

    rows.push_back({depth, label, scalar_field_to_string(member.type_id_, field_memory)});
  }

  std::string array_element_to_string(
    const MessageMember & member,
    const void * field_memory,
    std::size_t index) const
  {
    switch (member.type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN: {
        bool value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
        uint8_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
        int8_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
        uint16_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
        int16_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
        uint32_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
        int32_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
        uint64_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
        int64_t value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
        float value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
        double value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE: {
        long double value{};
        member.fetch_function(field_memory, index, &value);
        return scalar_to_string(value);
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
        signed char value{};
        member.fetch_function(field_memory, index, &value);
        return std::string(1, static_cast<char>(value));
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
        std::string value;
        member.fetch_function(field_memory, index, &value);
        return value;
      }
      default:
        return "<value>";
    }
  }

  std::string scalar_field_to_string(uint8_t type_id, const void * field_memory) const {
    switch (type_id) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
        return scalar_to_string(*static_cast<const bool *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        return scalar_to_string(*static_cast<const uint8_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        return scalar_to_string(*static_cast<const int8_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        return scalar_to_string(*static_cast<const uint16_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        return scalar_to_string(*static_cast<const int16_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        return scalar_to_string(*static_cast<const uint32_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        return scalar_to_string(*static_cast<const int32_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        return scalar_to_string(*static_cast<const uint64_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        return scalar_to_string(*static_cast<const int64_t *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        return scalar_to_string(*static_cast<const float *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        return scalar_to_string(*static_cast<const double *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        return scalar_to_string(*static_cast<const long double *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        return std::string(1, *static_cast<const char *>(field_memory));
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        return *static_cast<const std::string *>(field_memory);
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        return "<wstring>";
      default:
        return "<value>";
    }
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
    mvprintw(0, 1, "Service Commander ");
    if (view_mode_ == ViewMode::ServiceList) {
      draw_service_list(1, 1, content_bottom - 1, columns - 2);
    } else {
      draw_service_detail(1, 1, content_bottom - 1, columns - 2);
    }
    draw_status_line(status_row, columns);
    draw_help_line(help_row, columns);
    draw_search_box(rows, columns, search_state_);
    refresh();
  }

  void draw_service_list(int top, int left, int bottom, int right) {
    clamp_service_selection();
    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    const int name_width = std::max(32, width / 2);
    const int type_width = std::max(12, width - name_width - 1);
    const int separator_x = left + name_width;

    if (selected_service_index_ < service_scroll_) {
      service_scroll_ = selected_service_index_;
    }
    if (selected_service_index_ >= service_scroll_ + visible_rows - 1) {
      service_scroll_ = std::max(0, selected_service_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", name_width, "Service");
    draw_box_char(top, separator_x, WACS_VLINE, '|');
    mvprintw(top, separator_x + 1, "%-*s", type_width, "Type");
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = service_scroll_;
    const int last_row = std::min(static_cast<int>(services_.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_service_index_);

      mvhline(row, left, ' ', width);
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }
      draw_box_char(row, separator_x, WACS_VLINE, '|');
      if (!has_item) {
        continue;
      }

      const auto & entry = services_[static_cast<std::size_t>(first_row + (row - top - 1))];
      mvprintw(row, left, "%-*s", name_width, truncate_text(entry.name, name_width).c_str());
      mvprintw(row, separator_x + 1, "%-*s", type_width, truncate_text(entry.type, type_width).c_str());
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        mvaddch(row, separator_x, '|');
      }
    }
  }

  void draw_rows_panel(
    int top, int left, int bottom, int right,
    const std::string & title,
    const std::vector<DetailRow> & rows,
    const std::string & error) const
  {
    const int width = right - left + 1;
    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", width, truncate_text(title, width).c_str());
    attroff(COLOR_PAIR(kColorHeader));

    int row_y = top + 1;
    if (!error.empty()) {
      mvprintw(row_y, left, "%s", truncate_text(error, width).c_str());
      return;
    }

    for (const auto & row : rows) {
      if (row_y > bottom) {
        break;
      }
      const std::string line =
        std::string(static_cast<std::size_t>(row.depth * 2), ' ') + row.field + ": " + row.value;
      mvprintw(row_y, left, "%-*s", width, truncate_text(line, width).c_str());
      ++row_y;
    }
  }

  void draw_service_detail(int top, int left, int bottom, int right) const {
    const int width = right - left + 1;
    const int left_width = std::max(24, width / 2);
    const int separator_x = left + left_width;
    attron(COLOR_PAIR(kColorFrame));
    draw_text_vline(top, separator_x, bottom - top + 1);
    attroff(COLOR_PAIR(kColorFrame));

    draw_rows_panel(top, left, bottom, separator_x - 1, "Request", request_rows_, "");
    draw_rows_panel(top, separator_x + 1, bottom, right, "Response", response_rows_, response_error_);
  }

  void draw_status_line(int row, int columns) const {
    std::string line = status_line_;
    if (view_mode_ == ViewMode::ServiceDetail && !selected_service_.name.empty()) {
      line = truncate_text(
        selected_service_.name + " [" + selected_service_.type + "]  " + status_line_,
        columns - 2);
    }
    draw_status_bar(row, columns, line);
  }

  void draw_help_line(int row, int columns) const {
    const std::string help =
      view_mode_ == ViewMode::ServiceDetail
      ? "F2 Call  F3 Reset Request  F4 Refresh  Esc Services  F10 Exit"
      : "Enter Inspect  Alt+S Search  F4 Refresh  F10 Exit";
    draw_help_bar(row, columns, truncate_text(help, columns - 1));
  }

  std::vector<ServiceEntry> services_;
  std::map<std::string, ServiceIntrospection> introspection_cache_;
  ViewMode view_mode_{ViewMode::ServiceList};
  int selected_service_index_{0};
  int service_scroll_{0};
  ServiceEntry selected_service_;
  std::vector<DetailRow> request_rows_;
  std::vector<DetailRow> response_rows_;
  std::string response_error_;
  std::string status_line_{"Loading services..."};
  SearchState search_state_;
};

}  // namespace

}  // namespace ros2_console_tools

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_console_tools::ServiceCommanderNode>();

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
