#ifndef ROS2_CONSOLE_TOOLS__SERVICE_COMMANDER_HPP_
#define ROS2_CONSOLE_TOOLS__SERVICE_COMMANDER_HPP_

#include <rclcpp/create_generic_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_service_commander_tool(
  const std::string & initial_node = "",
  const std::string & initial_service = "",
  bool embedded_mode = false);

using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;
using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;
using ServiceMembers = rosidl_typesupport_introspection_cpp::ServiceMembers;

template<typename T>
inline std::string service_scalar_to_string(const T & value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

template<>
inline std::string service_scalar_to_string<bool>(const bool & value) {
  return value ? "true" : "false";
}

template<>
inline std::string service_scalar_to_string<uint8_t>(const uint8_t & value) {
  return std::to_string(static_cast<unsigned int>(value));
}

template<>
inline std::string service_scalar_to_string<int8_t>(const int8_t & value) {
  return std::to_string(static_cast<int>(value));
}

struct DetailRow {
  int depth{0};
  std::string field;
  std::string value;

  DetailRow() = default;
  DetailRow(int row_depth, std::string row_field, std::string row_value)
  : depth(row_depth), field(std::move(row_field)), value(std::move(row_value)) {}
};

struct RequestRow {
  int depth{0};
  std::string field;
  std::string value;
  bool editable{false};
  uint8_t type_id{0};
  void * field_memory{nullptr};

  RequestRow() = default;
  RequestRow(
    int row_depth,
    std::string row_field,
    std::string row_value,
    bool row_editable,
    uint8_t row_type_id,
    void * row_field_memory)
  : depth(row_depth),
    field(std::move(row_field)),
    value(std::move(row_value)),
    editable(row_editable),
    type_id(row_type_id),
    field_memory(row_field_memory) {}
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

enum class ServiceCommanderViewMode {
  NodeList,
  ServiceList,
  ServiceDetail,
};

class ServiceCommanderScreen;

class ServiceCommanderBackend : public rclcpp::Node {
public:
  explicit ServiceCommanderBackend(
    const std::string & target_node = "",
    const std::string & initial_service = "");
  ~ServiceCommanderBackend() override;

private:
  friend class ServiceCommanderScreen;

  using RequestStorageUnit = std::max_align_t;

  void refresh_node_list();
  void maybe_refresh_node_list();
  void warm_up_node_list();
  void select_current_node();
  void switch_to_node_list();
  void refresh_services();
  bool select_initial_node_if_present();
  bool resolve_initial_node_from_service();
  void clamp_service_selection();
  void clamp_node_selection();
  void open_selected_service();
  void close_service_detail();
  ServiceIntrospection & get_or_create_introspection(const std::string & type);
  std::size_t request_storage_units(std::size_t byte_count) const;
  void allocate_request_storage(std::size_t byte_count);
  void * request_storage_data();
  const void * request_storage_data() const;
  void finalize_request_storage();
  void reload_selected_service_request();
  void call_selected_service();
  RequestRow * selected_request_row();
  const RequestRow * selected_request_row() const;
  bool request_row_is_editable(const RequestRow & row) const;
  bool set_selected_request_value(const std::string & value_text);
  bool select_initial_service_if_present();
  std::vector<DetailRow> flatten_message(const MessageMembers * members, const void * message_memory) const;
  std::vector<RequestRow> flatten_request_message(const MessageMembers * members, void * message_memory) const;
  void append_message_members(
    std::vector<DetailRow> & rows, int depth, const MessageMembers * members, const void * message_memory) const;
  void append_request_members(
    std::vector<RequestRow> & rows, int depth, const MessageMembers * members, void * message_memory) const;
  std::string normalize_member_label(const MessageMembers & parent_members, const MessageMember & member) const;
  void append_member(
    std::vector<DetailRow> & rows, int depth, const std::string & label,
    const MessageMember & member, const void * field_memory) const;
  void append_request_member(
    std::vector<RequestRow> & rows, int depth, const std::string & label,
    const MessageMember & member, void * field_memory) const;
  std::string array_element_to_string(
    const MessageMember & member, const void * field_memory, std::size_t index) const;
  std::string scalar_field_to_string(uint8_t type_id, const void * field_memory) const;
  std::optional<bool> parse_bool_text(const std::string & value_text) const;
  std::vector<ServiceEntry> services_for_node(const std::string & full_name) const;
  std::string normalize_target_name(const std::string & name) const;
  std::string fully_qualified_node_name(const std::string & ns, const std::string & name) const;
  void split_node_name(const std::string & full_name, std::string & ns, std::string & name) const;

  std::string target_node_;
  std::vector<std::string> node_entries_;
  std::vector<ServiceEntry> services_;
  std::map<std::string, ServiceIntrospection> introspection_cache_;
  std::string initial_service_name_;
  bool auto_open_initial_service_{false};
  bool pending_initial_node_selection_{false};
  ServiceCommanderViewMode view_mode_{ServiceCommanderViewMode::NodeList};
  std::chrono::steady_clock::time_point last_node_refresh_time_{
    std::chrono::steady_clock::time_point::min()};
  int selected_node_index_{0};
  int node_scroll_{0};
  int selected_service_index_{0};
  int service_scroll_{0};
  ServiceEntry selected_service_;
  std::vector<RequestStorageUnit> request_storage_;
  std::vector<RequestRow> request_rows_;
  std::vector<DetailRow> response_rows_;
  std::string response_error_;
  std::string status_line_{"Loading nodes..."};
  int selected_request_index_{0};
  int request_scroll_{0};
};

class ServiceCommanderScreen {
public:
  explicit ServiceCommanderScreen(
    std::shared_ptr<ServiceCommanderBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_node_list_key(int key);
  bool handle_service_list_key(int key);
  bool handle_search_key(int key);
  bool handle_service_detail_key(int key);
  bool handle_edit_popup_key(int key);
  int page_step() const;
  void draw();
  void draw_node_list(int top, int left, int bottom, int right);
  void draw_service_list(int top, int left, int bottom, int right);
  void draw_response_panel(int top, int left, int bottom, int right) const;
  void draw_service_detail(int top, int left, int bottom, int right) const;
  void draw_edit_popup(int rows, int columns) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<ServiceCommanderBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
  bool edit_popup_open_{false};
  bool edit_popup_dirty_{false};
  std::string edit_popup_buffer_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__SERVICE_COMMANDER_HPP_
