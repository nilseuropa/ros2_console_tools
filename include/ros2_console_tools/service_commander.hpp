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

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

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
  ServiceList,
  ServiceDetail,
};

class ServiceCommanderScreen;

class ServiceCommanderBackend : public rclcpp::Node {
public:
  ServiceCommanderBackend();

private:
  friend class ServiceCommanderScreen;

  void refresh_services();
  void clamp_service_selection();
  void open_selected_service();
  void close_service_detail();
  ServiceIntrospection & get_or_create_introspection(const std::string & type);
  void reload_selected_service_request();
  void call_selected_service();
  std::vector<DetailRow> flatten_message(const MessageMembers * members, const void * message_memory) const;
  void append_message_members(
    std::vector<DetailRow> & rows, int depth, const MessageMembers * members, const void * message_memory) const;
  std::string normalize_member_label(const MessageMembers & parent_members, const MessageMember & member) const;
  void append_member(
    std::vector<DetailRow> & rows, int depth, const std::string & label,
    const MessageMember & member, const void * field_memory) const;
  std::string array_element_to_string(
    const MessageMember & member, const void * field_memory, std::size_t index) const;
  std::string scalar_field_to_string(uint8_t type_id, const void * field_memory) const;

  std::vector<ServiceEntry> services_;
  std::map<std::string, ServiceIntrospection> introspection_cache_;
  ServiceCommanderViewMode view_mode_{ServiceCommanderViewMode::ServiceList};
  int selected_service_index_{0};
  int service_scroll_{0};
  ServiceEntry selected_service_;
  std::vector<DetailRow> request_rows_;
  std::vector<DetailRow> response_rows_;
  std::string response_error_;
  std::string status_line_{"Loading services..."};
};

class ServiceCommanderScreen {
public:
  explicit ServiceCommanderScreen(std::shared_ptr<ServiceCommanderBackend> backend);
  int run();

private:
  bool handle_key(int key);
  bool handle_service_list_key(int key);
  bool handle_search_key(int key);
  bool handle_service_detail_key(int key);
  int page_step() const;
  void draw();
  void draw_service_list(int top, int left, int bottom, int right);
  void draw_rows_panel(
    int top, int left, int bottom, int right,
    const std::string & title,
    const std::vector<DetailRow> & rows,
    const std::string & error) const;
  void draw_service_detail(int top, int left, int bottom, int right) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<ServiceCommanderBackend> backend_;
  tui::SearchState search_state_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__SERVICE_COMMANDER_HPP_
