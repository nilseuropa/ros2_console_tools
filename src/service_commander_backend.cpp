#include "ros2_console_tools/service_commander.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <future>
#include <thread>
#include <utility>

namespace ros2_console_tools {

ServiceCommanderBackend::ServiceCommanderBackend(
  const std::string & target_node,
  const std::string & initial_service)
: Node("service_commander"),
  target_node_(normalize_target_name(
      this->declare_parameter<std::string>("target_node", target_node))),
  initial_service_name_(initial_service),
  auto_open_initial_service_(!initial_service.empty()),
  pending_initial_node_selection_(!target_node_.empty() || !initial_service.empty()),
  view_mode_(target_node_.empty()
      ? ServiceCommanderViewMode::NodeList
      : ServiceCommanderViewMode::ServiceList)
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }
}

ServiceCommanderBackend::~ServiceCommanderBackend() {
  finalize_request_storage();
}

void ServiceCommanderBackend::refresh_node_list() {
  const auto nodes = this->get_node_graph_interface()->get_node_names_and_namespaces();
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
  clamp_node_selection();
  last_node_refresh_time_ = std::chrono::steady_clock::now();

  if (pending_initial_node_selection_) {
    if (select_initial_node_if_present()) {
      pending_initial_node_selection_ = false;
    } else if (resolve_initial_node_from_service()) {
      pending_initial_node_selection_ = false;
    }
  }

  if (node_entries_.empty()) {
    status_line_ = "No ROS 2 nodes discovered.";
    return;
  }

  if (view_mode_ == ServiceCommanderViewMode::NodeList) {
    status_line_ = "Loaded " + std::to_string(node_entries_.size()) + " nodes. Press Enter to inspect services.";
  }
}

void ServiceCommanderBackend::maybe_refresh_node_list() {
  if (view_mode_ != ServiceCommanderViewMode::NodeList) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_node_refresh_time_ >= std::chrono::seconds(1)) {
    refresh_node_list();
  }
}

void ServiceCommanderBackend::warm_up_node_list() {
  auto target_visible = [this]() {
      return target_node_.empty() || std::find(node_entries_.begin(), node_entries_.end(), target_node_) != node_entries_.end();
    };

  if (!node_entries_.empty() && (!pending_initial_node_selection_ || target_visible())) {
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    if (!node_entries_.empty() && (!pending_initial_node_selection_ || target_visible())) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    refresh_node_list();
  }
}

void ServiceCommanderBackend::refresh_services() {
  if (target_node_.empty()) {
    view_mode_ = ServiceCommanderViewMode::NodeList;
    refresh_node_list();
    return;
  }

  try {
    services_ = services_for_node(target_node_);
  } catch (const std::exception & exception) {
    services_.clear();
    view_mode_ = ServiceCommanderViewMode::ServiceList;
    status_line_ =
      "Failed to load services from " + target_node_ + ": " + exception.what() + ". Press F4 to retry.";
    return;
  }
  view_mode_ = ServiceCommanderViewMode::ServiceList;
  clamp_service_selection();
  if (select_initial_service_if_present() && auto_open_initial_service_) {
    auto_open_initial_service_ = false;
    open_selected_service();
    return;
  }

  if (services_.empty()) {
    status_line_ = "No services found on " + target_node_ + ".";
    return;
  }

  auto_open_initial_service_ = false;
  status_line_ =
    "Loaded " + std::to_string(services_.size()) + " services from " + target_node_ + ".";
}

bool ServiceCommanderBackend::select_initial_node_if_present() {
  if (target_node_.empty()) {
    return false;
  }

  for (std::size_t index = 0; index < node_entries_.size(); ++index) {
    if (node_entries_[index] == target_node_) {
      selected_node_index_ = static_cast<int>(index);
      return true;
    }
  }
  return false;
}

bool ServiceCommanderBackend::resolve_initial_node_from_service() {
  if (!target_node_.empty() || initial_service_name_.empty()) {
    return false;
  }

  for (std::size_t index = 0; index < node_entries_.size(); ++index) {
    std::vector<ServiceEntry> services;
    try {
      services = services_for_node(node_entries_[index]);
    } catch (const std::exception &) {
      continue;
    }
    const auto found = std::find_if(
      services.begin(), services.end(),
      [this](const ServiceEntry & entry) { return entry.name == initial_service_name_; });
    if (found != services.end()) {
      target_node_ = node_entries_[index];
      selected_node_index_ = static_cast<int>(index);
      return true;
    }
  }

  return false;
}

bool ServiceCommanderBackend::select_initial_service_if_present() {
  if (initial_service_name_.empty()) {
    return false;
  }

  for (std::size_t index = 0; index < services_.size(); ++index) {
    if (services_[index].name == initial_service_name_) {
      selected_service_index_ = static_cast<int>(index);
      initial_service_name_.clear();
      return true;
    }
  }

  return false;
}

void ServiceCommanderBackend::clamp_service_selection() {
  if (services_.empty()) {
    selected_service_index_ = 0;
    service_scroll_ = 0;
    return;
  }
  selected_service_index_ = std::clamp(selected_service_index_, 0, static_cast<int>(services_.size()) - 1);
}

void ServiceCommanderBackend::clamp_node_selection() {
  if (node_entries_.empty()) {
    selected_node_index_ = 0;
    node_scroll_ = 0;
    return;
  }
  selected_node_index_ = std::clamp(selected_node_index_, 0, static_cast<int>(node_entries_.size()) - 1);
}

void ServiceCommanderBackend::select_current_node() {
  clamp_node_selection();
  if (node_entries_.empty()) {
    status_line_ = "No node selected.";
    return;
  }

  target_node_ = node_entries_[static_cast<std::size_t>(selected_node_index_)];
  pending_initial_node_selection_ = false;
  selected_service_index_ = 0;
  service_scroll_ = 0;
  close_service_detail();
  refresh_services();
}

void ServiceCommanderBackend::open_selected_service() {
  clamp_service_selection();
  if (services_.empty()) {
    status_line_ = "No service selected.";
    return;
  }

  selected_service_ = services_[static_cast<std::size_t>(selected_service_index_)];
  view_mode_ = ServiceCommanderViewMode::ServiceDetail;
  reload_selected_service_request();
  response_rows_.clear();
  response_error_.clear();
  status_line_ = "Inspecting " + selected_service_.name + ". Edit request fields before calling.";
}

void ServiceCommanderBackend::close_service_detail() {
  view_mode_ = ServiceCommanderViewMode::ServiceList;
  finalize_request_storage();
  selected_service_ = {};
  request_rows_.clear();
  response_rows_.clear();
  response_error_.clear();
  selected_request_index_ = 0;
  request_scroll_ = 0;
  status_line_ = "Returned to service list for " + target_node_ + ".";
}

void ServiceCommanderBackend::switch_to_node_list() {
  pending_initial_node_selection_ = false;
  finalize_request_storage();
  selected_service_ = {};
  request_rows_.clear();
  response_rows_.clear();
  response_error_.clear();
  selected_request_index_ = 0;
  request_scroll_ = 0;
  services_.clear();
  selected_service_index_ = 0;
  service_scroll_ = 0;
  view_mode_ = ServiceCommanderViewMode::NodeList;
  refresh_node_list();
}

ServiceIntrospection & ServiceCommanderBackend::get_or_create_introspection(const std::string & type) {
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

std::size_t ServiceCommanderBackend::request_storage_units(std::size_t byte_count) const {
  return (byte_count + sizeof(RequestStorageUnit) - 1) / sizeof(RequestStorageUnit);
}

void ServiceCommanderBackend::allocate_request_storage(std::size_t byte_count) {
  request_storage_.assign(request_storage_units(byte_count), RequestStorageUnit{});
}

void * ServiceCommanderBackend::request_storage_data() {
  return request_storage_.empty() ? nullptr : static_cast<void *>(request_storage_.data());
}

const void * ServiceCommanderBackend::request_storage_data() const {
  return request_storage_.empty() ? nullptr : static_cast<const void *>(request_storage_.data());
}

void ServiceCommanderBackend::finalize_request_storage() {
  if (selected_service_.name.empty() || request_storage_.empty()) {
    request_storage_.clear();
    return;
  }

  auto found = introspection_cache_.find(selected_service_.type);
  if (found != introspection_cache_.end() && found->second.request_members != nullptr) {
    found->second.request_members->fini_function(request_storage_data());
  }
  request_storage_.clear();
}

void ServiceCommanderBackend::reload_selected_service_request() {
  if (selected_service_.name.empty()) {
    return;
  }

  try {
    auto & introspection = get_or_create_introspection(selected_service_.type);
    finalize_request_storage();
    allocate_request_storage(introspection.request_members->size_of_);
    introspection.request_members->init_function(
      request_storage_data(), rosidl_runtime_cpp::MessageInitialization::ALL);
    request_rows_ = flatten_request_message(introspection.request_members, request_storage_data());
    selected_request_index_ = 0;
    request_scroll_ = 0;
    status_line_ = "Loaded default request for " + selected_service_.name + ".";
  } catch (const std::exception & exception) {
    request_storage_.clear();
    request_rows_.clear();
    response_rows_.clear();
    response_error_ = exception.what();
    status_line_ = "Request load failed: " + response_error_;
  }
}

void ServiceCommanderBackend::call_selected_service() {
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

    if (request_storage_.empty()) {
      allocate_request_storage(introspection.request_members->size_of_);
      introspection.request_members->init_function(
        request_storage_data(), rosidl_runtime_cpp::MessageInitialization::ALL);
      request_rows_ = flatten_request_message(introspection.request_members, request_storage_data());
    }

    auto future = client->async_send_request(request_storage_data());
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      client->remove_pending_request(future.request_id);
      status_line_ = "Timed out calling " + selected_service_.name + ".";
      return;
    }

    auto response = future.get();
    response_rows_ = flatten_message(introspection.response_members, response.get());
    response_error_.clear();
    status_line_ = "Called " + selected_service_.name + ".";
  } catch (const std::exception & exception) {
    response_rows_.clear();
    response_error_ = exception.what();
    status_line_ = "Call failed: " + response_error_;
  }
}

std::vector<ServiceEntry> ServiceCommanderBackend::services_for_node(const std::string & full_name) const {
  std::string ns;
  std::string name;
  split_node_name(full_name, ns, name);

  std::vector<ServiceEntry> services;
  const auto discovered = const_cast<ServiceCommanderBackend *>(this)
    ->get_node_graph_interface()
    ->get_service_names_and_types_by_node(name, ns);
  services.reserve(discovered.size());
  for (const auto & [service_name, types] : discovered) {
    if (service_name.empty()) {
      continue;
    }
    ServiceEntry entry;
    entry.name = service_name;
    entry.type = types.empty() ? "<unknown>" : types.front();
    services.push_back(std::move(entry));
  }

  std::sort(
    services.begin(), services.end(),
    [](const ServiceEntry & lhs, const ServiceEntry & rhs) { return lhs.name < rhs.name; });
  return services;
}

std::string ServiceCommanderBackend::normalize_target_name(const std::string & name) const {
  if (name.empty() || name.front() == '/') {
    return name;
  }
  return "/" + name;
}

std::string ServiceCommanderBackend::fully_qualified_node_name(
  const std::string & ns,
  const std::string & name) const
{
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

void ServiceCommanderBackend::split_node_name(
  const std::string & full_name,
  std::string & ns,
  std::string & name) const
{
  const std::size_t slash = full_name.rfind('/');
  if (slash == std::string::npos || slash == 0) {
    ns = "/";
    name = slash == std::string::npos ? full_name : full_name.substr(1);
    return;
  }
  ns = full_name.substr(0, slash);
  name = full_name.substr(slash + 1);
}

RequestRow * ServiceCommanderBackend::selected_request_row() {
  if (request_rows_.empty() || selected_request_index_ < 0 || selected_request_index_ >= static_cast<int>(request_rows_.size())) {
    return nullptr;
  }
  return &request_rows_[static_cast<std::size_t>(selected_request_index_)];
}

const RequestRow * ServiceCommanderBackend::selected_request_row() const {
  if (request_rows_.empty() || selected_request_index_ < 0 || selected_request_index_ >= static_cast<int>(request_rows_.size())) {
    return nullptr;
  }
  return &request_rows_[static_cast<std::size_t>(selected_request_index_)];
}

bool ServiceCommanderBackend::request_row_is_editable(const RequestRow & row) const {
  return row.editable && row.field_memory != nullptr;
}

bool ServiceCommanderBackend::set_selected_request_value(const std::string & value_text) {
  auto * row = selected_request_row();
  if (row == nullptr) {
    status_line_ = "No request field selected.";
    return false;
  }
  if (!request_row_is_editable(*row)) {
    status_line_ = "Selected request field is not editable.";
    return false;
  }

  try {
    switch (row->type_id) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN: {
        const auto parsed = parse_bool_text(value_text);
        if (!parsed.has_value()) {
          status_line_ = "Invalid bool. Use true/false.";
          return false;
        }
        *static_cast<bool *>(row->field_memory) = *parsed;
        break;
      }
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        *static_cast<uint8_t *>(row->field_memory) = static_cast<uint8_t>(std::stoul(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        *static_cast<int8_t *>(row->field_memory) = static_cast<int8_t>(std::stoi(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        *static_cast<uint16_t *>(row->field_memory) = static_cast<uint16_t>(std::stoul(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        *static_cast<int16_t *>(row->field_memory) = static_cast<int16_t>(std::stoi(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        *static_cast<uint32_t *>(row->field_memory) = static_cast<uint32_t>(std::stoul(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        *static_cast<int32_t *>(row->field_memory) = static_cast<int32_t>(std::stol(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        *static_cast<uint64_t *>(row->field_memory) = static_cast<uint64_t>(std::stoull(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        *static_cast<int64_t *>(row->field_memory) = static_cast<int64_t>(std::stoll(value_text));
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        *static_cast<float *>(row->field_memory) = std::stof(value_text);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        *static_cast<double *>(row->field_memory) = std::stod(value_text);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        *static_cast<long double *>(row->field_memory) = std::stold(value_text);
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        *static_cast<char *>(row->field_memory) = value_text.empty() ? '\0' : value_text.front();
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        *static_cast<std::string *>(row->field_memory) = value_text;
        break;
      default:
        status_line_ = "Unsupported request field type for editing.";
        return false;
    }
  } catch (const std::exception &) {
    status_line_ = "Invalid value for selected request field.";
    return false;
  }

  try {
    auto & introspection = get_or_create_introspection(selected_service_.type);
    request_rows_ = flatten_request_message(introspection.request_members, request_storage_data());
    status_line_ = "Updated request field.";
    return true;
  } catch (const std::exception & exception) {
    status_line_ = std::string("Failed to refresh request view: ") + exception.what();
    return false;
  }
}

std::vector<DetailRow> ServiceCommanderBackend::flatten_message(
  const MessageMembers * members, const void * message_memory) const
{
  std::vector<DetailRow> rows;
  append_message_members(rows, 0, members, message_memory);
  return rows;
}

std::vector<RequestRow> ServiceCommanderBackend::flatten_request_message(
  const MessageMembers * members, void * message_memory) const
{
  std::vector<RequestRow> rows;
  append_request_members(rows, 0, members, message_memory);
  return rows;
}

void ServiceCommanderBackend::append_message_members(
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

void ServiceCommanderBackend::append_request_members(
  std::vector<RequestRow> & rows,
  int depth,
  const MessageMembers * members,
  void * message_memory) const
{
  for (uint32_t index = 0; index < members->member_count_; ++index) {
    const auto & member = members->members_[index];
    auto * field_memory = static_cast<uint8_t *>(message_memory) + member.offset_;
    append_request_member(rows, depth, normalize_member_label(*members, member), member, field_memory);
  }
}

std::string ServiceCommanderBackend::normalize_member_label(
  const MessageMembers & parent_members, const MessageMember & member) const
{
  if (std::strcmp(member.name_, "structure_needs_at_least_one_member") != 0) {
    return member.name_;
  }
  if (std::strcmp(parent_members.message_name_, "Empty") == 0) {
    return "empty";
  }
  return "empty";
}

void ServiceCommanderBackend::append_member(
  std::vector<DetailRow> & rows,
  int depth,
  const std::string & label,
  const MessageMember & member,
  const void * field_memory) const
{
  if (member.is_array_) {
    const std::size_t element_count =
      member.size_function != nullptr ? member.size_function(field_memory) : member.array_size_;
    rows.emplace_back(depth, label, "[" + std::to_string(element_count) + "]");

    for (std::size_t index = 0; index < element_count; ++index) {
      const std::string child_label = "[" + std::to_string(index) + "]";
      if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
        const void * element_memory =
          member.get_const_function != nullptr ? member.get_const_function(field_memory, index) : nullptr;
        if (element_memory != nullptr) {
          rows.emplace_back(depth + 1, child_label, "<message>");
          append_message_members(rows, depth + 2, sub_members, element_memory);
        }
        continue;
      }

      rows.emplace_back(depth + 1, child_label, array_element_to_string(member, field_memory, index));
    }
    return;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    rows.emplace_back(depth, label, "<message>");
    const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
    append_message_members(rows, depth + 1, sub_members, field_memory);
    return;
  }

  rows.emplace_back(depth, label, scalar_field_to_string(member.type_id_, field_memory));
}

void ServiceCommanderBackend::append_request_member(
  std::vector<RequestRow> & rows,
  int depth,
  const std::string & label,
  const MessageMember & member,
  void * field_memory) const
{
  if (member.is_array_) {
    const std::size_t element_count =
      member.size_function != nullptr ? member.size_function(field_memory) : member.array_size_;
    rows.emplace_back(depth, label, "[" + std::to_string(element_count) + "]", false, member.type_id_, nullptr);

    for (std::size_t index = 0; index < element_count; ++index) {
      const std::string child_label = "[" + std::to_string(index) + "]";
      if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
        void * element_memory =
          member.get_function != nullptr ? member.get_function(field_memory, index) : nullptr;
        if (element_memory != nullptr) {
          rows.emplace_back(depth + 1, child_label, "<message>", false, member.type_id_, nullptr);
          append_request_members(rows, depth + 2, sub_members, element_memory);
        }
        continue;
      }

      rows.emplace_back(
        depth + 1, child_label, array_element_to_string(member, field_memory, index), false, member.type_id_,
        nullptr);
    }
    return;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    rows.emplace_back(depth, label, "<message>", false, member.type_id_, nullptr);
    const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
    append_request_members(rows, depth + 1, sub_members, field_memory);
    return;
  }

  const bool editable = member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING;
  rows.emplace_back(
    depth, label, scalar_field_to_string(member.type_id_, field_memory), editable, member.type_id_,
    editable ? field_memory : nullptr);
}

std::string ServiceCommanderBackend::array_element_to_string(
  const MessageMember & member,
  const void * field_memory,
  std::size_t index) const
{
  switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN: {
      bool value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
      uint8_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
      int8_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
      uint16_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
      int16_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
      uint32_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
      int32_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
      uint64_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
      int64_t value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
      float value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
      double value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE: {
      long double value{};
      member.fetch_function(field_memory, index, &value);
      return service_scalar_to_string(value);
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

std::string ServiceCommanderBackend::scalar_field_to_string(uint8_t type_id, const void * field_memory) const {
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      return service_scalar_to_string(*static_cast<const bool *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return service_scalar_to_string(*static_cast<const uint8_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return service_scalar_to_string(*static_cast<const int8_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return service_scalar_to_string(*static_cast<const uint16_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return service_scalar_to_string(*static_cast<const int16_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return service_scalar_to_string(*static_cast<const uint32_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return service_scalar_to_string(*static_cast<const int32_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return service_scalar_to_string(*static_cast<const uint64_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return service_scalar_to_string(*static_cast<const int64_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return service_scalar_to_string(*static_cast<const float *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return service_scalar_to_string(*static_cast<const double *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      return service_scalar_to_string(*static_cast<const long double *>(field_memory));
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

std::optional<bool> ServiceCommanderBackend::parse_bool_text(const std::string & value_text) const {
  std::string lowered;
  lowered.reserve(value_text.size());
  for (unsigned char character : value_text) {
    lowered.push_back(static_cast<char>(std::tolower(character)));
  }
  if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
    return false;
  }
  return std::nullopt;
}

}  // namespace ros2_console_tools
