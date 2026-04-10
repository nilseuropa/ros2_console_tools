#include "ros2_console_tools/parameter_commander.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace ros2_console_tools {

ParameterCommanderBackend::ParameterCommanderBackend(const std::string & target_node)
: Node("parameter_commander"),
  target_node_(normalize_target_name(this->declare_parameter<std::string>("target_node", target_node)))
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }
  current_view_ = target_node_.empty()
    ? ParameterCommanderViewMode::NodeList
    : ParameterCommanderViewMode::ParameterList;
  if (!target_node_.empty()) {
    reset_parameter_client();
  }
}

void ParameterCommanderBackend::refresh_all() {
  if (current_view_ == ParameterCommanderViewMode::NodeList) {
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
      selected_parameter_item_index_ = 0;
      set_status("No parameters found on " + target_node_ + ".");
      return;
    }

    const auto items = visible_parameter_items();
    if (items.empty()) {
      selected_parameter_item_index_ = 0;
    } else {
      selected_parameter_item_index_ = std::clamp(
        selected_parameter_item_index_, 0, static_cast<int>(items.size()) - 1);
    }
    sync_edit_buffer_from_selected();
    set_status("Loaded " + std::to_string(entries_.size()) + " parameters from " + target_node_ + ".");
  } catch (const std::exception & exception) {
    set_status(std::string("Refresh all failed: ") + exception.what());
  }
}

void ParameterCommanderBackend::refresh_selected() {
  if (current_view_ != ParameterCommanderViewMode::ParameterList) {
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
    set_status("Refreshed " + entry->name + ".");
  } catch (const std::exception & exception) {
    set_status(std::string("Refresh failed: ") + exception.what());
  }
}

void ParameterCommanderBackend::save_selected() {
  if (current_view_ != ParameterCommanderViewMode::ParameterList) {
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
  if (!is_scalar_type(entry->descriptor.type) && !is_array_type(entry->descriptor.type)) {
    set_status("This parameter type is not editable.");
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
    set_status("Saved " + entry->name + ".");
  } catch (const std::exception & exception) {
    set_status(std::string("Save failed: ") + exception.what());
  }
}

std::optional<rclcpp::Parameter> ParameterCommanderBackend::parse_edit_buffer(const ParameterEntry & entry) {
  const std::string trimmed = trim(entry.edit_buffer);
  auto parse_int64 = [](const std::string & token) {
      std::size_t parsed_chars = 0;
      const std::string value = trim(token);
      const auto parsed = static_cast<int64_t>(std::stoll(value, &parsed_chars));
      if (parsed_chars != value.size()) {
        throw std::invalid_argument("trailing characters");
      }
      return parsed;
    };
  auto parse_double = [](const std::string & token) {
      std::size_t parsed_chars = 0;
      const std::string value = trim(token);
      const double parsed = std::stod(value, &parsed_chars);
      if (parsed_chars != value.size()) {
        throw std::invalid_argument("trailing characters");
      }
      return parsed;
    };

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
        return rclcpp::Parameter(entry.name, parse_int64(trimmed));
      case ParameterType::PARAMETER_DOUBLE:
        return rclcpp::Parameter(entry.name, parse_double(trimmed));
      case ParameterType::PARAMETER_STRING:
        return rclcpp::Parameter(entry.name, entry.edit_buffer);
      case ParameterType::PARAMETER_BOOL_ARRAY: {
        std::vector<bool> values;
        for (const auto & token : parse_array_tokens(entry.edit_buffer)) {
          const std::string lowered = lowercase(trim(token));
          if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
            values.push_back(true);
          } else if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
            values.push_back(false);
          } else {
            set_status("Invalid bool array item. Use true/false.");
            return std::nullopt;
          }
        }
        return rclcpp::Parameter(entry.name, values);
      }
      case ParameterType::PARAMETER_INTEGER_ARRAY: {
        std::vector<int64_t> values;
        for (const auto & token : parse_array_tokens(entry.edit_buffer)) {
          values.push_back(parse_int64(token));
        }
        return rclcpp::Parameter(entry.name, values);
      }
      case ParameterType::PARAMETER_DOUBLE_ARRAY: {
        std::vector<double> values;
        for (const auto & token : parse_array_tokens(entry.edit_buffer)) {
          values.push_back(parse_double(token));
        }
        return rclcpp::Parameter(entry.name, values);
      }
      case ParameterType::PARAMETER_STRING_ARRAY:
        return rclcpp::Parameter(entry.name, parse_array_tokens(entry.edit_buffer));
      case ParameterType::PARAMETER_BYTE_ARRAY: {
        std::vector<uint8_t> values;
        for (const auto & token : parse_array_tokens(entry.edit_buffer)) {
          const int64_t parsed = parse_int64(token);
          if (parsed < 0 || parsed > 255) {
            set_status("Invalid byte array item. Use values from 0 to 255.");
            return std::nullopt;
          }
          values.push_back(static_cast<uint8_t>(parsed));
        }
        return rclcpp::Parameter(entry.name, values);
      }
      default:
        set_status("Unsupported parameter type for editing.");
        return std::nullopt;
    }
  } catch (const std::exception &) {
    set_status("Invalid value for type " + parameter_type_name(entry.descriptor.type) + ".");
    return std::nullopt;
  }
}

void ParameterCommanderBackend::refresh_node_list() {
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

void ParameterCommanderBackend::select_current_node() {
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
  selected_parameter_item_index_ = 0;
  list_scroll_ = 0;
  collapsed_namespaces_.clear();
  current_view_ = ParameterCommanderViewMode::ParameterList;
  refresh_all();
}

void ParameterCommanderBackend::switch_to_node_list() {
  current_view_ = ParameterCommanderViewMode::NodeList;
  entries_.clear();
  selected_parameter_item_index_ = 0;
  list_scroll_ = 0;
  refresh_node_list();
}

void ParameterCommanderBackend::reset_parameter_client() {
  client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, target_node_);
}

void ParameterCommanderBackend::maybe_refresh_node_list() {
  if (current_view_ != ParameterCommanderViewMode::NodeList) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_node_refresh_time_ >= std::chrono::seconds(1)) {
    refresh_node_list();
  }
}

void ParameterCommanderBackend::warm_up_node_list() {
  if (!node_entries_.empty()) {
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  while (node_entries_.empty() && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    refresh_node_list();
  }
}

std::string ParameterCommanderBackend::lowercase(std::string value) const {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string ParameterCommanderBackend::normalize_target_name(const std::string & name) const {
  if (name.empty() || name.front() == '/') {
    return name;
  }
  return "/" + name;
}

std::string ParameterCommanderBackend::fully_qualified_node_name(const std::string & ns, const std::string & name) const {
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

void ParameterCommanderBackend::toggle_bool_buffer(std::string & buffer) {
  const std::string lowered = lowercase(trim(buffer));
  buffer = (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on")
    ? "false"
    : "true";
}

bool ParameterCommanderBackend::wait_for_parameter_service() {
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

std::vector<std::string> ParameterCommanderBackend::parse_array_tokens(const std::string & buffer) {
  std::string text = trim(buffer);
  if (text.empty()) {
    return {};
  }
  if (text.front() == '[') {
    if (text.back() != ']') {
      set_status("Invalid array. Missing closing ].");
      throw std::invalid_argument("missing closing array bracket");
    }
    text = trim(text.substr(1, text.size() - 2));
  }
  if (text.empty()) {
    return {};
  }

  std::vector<std::string> tokens;
  std::string current;
  bool in_quote = false;
  char quote_char = '\0';
  bool token_was_quoted = false;
  bool quote_closed = false;
  bool escape = false;

  auto finish_token = [&]() {
      tokens.push_back(token_was_quoted ? current : trim(current));
      current.clear();
      token_was_quoted = false;
      quote_closed = false;
    };

  for (char character : text) {
    if (quote_closed) {
      if (std::isspace(static_cast<unsigned char>(character))) {
        continue;
      }
      if (character == ',') {
        finish_token();
        continue;
      }
      set_status("Invalid array. Unexpected text after quoted item.");
      throw std::invalid_argument("text after quoted array item");
    }
    if (escape) {
      current.push_back(character);
      escape = false;
      continue;
    }
    if (in_quote) {
      if (character == '\\') {
        escape = true;
        continue;
      }
      if (character == quote_char) {
        in_quote = false;
        token_was_quoted = true;
        quote_closed = true;
        continue;
      }
      current.push_back(character);
      continue;
    }
    if (character == '\'' || character == '"') {
      if (!trim(current).empty()) {
        set_status("Invalid array. Quote must start an item.");
        throw std::invalid_argument("quote inside unquoted array item");
      }
      current.clear();
      in_quote = true;
      quote_char = character;
      continue;
    }
    if (character == ',') {
      finish_token();
      continue;
    }
    current.push_back(character);
  }

  if (escape || in_quote) {
    set_status("Invalid array. Unterminated quoted item.");
    throw std::invalid_argument("unterminated quoted array item");
  }
  finish_token();
  return tokens;
}

void ParameterCommanderBackend::sync_edit_buffer_from_selected() {
  auto * entry = selected_entry();
  if (entry == nullptr || entry->dirty) {
    return;
  }
  entry->edit_buffer = parameter_value_to_string(entry->value);
}

ParameterEntry * ParameterCommanderBackend::selected_entry() {
  auto items = visible_parameter_items();
  if (items.empty()
    || selected_parameter_item_index_ < 0
    || selected_parameter_item_index_ >= static_cast<int>(items.size()))
  {
    return nullptr;
  }
  const auto & item = items[static_cast<std::size_t>(selected_parameter_item_index_)];
  return item.is_namespace ? nullptr : item.entry;
}

const ParameterEntry * ParameterCommanderBackend::selected_entry() const {
  auto items = visible_parameter_items();
  if (items.empty()
    || selected_parameter_item_index_ < 0
    || selected_parameter_item_index_ >= static_cast<int>(items.size()))
  {
    return nullptr;
  }
  const auto & item = items[static_cast<std::size_t>(selected_parameter_item_index_)];
  return item.is_namespace ? nullptr : item.entry;
}

void ParameterCommanderBackend::activate_selected_parameter_item() {
  auto items = visible_parameter_items();
  if (items.empty()
    || selected_parameter_item_index_ < 0
    || selected_parameter_item_index_ >= static_cast<int>(items.size()))
  {
    return;
  }

  const auto & item = items[static_cast<std::size_t>(selected_parameter_item_index_)];
  if (item.is_namespace) {
    collapsed_namespaces_[item.namespace_path] = is_namespace_expanded(item.namespace_path);
    const auto updated_items = visible_parameter_items();
    if (!updated_items.empty()) {
      selected_parameter_item_index_ = std::min(
        selected_parameter_item_index_, static_cast<int>(updated_items.size()) - 1);
    }
  }
}

void ParameterCommanderBackend::expand_selected_namespace() {
  auto items = visible_parameter_items();
  if (items.empty()
    || selected_parameter_item_index_ < 0
    || selected_parameter_item_index_ >= static_cast<int>(items.size()))
  {
    return;
  }
  const auto & item = items[static_cast<std::size_t>(selected_parameter_item_index_)];
  if (!item.is_namespace) {
    return;
  }
  collapsed_namespaces_[item.namespace_path] = false;
}

void ParameterCommanderBackend::collapse_selected_namespace() {
  auto items = visible_parameter_items();
  if (items.empty()
    || selected_parameter_item_index_ < 0
    || selected_parameter_item_index_ >= static_cast<int>(items.size()))
  {
    return;
  }
  const auto & item = items[static_cast<std::size_t>(selected_parameter_item_index_)];
  if (item.is_namespace) {
    collapsed_namespaces_[item.namespace_path] = true;
    return;
  }

  const std::string parent_namespace = parameter_namespace(item.entry->name);
  if (parent_namespace.empty()) {
    return;
  }

  for (std::size_t index = 0; index < items.size(); ++index) {
    if (items[index].is_namespace && items[index].namespace_path == parent_namespace) {
      selected_parameter_item_index_ = static_cast<int>(index);
      collapsed_namespaces_[parent_namespace] = true;
      return;
    }
  }
}

std::vector<ParameterViewItem> ParameterCommanderBackend::visible_parameter_items() const {
  std::vector<ParameterViewItem> items;
  append_namespace_children("", 0, items);
  return items;
}

void ParameterCommanderBackend::append_namespace_children(
  const std::string & parent_namespace, int depth, std::vector<ParameterViewItem> & items) const
{
  std::map<std::string, bool> namespace_children;
  std::vector<const ParameterEntry *> direct_entries;

  for (const auto & entry : entries_) {
    const std::string entry_namespace = parameter_namespace(entry.name);
    if (!is_in_namespace(entry_namespace, parent_namespace)) {
      continue;
    }

    const std::string remainder = parent_namespace.empty()
      ? entry_namespace
      : (entry_namespace == parent_namespace
          ? std::string()
          : entry_namespace.substr(parent_namespace.size() + 1));
    if (remainder.empty()) {
      direct_entries.push_back(&entry);
      continue;
    }

    const auto delimiter = remainder.find('.');
    const std::string child_segment = delimiter == std::string::npos ? remainder : remainder.substr(0, delimiter);
    const std::string child_namespace = parent_namespace.empty() ? child_segment : parent_namespace + "." + child_segment;
    namespace_children[child_namespace] = true;
  }

  for (const auto & [child_namespace, has_entries] : namespace_children) {
    (void)has_entries;
    ParameterViewItem folder;
    folder.is_namespace = true;
    folder.label = namespace_label(child_namespace);
    folder.namespace_path = child_namespace;
    folder.depth = depth;
    items.push_back(folder);
    if (is_namespace_expanded(child_namespace)) {
      append_namespace_children(child_namespace, depth + 1, items);
    }
  }

  for (const auto * entry : direct_entries) {
    ParameterViewItem leaf;
    leaf.is_namespace = false;
    leaf.label = parameter_leaf_name(entry->name);
    leaf.depth = depth;
    leaf.entry = const_cast<ParameterEntry *>(entry);
    items.push_back(std::move(leaf));
  }
}

std::string ParameterCommanderBackend::parameter_namespace(const std::string & parameter_name) const {
  const auto delimiter = parameter_name.rfind('.');
  if (delimiter == std::string::npos) {
    return "";
  }
  return parameter_name.substr(0, delimiter);
}

std::string ParameterCommanderBackend::parameter_leaf_name(const std::string & parameter_name) const {
  const auto delimiter = parameter_name.rfind('.');
  if (delimiter == std::string::npos) {
    return parameter_name;
  }
  return parameter_name.substr(delimiter + 1);
}

std::string ParameterCommanderBackend::namespace_label(const std::string & namespace_path) const {
  const auto delimiter = namespace_path.rfind('.');
  if (delimiter == std::string::npos) {
    return namespace_path;
  }
  return namespace_path.substr(delimiter + 1);
}

bool ParameterCommanderBackend::is_in_namespace(
  const std::string & entry_namespace, const std::string & parent_namespace) const
{
  if (parent_namespace.empty()) {
    return true;
  }
  if (entry_namespace == parent_namespace) {
    return true;
  }
  return entry_namespace.rfind(parent_namespace + ".", 0) == 0;
}

bool ParameterCommanderBackend::is_namespace_expanded(const std::string & namespace_path) const {
  const auto it = collapsed_namespaces_.find(namespace_path);
  if (it == collapsed_namespaces_.end()) {
    return false;
  }
  return !it->second;
}

std::string ParameterCommanderBackend::summary_value(const ParameterEntry & entry) const {
  if (!entry.has_value) {
    return "<no value>";
  }
  return single_line(parameter_value_to_string(entry.value));
}

std::string ParameterCommanderBackend::descriptor_summary(const ParameterDescriptor & descriptor) const {
  if (!descriptor.description.empty()) {
    return single_line(descriptor.description);
  }
  if (!descriptor.additional_constraints.empty()) {
    return single_line(descriptor.additional_constraints);
  }
  if (descriptor.read_only) {
    return "read-only";
  }
  return "-";
}

std::string ParameterCommanderBackend::parameter_min(const ParameterEntry & entry) const {
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

std::string ParameterCommanderBackend::parameter_max(const ParameterEntry & entry) const {
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

bool ParameterCommanderBackend::popup_is_editable(const ParameterEntry & entry) const {
  return (is_scalar_type(entry.descriptor.type) || is_array_type(entry.descriptor.type))
    && !entry.descriptor.read_only;
}

void ParameterCommanderBackend::set_status(const std::string & message) {
  status_message_ = message;
}

}  // namespace ros2_console_tools
