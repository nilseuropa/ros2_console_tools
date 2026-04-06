#include "ros2_console_tools/action_commander.hpp"

#include <algorithm>

namespace ros2_console_tools {

namespace {

bool ends_with(const std::string & text, const std::string & suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void sort_unique(std::vector<std::string> & values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

}  // namespace

ActionCommanderBackend::ActionCommanderBackend(const std::string & initial_action)
: Node("action_commander"),
  initial_action_name_(initial_action),
  auto_open_initial_action_(!initial_action.empty())
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

std::string ActionCommanderBackend::action_name_from_send_goal_service(const std::string & service_name) {
  static const std::string canonical_suffix = "/_action/send_goal";
  static const std::string short_suffix = "/send_goal";
  if (ends_with(service_name, canonical_suffix)) {
    return service_name.substr(0, service_name.size() - canonical_suffix.size());
  }
  if (ends_with(service_name, short_suffix)) {
    return service_name.substr(0, service_name.size() - short_suffix.size());
  }
  return "";
}

std::string ActionCommanderBackend::action_type_from_send_goal_type(const std::string & service_type) {
  static const std::string suffix = "_SendGoal";
  if (!ends_with(service_type, suffix)) {
    return service_type;
  }
  return service_type.substr(0, service_type.size() - suffix.size());
}

std::string ActionCommanderBackend::fully_qualified_node_name(const std::string & ns, const std::string & name) {
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

void ActionCommanderBackend::refresh_actions() {
  const auto services = this->get_service_names_and_types();
  std::map<std::string, ActionEntry> discovered;

  for (const auto & [service_name, service_types] : services) {
    const std::string action_name = action_name_from_send_goal_service(service_name);
    if (action_name.empty() || service_types.empty()) {
      continue;
    }
    ActionEntry entry;
    entry.name = action_name;
    entry.type = action_type_from_send_goal_type(service_types.front());
    discovered[action_name] = std::move(entry);
  }

  actions_.clear();
  actions_.reserve(discovered.size());
  for (auto & [name, entry] : discovered) {
    (void)name;
    actions_.push_back(std::move(entry));
  }

  std::sort(
    actions_.begin(), actions_.end(),
    [](const ActionEntry & lhs, const ActionEntry & rhs) { return lhs.name < rhs.name; });

  clamp_selection();
  select_initial_action_if_present();
  status_line_ = actions_.empty()
    ? "No ROS 2 actions discovered."
    : "Loaded " + std::to_string(actions_.size()) + " actions.";
}

void ActionCommanderBackend::clamp_selection() {
  if (actions_.empty()) {
    selected_index_ = 0;
    list_scroll_ = 0;
    return;
  }
  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(actions_.size()) - 1);
}

void ActionCommanderBackend::select_initial_action_if_present() {
  if (initial_action_name_.empty()) {
    return;
  }

  for (std::size_t index = 0; index < actions_.size(); ++index) {
    if (actions_[index].name == initial_action_name_) {
      selected_index_ = static_cast<int>(index);
      initial_action_name_.clear();
      if (auto_open_initial_action_) {
        auto_open_initial_action_ = false;
        open_selected_action();
      }
      return;
    }
  }
}

void ActionCommanderBackend::open_selected_action() {
  clamp_selection();
  if (actions_.empty()) {
    status_line_ = "No action selected.";
    return;
  }
  selected_action_ = actions_[static_cast<std::size_t>(selected_index_)];
  detail_open_ = true;
  status_line_ = "Inspecting " + selected_action_.name + ".";
}

void ActionCommanderBackend::close_action_detail() {
  detail_open_ = false;
  selected_action_ = {};
  status_line_ = "Returned to action list.";
}

ActionCommanderBackend::ActionDetails ActionCommanderBackend::build_action_details(const ActionEntry & entry) const {
  ActionDetails details;
  details.name = entry.name;
  details.type = entry.type;
  details.send_goal_service = entry.name + "/_action/send_goal";
  details.get_result_service = entry.name + "/_action/get_result";
  details.cancel_goal_service = entry.name + "/_action/cancel_goal";
  details.feedback_topic = entry.name + "/_action/feedback";
  details.status_topic = entry.name + "/_action/status";

  details.send_goal_servers = this->count_services(details.send_goal_service);
  details.send_goal_clients = this->count_clients(details.send_goal_service);
  details.get_result_servers = this->count_services(details.get_result_service);
  details.get_result_clients = this->count_clients(details.get_result_service);
  details.cancel_servers = this->count_services(details.cancel_goal_service);
  details.cancel_clients = this->count_clients(details.cancel_goal_service);
  details.feedback_publishers = this->count_publishers(details.feedback_topic);
  details.feedback_subscribers = this->count_subscribers(details.feedback_topic);
  details.status_publishers = this->count_publishers(details.status_topic);
  details.status_subscribers = this->count_subscribers(details.status_topic);

  auto graph = const_cast<ActionCommanderBackend *>(this)->get_node_graph_interface();
  const auto nodes = graph->get_node_names_and_namespaces();
  for (const auto & [node_name, node_ns] : nodes) {
    const std::string full_name = fully_qualified_node_name(node_ns, node_name);
    if (full_name == this->get_fully_qualified_name()) {
      continue;
    }

    const auto provided_services =
      graph->get_service_names_and_types_by_node(node_name, node_ns);
    for (const auto & item : provided_services) {
      if (
        item.first == details.send_goal_service ||
        item.first == details.get_result_service ||
        item.first == details.cancel_goal_service)
      {
        details.server_nodes.push_back(full_name);
        break;
      }
    }

    const auto used_clients =
      graph->get_client_names_and_types_by_node(node_name, node_ns);
    for (const auto & item : used_clients) {
      if (
        item.first == details.send_goal_service ||
        item.first == details.get_result_service ||
        item.first == details.cancel_goal_service)
      {
        details.client_nodes.push_back(full_name);
        break;
      }
    }
  }

  sort_unique(details.server_nodes);
  sort_unique(details.client_nodes);
  return details;
}

std::vector<ActionDetailLine> ActionCommanderBackend::selected_action_details() const {
  if (!detail_open_ || selected_action_.name.empty()) {
    return {{"No action selected.", false}};
  }

  const ActionDetails details = build_action_details(selected_action_);
  std::vector<ActionDetailLine> lines;
  auto add_header = [&lines](const std::string & text) { lines.push_back({text, true}); };
  auto add_line = [&lines](const std::string & text) { lines.push_back({text, false}); };

  add_header("Action");
  add_line(details.name);
  add_line("type: " + details.type);

  add_header("Protocol");
  add_line("send_goal:   " + details.send_goal_service);
  add_line("get_result:  " + details.get_result_service);
  add_line("cancel_goal: " + details.cancel_goal_service);
  add_line("feedback:    " + details.feedback_topic);
  add_line("status:      " + details.status_topic);

  add_header("Activity");
  add_line(
    "send_goal srv/cli: " + std::to_string(details.send_goal_servers) + "/" +
    std::to_string(details.send_goal_clients));
  add_line(
    "get_result srv/cli: " + std::to_string(details.get_result_servers) + "/" +
    std::to_string(details.get_result_clients));
  add_line(
    "cancel_goal srv/cli: " + std::to_string(details.cancel_servers) + "/" +
    std::to_string(details.cancel_clients));
  add_line(
    "feedback pub/sub: " + std::to_string(details.feedback_publishers) + "/" +
    std::to_string(details.feedback_subscribers));
  add_line(
    "status pub/sub:   " + std::to_string(details.status_publishers) + "/" +
    std::to_string(details.status_subscribers));

  add_header("Servers");
  if (details.server_nodes.empty()) {
    add_line("-");
  } else {
    for (const auto & node : details.server_nodes) {
      add_line(node);
    }
  }

  add_header("Clients");
  if (details.client_nodes.empty()) {
    add_line("-");
  } else {
    for (const auto & node : details.client_nodes) {
      add_line(node);
    }
  }

  return lines;
}

}  // namespace ros2_console_tools
