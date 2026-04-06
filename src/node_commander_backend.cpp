#include "ros2_console_tools/node_commander.hpp"

#include <rclcpp/parameter_client.hpp>

#include <thread>

namespace ros2_console_tools {

NodeCommanderBackend::NodeCommanderBackend()
: Node("node_commander")
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

void NodeCommanderBackend::refresh_nodes() {
  const auto nodes = this->get_node_graph_interface()->get_node_names_and_namespaces();
  std::vector<std::string> refreshed;
  refreshed.reserve(nodes.size());

  for (const auto & node : nodes) {
    const std::string full_name = fully_qualified_node_name(node.second, node.first);
    if (full_name == this->get_fully_qualified_name()) {
      continue;
    }
    refreshed.push_back(full_name);
  }

  std::sort(refreshed.begin(), refreshed.end());
  refreshed.erase(std::unique(refreshed.begin(), refreshed.end()), refreshed.end());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    node_entries_ = std::move(refreshed);
    clamp_selection();
    last_refresh_time_ = std::chrono::steady_clock::now();
    status_line_ = node_entries_.empty()
      ? "No ROS 2 nodes discovered."
      : "Loaded " + std::to_string(node_entries_.size()) + " nodes.";
  }
}

void NodeCommanderBackend::warm_up_node_list() {
  if (!node_entries_.empty()) {
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  while (node_entries_.empty() && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    refresh_nodes();
  }
}

void NodeCommanderBackend::maybe_refresh_nodes() {
  const auto now = std::chrono::steady_clock::now();
  if (now - last_refresh_time_ >= std::chrono::seconds(1)) {
    refresh_nodes();
  }
}

void NodeCommanderBackend::clamp_selection() {
  if (node_entries_.empty()) {
    selected_index_ = 0;
    node_scroll_ = 0;
    return;
  }
  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(node_entries_.size()) - 1);
}

std::string NodeCommanderBackend::fully_qualified_node_name(const std::string & ns, const std::string & name) const {
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

std::vector<DetailLine> NodeCommanderBackend::selected_node_details() const {
  std::string selected_node;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (node_entries_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(node_entries_.size())) {
      return {{"No node selected.", false, NodeDetailAction::None, ""}};
    }
    selected_node = node_entries_[static_cast<std::size_t>(selected_index_)];
  }

  const NodeDetails details = build_node_details(selected_node);
  std::vector<DetailLine> lines;
  lines.push_back({"Node", true, NodeDetailAction::None, ""});
  lines.push_back({details.node_name, false, NodeDetailAction::OpenParameters, details.node_name});
  lines.push_back({"Parameter Services", true, NodeDetailAction::None, ""});
  lines.push_back({
    details.parameter_services_ready ? "available" : "unavailable",
    false,
    NodeDetailAction::OpenParameters,
    details.node_name});

  auto append_section = [&lines](const std::string & title, const std::vector<GraphEndpoint> & entries) {
    lines.push_back({title, true, NodeDetailAction::None, ""});
    if (entries.empty()) {
      lines.push_back({"-", false, NodeDetailAction::None, ""});
      return;
    }
    for (const auto & entry : entries) {
      NodeDetailAction action = NodeDetailAction::None;
      if (title == "Publishers" || title == "Subscribers") {
        action = NodeDetailAction::OpenTopicMonitor;
      } else if (title == "Services") {
        action = NodeDetailAction::OpenServiceCommander;
      }
      lines.push_back({entry.name + " [" + entry.type + "]", false, action, entry.name});
    }
  };

  append_section("Publishers", details.publishers);
  append_section("Subscribers", details.subscribers);
  append_section("Services", details.services);
  return lines;
}

NodeDetails NodeCommanderBackend::build_node_details(const std::string & full_name) const {
  NodeDetails details;
  details.node_name = full_name;

  auto parse_name = [](const std::string & full, std::string & ns, std::string & name) {
    const std::size_t slash = full.rfind('/');
    if (slash == std::string::npos || slash == 0) {
      ns = "/";
      name = slash == std::string::npos ? full : full.substr(1);
      return;
    }
    ns = full.substr(0, slash);
    name = full.substr(slash + 1);
  };

  std::string ns;
  std::string name;
  parse_name(full_name, ns, name);

  auto append_graph = [](std::vector<GraphEndpoint> & output, const auto & names_and_types) {
    for (const auto & item : names_and_types) {
      GraphEndpoint entry;
      entry.name = item.first;
      entry.type = item.second.empty() ? "<unknown>" : item.second.front();
      output.push_back(std::move(entry));
    }
    std::sort(
      output.begin(), output.end(),
      [](const GraphEndpoint & lhs, const GraphEndpoint & rhs) { return lhs.name < rhs.name; });
  };

  auto graph = const_cast<NodeCommanderBackend *>(this)->get_node_graph_interface();
  append_graph(details.publishers, graph->get_publisher_names_and_types_by_node(name, ns));
  append_graph(details.subscribers, graph->get_subscriber_names_and_types_by_node(name, ns));
  append_graph(details.services, graph->get_service_names_and_types_by_node(name, ns));

  auto client = std::make_shared<rclcpp::AsyncParametersClient>(const_cast<NodeCommanderBackend *>(this), full_name);
  details.parameter_services_ready = client->wait_for_service(std::chrono::milliseconds(200));
  return details;
}

}  // namespace ros2_console_tools
