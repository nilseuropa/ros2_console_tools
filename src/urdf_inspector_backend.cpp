#include "ros2_console_tools/urdf_inspector.hpp"

#include <limits>
#include <thread>

namespace ros2_console_tools {

UrdfInspectorBackend::UrdfInspectorBackend(const std::string & target_node)
: Node("urdf_inspector"),
  target_node_(this->declare_parameter<std::string>("target_node", target_node))
{}

void UrdfInspectorBackend::refresh_model() {
  std::string xml;
  std::string source_node;
  const bool loaded = !target_node_.empty()
    ? load_from_target_node(xml, source_node)
    : discover_and_load(xml, source_node);
  last_refresh_time_ = std::chrono::steady_clock::now();

  if (!loaded) {
    model_.reset();
    rows_.clear();
    selected_index_ = 0;
    tree_scroll_ = 0;
    return;
  }

  auto model = std::make_shared<urdf::Model>();
  if (!model->initString(xml)) {
    model_.reset();
    rows_.clear();
    status_line_ = "Failed to parse URDF from " + source_node + ".";
    return;
  }

  model_ = std::move(model);
  source_node_ = source_node;
  rebuild_rows();
  status_line_ =
    "Loaded " + std::to_string(model_->links_.size()) + " links and "
    + std::to_string(model_->joints_.size()) + " joints from " + source_node_ + ".";
}

void UrdfInspectorBackend::warm_up_model() {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  while (model_ == nullptr && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    refresh_model();
    if (model_ != nullptr) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
}

void UrdfInspectorBackend::maybe_refresh_model() {
  if (model_ != nullptr) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_refresh_time_ != std::chrono::steady_clock::time_point::min() &&
    now - last_refresh_time_ < std::chrono::seconds(1))
  {
    return;
  }
  refresh_model();
}

bool UrdfInspectorBackend::load_from_target_node(std::string & xml_out, std::string & source_node_out) {
  if (!try_load_from_node(target_node_, xml_out)) {
    status_line_ = "Could not load " + parameter_name_ + " from " + target_node_ + ".";
    return false;
  }
  source_node_out = target_node_;
  return true;
}

bool UrdfInspectorBackend::discover_and_load(std::string & xml_out, std::string & source_node_out) {
  const auto nodes = this->get_node_graph_interface()->get_node_names_and_namespaces();
  std::vector<std::string> candidates;
  candidates.reserve(nodes.size());
  for (const auto & node : nodes) {
    std::string full_name;
    if (node.second.empty() || node.second == "/") {
      full_name = "/" + node.first;
    } else if (node.second.back() == '/') {
      full_name = node.second + node.first;
    } else {
      full_name = node.second + "/" + node.first;
    }
    if (full_name == this->get_fully_qualified_name()) {
      continue;
    }
    candidates.push_back(full_name);
  }
  std::sort(candidates.begin(), candidates.end());

  for (const auto & candidate : candidates) {
    if (try_load_from_node(candidate, xml_out)) {
      source_node_out = candidate;
      return true;
    }
  }

  status_line_ = "No node exposing robot_description was found.";
  return false;
}

bool UrdfInspectorBackend::try_load_from_node(const std::string & node_name, std::string & xml_out) const {
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(
    const_cast<UrdfInspectorBackend *>(this), node_name);
  if (!client->wait_for_service(std::chrono::milliseconds(250))) {
    return false;
  }

  auto future = client->get_parameters({parameter_name_});
  if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    return false;
  }

  const auto values = future.get();
  if (values.empty()) {
    return false;
  }

  const auto & value = values.front().get_parameter_value();
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    return false;
  }
  const std::string xml = value.get<std::string>();
  if (xml.empty()) {
    return false;
  }

  xml_out = xml;
  return true;
}

void UrdfInspectorBackend::rebuild_rows() {
  rows_.clear();
  if (!model_) {
    return;
  }

  append_link_rows(model_->getRoot(), 0);
  clamp_selection();
}

void UrdfInspectorBackend::append_link_rows(const urdf::LinkConstSharedPtr & link, int depth) {
  if (!link) {
    return;
  }

  const std::string parent_link_name =
    (link->getParent() != nullptr && link->getParent()->parent_joint != nullptr)
    ? link->getParent()->name
    : "";
  rows_.push_back(UrdfTreeRow{true, link->name, link->name, "", parent_link_name, depth});

  const auto collapsed = collapsed_links_.find(link->name);
  if (collapsed != collapsed_links_.end() && collapsed->second) {
    return;
  }

  for (const auto & child_link : link->child_links) {
    if (!child_link || !child_link->parent_joint) {
      continue;
    }
    rows_.push_back(UrdfTreeRow{
      false,
      child_link->parent_joint->name,
      child_link->name,
      child_link->parent_joint->name,
      link->name,
      depth + 1});
    append_link_rows(child_link, depth + 2);
  }
}

void UrdfInspectorBackend::clamp_selection() {
  if (rows_.empty()) {
    selected_index_ = 0;
    tree_scroll_ = 0;
    return;
  }
  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(rows_.size()) - 1);
}

void UrdfInspectorBackend::expand_selected() {
  clamp_selection();
  if (rows_.empty()) {
    return;
  }
  const auto & row = rows_[static_cast<std::size_t>(selected_index_)];
  if (!row.is_link) {
    return;
  }
  collapsed_links_[row.link_name] = false;
  rebuild_rows();
}

void UrdfInspectorBackend::collapse_selected() {
  clamp_selection();
  if (rows_.empty()) {
    return;
  }
  const auto & row = rows_[static_cast<std::size_t>(selected_index_)];
  if (row.is_link) {
    const auto collapsed = collapsed_links_.find(row.link_name);
    if (collapsed == collapsed_links_.end() || !collapsed->second) {
      collapsed_links_[row.link_name] = true;
      rebuild_rows();
      return;
    }
  }

  const int parent_index = selected_parent_link_index();
  if (parent_index >= 0) {
    selected_index_ = parent_index;
    const auto & parent_row = rows_[static_cast<std::size_t>(selected_index_)];
    if (parent_row.is_link) {
      collapsed_links_[parent_row.link_name] = true;
      rebuild_rows();
    }
  }
}

std::vector<std::string> UrdfInspectorBackend::selected_details() const {
  if (rows_.empty()) {
    return {"No URDF loaded."};
  }
  if (auto link = selected_link()) {
    return link_details(link);
  }
  if (auto joint = selected_joint()) {
    return joint_details(joint);
  }
  return {"No selection."};
}

std::vector<std::string> UrdfInspectorBackend::link_details(const urdf::LinkConstSharedPtr & link) const {
  std::vector<std::string> lines;
  lines.push_back("type: link");
  lines.push_back("name: " + link->name);
  lines.push_back("parent: " + (link->getParent() ? link->getParent()->name : "<root>"));
  lines.push_back("child links: " + std::to_string(link->child_links.size()));
  lines.push_back("child joints: " + std::to_string(link->child_joints.size()));
  lines.push_back("visuals: " + std::to_string(link->visual_array.size()));
  lines.push_back("collisions: " + std::to_string(link->collision_array.size()));

  if (link->inertial) {
    lines.push_back("mass: " + format_double(link->inertial->mass));
    lines.push_back("com xyz: " + format_vec3(link->inertial->origin.position));
    lines.push_back("com rpy: " + format_rpy(link->inertial->origin.rotation));
  } else {
    lines.push_back("mass: -");
  }

  if (!link->visual_array.empty() && link->visual_array.front() && link->visual_array.front()->geometry) {
    lines.push_back("visual geometry: " + geometry_type_name(link->visual_array.front()->geometry->type));
  }
  if (!link->collision_array.empty() && link->collision_array.front() && link->collision_array.front()->geometry) {
    lines.push_back("collision geometry: " + geometry_type_name(link->collision_array.front()->geometry->type));
  }

  return lines;
}

std::vector<std::string> UrdfInspectorBackend::joint_details(const urdf::JointConstSharedPtr & joint) const {
  std::vector<std::string> lines;
  lines.push_back("type: joint");
  lines.push_back("name: " + joint->name);
  lines.push_back("joint type: " + joint_type_name(joint->type));
  lines.push_back("parent: " + joint->parent_link_name);
  lines.push_back("child: " + joint->child_link_name);
  lines.push_back("origin xyz: " + format_vec3(joint->parent_to_joint_origin_transform.position));
  lines.push_back("origin rpy: " + format_rpy(joint->parent_to_joint_origin_transform.rotation));

  lines.push_back("axis xyz: " + format_vec3(joint->axis));
  if (joint->limits) {
    lines.push_back("lower: " + format_double(joint->limits->lower));
    lines.push_back("upper: " + format_double(joint->limits->upper));
    lines.push_back("effort: " + format_double(joint->limits->effort));
    lines.push_back("velocity: " + format_double(joint->limits->velocity));
  }

  return lines;
}

urdf::LinkConstSharedPtr UrdfInspectorBackend::selected_link() const {
  if (!model_ || rows_.empty()) {
    return {};
  }
  const auto & row = rows_[static_cast<std::size_t>(selected_index_)];
  if (!row.is_link) {
    return {};
  }
  return model_->getLink(row.link_name);
}

urdf::JointConstSharedPtr UrdfInspectorBackend::selected_joint() const {
  if (!model_ || rows_.empty()) {
    return {};
  }
  const auto & row = rows_[static_cast<std::size_t>(selected_index_)];
  if (row.is_link || row.joint_name.empty()) {
    return {};
  }
  auto found = model_->joints_.find(row.joint_name);
  if (found == model_->joints_.end()) {
    return {};
  }
  return found->second;
}

int UrdfInspectorBackend::selected_parent_link_index() const {
  if (rows_.empty()) {
    return -1;
  }
  const auto & row = rows_[static_cast<std::size_t>(selected_index_)];
  if (row.parent_link_name.empty()) {
    return -1;
  }
  for (std::size_t index = 0; index < rows_.size(); ++index) {
    if (rows_[index].is_link && rows_[index].link_name == row.parent_link_name) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

}  // namespace ros2_console_tools
