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
  model_xml_ = xml;
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

std::optional<std::string> UrdfInspectorBackend::selected_xml_section() const {
  if (rows_.empty()) {
    return std::nullopt;
  }
  const auto & row = rows_[static_cast<std::size_t>(selected_index_)];
  if (row.is_link) {
    return extract_tag_section("link", row.link_name);
  }
  if (!row.joint_name.empty()) {
    return extract_tag_section("joint", row.joint_name);
  }
  return std::nullopt;
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
    const auto & geometry = link->visual_array.front()->geometry;
    std::string geometry_text = geometry_type_name(geometry->type);
    if (geometry->type == urdf::Geometry::MESH) {
      if (auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry)) {
        geometry_text += "  " + mesh->filename;
      }
    }
    lines.push_back("visual geometry: " + geometry_text);
  }
  if (!link->collision_array.empty() && link->collision_array.front() && link->collision_array.front()->geometry) {
    const auto & geometry = link->collision_array.front()->geometry;
    std::string geometry_text = geometry_type_name(geometry->type);
    if (geometry->type == urdf::Geometry::MESH) {
      if (auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry)) {
        geometry_text += "  " + mesh->filename;
      }
    }
    lines.push_back("collision geometry: " + geometry_text);
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

std::optional<std::string> UrdfInspectorBackend::extract_tag_section(
  const std::string & tag_name, const std::string & element_name) const
{
  if (model_xml_.empty()) {
    return std::nullopt;
  }

  const std::string open_pattern = "<" + tag_name;
  std::size_t search_pos = 0;
  while (true) {
    const std::size_t tag_start = model_xml_.find(open_pattern, search_pos);
    if (tag_start == std::string::npos) {
      return std::nullopt;
    }

    const std::size_t tag_end = model_xml_.find('>', tag_start);
    if (tag_end == std::string::npos) {
      return std::nullopt;
    }

    const std::string open_tag = model_xml_.substr(tag_start, tag_end - tag_start + 1);
    const std::string name_attr = "name=\"" + element_name + "\"";
    const std::string alt_name_attr = "name='" + element_name + "'";
    if (open_tag.find(name_attr) == std::string::npos && open_tag.find(alt_name_attr) == std::string::npos) {
      search_pos = tag_end + 1;
      continue;
    }

    if (tag_end > tag_start && model_xml_[tag_end - 1] == '/') {
      return model_xml_.substr(tag_start, tag_end - tag_start + 1);
    }

    int depth = 1;
    std::size_t cursor = tag_end + 1;
    while (depth > 0 && cursor < model_xml_.size()) {
      const std::size_t next_open = model_xml_.find(open_pattern, cursor);
      const std::size_t next_close = model_xml_.find("</" + tag_name, cursor);
      if (next_close == std::string::npos) {
        return std::nullopt;
      }

      if (next_open != std::string::npos && next_open < next_close) {
        const std::size_t nested_end = model_xml_.find('>', next_open);
        if (nested_end == std::string::npos) {
          return std::nullopt;
        }
        if (!(nested_end > next_open && model_xml_[nested_end - 1] == '/')) {
          ++depth;
        }
        cursor = nested_end + 1;
        continue;
      }

      const std::size_t close_end = model_xml_.find('>', next_close);
      if (close_end == std::string::npos) {
        return std::nullopt;
      }
      --depth;
      cursor = close_end + 1;
      if (depth == 0) {
        return model_xml_.substr(tag_start, close_end - tag_start + 1);
      }
    }
    return std::nullopt;
  }
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
