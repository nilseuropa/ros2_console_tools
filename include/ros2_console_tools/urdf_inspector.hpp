#ifndef ROS2_CONSOLE_TOOLS__URDF_INSPECTOR_HPP_
#define ROS2_CONSOLE_TOOLS__URDF_INSPECTOR_HPP_

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_urdf_inspector_tool(const std::string & target_node = "", bool embedded_mode = false);

inline std::string format_double(double value, int precision = 3) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

inline std::string format_vec3(const urdf::Vector3 & vector) {
  return format_double(vector.x) + "  " + format_double(vector.y) + "  " + format_double(vector.z);
}

inline std::string format_rpy(const urdf::Rotation & rotation) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rotation.getRPY(roll, pitch, yaw);
  return format_double(roll) + "  " + format_double(pitch) + "  " + format_double(yaw);
}

inline std::string joint_type_name(int type) {
  switch (type) {
    case urdf::Joint::REVOLUTE:
      return "revolute";
    case urdf::Joint::CONTINUOUS:
      return "continuous";
    case urdf::Joint::PRISMATIC:
      return "prismatic";
    case urdf::Joint::FLOATING:
      return "floating";
    case urdf::Joint::PLANAR:
      return "planar";
    case urdf::Joint::FIXED:
      return "fixed";
    case urdf::Joint::UNKNOWN:
    default:
      return "unknown";
  }
}

inline std::string geometry_type_name(int type) {
  switch (type) {
    case urdf::Geometry::SPHERE:
      return "sphere";
    case urdf::Geometry::BOX:
      return "box";
    case urdf::Geometry::CYLINDER:
      return "cylinder";
    case urdf::Geometry::MESH:
      return "mesh";
    default:
      return "geometry";
  }
}

struct UrdfTreeRow {
  bool is_link{false};
  std::string label;
  std::string link_name;
  std::string joint_name;
  std::string parent_link_name;
  int depth{0};
};

struct XmlSpan {
  std::string text;
  int color{0};
};

class UrdfInspectorScreen;

class UrdfInspectorBackend : public rclcpp::Node {
public:
  explicit UrdfInspectorBackend(const std::string & target_node);

private:
  friend class UrdfInspectorScreen;

  void refresh_model();
  void warm_up_model();
  void maybe_refresh_model();
  bool load_from_target_node(std::string & xml_out, std::string & source_node_out);
  bool discover_and_load(std::string & xml_out, std::string & source_node_out);
  bool try_load_from_node(const std::string & node_name, std::string & xml_out) const;
  void rebuild_rows();
  void append_link_rows(const urdf::LinkConstSharedPtr & link, int depth);
  void clamp_selection();
  void expand_selected();
  void collapse_selected();
  std::optional<std::string> selected_xml_section() const;
  std::vector<std::string> selected_details() const;
  std::vector<std::string> link_details(const urdf::LinkConstSharedPtr & link) const;
  std::vector<std::string> joint_details(const urdf::JointConstSharedPtr & joint) const;
  std::optional<std::string> extract_tag_section(
    const std::string & tag_name, const std::string & element_name) const;
  urdf::LinkConstSharedPtr selected_link() const;
  urdf::JointConstSharedPtr selected_joint() const;
  int selected_parent_link_index() const;

  std::string target_node_;
  std::string parameter_name_{"robot_description"};
  std::string source_node_;
  std::string model_xml_;
  std::shared_ptr<urdf::Model> model_;
  std::vector<UrdfTreeRow> rows_;
  std::map<std::string, bool> collapsed_links_;
  int selected_index_{0};
  int tree_scroll_{0};
  std::string status_line_{"Loading robot_description..."};
  std::chrono::steady_clock::time_point last_refresh_time_{std::chrono::steady_clock::time_point::min()};
};

class UrdfInspectorScreen {
public:
  explicit UrdfInspectorScreen(std::shared_ptr<UrdfInspectorBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_popup_key(int key);
  static std::vector<std::string> pretty_print_xml_lines(const std::string & text);
  static std::vector<std::string> split_lines(const std::string & text);
  std::vector<XmlSpan> highlight_xml_line(const std::string & line) const;
  int page_step() const;
  void draw();
  void draw_tree_pane(int top, int left, int bottom, int right);
  void draw_details_pane(int top, int left, int bottom, int right) const;
  void draw_xml_popup(int rows, int columns) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<UrdfInspectorBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
  bool popup_open_{false};
  int popup_scroll_{0};
  std::string popup_title_;
  std::vector<std::string> popup_lines_;
  tui::TerminalPane terminal_pane_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__URDF_INSPECTOR_HPP_
