## ros2_console_tools

`ros2_console_tools` is a terminal-first toolbox for ROS 2 runtime inspection and operations.

The package contains several standalone ncurses-based tools with a shared TUI layer and a common architectural pattern, plus one older renderer node that does not follow that structure yet.

# Node Commander

![](doc/ros2_console_tools_demo.gif)

Binary:

```bash
ros2 run ros2_console_tools node_commander
```

Purpose:

- inspect the live ROS 2 node graph
- browse nodes and their graph interfaces
- check whether parameter services are reachable

Highlights:

- node list on the left
- detail pane with publishers, subscribers, and services
- parameter service availability indicator
- embedded tool launchers, including `F9` for `diagnostics_viewer`
- shared incremental `Alt+S` search

Main files:

- [include/ros2_console_tools/node_commander.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/node_commander.hpp)
- [src/node_commander_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/node_commander_backend.cpp)
- [src/node_commander_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/node_commander_screen.cpp)
- [src/node_commander.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/node_commander.cpp)

## Tools

### parameter_commander

Binary:

```bash
ros2 run ros2_console_tools parameter_commander
```

Purpose:

- browse ROS 2 nodes
- inspect parameters of a selected node
- edit scalar parameter values
- display descriptors and constraints

Highlights:

- node list view
- namespace-folded parameter tree
- parameter editor popup
- refresh selected/all
- read-only awareness

Main files:

- [include/ros2_console_tools/parameter_commander.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/parameter_commander.hpp)
- [src/parameter_commander_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/parameter_commander_backend.cpp)
- [src/parameter_commander_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/parameter_commander_screen.cpp)
- [src/parameter_commander.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/parameter_commander.cpp)

### topic_monitor

Binary:

```bash
ros2 run ros2_console_tools topic_monitor
```

Purpose:

- discover topics
- monitor selected topics for rate and bandwidth
- inspect decoded message structure and values

Highlights:

- namespace-folded topic list
- monitor toggling
- `Avg Hz`, `Min/Max Hz`, `Bandwidth`
- stale/monitored coloring
- structured message detail view

Main files:

- [include/ros2_console_tools/topic_monitor.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/topic_monitor.hpp)
- [src/topic_monitor_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/topic_monitor_backend.cpp)
- [src/topic_monitor_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/topic_monitor_screen.cpp)
- [src/topic_monitor.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/topic_monitor.cpp)

### service_commander

Binary:

```bash
ros2 run ros2_console_tools service_commander
```

Purpose:

- browse available ROS 2 services
- inspect request/response structures
- edit scalar request fields
- call services interactively

Highlights:

- service list view
- request/response split view
- editable scalar request fields
- introspection-based generic service calling

Main files:

- [include/ros2_console_tools/service_commander.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/service_commander.hpp)
- [src/service_commander_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/service_commander_backend.cpp)
- [src/service_commander_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/service_commander_screen.cpp)
- [src/service_commander.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/service_commander.cpp)

Current limitation:

- request editing is currently intended for scalar fields
- arrays and more advanced structured editing still need dedicated UI work

### action_commander

Binary:

```bash
ros2 run ros2_console_tools action_commander
```

Optional action argument:

```bash
ros2 run ros2_console_tools action_commander /navigate_to_pose
```

Purpose:

- discover ROS 2 actions from the live graph
- inspect action protocol endpoints
- inspect server/client activity for an action

Highlights:

- action list view
- action detail view
- protocol breakdown for `send_goal`, `get_result`, `cancel_goal`, `feedback`, and `status`
- server/client node lists derived from the graph

Main files:

- [include/ros2_console_tools/action_commander.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/action_commander.hpp)
- [src/action_commander_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/action_commander_backend.cpp)
- [src/action_commander_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/action_commander_screen.cpp)
- [src/action_commander.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/action_commander.cpp)

Current limitation:

- v1 is inspection-focused
- goal editing, goal sending, feedback streaming, and result retrieval still need dedicated action-client UI work

### log_viewer

Binary:

```bash
ros2 run ros2_console_tools log_viewer
```

Purpose:

- monitor `/rosout`
- filter by source, level, and text
- inspect log details
- inspect emitting source code when available locally

Highlights:

- source pane + log pane
- live per-source log view
- severity filtering
- text filter
- code inspection popup/fullscreen view with syntax highlighting

Main files:

- [include/ros2_console_tools/log_viewer.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/log_viewer.hpp)
- [src/log_viewer_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/log_viewer_backend.cpp)
- [src/log_viewer_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/log_viewer_screen.cpp)
- [src/log_viewer.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/log_viewer.cpp)

### diagnostics_viewer

Binary:

```bash
ros2 run ros2_console_tools diagnostics_viewer
```

Purpose:

- monitor `/diagnostics` and `/diagnostics_agg`
- inspect current diagnostic status levels and messages
- inspect detailed key/value pairs for a selected status

Highlights:

- status list on the left
- detail pane for the selected diagnostic entry
- severity filtering
- shared incremental `Alt+S` search
- embeddable in `node_commander` on `F9`

Main files:

- [include/ros2_console_tools/diagnostics_viewer.hpp](/home/nils/ros2_extras/src/ros2_console_tools/include/ros2_console_tools/diagnostics_viewer.hpp)
- [src/diagnostics_viewer_backend.cpp](/home/nils/ros2_extras/src/ros2_console_tools/src/diagnostics_viewer_backend.cpp)
- [src/diagnostics_viewer_screen.cpp](/home/nils/ros2_extras/src/ros2_console_tools/src/diagnostics_viewer_screen.cpp)
- [src/diagnostics_viewer.cpp](/home/nils/ros2_extras/src/ros2_console_tools/src/diagnostics_viewer.cpp)

### tf_monitor

Binary:

```bash
ros2 run ros2_console_tools tf_monitor
```

Purpose:

- inspect the live TF tree
- see transform freshness
- inspect relative transforms between selected frames

Highlights:

- tree view of frames
- freshness display for dynamic/static transforms
- stale transform highlighting
- two-frame selection and relative transform popup

Main files:

- [include/ros2_console_tools/tf_monitor.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/tf_monitor.hpp)
- [src/tf_monitor_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/tf_monitor_backend.cpp)
- [src/tf_monitor_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/tf_monitor_screen.cpp)
- [src/tf_monitor.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/tf_monitor.cpp)

### urdf_inspector

Binary:

```bash
ros2 run ros2_console_tools urdf_inspector
```

Optional target node:

```bash
ros2 run ros2_console_tools urdf_inspector /robot_state_publisher
```

Purpose:

- load `robot_description`
- inspect URDF link/joint tree
- inspect selected item details
- inspect the original XML section for a selected link or joint

Highlights:

- auto-discovery of a node exposing `robot_description`
- collapsible tree
- details pane
- XML inspection popup with syntax highlighting
- mesh filename display in details

Main files:

- [include/ros2_console_tools/urdf_inspector.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/urdf_inspector.hpp)
- [src/urdf_inspector_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/urdf_inspector_backend.cpp)
- [src/urdf_inspector_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/urdf_inspector_screen.cpp)
- [src/urdf_inspector.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/urdf_inspector.cpp)

### map_viewer

Binary:

```bash
ros2 run ros2_console_tools map_viewer
```

Optional topic argument:

```bash
ros2 run ros2_console_tools map_viewer /map
```

Purpose:

- terminal visualization of `nav_msgs/msg/OccupancyGrid`
- rotated map rendering
- costmap-style block rendering
- monochrome/legend control

Highlights:

- same backend/screen/thin-main structure as the other tools
- shared ncurses/TUI foundation
- launchable through a reusable `run_map_viewer_tool(...)` entry point
- preserves the existing occupancy-grid oriented parameters

Main files:

- [include/ros2_console_tools/map_viewer.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/map_viewer.hpp)
- [src/map_viewer_backend.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/map_viewer_backend.cpp)
- [src/map_viewer_screen.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/map_viewer_screen.cpp)
- [src/map_viewer.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/map_viewer.cpp)

## Current Architecture

Most tools now follow the same 3-layer structure:

1. `tui`
   Shared terminal infrastructure in:
   - [include/ros2_console_tools/tui.hpp](/home/nils/dev_ws/src/ros2_console_tools/include/ros2_console_tools/tui.hpp)
   - [src/tui.cpp](/home/nils/dev_ws/src/ros2_console_tools/src/tui.cpp)

   Responsibilities:
   - ncurses session setup/teardown
   - Unicode-safe line drawing with fallback behavior
   - global theme/color roles
   - status/help bar rendering
   - popup help-bar rendering
   - incremental `Alt+S` search support

2. `backend`
   ROS-facing logic and tool state.

   Responsibilities:
   - node creation
   - subscriptions/clients/service calls
   - data refresh and caching
   - domain-specific model preparation for the screen

   This layer should avoid ncurses concerns as much as possible.

3. `screen`
   TUI controller/view.

   Responsibilities:
   - key handling
   - list navigation
   - pane and popup drawing
   - selection/search/edit workflows

4. thin executable wrapper
   The executable `main()` should do only:
   - `rclcpp::init`
   - create backend
   - create screen
   - run executor thread
   - run screen loop
   - shutdown cleanly

## Tool Structure

For a tool following the current structure, the files should look like this:

- `include/ros2_console_tools/<tool>.hpp`
- `src/<tool>_backend.cpp`
- `src/<tool>_screen.cpp`
- `src/<tool>.cpp`

The CMake layout should also follow the same pattern:

- `<tool>_lib`
  Contains backend + screen + shared `tui.cpp`
- `<tool>`
  Thin executable linked against `<tool>_lib`

## Shared TUI Layer

The shared TUI is the most important foundation for future growth.

It already centralizes:

- terminal session setup
- global theme roles
- line drawing
- status/help bars
- popup help bars
- `Alt+S` search behavior

Future work should continue moving common UI behavior here, for example:

- standard list/tree widgets
- split-pane layout helpers
- popup form controls
- reusable code/text viewers
- configuration-driven theming

## Expansion API

The current implementation already gives a clear template for adding new tools.

### 1. Add a public header

Define:

- domain structs used by backend/screen
- `Backend : public rclcpp::Node`
- `Screen`

Typical shape:

```cpp
class ExampleScreen;

class ExampleBackend : public rclcpp::Node {
public:
  ExampleBackend();

private:
  friend class ExampleScreen;

  void refresh();
  void set_status(const std::string & text);

  std::string status_line_;
};

class ExampleScreen {
public:
  explicit ExampleScreen(std::shared_ptr<ExampleBackend> backend);
  int run();

private:
  bool handle_key(int key);
  void draw();

  std::shared_ptr<ExampleBackend> backend_;
  tui::SearchState search_state_;
};
```

### 2. Keep backend ownership clear

The backend should own:

- ROS interfaces
- cached data
- selected domain objects
- refresh/load/save/call logic

The screen should not duplicate backend state unless it is strictly UI-local, such as:

- popup open/closed flags
- temporary edit buffers
- search state
- local scroll state for a popup

### 3. Reuse the shared TUI

Prefer the shared TUI helpers instead of per-tool ad hoc rendering:

- `tui::Session`
- `tui::draw_box`
- `tui::draw_status_bar`
- `tui::draw_help_bar`
- `tui::draw_help_bar_region`
- `tui::draw_search_box`
- `tui::find_best_match`
- `tui::handle_search_input`

### 4. Keep keybindings consistent

Default conventions currently in use:

- `F10`: exit
- `Esc`: back/close
- `Enter`: inspect/open/edit depending on context
- `Alt+S`: incremental search
- `F4`: refresh current scope
- `F2`: primary action in current context
- `F3`: secondary non-destructive action in current context
- `Space`: mark/select where multiselect or monitoring exists
- `Left` / `Right`: fold/unfold tree structures

New tools should follow these unless there is a strong reason not to.

### 5. Prefer additive evolution

- keep each tool independently runnable
- factor shared behavior into `tui`
- avoid pushing unrelated workflows into one tool prematurely
