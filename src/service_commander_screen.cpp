#include "ros2_console_tools/service_commander.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_service_commander_tool(const std::string & initial_service, bool embedded_mode) {
  auto backend = std::make_shared<ServiceCommanderBackend>(initial_service);
  ServiceCommanderScreen screen(backend, embedded_mode);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backend);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = screen.run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return result;
}

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorPopup = tui::kColorPopup,
  kColorInput = tui::kColorInput,
};

using tui::Session;
using tui::draw_box;
using tui::draw_box_char;
using tui::draw_help_bar;
using tui::draw_help_bar_region;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::draw_text_hline;
using tui::draw_text_vline;
using tui::apply_role_chgat;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;
using tui::truncate_text;

}  // namespace

ServiceCommanderScreen::ServiceCommanderScreen(
  std::shared_ptr<ServiceCommanderBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int ServiceCommanderScreen::run() {
  Session ncurses_session;
  backend_->refresh_services();

  bool running = true;
  while (running && rclcpp::ok()) {
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool ServiceCommanderScreen::handle_key(int key) {
  if (edit_popup_open_) {
    return handle_edit_popup_key(key);
  }
  if (search_state_.active) {
    return handle_search_key(key);
  }
  switch (backend_->view_mode_) {
    case ServiceCommanderViewMode::ServiceList:
      return handle_service_list_key(key);
    case ServiceCommanderViewMode::ServiceDetail:
      return handle_service_detail_key(key);
  }
  return true;
}

bool ServiceCommanderScreen::handle_service_list_key(int key) {
  backend_->clamp_service_selection();
  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_services();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->status_line_ = "Search.";
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      return true;
    case KEY_UP:
    case 'k':
      if (backend_->selected_service_index_ > 0) {
        --backend_->selected_service_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_service_index_ + 1 < static_cast<int>(backend_->services_.size())) {
        ++backend_->selected_service_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_service_index_ = std::max(0, backend_->selected_service_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->services_.empty()) {
        backend_->selected_service_index_ = std::min(
          static_cast<int>(backend_->services_.size()) - 1,
          backend_->selected_service_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_selected_service();
      return true;
    default:
      return true;
  }
}

bool ServiceCommanderScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    backend_->status_line_ =
      search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query;
    return true;
  }
  if (result != SearchInputResult::Changed || backend_->view_mode_ != ServiceCommanderViewMode::ServiceList) {
    return true;
  }

  std::vector<std::string> labels;
  labels.reserve(backend_->services_.size());
  for (const auto & entry : backend_->services_) {
    labels.push_back(entry.name);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_service_index_);
  if (match >= 0) {
    backend_->selected_service_index_ = match;
  }
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

bool ServiceCommanderScreen::handle_service_detail_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
      if (embedded_mode_) {
        return false;
      }
      backend_->close_service_detail();
      return true;
    case KEY_F(4):
      backend_->refresh_services();
      backend_->reload_selected_service_request();
      return true;
    case KEY_F(3):
      backend_->reload_selected_service_request();
      return true;
    case KEY_F(2):
      backend_->call_selected_service();
      return true;
    case KEY_UP:
    case 'k':
      if (backend_->selected_request_index_ > 0) {
        --backend_->selected_request_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_request_index_ + 1 < static_cast<int>(backend_->request_rows_.size())) {
        ++backend_->selected_request_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_request_index_ = std::max(0, backend_->selected_request_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->request_rows_.empty()) {
        backend_->selected_request_index_ = std::min(
          static_cast<int>(backend_->request_rows_.size()) - 1,
          backend_->selected_request_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER: {
      const auto * row = backend_->selected_request_row();
      if (row == nullptr) {
        backend_->status_line_ = "No request field selected.";
        return true;
      }
      if (!backend_->request_row_is_editable(*row)) {
        backend_->status_line_ = "Selected request field is not editable.";
        return true;
      }
      edit_popup_open_ = true;
      edit_popup_dirty_ = false;
      edit_popup_buffer_ = row->value;
      return true;
    }
    default:
      return true;
  }
}

bool ServiceCommanderScreen::handle_edit_popup_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
      edit_popup_open_ = false;
      edit_popup_dirty_ = false;
      edit_popup_buffer_.clear();
      return true;
    case KEY_F(2):
    case '\n':
    case KEY_ENTER:
      if (backend_->set_selected_request_value(edit_popup_buffer_)) {
        edit_popup_open_ = false;
        edit_popup_dirty_ = false;
        edit_popup_buffer_.clear();
      }
      return true;
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      if (!edit_popup_buffer_.empty()) {
        edit_popup_buffer_.pop_back();
        edit_popup_dirty_ = true;
      }
      return true;
    case KEY_DC:
      edit_popup_buffer_.clear();
      edit_popup_dirty_ = true;
      return true;
    default:
      if (key >= 32 && key <= 126) {
        edit_popup_buffer_.push_back(static_cast<char>(key));
        edit_popup_dirty_ = true;
      }
      return true;
  }
}

int ServiceCommanderScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void ServiceCommanderScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Service Commander ");
  attroff(theme_attr(kColorTitle));
  if (backend_->view_mode_ == ServiceCommanderViewMode::ServiceList) {
    draw_service_list(1, 1, content_bottom - 1, columns - 2);
  } else {
    draw_service_detail(1, 1, content_bottom - 1, columns - 2);
  }
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  if (edit_popup_open_) {
    draw_edit_popup(rows, columns);
  }
  refresh();
}

void ServiceCommanderScreen::draw_service_list(int top, int left, int bottom, int right) {
  backend_->clamp_service_selection();
  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  const int name_width = std::max(32, width / 2);
  const int type_width = std::max(12, width - name_width - 1);
  const int separator_x = left + name_width;

  if (backend_->selected_service_index_ < backend_->service_scroll_) {
    backend_->service_scroll_ = backend_->selected_service_index_;
  }
  if (backend_->selected_service_index_ >= backend_->service_scroll_ + visible_rows - 1) {
    backend_->service_scroll_ = std::max(0, backend_->selected_service_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", name_width, "Service");
  draw_box_char(top, separator_x, WACS_VLINE, '|');
  mvprintw(top, separator_x + 1, "%-*s", type_width, "Type");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->service_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->services_.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_service_index_);

    mvhline(row, left, ' ', width);
    if (selected) {
      apply_role_chgat(row, left, width, tui::kColorSelection);
    }
    draw_box_char(row, separator_x, WACS_VLINE, '|');
    if (!has_item) {
      continue;
    }

    const auto & entry = backend_->services_[static_cast<std::size_t>(first_row + (row - top - 1))];
    mvprintw(row, left, "%-*s", name_width, truncate_text(entry.name, name_width).c_str());
    mvprintw(row, separator_x + 1, "%-*s", type_width, truncate_text(entry.type, type_width).c_str());
    if (selected) {
      apply_role_chgat(row, left, width, tui::kColorSelection);
      mvaddch(row, separator_x, '|');
    }
  }
}

void ServiceCommanderScreen::draw_response_panel(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Response");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  if (!backend_->response_error_.empty()) {
    mvprintw(row_y, left, "%s", truncate_text(backend_->response_error_, width).c_str());
    return;
  }

  for (const auto & row : backend_->response_rows_) {
    if (row_y > bottom) {
      break;
    }
    const std::string line =
      std::string(static_cast<std::size_t>(row.depth * 2), ' ') + row.field + ": " + row.value;
    mvprintw(row_y, left, "%-*s", width, truncate_text(line, width).c_str());
    ++row_y;
  }
}

void ServiceCommanderScreen::draw_service_detail(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  const int left_width = std::max(24, width / 2);
  const int separator_x = left + left_width;
  const int request_width = separator_x - left;
  const int response_width = right - separator_x;
  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(top, separator_x, bottom - top + 1);
  attroff(COLOR_PAIR(kColorFrame));

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", request_width, "Request");
  attroff(theme_attr(kColorHeader));

  const int visible_rows = std::max(1, bottom - top);
  if (backend_->selected_request_index_ < backend_->request_scroll_) {
    backend_->request_scroll_ = backend_->selected_request_index_;
  }
  if (backend_->selected_request_index_ >= backend_->request_scroll_ + visible_rows) {
    backend_->request_scroll_ = backend_->selected_request_index_ - visible_rows + 1;
  }

  const int first_row = backend_->request_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->request_rows_.size()), first_row + visible_rows);
  for (int row_y = top + 1; row_y <= bottom; ++row_y) {
    const bool has_item = first_row + (row_y - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row_y - top - 1) == backend_->selected_request_index_);
    mvhline(row_y, left, ' ', request_width);
    if (selected) {
      apply_role_chgat(row_y, left, request_width, kColorSelection);
    }
    if (!has_item) {
      continue;
    }
    const auto & row = backend_->request_rows_[static_cast<std::size_t>(first_row + (row_y - top - 1))];
    std::string line = std::string(static_cast<std::size_t>(row.depth * 2), ' ') + row.field + ": " + row.value;
    if (row.editable) {
      line += " *";
    }
    mvprintw(row_y, left, "%-*s", request_width, truncate_text(line, request_width).c_str());
    if (selected) {
      apply_role_chgat(row_y, left, request_width, kColorSelection);
    }
  }

  draw_response_panel(top, separator_x + 1, bottom, right);
  (void)response_width;
}

void ServiceCommanderScreen::draw_edit_popup(int rows, int columns) const {
  const auto * row = backend_->selected_request_row();
  if (row == nullptr) {
    return;
  }

  const int popup_width = std::min(columns - 6, 80);
  const int popup_height = 9;
  const int left = std::max(2, (columns - popup_width) / 2);
  const int top = std::max(2, (rows - popup_height) / 2);
  const int right = left + popup_width - 1;
  const int bottom = top + popup_height - 1;
  const int inner_width = popup_width - 2;

  for (int row_y = top + 1; row_y < bottom; ++row_y) {
    attron(COLOR_PAIR(kColorPopup));
    mvhline(row_y, left + 1, ' ', inner_width);
    attroff(COLOR_PAIR(kColorPopup));
  }
  draw_box(top, left, bottom, right, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(top, left + 2, " Edit Request Field ");
  attroff(theme_attr(kColorTitle));

  mvprintw(top + 1, left + 2, "%s", truncate_text(row->field, popup_width - 4).c_str());
  mvprintw(top + 2, left + 2, "type: %u", static_cast<unsigned int>(row->type_id));
  attron(COLOR_PAIR(kColorPopup));
  draw_text_hline(top + 3, left + 1, inner_width);
  attroff(COLOR_PAIR(kColorPopup));
  draw_box(top + 4, left + 2, top + 6, right - 2, kColorFrame);
  attron(COLOR_PAIR(kColorInput));
  mvhline(top + 5, left + 3, ' ', popup_width - 6);
  mvaddnstr(top + 5, left + 3, edit_popup_buffer_.c_str(), popup_width - 6);
  attroff(COLOR_PAIR(kColorInput));

  draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "F2 Save  Enter Save  Esc Close  F10 Exit");
}

void ServiceCommanderScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;
  if (backend_->view_mode_ == ServiceCommanderViewMode::ServiceDetail && !backend_->selected_service_.name.empty()) {
    line = truncate_text(
      backend_->selected_service_.name + " [" + backend_->selected_service_.type + "]  " + backend_->status_line_,
      columns - 2);
  }
  draw_status_bar(row, columns, line);
}

void ServiceCommanderScreen::draw_help_line(int row, int columns) const {
  const std::string help =
    edit_popup_open_
    ? "F2 Save  Enter Save  Esc Close  F10 Exit"
    : backend_->view_mode_ == ServiceCommanderViewMode::ServiceDetail
    ? "Enter Edit  F2 Call  F3 Reset  F4 Refresh  Esc Back  F10 Exit"
    : "Enter Inspect  Alt+S Search  F4 Refresh  F10 Exit";
  draw_help_bar(row, columns, truncate_text(help, columns - 1));
}

}  // namespace ros2_console_tools
