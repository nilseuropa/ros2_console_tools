#include "ros2_console_tools/parameter_commander.hpp"

#include <ncursesw/ncurses.h>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorStatus = tui::kColorStatus,
  kColorHelp = tui::kColorHelp,
  kColorPopup = tui::kColorPopup,
  kColorInput = tui::kColorInput,
  kColorDirty = tui::kColorDirty,
  kColorCursor = tui::kColorCursor,
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
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;

}  // namespace

ParameterCommanderScreen::ParameterCommanderScreen(std::shared_ptr<ParameterCommanderBackend> backend)
: backend_(std::move(backend)) {}

int ParameterCommanderScreen::run() {
  Session ncurses_session;
  backend_->refresh_node_list();
  if (backend_->current_view_ == ParameterCommanderViewMode::NodeList) {
    backend_->warm_up_node_list();
    backend_->set_status("Choose a node and press Enter.");
  } else {
    backend_->set_status("Connecting to " + backend_->target_node_ + "...");
    if (!backend_->client_ || !backend_->client_->wait_for_service(std::chrono::seconds(2))) {
      backend_->set_status("Parameter services for " + backend_->target_node_ + " are not available.");
    } else {
      backend_->refresh_all();
    }
  }

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->maybe_refresh_node_list();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool ParameterCommanderScreen::handle_key(int key) {
  if (popup_open_) {
    return handle_popup_key(key);
  }
  if (search_state_.active) {
    return handle_search_key(key);
  }

  switch (key) {
    case KEY_F(10):
      curs_set(1);
      return false;
    case KEY_F(4):
      backend_->refresh_all();
      return true;
    case KEY_F(3):
      if (backend_->current_view_ == ParameterCommanderViewMode::ParameterList) {
        backend_->refresh_selected();
        return true;
      }
      break;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->set_status("Search.");
        return true;
      }
      if (backend_->current_view_ == ParameterCommanderViewMode::ParameterList) {
        close_popup();
        backend_->switch_to_node_list();
        return true;
      }
      break;
    case '\n':
    case KEY_ENTER:
      if (backend_->current_view_ == ParameterCommanderViewMode::NodeList) {
        backend_->select_current_node();
        return true;
      }
      break;
    default:
      break;
  }

  if (backend_->current_view_ == ParameterCommanderViewMode::NodeList) {
    handle_node_list_key(key);
  } else {
    handle_list_key(key);
  }
  return true;
}

bool ParameterCommanderScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    backend_->set_status("Search cancelled.");
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    backend_->set_status(search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query);
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  if (backend_->current_view_ == ParameterCommanderViewMode::NodeList) {
    const int match = find_best_match(backend_->node_entries_, search_state_.query, backend_->selected_node_index_);
    if (match >= 0) {
      backend_->selected_node_index_ = match;
    }
    backend_->set_status("Search: " + search_state_.query);
    return true;
  }

  const auto items = backend_->visible_parameter_items();
  std::vector<std::string> labels;
  labels.reserve(items.size());
  for (const auto & item : items) {
    labels.push_back(item.label);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_parameter_item_index_);
  if (match >= 0) {
    backend_->selected_parameter_item_index_ = match;
    backend_->sync_edit_buffer_from_selected();
  }
  backend_->set_status("Search: " + search_state_.query);
  return true;
}

bool ParameterCommanderScreen::handle_popup_key(int key) {
  auto * entry = backend_->selected_entry();
  if (entry == nullptr) {
    popup_open_ = false;
    return true;
  }

  switch (key) {
    case KEY_F(10):
      close_popup();
      curs_set(1);
      return false;
    case 27:
      close_popup();
      return true;
    case KEY_F(3):
      backend_->refresh_selected();
      popup_buffer_ = entry->edit_buffer;
      popup_dirty_ = false;
      return true;
    case KEY_F(2):
      entry->edit_buffer = popup_buffer_;
      entry->dirty = popup_dirty_;
      backend_->save_selected();
      popup_buffer_ = entry->edit_buffer;
      popup_dirty_ = entry->dirty;
      return true;
    case '\n':
    case KEY_ENTER:
      entry->edit_buffer = popup_buffer_;
      entry->dirty = popup_dirty_;
      backend_->save_selected();
      if (!entry->dirty) {
        close_popup();
      } else {
        popup_buffer_ = entry->edit_buffer;
        popup_dirty_ = entry->dirty;
      }
      return true;
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      if (!popup_buffer_.empty() && backend_->popup_is_editable(*entry)) {
        popup_buffer_.pop_back();
        popup_dirty_ = true;
      }
      return true;
    case KEY_DC:
      if (backend_->popup_is_editable(*entry)) {
        popup_buffer_.clear();
        popup_dirty_ = true;
      }
      return true;
    case ' ':
      if (!backend_->popup_is_editable(*entry)) {
        return true;
      }
      if (entry->descriptor.type == ParameterType::PARAMETER_BOOL) {
        backend_->toggle_bool_buffer(popup_buffer_);
      } else {
        popup_buffer_.push_back(' ');
      }
      popup_dirty_ = true;
      return true;
    default:
      if (key >= 32 && key <= 126 && backend_->popup_is_editable(*entry)) {
        popup_buffer_.push_back(static_cast<char>(key));
        popup_dirty_ = true;
      }
      return true;
  }
}

void ParameterCommanderScreen::handle_list_key(int key) {
  const auto items = backend_->visible_parameter_items();
  if (items.empty()) {
    return;
  }

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_parameter_item_index_ > 0) {
        --backend_->selected_parameter_item_index_;
      }
      backend_->sync_edit_buffer_from_selected();
      break;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_parameter_item_index_ + 1 < static_cast<int>(items.size())) {
        ++backend_->selected_parameter_item_index_;
      }
      backend_->sync_edit_buffer_from_selected();
      break;
    case KEY_PPAGE:
      backend_->selected_parameter_item_index_ =
        std::max(0, backend_->selected_parameter_item_index_ - page_step());
      backend_->sync_edit_buffer_from_selected();
      break;
    case KEY_NPAGE:
      backend_->selected_parameter_item_index_ = std::min(
        static_cast<int>(items.size()) - 1,
        backend_->selected_parameter_item_index_ + page_step());
      backend_->sync_edit_buffer_from_selected();
      break;
    case KEY_RIGHT:
    case 'l':
      backend_->expand_selected_namespace();
      break;
    case KEY_LEFT:
    case 'h':
      backend_->collapse_selected_namespace();
      break;
    case '\n':
    case KEY_ENTER:
      backend_->activate_selected_parameter_item();
      if (backend_->selected_entry() != nullptr) {
        open_popup();
      }
      break;
    default:
      break;
  }
}

void ParameterCommanderScreen::handle_node_list_key(int key) {
  if (backend_->node_entries_.empty()) {
    return;
  }

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_node_index_ > 0) {
        --backend_->selected_node_index_;
      }
      break;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_node_index_ + 1 < static_cast<int>(backend_->node_entries_.size())) {
        ++backend_->selected_node_index_;
      }
      break;
    case KEY_PPAGE:
      backend_->selected_node_index_ = std::max(0, backend_->selected_node_index_ - page_step());
      break;
    case KEY_NPAGE:
      backend_->selected_node_index_ = std::min(
        static_cast<int>(backend_->node_entries_.size()) - 1,
        backend_->selected_node_index_ + page_step());
      break;
    default:
      break;
  }
}

int ParameterCommanderScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  return std::max(5, rows - 8);
}

void ParameterCommanderScreen::open_popup() {
  auto * entry = backend_->selected_entry();
  if (entry == nullptr) {
    backend_->set_status("No parameter selected.");
    return;
  }
  popup_open_ = true;
  popup_buffer_ = entry->edit_buffer;
  popup_dirty_ = entry->dirty;
  curs_set(0);
}

void ParameterCommanderScreen::close_popup() {
  popup_open_ = false;
  popup_buffer_.clear();
  popup_dirty_ = false;
  curs_set(0);
}

void ParameterCommanderScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);
  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  mvprintw(0, 1, "Parameter Commander ");
  if (backend_->current_view_ == ParameterCommanderViewMode::NodeList) {
    draw_node_list(1, 1, content_bottom - 1, columns - 2);
  } else {
    draw_parameter_list(1, 1, content_bottom - 1, columns - 2);
  }
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  if (popup_open_) {
    draw_popup(rows, columns);
  }
  refresh();
}

void ParameterCommanderScreen::draw_parameter_list(int top, int left, int bottom, int right) {
  const auto items = backend_->visible_parameter_items();
  const int visible_rows = std::max(1, bottom - top + 1);
  const int width = right - left + 1;
  const int name_width = std::max(22, width / 3);
  const int value_width = std::max(12, width / 6);
  const int desc_width = std::max(10, width - name_width - value_width - 2);
  const int separator_one_x = left + name_width;
  const int separator_two_x = left + name_width + 1 + value_width;
  if (backend_->selected_parameter_item_index_ < backend_->list_scroll_) {
    backend_->list_scroll_ = backend_->selected_parameter_item_index_;
  }
  if (backend_->selected_parameter_item_index_ >= backend_->list_scroll_ + visible_rows) {
    backend_->list_scroll_ = backend_->selected_parameter_item_index_ - visible_rows + 1;
  }

  const std::string header = pad_column("Name", name_width) + " "
    + pad_column("Current", value_width) + " "
    + pad_column("Descriptor", desc_width);
  attron(COLOR_PAIR(kColorHeader) | A_BOLD);
  mvaddnstr(top, left, header.c_str(), width);
  attroff(COLOR_PAIR(kColorHeader) | A_BOLD);

  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(top, separator_one_x, visible_rows);
  draw_text_vline(top, separator_two_x, visible_rows);
  attroff(COLOR_PAIR(kColorFrame));

  for (int row = 1; row < visible_rows; ++row) {
    const int entry_index = backend_->list_scroll_ + row - 1;
    const int row_y = top + row;
    const bool is_selected = entry_index == backend_->selected_parameter_item_index_;

    if (is_selected) {
      attron(COLOR_PAIR(kColorSelection) | A_BOLD);
    }
    mvhline(row_y, left, ' ', width);
    if (is_selected) {
      attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
    }
    if (entry_index >= static_cast<int>(items.size())) {
      if (is_selected) {
        attron(COLOR_PAIR(kColorSelection) | A_BOLD);
      } else {
        attron(COLOR_PAIR(kColorFrame));
      }
      draw_box_char(row_y, separator_one_x, WACS_VLINE, '|');
      draw_box_char(row_y, separator_two_x, WACS_VLINE, '|');
      if (is_selected) {
        attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
      } else {
        attroff(COLOR_PAIR(kColorFrame));
      }
      continue;
    }
    const auto & item = items[static_cast<std::size_t>(entry_index)];

    if (is_selected) {
      attron(COLOR_PAIR(kColorSelection) | A_BOLD);
    } else if (item.is_namespace) {
      attron(COLOR_PAIR(kColorFrame) | A_BOLD);
    }
    draw_parameter_name_cell(row_y, left, name_width, item);
    if (is_selected) {
      attron(COLOR_PAIR(kColorSelection) | A_BOLD);
    } else {
      attron(COLOR_PAIR(kColorFrame));
    }
    draw_box_char(row_y, separator_one_x, WACS_VLINE, '|');
    draw_box_char(row_y, separator_two_x, WACS_VLINE, '|');
    if (is_selected) {
      attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
    } else {
      attroff(COLOR_PAIR(kColorFrame));
    }
    mvaddnstr(
      row_y,
      left + name_width + 1,
      pad_column(item.is_namespace ? "" : backend_->summary_value(*item.entry), value_width).c_str(),
      value_width);
    mvaddnstr(
      row_y,
      left + name_width + 1 + value_width + 1,
      pad_column(item.is_namespace ? "" : backend_->descriptor_summary(item.entry->descriptor), desc_width).c_str(),
      desc_width);
    if (is_selected) {
      mvchgat(row_y, left, width, A_BOLD, kColorSelection, nullptr);
      attron(COLOR_PAIR(kColorSelection) | A_BOLD);
      draw_box_char(row_y, separator_one_x, WACS_VLINE, '|');
      draw_box_char(row_y, separator_two_x, WACS_VLINE, '|');
      attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
    } else if (item.is_namespace) {
      attroff(COLOR_PAIR(kColorFrame) | A_BOLD);
    }
  }
}

void ParameterCommanderScreen::draw_parameter_name_cell(
  int row, int left, int width, const ParameterViewItem & item) const
{
  mvhline(row, left, ' ', width);
  const int indent = item.depth * 2;
  if (item.is_namespace) {
    if (indent < width) {
      mvaddnstr(
        row,
        left + indent,
        truncate_parameter_line(item.label, width - indent).c_str(),
        width - indent);
    }
    return;
  }

  const std::string name = item.entry->dirty ? "*" + item.label : item.label;
  const std::string rendered = std::string(static_cast<std::size_t>(indent), ' ') + "  " + name;
  mvaddnstr(row, left, pad_column(rendered, width).c_str(), width);
}

void ParameterCommanderScreen::draw_node_list(int top, int left, int bottom, int right) {
  const int visible_rows = std::max(1, bottom - top + 1);
  const int width = right - left + 1;
  if (backend_->selected_node_index_ < backend_->node_scroll_) {
    backend_->node_scroll_ = backend_->selected_node_index_;
  }
  if (backend_->selected_node_index_ >= backend_->node_scroll_ + visible_rows - 1) {
    backend_->node_scroll_ = backend_->selected_node_index_ - visible_rows + 2;
  }

  attron(COLOR_PAIR(kColorHeader) | A_BOLD);
  mvaddnstr(top, left, pad_column("Discovered Nodes", width).c_str(), width);
  attroff(COLOR_PAIR(kColorHeader) | A_BOLD);

  for (int row = 1; row < visible_rows; ++row) {
    const int entry_index = backend_->node_scroll_ + row - 1;
    mvhline(top + row, left, ' ', width);
    if (entry_index >= static_cast<int>(backend_->node_entries_.size())) {
      continue;
    }
    const std::string rendered = pad_column(backend_->node_entries_[static_cast<std::size_t>(entry_index)], width);
    if (entry_index == backend_->selected_node_index_) {
      attron(COLOR_PAIR(kColorSelection) | A_BOLD);
    }
    mvaddnstr(top + row, left, rendered.c_str(), width);
    if (entry_index == backend_->selected_node_index_) {
      attroff(COLOR_PAIR(kColorSelection) | A_BOLD);
    }
  }
}

void ParameterCommanderScreen::draw_popup(int rows, int columns) {
  const ParameterEntry * entry = backend_->selected_entry();
  if (entry == nullptr) {
    return;
  }

  const int popup_width = std::min(columns - 4, 80);
  const int popup_height = std::min(rows - 4, 13);
  const int top = std::max(1, (rows - popup_height) / 2);
  const int left = std::max(2, (columns - popup_width) / 2);
  const int bottom = top + popup_height - 1;
  const int right = left + popup_width - 1;
  const int inner_width = popup_width - 2;
  const int field_top = top + 7;
  const int field_left = left + 2;
  const int field_right = right - 2;
  const int field_inner_width = std::max(1, field_right - field_left - 1);
  const int edit_text_width = std::max(1, field_inner_width - 1);

  for (int row = top + 1; row < bottom; ++row) {
    attron(COLOR_PAIR(kColorPopup));
    mvhline(row, left + 1, ' ', inner_width);
    attroff(COLOR_PAIR(kColorPopup));
  }

  if (popup_dirty_) {
    attron(COLOR_PAIR(kColorDirty));
    draw_box_char(top, left, WACS_ULCORNER, '+');
    draw_box_char(top, right, WACS_URCORNER, '+');
    draw_box_char(bottom, left, WACS_LLCORNER, '+');
    draw_box_char(bottom, right, WACS_LRCORNER, '+');
    draw_text_hline(top, left + 1, right - left - 1);
    draw_text_hline(bottom, left + 1, right - left - 1);
    draw_text_vline(top + 1, left, bottom - top - 1);
    draw_text_vline(top + 1, right, bottom - top - 1);
    attroff(COLOR_PAIR(kColorDirty));
  } else {
    draw_box(top, left, bottom, right, kColorFrame);
  }
  attron(COLOR_PAIR(kColorTitle) | A_BOLD);
  mvprintw(top, left + 2, " Edit Parameter ");
  attroff(COLOR_PAIR(kColorTitle) | A_BOLD);

  constexpr int label_width = 11;
  auto draw_popup_field = [&](int row, const std::string & label, const std::string & value) {
    const std::string label_text = pad_column(label, label_width) + ": ";
    mvaddnstr(row, left + 1, label_text.c_str(), inner_width);
    mvaddnstr(
      row,
      left + 1 + static_cast<int>(label_text.size()),
      truncate_parameter_line(value, inner_width - static_cast<int>(label_text.size())).c_str(),
      inner_width - static_cast<int>(label_text.size()));
  };

  attron(COLOR_PAIR(kColorPopup));
  draw_popup_field(top + 1, "name", "");
  attron(COLOR_PAIR(kColorHeader) | A_BOLD);
  mvaddnstr(
    top + 1,
    left + 1 + label_width + 2,
    truncate_parameter_line(entry->name, inner_width - label_width - 2).c_str(),
    inner_width - label_width - 2);
  attroff(COLOR_PAIR(kColorHeader) | A_BOLD);
  draw_popup_field(top + 2, "descriptor", backend_->descriptor_summary(entry->descriptor));
  draw_popup_field(
    top + 3,
    "type",
    parameter_type_name(entry->descriptor.type)
      + "  [" + backend_->parameter_min(*entry) + "]"
      + "  [" + backend_->parameter_max(*entry) + "]");
  draw_popup_field(top + 4, "current", backend_->summary_value(*entry));
  draw_text_hline(top + 5, left + 1, inner_width);
  attroff(COLOR_PAIR(kColorPopup));
  draw_box(field_top, field_left, field_top + 2, field_right, kColorFrame);
  attron(COLOR_PAIR(kColorHeader) | A_BOLD);
  mvhline(field_top + 1, field_left + 1, ' ', field_inner_width);
  const std::string visible_buffer = tail_fit(popup_buffer_, edit_text_width);
  mvaddnstr(field_top + 1, field_left + 1, visible_buffer.c_str(), edit_text_width);
  attroff(COLOR_PAIR(kColorHeader) | A_BOLD);
  if (backend_->popup_is_editable(*entry) && software_caret_visible()) {
    const int visible_buffer_width = std::min(edit_text_width, static_cast<int>(visible_buffer.size()));
    const int cursor_x = std::min(field_left + field_inner_width - 1, field_left + 1 + visible_buffer_width);
    attron(COLOR_PAIR(kColorCursor) | A_BOLD);
    mvaddch(field_top + 1, cursor_x, ' ');
    attroff(COLOR_PAIR(kColorCursor) | A_BOLD);
  }

  attron(COLOR_PAIR(kColorPopup));
  draw_text_hline(bottom - 2, left + 1, inner_width);
  attroff(COLOR_PAIR(kColorPopup));
  draw_help_bar_region(bottom - 1, left + 1, inner_width, "F3 Load  F2 Save  Esc Close  F10 Exit");
}

std::string ParameterCommanderScreen::pad_column(const std::string & value, int width) const {
  const std::string truncated = truncate_parameter_line(value, width);
  if (visible_width(truncated) >= width) {
    return truncated;
  }
  return truncated + std::string(static_cast<std::size_t>(width - visible_width(truncated)), ' ');
}

void ParameterCommanderScreen::draw_status_line(int row, int columns) const {
  draw_status_bar(row, columns, truncate_parameter_line(backend_->status_message_, columns - 2));
}

void ParameterCommanderScreen::draw_help_line(int row, int columns) const {
  std::string help;
  if (popup_open_) {
    help = "F2 Save  F3 Reload  Enter Save+Close  Esc Close  F10 Exit";
  } else if (backend_->current_view_ == ParameterCommanderViewMode::NodeList) {
    help = "Enter Open  Alt+S Search  F4 Refresh  F10 Exit";
  } else {
    help = "Enter Edit  Alt+S Search  F3 Refresh  F4 Refresh All  Esc Back  F10 Exit";
  }
  draw_help_bar(row, columns, truncate_parameter_line(help, columns));
}

}  // namespace ros2_console_tools
