#include "ros2_console_tools/journal_viewer.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace ros2_console_tools {

int run_journal_viewer_tool(const std::string & initial_unit, bool embedded_mode) {
  auto backend = std::make_shared<JournalViewerBackend>(initial_unit);
  JournalViewerScreen screen(backend, embedded_mode);
  return screen.run();
}

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorAccent = tui::kColorAccent,
  kColorWarn = tui::kColorWarn,
  kColorError = tui::kColorError,
  kColorFatal = tui::kColorFatal,
  kColorPopup = tui::kColorPopup,
  kColorInput = tui::kColorInput,
};

using tui::Session;
using tui::apply_role_chgat;
using tui::draw_box;
using tui::draw_help_bar;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;
using tui::truncate_text;

int journal_row_color(int priority, bool selected) {
  if (selected) {
    return kColorSelection;
  }
  if (priority <= 2) {
    return kColorFatal;
  }
  if (priority == 3) {
    return kColorError;
  }
  if (priority == 4) {
    return kColorWarn;
  }
  if (priority == 5) {
    return kColorAccent;
  }
  return 0;
}

std::string short_timestamp(const std::string & timestamp) {
  if (timestamp.size() >= 19) {
    return timestamp.substr(11);
  }
  return timestamp;
}

}  // namespace

JournalViewerScreen::JournalViewerScreen(
  std::shared_ptr<JournalViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int JournalViewerScreen::run() {
  std::unique_ptr<Session> ncurses_session;
  if (!embedded_mode_) {
    ncurses_session = std::make_unique<Session>();
  } else {
    curs_set(0);
    keypad(stdscr, TRUE);
    cbreak();
    noecho();
  }

  timeout(100);
  if (embedded_mode_) {
    flushinp();
    timeout(0);
    while (getch() != ERR) {
    }
    timeout(100);
  }

  bool running = true;
  while (running) {
    terminal_pane_.update();
    draw();
    const int key = getch();
    if (key == ERR) {
      if (!filter_prompt_open_ && !search_state_.active) {
        backend_->maybe_poll_live_updates();
      }
      continue;
    }
    running = handle_key(key);
  }

  if (embedded_mode_) {
    timeout(100);
    curs_set(0);
    clear();
    clearok(stdscr, TRUE);
    refresh();
  }

  return 0;
}

bool JournalViewerScreen::handle_key(int key) {
  if (filter_prompt_open_) {
    return handle_filter_prompt_key(key);
  }

  if (is_alt_binding(key, 't')) {
    search_state_.active = false;
    terminal_pane_.toggle();
    return true;
  }

  if (terminal_pane_.visible()) {
    if (key == KEY_F(10)) {
      return false;
    }
    return terminal_pane_.handle_key(key);
  }

  if (search_state_.active) {
    return handle_search_key(key);
  }

  if (detail_popup_open_) {
    return handle_detail_popup_key(key);
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      backend_->toggle_live_mode();
      return true;
    case KEY_F(4):
      detail_scroll_ = 0;
      backend_->refresh_entries();
      return true;
    case KEY_F(5):
      detail_scroll_ = 0;
      backend_->cycle_priority_filter();
      return true;
    case KEY_F(6):
      {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        filter_buffer_ = backend_->text_filter_;
      }
      filter_prompt_open_ = true;
      return true;
    case '\n':
    case KEY_ENTER:
      {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        detail_popup_open_ = !backend_->entries_.empty();
      }
      detail_scroll_ = 0;
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->status_line_ = "Search journal entries.";
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      return true;
    default:
      break;
  }

  return handle_entry_list_key(key);
}

bool JournalViewerScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ =
      search_state_.query.empty() ? "Search closed." : ("Search: " + search_state_.query);
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  std::vector<JournalEntry> entries;
  int current_index = 0;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    entries = backend_->entries_;
    current_index = backend_->selected_index_;
  }

  std::vector<std::string> labels;
  labels.reserve(entries.size());
  for (const auto & entry : entries) {
    labels.push_back(
      entry.timestamp + " " + journal_priority_label(entry.priority) + " " +
      entry.unit + " " + entry.identifier + " " + entry.message);
  }

  const int match = find_best_match(labels, search_state_.query, current_index);
  if (match >= 0) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->selected_index_ = match;
    backend_->clamp_selection();
    detail_scroll_ = 0;
  }

  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

bool JournalViewerScreen::handle_filter_prompt_key(int key) {
  switch (key) {
    case 27:
      filter_prompt_open_ = false;
      return true;
    case '\n':
    case KEY_ENTER:
      filter_prompt_open_ = false;
      detail_scroll_ = 0;
      backend_->set_text_filter(filter_buffer_);
      return true;
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      if (!filter_buffer_.empty()) {
        filter_buffer_.pop_back();
      }
      return true;
    default:
      if (key >= 32 && key <= 126) {
        filter_buffer_.push_back(static_cast<char>(key));
      }
      return true;
  }
}

bool JournalViewerScreen::handle_entry_list_key(int key) {
  std::vector<JournalEntry> entries;
  int selected_index = 0;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    entries = backend_->entries_;
    selected_index = backend_->selected_index_;
  }

  switch (key) {
    case KEY_UP:
    case 'k':
      if (selected_index > 0) {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        --backend_->selected_index_;
        backend_->clamp_selection();
        detail_scroll_ = 0;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (selected_index + 1 < static_cast<int>(entries.size())) {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        ++backend_->selected_index_;
        backend_->clamp_selection();
        detail_scroll_ = 0;
      }
      return true;
    case KEY_PPAGE:
      {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
        backend_->clamp_selection();
        detail_scroll_ = 0;
      }
      return true;
    case KEY_NPAGE:
      if (!entries.empty()) {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->selected_index_ = std::min(
          static_cast<int>(entries.size()) - 1, backend_->selected_index_ + page_step());
        backend_->clamp_selection();
        detail_scroll_ = 0;
      }
      return true;
    case '\n':
    case KEY_ENTER:
      detail_popup_open_ = !entries.empty();
      detail_scroll_ = 0;
      return true;
    default:
      return true;
  }
}

bool JournalViewerScreen::handle_detail_popup_key(int key) {
  const auto rows = backend_->detail_rows_snapshot();
  int screen_rows = 0;
  int screen_columns = 0;
  getmaxyx(stdscr, screen_rows, screen_columns);
  const int box_height = std::min(screen_rows - 4, std::max(8, screen_rows * 3 / 4));
  const int visible_rows = std::max(1, box_height - 3);
  const int max_scroll = std::max(0, static_cast<int>(rows.size()) - visible_rows);

  switch (key) {
    case 27:
      detail_popup_open_ = false;
      return true;
    case KEY_UP:
    case 'k':
      detail_scroll_ = std::max(0, detail_scroll_ - 1);
      return true;
    case KEY_DOWN:
    case 'j':
      detail_scroll_ = std::min(max_scroll, detail_scroll_ + 1);
      return true;
    case KEY_PPAGE:
      detail_scroll_ = std::max(0, detail_scroll_ - page_step());
      return true;
    case KEY_NPAGE:
      detail_scroll_ = std::min(max_scroll, detail_scroll_ + page_step());
      return true;
    case '\n':
    case KEY_ENTER:
      detail_popup_open_ = false;
      return true;
    default:
      return true;
  }
}

int JournalViewerScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void JournalViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const auto layout = tui::make_commander_layout(rows, terminal_pane_.visible());
  const int help_row = layout.help_row;
  const int status_row = layout.status_row;
  const int content_bottom = layout.content_bottom;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Journal Viewer ");
  attroff(theme_attr(kColorTitle));

  draw_entry_list(1, 1, content_bottom - 1, columns - 2);
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(layout.pane_rows, columns, search_state_);
  if (filter_prompt_open_) {
    draw_filter_popup(layout.pane_rows, columns);
  }
  if (detail_popup_open_) {
    draw_detail_popup(layout.pane_rows, columns);
  }
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
  }
  refresh();
}

void JournalViewerScreen::draw_entry_list(int top, int left, int bottom, int right) {
  std::vector<JournalEntry> entries;
  int selected_index = 0;
  int scroll = 0;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    entries = backend_->entries_;
    selected_index = backend_->selected_index_;
    scroll = backend_->entry_scroll_;
  }

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top);
  if (selected_index < scroll) {
    scroll = selected_index;
  }
  if (selected_index >= scroll + visible_rows) {
    scroll = std::max(0, selected_index - visible_rows + 1);
  }
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->entry_scroll_ = scroll;
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Entries");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = scroll;
  const int last_row = std::min(static_cast<int>(entries.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & entry = entries[static_cast<std::size_t>(index)];
    const bool selected = index == selected_index;
    const std::string source = !entry.unit.empty()
      ? entry.unit
      : (!entry.identifier.empty() ? entry.identifier : "-");
    const std::string text =
      short_timestamp(entry.timestamp) +
      " [" + journal_priority_label(entry.priority) + "] " +
      source + " " + entry.message;
    mvhline(row_y, left, ' ', width);
    mvaddnstr(row_y, left, truncate_text(text, width).c_str(), width);
    const int color = journal_row_color(entry.priority, selected);
    if (color != 0) {
      apply_role_chgat(row_y, left, width, color);
    }
  }

  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void JournalViewerScreen::draw_detail_popup(int total_rows, int total_columns) {
  const auto detail_rows = backend_->detail_rows_snapshot();
  if (total_rows < 8 || total_columns < 32) {
    return;
  }

  const int box_width = std::min(total_columns - 4, std::max(32, total_columns * 4 / 5));
  const int box_height = std::min(total_rows - 4, std::max(8, total_rows * 3 / 4));
  const int left = std::max(2, (total_columns - box_width) / 2);
  const int top = std::max(1, (total_rows - box_height) / 2);
  const int right = left + box_width - 1;
  const int bottom = top + box_height - 1;
  const int width = right - left + 1;
  const int visible_rows = std::max(1, box_height - 3);
  const int max_scroll = std::max(0, static_cast<int>(detail_rows.size()) - visible_rows);
  detail_scroll_ = std::clamp(detail_scroll_, 0, max_scroll);

  attron(theme_attr(kColorPopup));
  for (int row = top + 1; row < bottom; ++row) {
    mvhline(row, left + 1, ' ', box_width - 2);
  }
  attroff(theme_attr(kColorPopup));
  draw_box(top, left, bottom, right, kColorFrame);

  attron(theme_attr(kColorHeader));
  mvprintw(top, left + 2, " Entry Details ");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = detail_scroll_;
  const int last_row = std::min(static_cast<int>(detail_rows.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & row = detail_rows[static_cast<std::size_t>(index)];
    mvhline(row_y, left + 1, ' ', box_width - 2);
    if (row.is_header) {
      attron(theme_attr(kColorHeader));
      mvaddnstr(row_y, left + 1, truncate_text(row.text, width - 2).c_str(), width - 2);
      attroff(theme_attr(kColorHeader));
    } else {
      mvaddnstr(row_y, left + 1, truncate_text(row.text, width - 2).c_str(), width - 2);
    }
  }

  for (; row_y < bottom; ++row_y) {
    mvhline(row_y, left + 1, ' ', box_width - 2);
  }
}

void JournalViewerScreen::draw_filter_popup(int rows, int columns) const {
  if (rows < 6 || columns < 24) {
    return;
  }

  const int box_width = std::min(columns - 4, 52);
  const int left = std::max(2, (columns - box_width) / 2);
  const int top = std::max(1, rows - 5);
  const int bottom = top + 2;
  const int right = left + box_width - 1;
  const int inner_width = box_width - 2;

  attron(theme_attr(kColorPopup));
  mvhline(top + 1, left + 1, ' ', inner_width);
  attroff(theme_attr(kColorPopup));
  draw_box(top, left, bottom, right, kColorFrame);

  const std::string label = "Text Filter: ";
  attron(theme_attr(kColorHeader));
  mvaddnstr(top + 1, left + 1, label.c_str(), inner_width);
  attroff(theme_attr(kColorHeader));

  const int input_left = left + 1 + static_cast<int>(label.size());
  const int input_width = std::max(0, right - input_left);
  attron(theme_attr(kColorInput));
  mvhline(top + 1, input_left, ' ', input_width);
  mvaddnstr(top + 1, input_left, truncate_text(filter_buffer_, input_width).c_str(), input_width);
  attroff(theme_attr(kColorInput));
}

void JournalViewerScreen::draw_status_line(int row, int columns) const {
  std::string status_line;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    status_line = backend_->status_line_;
  }
  draw_status_bar(row, columns, status_line);
}

void JournalViewerScreen::draw_help_line(int row, int columns) const {
  const std::string priority_label = backend_->priority_filter_label();
  const bool live_mode = backend_->live_mode_enabled();
  draw_help_bar(
    row,
    columns,
    tui::with_terminal_help(
      "Up/Down Move  Enter Details  F2 Live:" + std::string(live_mode ? "On" : "Off") +
      "  F4 Refresh  F5 Priority:" + priority_label +
      "  F6 Text Filter  Alt+S Search  F10 Exit",
      terminal_pane_.visible()));
}

}  // namespace ros2_console_tools
