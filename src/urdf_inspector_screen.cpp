#include "ros2_console_tools/urdf_inspector.hpp"

#include <ncursesw/ncurses.h>

#include <cctype>
#include <thread>

namespace ros2_console_tools {

int run_urdf_inspector_tool(const std::string & target_node) {
  auto backend = std::make_shared<UrdfInspectorBackend>(target_node);
  UrdfInspectorScreen screen(backend);

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
  kColorLink = tui::kColorAccent,
};

using tui::Session;
using tui::draw_box;
using tui::draw_box_char;
using tui::draw_help_bar;
using tui::draw_help_bar_region;
using tui::draw_search_box;
using tui::draw_status_bar;
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

UrdfInspectorScreen::UrdfInspectorScreen(std::shared_ptr<UrdfInspectorBackend> backend)
: backend_(std::move(backend)) {}

int UrdfInspectorScreen::run() {
  Session ncurses_session;
  backend_->refresh_model();
  backend_->warm_up_model();

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->maybe_refresh_model();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool UrdfInspectorScreen::handle_key(int key) {
  if (popup_open_) {
    return handle_popup_key(key);
  }
  if (search_state_.active) {
    return handle_search_key(key);
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_model();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->status_line_ = "Search.";
        return true;
      }
      return true;
    case KEY_UP:
    case 'k':
      if (backend_->selected_index_ > 0) {
        --backend_->selected_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_index_ + 1 < static_cast<int>(backend_->rows_.size())) {
        ++backend_->selected_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->rows_.empty()) {
        backend_->selected_index_ = std::min(
          static_cast<int>(backend_->rows_.size()) - 1,
          backend_->selected_index_ + page_step());
      }
      return true;
    case KEY_RIGHT:
    case 'l':
      backend_->expand_selected();
      return true;
    case KEY_LEFT:
    case 'h':
      backend_->collapse_selected();
      return true;
    case '\n':
    case KEY_ENTER:
      if (const auto section = backend_->selected_xml_section()) {
        popup_open_ = true;
        popup_scroll_ = 0;
        popup_lines_ = split_lines(*section);
        if (!backend_->rows_.empty()) {
          const auto & row = backend_->rows_[static_cast<std::size_t>(backend_->selected_index_)];
          popup_title_ = row.is_link ? ("Link: " + row.link_name) : ("Joint: " + row.joint_name);
        } else {
          popup_title_ = "URDF Section";
        }
      } else {
        backend_->status_line_ = "No XML section available for the selected item.";
      }
      return true;
    default:
      return true;
  }
}

bool UrdfInspectorScreen::handle_popup_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
    case '\n':
    case KEY_ENTER:
      popup_open_ = false;
      popup_scroll_ = 0;
      popup_lines_.clear();
      popup_title_.clear();
      return true;
    case KEY_UP:
    case 'k':
      if (popup_scroll_ > 0) {
        --popup_scroll_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (popup_scroll_ + 1 < static_cast<int>(popup_lines_.size())) {
        ++popup_scroll_;
      }
      return true;
    case KEY_PPAGE:
      popup_scroll_ = std::max(0, popup_scroll_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!popup_lines_.empty()) {
        popup_scroll_ = std::min(static_cast<int>(popup_lines_.size()) - 1, popup_scroll_ + page_step());
      }
      return true;
    default:
      return true;
  }
}

bool UrdfInspectorScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    backend_->status_line_ = search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query;
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  std::vector<std::string> labels;
  labels.reserve(backend_->rows_.size());
  for (const auto & row : backend_->rows_) {
    labels.push_back(row.label);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_index_);
  if (match >= 0) {
    backend_->selected_index_ = match;
  }
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

int UrdfInspectorScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

std::vector<std::string> UrdfInspectorScreen::pretty_print_xml_lines(const std::string & text) {
  std::vector<std::string> lines;
  std::size_t index = 0;
  int indent = 0;

  auto push_line = [&lines](int line_indent, const std::string & content) {
    if (content.empty()) {
      return;
    }
    lines.push_back(std::string(static_cast<std::size_t>(std::max(0, line_indent) * 2), ' ') + content);
  };

  while (index < text.size()) {
    const std::size_t tag_start = text.find('<', index);
    if (tag_start == std::string::npos) {
      const auto first = text.find_first_not_of(" \t\r\n", index);
      if (first != std::string::npos) {
        const auto last = text.find_last_not_of(" \t\r\n");
        push_line(indent, text.substr(first, last - first + 1));
      }
      break;
    }

    const std::string between = text.substr(index, tag_start - index);
    const auto text_first = between.find_first_not_of(" \t\r\n");
    if (text_first != std::string::npos) {
      const auto text_last = between.find_last_not_of(" \t\r\n");
      push_line(indent, between.substr(text_first, text_last - text_first + 1));
    }

    const std::size_t tag_end = text.find('>', tag_start);
    if (tag_end == std::string::npos) {
      push_line(indent, text.substr(tag_start));
      break;
    }

    const std::string tag = text.substr(tag_start, tag_end - tag_start + 1);
    const bool closing = tag.size() >= 2 && tag[1] == '/';
    const bool declaration = tag.size() >= 2 && tag[1] == '?';
    const bool comment = tag.size() >= 4 && tag.compare(0, 4, "<!--") == 0;
    const bool self_closing =
      (!closing && !declaration && !comment && tag.size() >= 2 && tag[tag.size() - 2] == '/');

    if (closing) {
      indent = std::max(0, indent - 1);
      push_line(indent, tag);
    } else {
      push_line(indent, tag);
      if (!self_closing && !declaration && !comment) {
        ++indent;
      }
    }

    index = tag_end + 1;
  }

  if (lines.empty()) {
    lines.push_back(text);
  }
  return lines;
}

std::vector<std::string> UrdfInspectorScreen::split_lines(const std::string & text) {
  std::vector<std::string> lines;
  if (text.find('\n') == std::string::npos) {
    lines = pretty_print_xml_lines(text);
  } else {
    std::stringstream stream(text);
  std::string line;
    while (std::getline(stream, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      std::string normalized;
      normalized.reserve(line.size());
      int column = 0;
      for (char ch : line) {
        if (ch == '\t') {
          const int spaces = 2 - (column % 2);
          normalized.append(static_cast<std::size_t>(spaces), ' ');
          column += spaces;
        } else {
          normalized.push_back(ch);
          ++column;
        }
      }
      lines.push_back(std::move(normalized));
    }
  }
  if (lines.empty()) {
    lines.push_back(text);
  }

  std::size_t common_indent = std::string::npos;
  for (std::size_t index = 1; index < lines.size(); ++index) {
    const auto & current = lines[index];
    const std::size_t first_non_space = current.find_first_not_of(' ');
    if (first_non_space == std::string::npos) {
      continue;
    }
    if (common_indent == std::string::npos) {
      common_indent = first_non_space;
    } else {
      common_indent = std::min(common_indent, first_non_space);
    }
  }

  if (common_indent != std::string::npos && common_indent > 0) {
    for (std::size_t index = 1; index < lines.size(); ++index) {
      auto & current = lines[index];
      if (current.size() >= common_indent) {
        current.erase(0, common_indent);
      }
    }
  }

  return lines;
}

std::vector<XmlSpan> UrdfInspectorScreen::highlight_xml_line(const std::string & line) const {
  std::vector<XmlSpan> spans;
  auto push_span = [&](const std::string & text, int color) {
    if (text.empty()) {
      return;
    }
    if (!spans.empty() && spans.back().color == color) {
      spans.back().text += text;
    } else {
      spans.push_back({text, color});
    }
  };

  std::size_t index = 0;
  while (index < line.size()) {
    if (line[index] == '<') {
      std::size_t end = line.find('>', index);
      if (end == std::string::npos) {
        push_span(line.substr(index), kColorHeader);
        break;
      }
      const std::string tag = line.substr(index, end - index + 1);
      std::size_t inner = 0;
      while (inner < tag.size()) {
        if (tag[inner] == '<' || tag[inner] == '>' || tag[inner] == '/' || tag[inner] == '?') {
          push_span(std::string(1, tag[inner]), kColorHeader);
          ++inner;
          continue;
        }
        if (std::isspace(static_cast<unsigned char>(tag[inner]))) {
          push_span(std::string(1, tag[inner]), 0);
          ++inner;
          continue;
        }
        std::size_t token_end = inner;
        while (token_end < tag.size() &&
          !std::isspace(static_cast<unsigned char>(tag[token_end])) &&
          tag[token_end] != '=' && tag[token_end] != '>' && tag[token_end] != '/')
        {
          ++token_end;
        }
        const std::string token = tag.substr(inner, token_end - inner);
        if (token_end < tag.size() && tag[token_end] == '=') {
          push_span(token, kColorTitle);
        } else {
          push_span(token, kColorHeader);
        }
        inner = token_end;
        if (inner < tag.size() && tag[inner] == '=') {
          push_span("=", kColorHeader);
          ++inner;
          if (inner < tag.size() && (tag[inner] == '"' || tag[inner] == '\'')) {
            const char quote = tag[inner];
            std::size_t value_end = inner + 1;
            while (value_end < tag.size() && tag[value_end] != quote) {
              ++value_end;
            }
            if (value_end < tag.size()) {
              ++value_end;
            }
            push_span(tag.substr(inner, value_end - inner), tui::kColorPositive);
            inner = value_end;
          }
        }
      }
      index = end + 1;
      continue;
    }

    std::size_t next_tag = line.find('<', index);
    push_span(line.substr(index, next_tag == std::string::npos ? std::string::npos : next_tag - index), 0);
    if (next_tag == std::string::npos) {
      break;
    }
    index = next_tag;
  }

  return spans;
}

void UrdfInspectorScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "URDF Inspector ");
  attroff(theme_attr(kColorTitle));

  const int left_width = std::max(28, (columns - 2) / 2);
  const int separator_x = 1 + left_width;
  draw_tree_pane(1, 1, content_bottom - 1, separator_x - 1);
  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(1, separator_x, content_bottom - 1);
  attroff(COLOR_PAIR(kColorFrame));
  draw_details_pane(1, separator_x + 1, content_bottom - 1, columns - 2);
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  if (popup_open_) {
    draw_xml_popup(rows, columns);
  }
  refresh();
}

void UrdfInspectorScreen::draw_tree_pane(int top, int left, int bottom, int right) {
  backend_->clamp_selection();

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  if (backend_->selected_index_ < backend_->tree_scroll_) {
    backend_->tree_scroll_ = backend_->selected_index_;
  }
  if (backend_->selected_index_ >= backend_->tree_scroll_ + visible_rows - 1) {
    backend_->tree_scroll_ = std::max(0, backend_->selected_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Tree");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->tree_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->rows_.size()), first_row + visible_rows - 1);
  for (int row_y = top + 1; row_y <= bottom; ++row_y) {
    const bool has_item = first_row + (row_y - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row_y - top - 1) == backend_->selected_index_);
    mvhline(row_y, left, ' ', width);
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
    }
    if (!has_item) {
      continue;
    }

    const auto & row = backend_->rows_[static_cast<std::size_t>(first_row + (row_y - top - 1))];
    const std::string text =
      std::string(static_cast<std::size_t>(row.depth * 2), ' ') + row.label;
    const int color = selected ? kColorSelection : (row.is_link ? kColorLink : 0);
    if (color != 0) {
      attron(COLOR_PAIR(color));
    }
    mvprintw(row_y, left, "%-*s", width, truncate_text(text, width).c_str());
    if (color != 0) {
      attroff(COLOR_PAIR(color));
    }
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
      apply_role_chgat(row_y, left, std::min(width, static_cast<int>(text.size())), color);
    }
  }
}

void UrdfInspectorScreen::draw_details_pane(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  const auto lines = backend_->selected_details();

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Details");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  for (const auto & line : lines) {
    if (row_y > bottom) {
      break;
    }
    mvprintw(row_y, left, "%-*s", width, truncate_text(line, width).c_str());
    ++row_y;
  }
  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void UrdfInspectorScreen::draw_xml_popup(int rows, int columns) const {
  const int popup_width = std::min(columns - 6, 100);
  const int popup_height = std::min(rows - 6, 20);
  const int left = std::max(2, (columns - popup_width) / 2);
  const int top = std::max(2, (rows - popup_height) / 2);
  const int right = left + popup_width - 1;
  const int bottom = top + popup_height - 1;
  const int inner_width = popup_width - 2;
  const int visible_rows = std::max(1, popup_height - 3);

  for (int row = top + 1; row < bottom; ++row) {
    attron(COLOR_PAIR(tui::kColorPopup));
    mvhline(row, left + 1, ' ', inner_width);
    attroff(COLOR_PAIR(tui::kColorPopup));
  }
  draw_box(top, left, bottom, right, kColorFrame);

  attron(theme_attr(kColorHeader));
  mvprintw(top, left + 2, " %s ", truncate_text(popup_title_, popup_width - 6).c_str());
  attroff(theme_attr(kColorHeader));

  const int max_scroll = std::max(0, static_cast<int>(popup_lines_.size()) - visible_rows);
  const int scroll = std::clamp(popup_scroll_, 0, max_scroll);
  for (int row = 0; row < visible_rows; ++row) {
    const int screen_row = top + 1 + row;
    const int line_index = scroll + row;
    mvhline(screen_row, left + 1, ' ', inner_width);
    if (line_index >= static_cast<int>(popup_lines_.size())) {
      continue;
    }

    int column = left + 2;
    int remaining = popup_width - 4;
    for (const auto & span : highlight_xml_line(popup_lines_[static_cast<std::size_t>(line_index)])) {
      if (remaining <= 0) {
        break;
      }
      const std::string text = truncate_text(span.text, remaining);
      if (span.color != 0) {
        attron(COLOR_PAIR(span.color));
      }
      mvaddnstr(screen_row, column, text.c_str(), remaining);
      if (span.color != 0) {
        attroff(COLOR_PAIR(span.color));
      }
      column += static_cast<int>(text.size());
      remaining -= static_cast<int>(text.size());
    }
  }

  draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "Enter Close  Esc Close  Up Down Scroll  F10 Exit");
}

void UrdfInspectorScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;
  if (!backend_->source_node_.empty()) {
    line = "src=" + backend_->source_node_ + "  " + line;
  }
  draw_status_bar(row, columns, line);
}

void UrdfInspectorScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(row, columns, "Enter Inspect  Left Fold  Right Unfold  Alt+S Search  F4 Reload  F10 Exit");
}

}  // namespace ros2_console_tools
