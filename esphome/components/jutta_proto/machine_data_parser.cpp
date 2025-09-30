#include "machine_data_parser.h"

#include <algorithm>
#include <cctype>
#include <unordered_map>

namespace esphome {
namespace jutta_component {
namespace {

std::string encode_utf8(uint32_t codepoint) {
  std::string result;
  if (codepoint <= 0x7F) {
    result.push_back(static_cast<char>(codepoint));
  } else if (codepoint <= 0x7FF) {
    result.push_back(static_cast<char>(0xC0 | ((codepoint >> 6) & 0x1F)));
    result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
  } else if (codepoint <= 0xFFFF) {
    result.push_back(static_cast<char>(0xE0 | ((codepoint >> 12) & 0x0F)));
    result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
    result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
  } else if (codepoint <= 0x10FFFF) {
    result.push_back(static_cast<char>(0xF0 | ((codepoint >> 18) & 0x07)));
    result.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
    result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
    result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
  }
  return result;
}

std::optional<uint32_t> parse_numeric_entity(const std::string &entity) {
  if (entity.size() < 2 || entity[0] != '#') {
    return std::nullopt;
  }

  int base = 10;
  size_t index = 1;
  if (entity.size() >= 3 && (entity[1] == 'x' || entity[1] == 'X')) {
    base = 16;
    index = 2;
  }

  if (index >= entity.size()) {
    return std::nullopt;
  }

  uint32_t value = 0;
  for (; index < entity.size(); ++index) {
    unsigned char c = static_cast<unsigned char>(entity[index]);
    int digit = -1;
    if (std::isdigit(c)) {
      digit = c - '0';
    } else if (base == 16 && c >= 'a' && c <= 'f') {
      digit = 10 + (c - 'a');
    } else if (base == 16 && c >= 'A' && c <= 'F') {
      digit = 10 + (c - 'A');
    } else {
      return std::nullopt;
    }

    value = value * base + static_cast<uint32_t>(digit);
    if (value > 0x10FFFF) {
      return std::nullopt;
    }
  }

  return value;
}

std::string decode_xml_entities(const std::string &value) {
  static const std::unordered_map<std::string, std::string> named_entities = {
      {"amp", "&"}, {"lt", "<"}, {"gt", ">"}, {"apos", "'"}, {"quot", "\""}};

  std::string result;
  result.reserve(value.size());

  for (size_t i = 0; i < value.size(); ++i) {
    char c = value[i];
    if (c != '&') {
      result.push_back(c);
      continue;
    }

    size_t end = value.find(';', i + 1);
    if (end == std::string::npos) {
      result.push_back('&');
      continue;
    }

    std::string entity = value.substr(i + 1, end - i - 1);
    if (entity.empty()) {
      result.push_back('&');
      i = end;
      continue;
    }

    if (entity[0] == '#') {
      auto numeric = parse_numeric_entity(entity);
      if (numeric.has_value()) {
        result.append(encode_utf8(numeric.value()));
        i = end;
        continue;
      }
    } else {
      auto it = named_entities.find(entity);
      if (it != named_entities.end()) {
        result.append(it->second);
        i = end;
        continue;
      }
    }

    // Unknown entity - keep original text.
    result.append(value.substr(i, end - i + 1));
    i = end;
  }

  return result;
}

class SimpleXmlParser {
 public:
  explicit SimpleXmlParser(const std::string &input) : input_(input) {}

  std::optional<MachineDataNode> parse() {
    position_ = 0;
    skip_whitespace();
    // Skip XML declaration if present.
    if (match("<?")) {
      if (!skip_until("?>")) {
        return std::nullopt;
      }
      skip_whitespace();
    }
    // Skip comments or directives before the root element.
    while (peek("<!--") || peek("<!")) {
      if (peek("<!--")) {
        if (!skip_comment()) {
          return std::nullopt;
        }
      } else if (peek("<!")) {
        if (!skip_until(">")) {
          return std::nullopt;
        }
      }
      skip_whitespace();
    }
    auto node = parse_element();
    if (!node.has_value()) {
      return std::nullopt;
    }
    return node;
  }

 private:
  const std::string &input_;
  size_t position_{0};

  bool at_end() const { return position_ >= input_.size(); }

  void skip_whitespace() {
    while (!at_end() && std::isspace(static_cast<unsigned char>(input_[position_]))) {
      ++position_;
    }
  }

  bool match(const std::string &token) {
    if (!peek(token)) {
      return false;
    }
    position_ += token.size();
    return true;
  }

  bool peek(const std::string &token) const {
    if (position_ + token.size() > input_.size()) {
      return false;
    }
    return input_.compare(position_, token.size(), token) == 0;
  }

  bool skip_until(const std::string &token) {
    auto pos = input_.find(token, position_);
    if (pos == std::string::npos) {
      position_ = input_.size();
      return false;
    }
    position_ = pos + token.size();
    return true;
  }

  bool skip_comment() {
    if (!match("<!--")) {
      return false;
    }
    return skip_until("-->");
  }

  std::optional<MachineDataNode> parse_element() {
    skip_whitespace();
    if (!match("<")) {
      return std::nullopt;
    }
    if (match("/")) {
      return std::nullopt;
    }
    if (peek("?")) {
      // Processing instruction within the document, skip it.
      if (!skip_until("?>")) {
        return std::nullopt;
      }
      skip_whitespace();
      return parse_element();
    }
    if (peek("!")) {
      // Comment or directive.
      if (peek("!--")) {
        if (!skip_comment()) {
          return std::nullopt;
        }
      } else {
        if (!skip_until(">")) {
          return std::nullopt;
        }
      }
      skip_whitespace();
      return parse_element();
    }

    MachineDataNode node;
    node.name = parse_name();
    if (node.name.empty()) {
      return std::nullopt;
    }
    skip_whitespace();

    while (!at_end() && input_[position_] != '>' && input_[position_] != '/') {
      std::string attr_name = parse_name();
      if (attr_name.empty()) {
        return std::nullopt;
      }
      skip_whitespace();
      if (!match("=")) {
        return std::nullopt;
      }
      skip_whitespace();
      std::string attr_value = parse_attribute_value();
      node.attributes.emplace_back(std::move(attr_name), std::move(attr_value));
      skip_whitespace();
    }

    if (match("/>")) {
      return node;
    }

    if (!match(">")) {
      return std::nullopt;
    }

    while (true) {
      skip_whitespace();
      if (peek("</")) {
        position_ += 2;  // Skip '</'
        std::string end_name = parse_name();
        skip_whitespace();
        if (!match(">")) {
          return std::nullopt;
        }
        // Name mismatch is ignored to keep parser permissive.
        (void)end_name;
        break;
      }
      if (peek("<!--")) {
        if (!skip_comment()) {
          return std::nullopt;
        }
        continue;
      }
      if (peek("<?")) {
        if (!skip_until("?>")) {
          return std::nullopt;
        }
        continue;
      }
      if (peek("<!")) {
        if (!skip_until(">")) {
          return std::nullopt;
        }
        continue;
      }
      if (peek("<")) {
        auto child = parse_element();
        if (!child.has_value()) {
          return std::nullopt;
        }
        node.children.emplace_back(std::move(child.value()));
      } else {
        std::string text = parse_text();
        if (!text.empty()) {
          MachineDataNode text_node;
          text_node.name = "#text";
          text_node.text = std::move(text);
          node.children.emplace_back(std::move(text_node));
        }
      }
    }

    return node;
  }

  std::string parse_name() {
    size_t start = position_;
    while (!at_end()) {
      char c = input_[position_];
      if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == ':' || c == '-') {
        ++position_;
      } else {
        break;
      }
    }
    return input_.substr(start, position_ - start);
  }

  std::string parse_attribute_value() {
    if (match("\"")) {
      size_t start = position_;
      while (!at_end() && input_[position_] != '\"') {
        ++position_;
      }
      std::string value = input_.substr(start, position_ - start);
      match("\"");
      return decode_xml_entities(value);
    }
    if (match("'")) {
      size_t start = position_;
      while (!at_end() && input_[position_] != '\'' ) {
        ++position_;
      }
      std::string value = input_.substr(start, position_ - start);
      match("'");
      return decode_xml_entities(value);
    }
    // Unquoted value
    size_t start = position_;
    while (!at_end() && !std::isspace(static_cast<unsigned char>(input_[position_])) && input_[position_] != '>') {
      ++position_;
    }
    return decode_xml_entities(input_.substr(start, position_ - start));
  }

  std::string parse_text() {
    size_t start = position_;
    while (!at_end() && input_[position_] != '<') {
      ++position_;
    }
    return decode_xml_entities(trim(input_.substr(start, position_ - start)));
  }

  static std::string trim(const std::string &value) {
    size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
      ++start;
    }
    size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
      --end;
    }
    return value.substr(start, end - start);
  }
};

std::string to_upper(const std::string &value) {
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return result;
}

bool is_text_node(const MachineDataNode &node) { return node.name == "#text"; }

void format_node_recursive(const MachineDataNode &node, std::string &out, int indent) {
  if (is_text_node(node)) {
    if (!node.text.empty()) {
      out.append(std::string(indent, ' '));
      out.append(node.text);
      out.push_back('\n');
    }
    return;
  }

  out.append(std::string(indent, ' '));
  out.append(node.name);
  if (!node.attributes.empty()) {
    out.append(" {");
    bool first = true;
    for (const auto &attr : node.attributes) {
      if (!first) {
        out.append(", ");
      }
      first = false;
      out.append(attr.first);
      out.append("=");
      out.append(attr.second);
    }
    out.append("}");
  }
  if (node.children.empty()) {
    if (!node.text.empty()) {
      out.append(": ");
      out.append(node.text);
    }
    out.push_back('\n');
    return;
  }
  out.push_back('\n');
  for (const auto &child : node.children) {
    format_node_recursive(child, out, indent + 2);
  }
}

void collect_text_recursive(const MachineDataNode &node, std::string &out) {
  if (is_text_node(node)) {
    out.append(node.text);
    return;
  }
  for (const auto &child : node.children) {
    collect_text_recursive(child, out);
  }
}

}  // namespace

const MachineDataNode *MachineDataNode::find_child_case_insensitive(const std::string &name) const {
  std::string target = to_upper(name);
  for (const auto &child : this->children) {
    if (is_text_node(child)) {
      continue;
    }
    if (to_upper(child.name) == target) {
      return &child;
    }
  }
  return nullptr;
}

std::optional<std::string> MachineDataNode::get_attribute_case_insensitive(const std::string &name) const {
  std::string target = to_upper(name);
  for (const auto &attribute : this->attributes) {
    if (to_upper(attribute.first) == target) {
      return attribute.second;
    }
  }
  return std::nullopt;
}

std::string MachineDataNode::collect_text_content() const {
  std::string text;
  collect_text_recursive(*this, text);
  return text;
}

std::optional<MachineDataNode> MachineDataParser::parse(const std::string &input) {
  SimpleXmlParser parser(input);
  return parser.parse();
}

std::string format_machine_data_tree(const MachineDataNode &node) {
  std::string output;
  format_node_recursive(node, output, 0);
  if (!output.empty() && output.back() == '\n') {
    output.pop_back();
  }
  return output;
}

std::string format_machine_data_section(const MachineDataNode *node) {
  if (node == nullptr) {
    return "";
  }
  return format_machine_data_tree(*node);
}

}  // namespace jutta_component
}  // namespace esphome

