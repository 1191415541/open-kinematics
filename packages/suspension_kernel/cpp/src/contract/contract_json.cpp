/// Canonical JSON parsing and serialisation for the multibody contract.
///
/// The parser accepts ordinary JSON; the writer always emits the canonical
/// form.  `contract_verify_canonical` combines the two, so the boundary can
/// insist that a payload is byte-identical to its canonical form before any
/// hash or identity is derived from it.

#include "mb_contract/functions.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace axle_kernel {
namespace {

struct Parser {
  const char* cursor;
  const char* end;
  std::string error;

  bool fail(const std::string& message) {
    if (error.empty()) {
      error = message;
    }
    return false;
  }

  void skip_whitespace() {
    while (cursor < end) {
      const char c = *cursor;
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++cursor;
      } else {
        break;
      }
    }
  }

  bool literal(const char* text) {
    const std::size_t length = std::strlen(text);
    if (static_cast<std::size_t>(end - cursor) < length ||
        std::memcmp(cursor, text, length) != 0) {
      return false;
    }
    cursor += length;
    return true;
  }

  static void append_utf8(std::string& out, unsigned int code) {
    if (code <= 0x7F) {
      out.push_back(static_cast<char>(code));
    } else if (code <= 0x7FF) {
      out.push_back(static_cast<char>(0xC0 | (code >> 6)));
      out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
    } else if (code <= 0xFFFF) {
      out.push_back(static_cast<char>(0xE0 | (code >> 12)));
      out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
    } else {
      out.push_back(static_cast<char>(0xF0 | (code >> 18)));
      out.push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
      out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
    }
  }

  bool parse_hex4(unsigned int& value) {
    if (end - cursor < 4) {
      return fail("truncated \\u escape");
    }
    value = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = *cursor++;
      unsigned int digit = 0;
      if (c >= '0' && c <= '9') {
        digit = static_cast<unsigned int>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        digit = static_cast<unsigned int>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        digit = static_cast<unsigned int>(c - 'A' + 10);
      } else {
        return fail("invalid hex digit in \\u escape");
      }
      value = (value << 4) | digit;
    }
    return true;
  }

  bool parse_string(std::string& out) {
    if (cursor >= end || *cursor != '"') {
      return fail("expected a string");
    }
    ++cursor;
    out.clear();
    while (true) {
      if (cursor >= end) {
        return fail("unterminated string");
      }
      const unsigned char c = static_cast<unsigned char>(*cursor++);
      if (c == '"') {
        return true;
      }
      if (c < 0x20) {
        return fail("raw control character in string");
      }
      if (c != '\\') {
        out.push_back(static_cast<char>(c));
        continue;
      }
      if (cursor >= end) {
        return fail("truncated escape");
      }
      const char escape = *cursor++;
      switch (escape) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned int code = 0;
          if (!parse_hex4(code)) {
            return false;
          }
          if (code >= 0xD800 && code <= 0xDBFF) {
            if (end - cursor < 2 || cursor[0] != '\\' || cursor[1] != 'u') {
              return fail("unpaired high surrogate");
            }
            cursor += 2;
            unsigned int low = 0;
            if (!parse_hex4(low) || low < 0xDC00 || low > 0xDFFF) {
              return fail("invalid low surrogate");
            }
            code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
          } else if (code >= 0xDC00 && code <= 0xDFFF) {
            return fail("unpaired low surrogate");
          }
          append_utf8(out, code);
          break;
        }
        default:
          return fail("unknown escape sequence");
      }
    }
  }

  bool parse_number(JsonValue& out) {
    const char* start = cursor;
    if (cursor < end && *cursor == '-') {
      ++cursor;
    }
    if (cursor >= end || !std::isdigit(static_cast<unsigned char>(*cursor))) {
      return fail("malformed number");
    }
    if (*cursor == '0') {
      ++cursor;
    } else {
      while (cursor < end && std::isdigit(static_cast<unsigned char>(*cursor))) {
        ++cursor;
      }
    }
    bool is_integer = true;
    if (cursor < end && *cursor == '.') {
      is_integer = false;
      ++cursor;
      if (cursor >= end || !std::isdigit(static_cast<unsigned char>(*cursor))) {
        return fail("malformed fraction");
      }
      while (cursor < end && std::isdigit(static_cast<unsigned char>(*cursor))) {
        ++cursor;
      }
    }
    if (cursor < end && (*cursor == 'e' || *cursor == 'E')) {
      is_integer = false;
      ++cursor;
      if (cursor < end && (*cursor == '+' || *cursor == '-')) {
        ++cursor;
      }
      if (cursor >= end || !std::isdigit(static_cast<unsigned char>(*cursor))) {
        return fail("malformed exponent");
      }
      while (cursor < end && std::isdigit(static_cast<unsigned char>(*cursor))) {
        ++cursor;
      }
    }
    const std::string literal_text(start, static_cast<std::size_t>(cursor - start));
    out = JsonValue();
    out.kind = JsonKind::Number;
    out.number_is_integer = is_integer;
    if (is_integer) {
      errno = 0;
      out.integer = std::strtoll(literal_text.c_str(), nullptr, 10);
      out.number = static_cast<double>(out.integer);
    } else {
      out.number = std::strtod(literal_text.c_str(), nullptr);
      out.integer = static_cast<long long>(out.number);
    }
    if (!std::isfinite(out.number)) {
      return fail("number is not finite");
    }
    return true;
  }

  bool parse_value(JsonValue& out) {
    skip_whitespace();
    if (cursor >= end) {
      return fail("unexpected end of input");
    }
    const char c = *cursor;
    if (c == '{') {
      ++cursor;
      out = JsonValue();
      out.kind = JsonKind::Object;
      skip_whitespace();
      if (cursor < end && *cursor == '}') {
        ++cursor;
        return true;
      }
      while (true) {
        skip_whitespace();
        std::string key;
        if (!parse_string(key)) {
          return false;
        }
        skip_whitespace();
        if (cursor >= end || *cursor != ':') {
          return fail("expected ':' after object key");
        }
        ++cursor;
        JsonValue value;
        if (!parse_value(value)) {
          return false;
        }
        for (const auto& field : out.fields) {
          if (field.first == key) {
            return fail("duplicate object key");
          }
        }
        out.fields.emplace_back(std::move(key), std::move(value));
        skip_whitespace();
        if (cursor < end && *cursor == ',') {
          ++cursor;
          continue;
        }
        if (cursor < end && *cursor == '}') {
          ++cursor;
          return true;
        }
        return fail("expected ',' or '}' in object");
      }
    }
    if (c == '[') {
      ++cursor;
      out = JsonValue();
      out.kind = JsonKind::Array;
      skip_whitespace();
      if (cursor < end && *cursor == ']') {
        ++cursor;
        return true;
      }
      while (true) {
        JsonValue value;
        if (!parse_value(value)) {
          return false;
        }
        out.items.push_back(std::move(value));
        skip_whitespace();
        if (cursor < end && *cursor == ',') {
          ++cursor;
          continue;
        }
        if (cursor < end && *cursor == ']') {
          ++cursor;
          return true;
        }
        return fail("expected ',' or ']' in array");
      }
    }
    if (c == '"') {
      out = JsonValue();
      out.kind = JsonKind::String;
      return parse_string(out.text);
    }
    if (literal("true")) {
      out = JsonValue();
      out.kind = JsonKind::Bool;
      out.boolean = true;
      return true;
    }
    if (literal("false")) {
      out = JsonValue();
      out.kind = JsonKind::Bool;
      out.boolean = false;
      return true;
    }
    if (literal("null")) {
      out = JsonValue();
      out.kind = JsonKind::Null;
      return true;
    }
    return parse_number(out);
  }
};

void write_string(const std::string& value, std::string& out) {
  out.push_back('"');
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned int>(c));
          out += buffer;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  out.push_back('"');
}

void write_number(const JsonValue& value, std::string& out) {
  if (value.number_is_integer) {
    out += std::to_string(value.integer);
    return;
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.17g", value.number);
  std::string text(buffer);
  // Keep float literals unambiguous: ``%.17g`` renders -0.0 as "-0" and 1.0 as
  // "1", which JSON would read back as integers.
  if (text.find_first_of(".eE") == std::string::npos) {
    text += ".0";
  }
  out += text;
}

void write_value(const JsonValue& value, std::string& out) {
  switch (value.kind) {
    case JsonKind::Null:
      out += "null";
      return;
    case JsonKind::Bool:
      out += value.boolean ? "true" : "false";
      return;
    case JsonKind::Number:
      write_number(value, out);
      return;
    case JsonKind::String:
      write_string(value.text, out);
      return;
    case JsonKind::Array: {
      out.push_back('[');
      for (std::size_t i = 0; i < value.items.size(); ++i) {
        if (i != 0) {
          out.push_back(',');
        }
        write_value(value.items[i], out);
      }
      out.push_back(']');
      return;
    }
    case JsonKind::Object: {
      std::vector<std::size_t> order(value.fields.size());
      for (std::size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
      }
      std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return value.fields[a].first < value.fields[b].first;
      });
      out.push_back('{');
      for (std::size_t i = 0; i < order.size(); ++i) {
        if (i != 0) {
          out.push_back(',');
        }
        write_string(value.fields[order[i]].first, out);
        out.push_back(':');
        write_value(value.fields[order[i]].second, out);
      }
      out.push_back('}');
      return;
    }
  }
}

}  // namespace

const JsonValue* JsonValue::find(const std::string& key) const {
  for (const auto& field : fields) {
    if (field.first == key) {
      return &field.second;
    }
  }
  return nullptr;
}

const std::string* JsonValue::find_string(const std::string& key) const {
  const JsonValue* value = find(key);
  if (value == nullptr || value->kind != JsonKind::String) {
    return nullptr;
  }
  return &value->text;
}

bool contract_parse_json(const std::string& text, JsonValue& out, std::string& error) {
  Parser parser{text.data(), text.data() + text.size(), std::string()};
  JsonValue value;
  if (!parser.parse_value(value)) {
    error = parser.error.empty() ? "malformed JSON" : parser.error;
    return false;
  }
  parser.skip_whitespace();
  if (parser.cursor != parser.end) {
    error = "trailing characters after the document";
    return false;
  }
  out = std::move(value);
  error.clear();
  return true;
}

bool contract_write_canonical(const JsonValue& value, std::string& out,
                              std::string& error) {
  out.clear();
  write_value(value, out);
  error.clear();
  return true;
}

bool contract_verify_canonical(const std::string& text, std::string& error) {
  JsonValue value;
  if (!contract_parse_json(text, value, error)) {
    return false;
  }
  std::string canonical;
  std::string write_error;
  if (!contract_write_canonical(value, canonical, write_error)) {
    error = write_error;
    return false;
  }
  if (canonical != text) {
    error = "payload is not in canonical form; re-serialise with canonical_json_bytes";
    return false;
  }
  error.clear();
  return true;
}

}  // namespace axle_kernel