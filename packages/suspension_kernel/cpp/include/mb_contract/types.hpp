#pragma once

/// Value types for the multibody contract boundary.
///
/// The kernel only ever receives documents produced by the Python side in
/// canonical form (sorted keys, no incidental whitespace, ``%.17g`` floats).
/// ``JsonValue`` therefore exists to *parse and re-serialise* those documents
/// and to prove the re-serialisation is byte-identical, not to be a general
/// purpose JSON library.

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace axle_kernel {

enum class JsonKind { Null, Bool, Number, String, Array, Object };

struct JsonValue {
  JsonKind kind = JsonKind::Null;
  bool boolean = false;
  double number = 0.0;
  bool number_is_integer = false;
  long long integer = 0;
  std::string text;
  std::vector<JsonValue> items;
  std::vector<std::pair<std::string, JsonValue>> fields;

  const JsonValue* find(const std::string& key) const;
  const std::string* find_string(const std::string& key) const;
  bool is_object() const { return kind == JsonKind::Object; }
  bool is_array() const { return kind == JsonKind::Array; }
};

/// One named numeric array inside a payload's blob section.
struct BlobDescriptor {
  std::string name;
  std::size_t offset = 0;
  std::size_t length = 0;
  std::string dtype;
  std::vector<std::size_t> shape;
};

/// A parsed payload: the canonical JSON document plus its raw blob section.
struct ContractPayload {
  JsonValue document;
  std::string blob;
  std::size_t contract_version = 0;
};

}  // namespace axle_kernel