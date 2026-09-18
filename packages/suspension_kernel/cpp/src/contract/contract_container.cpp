/// Container layout for the multibody contract payload.
///
/// `magic(4) | version(4 LE) | json_length(8 LE) | blob_length(8 LE) | json | blob`

#include "mb_contract/functions.hpp"

#include <cstring>

namespace axle_kernel {

const char kContractMagic[4] = {'M', 'B', 'C', '1'};
const std::uint32_t kContractVersion = 1;
constexpr std::size_t kHeaderSize = 4 + 4 + 8 + 8;

namespace {

std::uint32_t read_u32(const std::uint8_t* data) {
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8) |
         (static_cast<std::uint32_t>(data[2]) << 16) |
         (static_cast<std::uint32_t>(data[3]) << 24);
}

std::uint64_t read_u64(const std::uint8_t* data) {
  std::uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | static_cast<std::uint64_t>(data[i]);
  }
  return value;
}

void write_u32(std::string& out, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<char>((value >> (8 * i)) & 0xFF));
  }
}

void write_u64(std::string& out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<char>((value >> (8 * i)) & 0xFF));
  }
}

}  // namespace

bool contract_parse_container(const std::uint8_t* payload, std::size_t size,
                              ContractPayload& out, std::string& error) {
  if (payload == nullptr || size < kHeaderSize) {
    error = "payload is shorter than the container header";
    return false;
  }
  if (std::memcmp(payload, kContractMagic, 4) != 0) {
    error = "unexpected container magic";
    return false;
  }
  const std::uint32_t version = read_u32(payload + 4);
  if (version != kContractVersion) {
    error = "unsupported container version";
    return false;
  }
  const std::uint64_t json_length = read_u64(payload + 8);
  const std::uint64_t blob_length = read_u64(payload + 16);
  if (json_length > size ||
      blob_length > size ||
      kHeaderSize + json_length + blob_length != size) {
    error = "container length mismatch";
    return false;
  }
  const char* json_begin = reinterpret_cast<const char*>(payload + kHeaderSize);
  const std::string text(json_begin, static_cast<std::size_t>(json_length));
  JsonValue document;
  std::string parse_error;
  if (!contract_parse_json(text, document, parse_error)) {
    error = "container document: " + parse_error;
    return false;
  }
  if (!document.is_object()) {
    error = "container document must be a JSON object";
    return false;
  }
  std::string canonical_error;
  if (!contract_verify_canonical(text, canonical_error)) {
    error = canonical_error;
    return false;
  }
  out.document = std::move(document);
  out.blob.assign(json_begin + json_length, static_cast<std::size_t>(blob_length));
  out.contract_version = version;
  error.clear();
  return true;
}

std::string contract_build_container(const std::string& canonical_json,
                                     const std::string& blob) {
  std::string payload;
  payload.reserve(kHeaderSize + canonical_json.size() + blob.size());
  payload.append(kContractMagic, 4);
  write_u32(payload, kContractVersion);
  write_u64(payload, canonical_json.size());
  write_u64(payload, blob.size());
  payload += canonical_json;
  payload += blob;
  return payload;
}

}  // namespace axle_kernel