/// Self-test for the `mb_contract` module.
///
/// Built as a standalone executable so the contract layer can be verified
/// without exporting anything through the product C ABI.

#include "mb_contract/functions.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_checks = 0;
int g_failures = 0;

void check(bool condition, const char* label) {
  ++g_checks;
  if (!condition) {
    ++g_failures;
    std::printf("FAIL: %s\n", label);
  }
}

void expect_canonical(const std::string& text, bool expected, const char* label) {
  std::string error;
  const bool ok = axle_kernel::contract_verify_canonical(text, error);
  check(ok == expected, label);
}

std::string round_trip(const std::string& canonical_text) {
  axle_kernel::JsonValue value;
  std::string error;
  if (!axle_kernel::contract_parse_json(canonical_text, value, error)) {
    check(false, "round trip: parse failed");
    return {};
  }
  std::string out;
  if (!axle_kernel::contract_write_canonical(value, out, error)) {
    check(false, "round trip: write failed");
    return {};
  }
  return out;
}

}  // namespace

int main() {
  using namespace axle_kernel;

  // --- canonical form -----------------------------------------------------
  expect_canonical("{\"a\":1,\"b\":2}", true, "canonical object accepted");
  expect_canonical("{\"b\":2,\"a\":1}", false, "unsorted object rejected");
  expect_canonical("{ \"a\": 1 }", false, "incidental whitespace rejected");
  expect_canonical("{\"a\":null,\"b\":true,\"c\":false}", true, "literals accepted");

  check(round_trip("{\"x\":-0.0}") == "{\"x\":-0.0}", "negative zero keeps its sign");
  check(round_trip("{\"x\":1.0}") == "{\"x\":1.0}", "integral float keeps its float form");
  check(round_trip("{\"x\":1}") == "{\"x\":1}", "integer stays an integer");
  check(round_trip("{\"x\":0.10000000000000001}") == "{\"x\":0.10000000000000001}",
        "seventeen significant digits are stable");
  check(round_trip("{\"x\":1.0000000000000001e+300}") ==
            "{\"x\":1.0000000000000001e+300}",
        "large exponent is stable at seventeen digits");
  expect_canonical("{\"x\":1e+300}", false,
                   "a truncated exponent form is not canonical");
  check(round_trip("{\"s\":\"a\\u0001b\"}") == "{\"s\":\"a\\u0001b\"}",
        "control characters round trip");
  // The canonical form emits raw UTF-8, so an escaped non-ASCII literal is
  // not canonical even though it decodes to the same text.
  expect_canonical("{\"s\":\"\\u4fa7\\u503e\"}", false,
                   "escaped non-ASCII is not canonical");
  check(round_trip("{\"s\":\"\xe4\xbe\xa7\xe5\x80\xbe\"}") ==
            "{\"s\":\"\xe4\xbe\xa7\xe5\x80\xbe\"}",
        "raw UTF-8 round trips");

  std::string error;
  check(!contract_verify_canonical("{\"a\":1,}", error), "trailing comma rejected");
  check(!contract_verify_canonical("{\"a\":1}{\"b\":2}", error), "two documents rejected");
  check(!contract_verify_canonical("{\"a\":1,\"a\":2}", error), "duplicate key rejected");

  // --- identity -----------------------------------------------------------
  // Published SHA-256 test vectors; a mismatch here would invalidate every
  // contract identity the kernel derives.
  check(contract_sha256_hex("") ==
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
        "sha256 of the empty string");
  check(contract_sha256_hex("abc") ==
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "sha256 of abc");
  check(contract_sha256_hex(std::string(1000000, 'a')) ==
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0",
        "sha256 of a million a");

  // --- container ----------------------------------------------------------
  const std::string document = "{\"contract\":\"multibody-case\",\"kind\":\"case\"}";
  const std::string blob("\x00\x01\x02\x03", 4);
  const std::string payload = contract_build_container(document, blob);
  ContractPayload parsed;
  check(contract_parse_container(reinterpret_cast<const std::uint8_t*>(payload.data()),
                                payload.size(), parsed, error),
        "container parses");
  check(parsed.blob == blob, "blob section survives the round trip");
  check(parsed.contract_version == kContractVersion, "container carries its version");
  check(parsed.document.find_string("contract") != nullptr, "document fields are readable");

  std::string truncated = payload.substr(0, payload.size() - 1);
  check(!contract_parse_container(reinterpret_cast<const std::uint8_t*>(truncated.data()),
                                 truncated.size(), parsed, error),
        "truncated container rejected");
  std::string foreign = payload;
  foreign[0] = 'X';
  check(!contract_parse_container(reinterpret_cast<const std::uint8_t*>(foreign.data()),
                                 foreign.size(), parsed, error),
        "foreign magic rejected");
  std::string non_canonical = contract_build_container("{\"b\":1,\"a\":1}", blob);
  check(!contract_parse_container(
            reinterpret_cast<const std::uint8_t*>(non_canonical.data()),
            non_canonical.size(), parsed, error),
        "non-canonical document rejected");

  // --- registries ---------------------------------------------------------
  check(contract_joint_rows("spherical") == 3, "spherical joint rows");
  check(contract_joint_rows("revolute") == 5, "revolute joint rows");
  check(contract_joint_rows("fixed") == 6, "fixed joint rows");
  check(contract_joint_rows("nope") == -1, "unknown joint reported as -1");
  check(contract_element_known("bushing"), "bushing element known");
  check(!contract_element_known("sprocket"), "unknown element rejected");
  check(contract_tire_known("pac2002"), "pac2002 tire known");
  check(contract_case_family_known("ride_four_post"), "four-post case family known");
  check(!contract_case_family_known("rally"), "unknown case family rejected");
  check(contract_registry_size(0) == 10, "ten joint types registered");
  check(contract_registry_size(1) == 9, "nine element types registered");
  check(contract_registry_size(2) == 4, "four tire models registered");
  check(contract_registry_size(3) == 8, "eight case families registered");

  if (g_failures == 0) {
    std::printf("mb_contract selftest: OK (%d checks)\n", g_checks);
    return 0;
  }
  std::printf("mb_contract selftest: FAILED (%d/%d checks)\n", g_failures, g_checks);
  return 1;
}