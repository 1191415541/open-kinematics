/// Consistency self-test for the case-family table.
///
/// The point of subtask 08 is that three questions about a case family are
/// answered by *one* description:
///
///   1. does the protocol define the name?         -- `mb_contract`'s registry
///   2. does this build implement it?              -- the case table
///   3. which function expands it?                 -- the same row as (2)
///
/// (1) and (2) are necessarily two tables, in two modules: the contract layer is
/// not allowed to depend on the case layer, and it must be able to name a family
/// that no build implements (`comparison` is exactly that).  What must *not*
/// happen is the two silently diverging -- a protocol name no row mentions, or a
/// row claiming support with no handler behind it.  That is what this executable
/// checks, against the real tables rather than against a copied list.

#include "mb_cases/functions.hpp"
#include "mb_contract/functions.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool condition, const char* what) {
  ++g_checks;
  if (!condition) {
    ++g_failures;
    std::printf("FAIL: %s\n", what);
  }
}

/// Every name the contract module declares.
std::vector<std::string> protocol_names() {
  std::vector<std::string> names;
  // Table 3 is the case families; see `contract_registry_size`'s switch.
  const int size = axle_kernel::contract_registry_size(3);
  for (int index = 0; index < size; ++index) {
    const char* name = axle_kernel::contract_registry_name(3, index);
    if (name != nullptr) {
      names.emplace_back(name);
    }
  }
  return names;
}

}  // namespace

int main() {
  using namespace axle_kernel;

  // --- the query and the dispatch agree -----------------------------------
  // A family reported supported must be exactly one the protocol knows and this
  // build has a handler for.
  const int declared = contract_case_family_count(/*supported_only=*/false);
  const int supported = contract_case_family_count(/*supported_only=*/true);
  check(declared > 0, "the case table is not empty");
  check(supported > 0, "at least one family is implemented");
  check(supported <= declared, "supported is a subset of declared");

  for (int index = 0; index < declared; ++index) {
    const char* name = contract_case_family_name(index);
    check(name != nullptr, "every table row has a name");
    if (name == nullptr) {
      continue;
    }
    const std::string family(name);
    // The three predicates must agree with each other for every row.
    const bool is_supported = contract_case_supported(family);
    const bool in_protocol = contract_case_family_in_protocol(family);
    check(in_protocol, "a table row is a protocol name");
    if (is_supported) {
      // Supported implies the protocol knows it: a handler for a name no
      // contract defines would be unreachable through a real document.
      check(in_protocol, "a supported family is in the protocol");
    }
    if (!is_supported) {
      // The interesting direction: known but unimplemented must *say* so, and
      // must not be reported as an unknown name.
      check(in_protocol,
            "an unimplemented family is still reported as known to the protocol");
    }
  }

  // --- unknown names are not in the protocol -------------------------------
  check(!contract_case_supported("rally"), "an unknown family is not supported");
  check(!contract_case_family_in_protocol("rally"),
        "an unknown family is not in the protocol");
  check(!contract_case_supported(""),
        "the empty family name is not supported");

  // --- the protocol list and the table agree on membership -----------------
  // Every protocol name must appear in the case table.  The table may name
  // nothing extra in the protocol column; it exists so the refusal can say why.
  for (const std::string& name : protocol_names()) {
    check(contract_case_family_in_protocol(name),
          "every protocol family is classified by the case table");
  }
  const std::vector<std::string> known = protocol_names();
  check(static_cast<int>(known.size()) == declared,
        "the case table mentions exactly the protocol's families");

  // --- `comparison` is the live known-but-unimplemented case --------------
  // It is a per-target gate rather than a solve, so it must be refused *by name*
  // and the refusal must distinguish it from a typo.
  check(contract_case_family_in_protocol("comparison"),
        "comparison is a protocol family");
  check(!contract_case_supported("comparison"),
        "comparison has no expander in this build");

  if (g_failures == 0) {
    std::printf("mb_cases selftest: OK (%d checks)\n", g_checks);
    return 0;
  }
  std::printf("mb_cases selftest: FAILED (%d/%d checks)\n", g_failures, g_checks);
  return 1;
}
