/// The case-family dispatcher.
///
/// One table decides everything asked about a family: whether the protocol knows
/// the name, whether this build implements it, and which function expands it.
/// Before the table those three facts lived in three places -- `mb_contract`'s
/// registry, a hand-written `||` chain, and an `if` ladder -- and nothing checked
/// that they agreed.  A family declared and unimplemented was refused only by the
/// accident of two lists differing; one implemented but left off a list was
/// silently reported as missing.
///
/// The two failure modes are kept distinct on purpose:
///
///   * a name the protocol does not define is an **unknown family**;
///   * a name the protocol defines but this build cannot run is **not implemented
///     by this build**.
///
/// Collapsing them would tell a user their typo and their missing feature are the
/// same problem.  `comparison` is the live example of the second kind: the
/// contract names it, and it is a per-target gate rather than a solve, so no
/// expander exists for it.

#include "case_common.hpp"

#include <cstddef>
#include <string>

namespace axle_kernel {
namespace {

using axle_kernel::ContractPlan;

/// How well a family name is known to this build.
enum class Support {
  kSupported,
  kKnownUnimplemented,
  kNotInProtocol,
};

/// One expander's signature, matching the per-family functions.
using Expander = bool (*)(const JsonValue& document, const std::string& blob,
                          const ContractModel& model, ContractPlan& out,
                          std::string& error);

/// One family: its name, which failure applies, and how it expands.
///
/// `expander` is non-null exactly when the family is supported, so "can we run
/// it" and "which function runs it" cannot disagree.  `in_protocol` separates the
/// two failure modes above.
struct FamilyRow {
  const char* name;
  bool in_protocol;
  Expander expander;
};

/// Every name the contract defines, with the handler when there is one.
///
/// This table is the single source of truth for the case layer.  The contract
/// module keeps its own list of names -- it must, because the protocol half is
/// allowed to know a family that no build implements -- and the self-test asserts
/// the two agree on the `in_protocol` column, so a protocol name that no row
/// mentions is a test failure rather than a silent omission.
/// One expander's signature, uniform across the table.
///
/// Two families ignore the blob (`kc_quasi_static`, `vehicle_kc`: a grid has no
/// sampled tables), so they are reached through thin adapters below.  The table
/// stays uniform on purpose -- a table whose rows had different types could not be
/// searched, and the adapters are where the difference is visible instead of
/// spread across the dispatch.
using Expander = bool (*)(const JsonValue& document, const std::string& blob,
                          const ContractModel& model, ContractPlan& out,
                          std::string& error);

bool expand_kc_without_blob(const JsonValue& document, const std::string& blob,
                            const ContractModel& model, ContractPlan& out,
                            std::string& error) {
  (void)blob;
  return case_detail::expand_kc_quasi_static(document, model, out, error);
}

bool expand_vehicle_kc_without_blob(const JsonValue& document,
                                    const std::string& blob,
                                    const ContractModel& model, ContractPlan& out,
                                    std::string& error) {
  (void)blob;
  return case_detail::expand_vehicle_kc(document, model, out, error);
}
const FamilyRow kFamilies[] = {
    /// The grid families take no blob, so they arrive through the adapters above.
    {"kc_quasi_static", true, &expand_kc_without_blob},
    {"vehicle_kc", true, &expand_vehicle_kc_without_blob},
    {"axle_dynamic", true, &case_detail::expand_axle_dynamic},
    {"vehicle_dynamic", true, &case_detail::expand_vehicle_dynamic},
    {"handling", true, &case_detail::expand_handling},
    {"ride_four_post", true, &case_detail::expand_ride_four_post},
    {"ride_random_road", true, &case_detail::expand_ride_random_road},
    // Declared by the protocol and deliberately without a handler: `comparison`
    // is a per-target gate, not a solve.  The row keeps the name visible so the
    // refusal can say *why* it is refused.
    {"comparison", true, nullptr},
};

const FamilyRow* find_row(const std::string& family) {
  for (const FamilyRow& row : kFamilies) {
    if (family == row.name) {
      return &row;
    }
  }
  return nullptr;
}

Support support_of(const std::string& family) {
  const FamilyRow* row = find_row(family);
  if (row == nullptr) {
    return Support::kNotInProtocol;
  }
  return row->expander != nullptr ? Support::kSupported
                                  : Support::kKnownUnimplemented;
}

}  // namespace

bool contract_case_supported(const std::string& family) {
  return support_of(family) == Support::kSupported;
}

/// Whether the protocol defines a family, whether or not this build runs it.
///
/// Separate from `contract_case_supported` because they answer different
/// questions, and the dispatcher's error message depends on telling them apart.
bool contract_case_family_in_protocol(const std::string& family) {
  return support_of(family) != Support::kNotInProtocol;
}

int contract_case_family_count(bool supported_only) {
  int count = 0;
  for (const FamilyRow& row : kFamilies) {
    if (!supported_only || row.expander != nullptr) {
      ++count;
    }
  }
  return count;
}

const char* contract_case_family_name(int index) {
  const int size = static_cast<int>(sizeof(kFamilies) / sizeof(kFamilies[0]));
  if (index < 0 || index >= size) {
    return nullptr;
  }
  return kFamilies[index].name;
}

bool contract_expand_case(const JsonValue& document, const std::string& blob,
                          const ContractModel& model, ContractPlan& out,
                          std::string& error) {
  using case_detail::fail;
  error.clear();
  out = ContractPlan{};

  const std::string* contract = document.find_string("contract");
  const std::string* kind = document.find_string("kind");
  const std::string* family = document.find_string("family");
  if (contract == nullptr || *contract != "multibody-case" || kind == nullptr ||
      *kind != "case" || family == nullptr) {
    return fail(error, "expected a multibody-case document");
  }
  // The classification and the dispatch read the *same* row, so a family cannot
  // be reported runnable and then fall through to the wrong expander.
  switch (support_of(*family)) {
    case Support::kNotInProtocol:
      return fail(error, "unknown case family \"" + *family + "\"");
    case Support::kKnownUnimplemented:
      return fail(error, "case family \"" + *family +
                             "\" is defined by the contract but not implemented "
                             "by this build");
    case Support::kSupported:
      break;
  }
  return find_row(*family)->expander(document, blob, model, out, error);
}

void contract_apply_solver(const ContractPlan& plan, AxleInput& input) {
  case_detail::apply_solver(plan, input);
}

}  // namespace axle_kernel
