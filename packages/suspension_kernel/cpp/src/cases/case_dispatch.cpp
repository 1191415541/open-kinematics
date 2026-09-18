/// The case-family dispatcher.
///
/// One place decides which family a document belongs to and hands it to that
/// family's expander.  Adding a family is a new file plus a row here -- no
/// change to the ABI, to the solver, or to the other families.  A family that
/// is declared in the contract but not implemented here is rejected by name,
/// which is the difference between "not supported yet" and "silently ran
/// something else".

#include "case_common.hpp"

namespace axle_kernel {

bool contract_case_supported(const std::string& family) {
  return family == "kc_quasi_static" || family == "axle_dynamic" ||
         family == "vehicle_dynamic" || family == "ride_four_post" ||
         family == "ride_random_road" || family == "handling" ||
         family == "vehicle_kc";
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
  if (!contract_case_supported(*family)) {
    return fail(error, "case family " + *family +
                            " is not implemented by this build");
  }
  if (*family == "kc_quasi_static") {
    return case_detail::expand_kc_quasi_static(document, model, out, error);
  }
  if (*family == "vehicle_kc") {
    return case_detail::expand_vehicle_kc(document, model, out, error);
  }
  if (*family == "axle_dynamic") {
    return case_detail::expand_axle_dynamic(document, blob, model, out, error);
  }
  if (*family == "ride_four_post") {
    return case_detail::expand_ride_four_post(document, blob, model, out, error);
  }
  if (*family == "ride_random_road") {
    return case_detail::expand_ride_random_road(document, blob, model, out, error);
  }
  if (*family == "handling") {
    return case_detail::expand_handling(document, blob, model, out, error);
  }
  return case_detail::expand_vehicle_dynamic(document, blob, model, out, error);
}

void contract_apply_solver(const ContractPlan& plan, AxleInput& input) {
  case_detail::apply_solver(plan, input);
}

}  // namespace axle_kernel
