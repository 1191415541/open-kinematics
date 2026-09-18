/// Expand a `vehicle_kc` case document into concrete solver runs.
///
/// A vehicle-level K/C sweep prescribes the driven coordinates of a *whole*
/// vehicle -- four wheel-centre travels and the rack -- and measures what the
/// body does, where the axle family prescribes two wheel centres and the rack
/// and measures the axle.  The expansion is therefore the same cartesian grid
/// over named driven coordinates, and it is deliberately the same code: a
/// second copy would be a second place for the grid order, the absolute-target
/// rule and the case naming to drift.
///
/// What differs is only the model the grid runs on, and that difference lives
/// in the model document -- which driven joints it declares -- not here.

#include "case_common.hpp"

namespace axle_kernel {
namespace case_detail {

bool expand_vehicle_kc(const Json& document, const ContractModel& model,
                       ContractPlan& out, std::string& error) {
  if (!read_identity(document, "vehicle_kc", out, error)) return false;
  return expand_driven_grid(document, model, out, error);
}

}  // namespace case_detail
}  // namespace axle_kernel
