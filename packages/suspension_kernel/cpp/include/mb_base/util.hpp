#pragma once

/// Small whole-vector utilities (MB_BASE).
///
/// `finite_vec` is used by the linear-algebra backends (which is why it lives
/// below them rather than beside `max_abs`) and by the integrator's own
/// checks; `max_abs` is used by the linear solvers and the statics layer.  Both
/// are dependency-free, so they belong to the base layer rather than to whichever
/// layer happened to need them first.

#include <cstddef>
#include <vector>

namespace axle_kernel {

bool finite_vec(const std::vector<double>& v);

double max_abs(const std::vector<double>& x);

} // namespace axle_kernel
