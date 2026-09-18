#pragma once

/// The free functions of the `mb_linalg` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_base/prelude.hpp"
#include "mb_base/vector.hpp"
#include "mb_base/monotone_cubic.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_linalg/factorization_types.hpp"
#include "mb_base/prelude.hpp"

namespace axle_kernel {
bool solve_linear(std::vector<double> A, std::vector<double> b, std::vector<double>& x);

int matrix_rank( std::vector<double> matrix, int rows, int columns );
} // namespace axle_kernel
