#pragma once

/// The free functions of the `mb_tire/brush` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"
#include "mb_config/diagnostics.hpp"
#include "mb_tire_state/tire_state.hpp"
#include "mb_config/env.hpp"
#include "mb_numeric/util.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_config/prelude.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
void project_brush_state( const Tire& tire, double normal_force, double sx, double sy, double& projected_sx, double& projected_sy, double& trial_utilization );
} // namespace axle_kernel
