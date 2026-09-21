#pragma once

/// The free functions of the `abi` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_input/types.hpp"
#include "core_abi.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_tire/model.hpp"
#include "mb_tire/pac2002/parameters.hpp"
#include "mb_tire/fiala/parameters.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"
#include "mb_config/diagnostics.hpp"
#include "mb_tire_state/tire_state.hpp"
#include "mb_config/env.hpp"
#include "mb_numeric/util.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_energy/types.hpp"
#include "mb_linear/factorization_types.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include "mb_tire/pac2002/spin.hpp"
#include "mb_solve_dynamic/context.hpp"
#include "mb_joint/types.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_tire/assembly.hpp"
#include "mb_tire/force_context.hpp"
#include "mb_output/measurement.hpp"
#include "abi/version.hpp"
#include "mb_config/prelude.hpp"
#include "mb_joint/registry.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
int run_model( const AxleInput* input, AxleOutput* output, char* error_buffer, std::size_t error_capacity, const Model* model_override );
} // namespace axle_kernel
