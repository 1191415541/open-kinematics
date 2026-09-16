#pragma once

/// The free functions of the `mb_tire/common` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_base/prelude.hpp"
#include "mb_base/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_base/monotone_cubic.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"

namespace axle_kernel {
TireContactKinematics tire_contact_kinematics( const Model& model, const State& state, const SampleInput& input, std::size_t tire_index );

DirectionalScalar tire_contact_compression_directional( const Model& model, const State& state, const SampleInput& input, const DirectionalState& direction, std::size_t tire_index, bool& smooth );
} // namespace axle_kernel
