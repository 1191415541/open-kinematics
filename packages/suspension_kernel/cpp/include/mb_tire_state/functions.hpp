#pragma once

/// The free functions of the `mb_tire_state` module.
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
#include "mb_tire_state/tire_state.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"

namespace axle_kernel {
void resize_tire_states(const Model& model, State& state);

double tire_state_value(const State& state, std::size_t tire, int slot);

int tire_state_width(const Model& model);

int tire_block_width(const Model& model);

void resize_tire_states(const Model& model, State& state);

void read_tire_states( const std::vector<double>& x, int base, int index, int per_tire, State& state );

double tire_state_value(const State& state, std::size_t tire, int slot);
} // namespace axle_kernel
