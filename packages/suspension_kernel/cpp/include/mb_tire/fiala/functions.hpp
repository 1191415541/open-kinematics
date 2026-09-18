#pragma once

/// The free functions of the `mb_tire/fiala` module.
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
#include "mb_tire/fiala/parameters.hpp"
#include "mb_base/monotone_cubic.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_tire_state/tire_state.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_base/prelude.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
double fiala_parameter(const Tire& tire, int index, double fallback);

int fiala_use_mode(const Tire& tire);

bool fiala_startup_smoothing_enabled();

double fiala_startup_scale(int use_mode, double time);

bool fiala_transient_forced();

bool fiala_transient_enabled(int use_mode);

double fiala_rolling_resistance_factor(double spin_rate);

void fiala_forces( const Tire& tire, double kappa, double alpha, double normal_force, double& fx, double& fy, double& mz );

double fiala_elastic_force(const Tire& tire, double penetration);

double fiala_vertical_force( const Tire& tire, double penetration, double penetration_rate );

bool fiala_relaxation_target( const Model& model, const State& state, const SampleInput& input, std::size_t tire_index, double& longitudinal_target, double& lateral_target, double& longitudinal_rate, double& lateral_rate );

void fiala_forces_directional( const Tire& tire, const DirectionalScalar& kappa, const DirectionalScalar& alpha, const DirectionalScalar& normal_force, DirectionalScalar& fx, DirectionalScalar& fy, DirectionalScalar& mz, bool& smooth );

DirectionalScalar fiala_vertical_force_directional( const Tire& tire, const DirectionalScalar& penetration, const DirectionalScalar& penetration_rate );
} // namespace axle_kernel
