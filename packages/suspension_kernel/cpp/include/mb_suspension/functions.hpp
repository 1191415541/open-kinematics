#pragma once

/// The free functions of the `mb_suspension` module.
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
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_suspension/functions.hpp"
#include "mb_vehicle/energy.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"

namespace axle_kernel {
std::pair<double, double> bushing_curve_value_slope( const Bushing& bushing, std::size_t axis, double value );

double integrate_bushing_curve_from_zero( const Bushing& bushing, std::size_t axis, double value );

void assemble_bushing_forces( const Model& model, const State& state, std::vector<Vec3>& force, std::vector<Vec3>& torque, std::vector<double>* bushing_component_output, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, bool brush_only, double internal_force_scale, double& dissipation, double& potential );

void assemble_anti_roll_forces( const Model& model, const State& state, std::vector<Vec3>& torque, std::vector<double>* anti_roll_component_output, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, bool brush_only, double internal_force_scale, double& dissipation, double& potential );

void assemble_spring_forces( const Model& model, const State& state, std::vector<Vec3>& force, std::vector<Vec3>& torque, std::vector<double>* spring_component_output, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, bool brush_only, double internal_force_scale, double& dissipation, double& potential );
} // namespace axle_kernel
