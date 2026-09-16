#pragma once

/// The free functions of the `mb_tire` module.
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
#include "mb_tire/model.hpp"
#include "mb_tire/pac2002/parameters.hpp"
#include "mb_tire/fiala/parameters.hpp"
#include "mb_base/monotone_cubic.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_tire_state/tire_state.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include "mb_tire/pac2002/spin.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_tire/assembly.hpp"
#include "mb_tire/force_context.hpp"
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire/brush/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_vehicle/energy.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_tire/brush/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"

namespace axle_kernel {
void pac2002_evaluate_forces( const Tire& tire, double normal_force, bool allow_longitudinal, bool allow_lateral, bool combined_slip, double limited_longitudinal_slip, double limited_lateral_slip, double limited_camber, const Pac2002TurnSlip& turn_slip, double& fx, double& fy, double& utilization );

Pac2002TurnSlipDiagnostics pac2002_turn_slip_diagnostics( const Tire& t, const State& state, const Model& model, std::size_t i, bool turn_slip_mode, int frame_body, const Vec3& normal, const Vec3& lateral, double fn, double spin_rate, double camber, const Vec3& vc, const Vec3& forward );

// The tire loop itself: `kernel_tire.cpp` held it and `external_force_vector`
// in the same unit, so it needed no declaration until K6 split them.
void assemble_tire_forces(
    const Model& model, const State& state, const SampleInput& input,
    std::vector<Vec3>& force, std::vector<Vec3>& torque,
    std::vector<double>& tire_forces, std::vector<double>& tire_state_derivatives,
    std::vector<double>& tire_output, std::vector<double>* tire_relaxation_rates,
    std::vector<double>* tire_deflections,
    const StaticContactOverride* static_contact, EnergyRates* energy_rates,
    EnergyStorage* energy_storage, bool brush_only, bool record_output,
    bool record_energy, double internal_force_scale, int stride,
    double& potential, double& external_power, double& dissipation
);

void write_tire_output_prefix( const ForceAssemblyContext& ctx, const State& state, std::size_t tire_index, std::size_t output_offset, double delta, double delta_dot, double vx, double vy, double sx, double sy );

void write_detachment_relaxation( const ForceAssemblyContext& ctx, const Tire& tire, std::size_t tire_index );

bool tire_is_detached_and_relaxing( const ForceAssemblyContext& ctx, const Tire& tire, std::size_t tire_index, bool static_active, double delta );

bool assemble_brush_tire( const ForceAssemblyContext& ctx, const Tire& tire, std::size_t tire_index, std::size_t output_offset, double delta, double delta_dot, double vx, double vy, double sx, double sy, double fn, double road_v, const Vec3& forward, const Vec3& lateral, const Vec3& normal, const Vec3& center, const Vec3& patch_arm );

NonPositiveNormalForceResult handle_non_positive_normal_force( const ForceAssemblyContext& ctx, const Tire& tire, std::size_t tire_index, double delta, double fn, bool pac2002 );

bool assemble_fiala_tire( const ForceAssemblyContext& ctx, const Tire& tire, const State& state, const Model& model, std::size_t tire_index, std::size_t output_offset, double spin_rate, double vx, double vy, double sx, double sy, double fn, double rolling_speed, const Vec3& forward, const Vec3& lateral, const Vec3& normal, const Vec3& center, const Vec3& patch_arm );

void write_pac2002_output_channels( const ForceAssemblyContext& ctx, std::size_t output_offset, double fn, double fx, double fy, double utilization, double overturning_moment, double rolling_resistance_moment, double aligning_moment, double rolling_speed, double slip_speed, double lateral_slip_base, double yaw_beta_component, double yaw_beta_st_component, double lateral_slip_target, double lateral_slip, const Pac2002TurnSlip& turn_slip, double turn_slip_drive, double turn_slip_yaw_rate, double turn_slip_camber_term, double total_spin_rate, double relaxation_length_lateral_output, double limited_longitudinal_slip, double limited_lateral_slip );

void accumulate_normal_potential_energy( const ForceAssemblyContext& ctx, const Tire& tire, double compression );

void accumulate_pac2002_energy( const ForceAssemblyContext& ctx, const Tire& tire, double delta, double delta_dot, double fn, double fx, double fy, double vx, double vy, double road_v );

void assemble_pac2002_tire( const ForceAssemblyContext& ctx, const Model& model, const State& state, const Tire& t, std::size_t i, int frame_body, const Vec3& center, const Vec3& vc, const Vec3& normal, double road_v, double sx, double sy, double delta, double delta_dot, double spin_rate, double rolling_speed, const Vec3& forward, const Vec3& lateral, const Vec3& patch_arm, double vx, double vy, std::size_t output_offset, double& fn, double pac2002_camber, double loaded_radius, double rolling_radius, bool static_active, double maxwell_displacement );
} // namespace axle_kernel
