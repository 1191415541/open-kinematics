#pragma once

/// The free functions of the `mb_element` module.
///
/// This is the element layer: the spring, bushing and anti-roll constitutive
/// wrenches with their curve helpers, the steering actuator and drive/brake
/// torque elements, and the element half of the directional (dual-number) force
/// pass.  The declarations are grouped by the module that defines them, not by
/// the module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"
#include "mb_config/diagnostics.hpp"
#include "mb_config/env.hpp"
#include "mb_config/element_wrench.hpp"
#include "mb_numeric/util.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_model/enums.hpp"
#include "mb_energy/types.hpp"

namespace axle_kernel {
std::pair<double, double> bushing_curve_value_slope( const Bushing& bushing, std::size_t axis, double value );

double integrate_bushing_curve_from_zero( const Bushing& bushing, std::size_t axis, double value );

void assemble_bushing_forces( const Model& model, const State& state, std::vector<Vec3>& force, std::vector<Vec3>& torque, std::vector<double>* bushing_component_output, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, bool brush_only, double internal_force_scale, double& dissipation, double& potential );

void assemble_anti_roll_forces( const Model& model, const State& state, std::vector<Vec3>& torque, std::vector<double>* anti_roll_component_output, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, bool brush_only, double internal_force_scale, double& dissipation, double& potential );

void assemble_spring_forces( const Model& model, const State& state, std::vector<Vec3>& force, std::vector<Vec3>& torque, std::vector<double>* spring_component_output, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, bool brush_only, double internal_force_scale, double& dissipation, double& potential );

void assemble_steering_forces( const Model& model, const State& state, const SampleInput& input, std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, bool brush_only, double& external_power );

void assemble_drive_brake_torques( const Model& model, const State& state, const SampleInput& input, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, bool brush_only, double& external_power );

// K4 (epic MODULES.md section 3.2): the directional element assembly.  It is the
// `if (!brush_only)` block that used to front `external_force_directional`:
// aerodynamic drag, then the spring, bushing and anti-roll elements, each with
// its analytic derivative.  The pack is returned by value -- the block has no
// floating-point accumulation-order constraint, so nothing has to be passed in
// and added back.
struct DirectionalElementForces {
    std::vector<Vec3> force;
    std::vector<Vec3> torque;
    bool smooth{true};
};

DirectionalElementForces assemble_directional_elements( const Model& model, const State& state, const SampleInput& input, const DirectionalState& direction, const StaticContactOverride* static_contact, bool brush_only, double external_load_scale, double internal_force_scale );

// K4 (epic MODULES.md section 3.2): the directional drive and brake torques.
// It is the `if (!brush_only)` block that followed the tire loop in
// `external_force_directional`: the mapped drive torque and the brake torque,
// each with its analytic derivative.
struct DirectionalDriveTorques {
    std::vector<Vec3> torque;
    bool smooth{true};
};

DirectionalDriveTorques assemble_directional_drive_torques( const Model& model, const State& state, const SampleInput& input, const DirectionalState& direction, const std::vector<Vec3>& torque_in );

void external_force_aerodynamic_directional( const Model& model, const State& state, const DirectionalState& direction, double external_load_scale, std::vector<Vec3>& force, std::vector<Vec3>& torque, bool& smooth);

void external_force_spring_directional( const Model& model, const State& state, const DirectionalState& direction, const StaticContactOverride* static_contact, double internal_force_scale, std::vector<Vec3>& force, std::vector<Vec3>& torque, bool& smooth);

void external_force_bushing_directional( const Model& model, const State& state, const DirectionalState& direction, double internal_force_scale, std::vector<Vec3>& force, std::vector<Vec3>& torque, bool& smooth);

void external_force_anti_roll_directional( const Model& model, const State& state, const DirectionalState& direction, double internal_force_scale, std::vector<Vec3>& torque, bool& smooth);

void external_force_steering_directional( const Model& model, const State& state, const SampleInput& input, const DirectionalState& direction, std::vector<Vec3>& force, std::vector<Vec3>& torque, bool& smooth);

void write_directional_detachment( const Tire& t, const DirectionalScalar& sx, const DirectionalScalar& sy, std::size_t i, int stride, std::vector<double>& tire_state_derivatives );
// The optional `sink` records the wrench the call applies.  A caller that does
// not pass one -- every caller on the default path -- keeps the call it always
// made: the sink only reads values that were already computed, and it records
// after the accumulation, so no existing expression or order changes.
Vec3 add_force_on_body( std::vector<Vec3>& force, std::vector<Vec3>& torque, const Model& model, const State& state, int body, const Vec3& point_local, const Vec3& f_world, ElementWrenchSink* sink = nullptr);
void add_torque_on_body( std::vector<Vec3>& torque, const Model& model, int body, const Vec3& tau_world, ElementWrenchSink* sink = nullptr );
std::array<double, 6> mat6_mul( const std::array<double, 36>& matrix, const std::array<double, 6>& vector );
std::array<double, 6> bushing_deformation( const Bushing& bushing, const Model& model, const State& state, std::array<double, 6>& rate );

} // namespace axle_kernel
