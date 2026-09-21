#pragma once

/// The free functions of the `mb_force` module.
///
/// This is the external-force layer: the scalar bus `external_force_vector`, the
/// directional bus `external_force_directional`, the applied-wrench/aerodynamic/
/// gravity assembly, the generalized-force assembly, and the force-assembly
/// arithmetic the model layer used to hold (`add_force_on_body`,
/// `add_torque_on_body`, `mat6_mul`, `bushing_deformation`).  The declarations
/// are grouped by the module that defines them, not by the module that calls
/// them, so the layering the project checks with `check_module_layering.py` is
/// also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_energy/types.hpp"

namespace axle_kernel {
double assemble_aerodynamic_force( const Model& model, const State& state, std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, double external_load_scale );

int reset_force_outputs( const Model& model, std::vector<double>& tire_forces, std::vector<double>& tire_state_derivatives, std::vector<double>* tire_relaxation_rates, std::vector<double>& tire_output, std::vector<double>* spring_component_output, std::vector<double>* bushing_component_output, std::vector<double>* anti_roll_component_output, bool record_output );

void assemble_external_and_gravity( const Model& model, const State& state, const SampleInput& input, std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, double external_load_scale, double gravity_x, double gravity_y, double gravity_z, int n, double& external_power, double& potential );

void assemble_generalized_force( const Model& model, const State& state, const std::vector<Vec3>& force, const std::vector<Vec3>& torque, std::vector<double>& generalized_force, bool brush_only );

void external_force_vector( const Model& model, const State& state, const SampleInput& input, double gravity_x, double gravity_y, double gravity_z, std::vector<double>& tire_forces, std::vector<double>& tire_state_derivatives, std::vector<double>& tire_output, double& potential, double& external_power, double& dissipation, std::vector<double>& generalized_force, std::vector<double>* spring_component_output = nullptr, std::vector<double>* bushing_component_output = nullptr, std::vector<double>* anti_roll_component_output = nullptr, const StaticContactOverride* static_contact = nullptr, EnergyRates* energy_rates = nullptr, EnergyStorage* energy_storage = nullptr, bool brush_only = false, std::vector<Vec3>* force_workspace = nullptr, std::vector<Vec3>* torque_workspace = nullptr, bool dynamics_only = false, std::vector<double>* tire_relaxation_rates = nullptr, std::vector<double>* tire_deflections = nullptr);

bool external_force_directional( const Model& model, const State& state, const SampleInput& input, const DirectionalState& direction, std::vector<double>& generalized_force, std::vector<double>& tire_state_derivatives, bool brush_only = false, const StaticContactOverride* static_contact = nullptr, DirectionalForceScratch* scratch = nullptr );

void assemble_directional_generalized_force( std::vector<double>& generalized_force, const std::vector<Vec3>& force, const std::vector<Vec3>& torque, const Model& model, const State& state, const DirectionalState& direction, int n);

// Force-assembly arithmetic.  It used to live in `mb_model`; it names only the
// model types and the numeric layer, so the force layer owns it now.



} // namespace axle_kernel
