#pragma once

/// The free functions of the `mb_vehicle` module.
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
#include "mb_vehicle/energy.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include "mb_tire/pac2002/spin.hpp"
#include "mb_constraint/types.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_tire/assembly.hpp"
#include "mb_tire/force_context.hpp"
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_constraint/functions.hpp"
#include "mb_constraint/registry.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_static/functions.hpp"
#include "mb_suspension/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_vehicle/functions.hpp"
#include "axle_kernel.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_suspension/functions.hpp"
#include "mb_constraint/functions.hpp"
#include "mb_static/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"

namespace axle_kernel {
// --- K4 element and source assemblers (kernel_force_elements.cpp) ---
// These moved out of `kernel_tire.cpp` as one contiguous run of top-level
// functions; `external_force_vector` stayed behind and calls them all.
double assemble_aerodynamic_force( const Model& model, const State& state, std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, double external_load_scale );

int reset_force_outputs( const Model& model, std::vector<double>& tire_forces, std::vector<double>& tire_state_derivatives, std::vector<double>* tire_relaxation_rates, std::vector<double>& tire_output, std::vector<double>* spring_component_output, std::vector<double>* bushing_component_output, std::vector<double>* anti_roll_component_output, bool record_output );

void assemble_steering_forces( const Model& model, const State& state, const SampleInput& input, std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, bool brush_only, double& external_power );

void assemble_external_and_gravity( const Model& model, const State& state, const SampleInput& input, std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, EnergyStorage* energy_storage, bool record_energy, double external_load_scale, double gravity_x, double gravity_y, double gravity_z, int n, double& external_power, double& potential );

void assemble_generalized_force( const Model& model, const State& state, const std::vector<Vec3>& force, const std::vector<Vec3>& torque, std::vector<double>& generalized_force, bool brush_only );

void assemble_drive_brake_torques( const Model& model, const State& state, const SampleInput& input, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, bool brush_only, double& external_power );

void external_force_vector( const Model& model, const State& state, const SampleInput& input, double gravity_x, double gravity_y, double gravity_z, std::vector<double>& tire_forces, std::vector<double>& tire_state_derivatives, std::vector<double>& tire_output, double& potential, double& external_power, double& dissipation, std::vector<double>& generalized_force, std::vector<double>* spring_component_output = nullptr, std::vector<double>* bushing_component_output = nullptr, std::vector<double>* anti_roll_component_output = nullptr, const StaticContactOverride* static_contact = nullptr, EnergyRates* energy_rates = nullptr, EnergyStorage* energy_storage = nullptr, bool brush_only = false, std::vector<Vec3>* force_workspace = nullptr, std::vector<Vec3>* torque_workspace = nullptr, bool dynamics_only = false, std::vector<double>* tire_relaxation_rates = nullptr, std::vector<double>* tire_deflections = nullptr);

bool external_force_directional( const Model& model, const State& state, const SampleInput& input, const DirectionalState& direction, std::vector<double>& generalized_force, std::vector<double>& tire_state_derivatives, bool brush_only = false, const StaticContactOverride* static_contact = nullptr, DirectionalForceScratch* scratch = nullptr );

bool add_vehicle_steering_actuators( const VehicleInput& input, Model& model, std::string& error );

bool add_driven_coordinates( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_aerodynamic_drags( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_road_profile( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_static_rotation_gauges( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_tire_frames( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_tire_models( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_drive_torque_mappings( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_spring_curves( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_bushing_curves( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_bushing_rotation_coordinates( const VehicleInput& input, Model& model, std::string& error );
} // namespace axle_kernel
