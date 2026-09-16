#pragma once

/// The free functions of the `mb_output` module.
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
#include "mb_vehicle/energy.hpp"
#include "mb_integrator/context.hpp"
#include "mb_constraint/types.hpp"
#include "mb_output/measurement.hpp"
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_constraint/functions.hpp"
#include "mb_constraint/registry.hpp"
#include "mb_integrator/functions.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_output/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_vehicle/functions.hpp"
#include "axle_kernel.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_vehicle/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_constraint/functions.hpp"
#include "mb_integrator/functions.hpp"

namespace axle_kernel {
void copy_state(const State& s, const Model& m, double* out, std::size_t sample);

SteeringMeasurement measure_steering( const SteeringActuator& actuator, const State& state, const SampleInput& input, std::size_t index );

void write_vehicle_steering_output( const Model& model, const AxleInput& input, const AxleOutput& axle_output, VehicleOutput& output );

void write_constraint_wrenches( const Model& model, const State& state, const std::vector<double>& multiplier, double* output, std::size_t sample );

double kinetic_energy(const Model& model, const State& state);

bool accumulate_energy_step( const Model& model, const AxleInput& input, const State& start, const State& end, double time, double h, double alpha_f, const std::vector<double>& multiplier, EnergyInterval& interval );

void write_physics_output( const Model& model, const AxleInput& input, const State& state, double time, AxleOutput& output, std::size_t sample, double previous_energy, const EnergyInterval& interval, bool first );

double performance_seconds(const std::atomic<std::uint64_t>& nanoseconds);

void write_performance_metrics( const PerformanceCounters& counters, bool enabled, std::size_t sample_count, AxleOutput& output );
} // namespace axle_kernel
