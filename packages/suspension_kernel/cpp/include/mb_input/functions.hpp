#pragma once

/// The shared input-sampling functions of `mb_input` (subtask 03 step 6).
///
/// Interpolating the caller's sampled tables and finding the next prescribed
/// breakpoint is input preparation, not integration: the static solver, the
/// dynamic solver and the output layer all call these, so they live with the
/// input payload types rather than under either solver.

#include <cstddef>
#include <vector>

#include "mb_config/prelude.hpp"
#include "mb_config/constants.hpp"
#include "mb_model/types.hpp"
#include "mb_input/types.hpp"

namespace axle_kernel {

void interpolate_input(const AxleInput& in, double t, SampleInput& out);

void interpolate_input( const Model& model, const AxleInput& in, double t, SampleInput& out );

double next_prescribed_input_breakpoint( const AxleInput& in, double from_time, double to_time );

} // namespace axle_kernel
