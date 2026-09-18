#pragma once

/// The free functions of the `abi` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_base/prelude.hpp"
#include "mb_input/types.hpp"
#include "core_abi.hpp"
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
#include "mb_energy/types.hpp"
#include "mb_linalg/factorization_types.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include "mb_tire/pac2002/spin.hpp"
#include "mb_integrator/context.hpp"
#include "mb_constraint/types.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_tire/assembly.hpp"
#include "mb_tire/force_context.hpp"
#include "mb_output/measurement.hpp"
#include "abi/version.hpp"
#include "mb_base/prelude.hpp"
#include "mb_constraint/registry.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
// Generic element surface (K7): reads element blocks into a model.  Declared
// here because the build path and the vehicle registration path both call it
// while it is defined in the model translation unit.
bool read_element_blocks(
    const ElementBlock* elements,
    std::size_t element_count,
    const ElementCurveReference* element_curves,
    std::size_t topology_extension_count,
    const TopologyExtensionBlock* topology_extensions,
    Model& model,
    std::string& error
);

Model build_model( const AxleInput& in, std::string& error, const double* axis_a_secondary = nullptr, const double* axis_b_secondary = nullptr, const double* convel_angle_target = nullptr, std::size_t coordinate_coupler_count = 0, const int* coordinate_coupler_joint_a = nullptr, const int* coordinate_coupler_coordinate_a = nullptr, const double* coordinate_coupler_scale_a = nullptr, const int* coordinate_coupler_joint_b = nullptr, const int* coordinate_coupler_coordinate_b = nullptr, const double* coordinate_coupler_scale_b = nullptr );

int run_model( const AxleInput* input, AxleOutput* output, char* error_buffer, std::size_t error_capacity, const Model* model_override );
} // namespace axle_kernel
