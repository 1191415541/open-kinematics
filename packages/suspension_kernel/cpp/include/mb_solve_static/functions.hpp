#pragma once

/// The free functions of the `mb_static` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"
#include "mb_config/diagnostics.hpp"
#include "mb_tire_state/tire_state.hpp"
#include "mb_config/env.hpp"
#include "mb_numeric/util.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_energy/types.hpp"
#include "mb_linear/factorization_types.hpp"
#include "mb_solve_dynamic/context.hpp"
#include "mb_joint/types.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_config/prelude.hpp"
#include "mb_joint/registry.hpp"
#include "mb_model/enums.hpp"
#include "mb_input/types.hpp"

namespace axle_kernel {

double static_rotation_gauge_value( const Model& model, const StaticRotationGauge& gauge, const std::vector<double>& pose_increment );

std::vector<double> static_residual( const Model& model, const State& base, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, const std::vector<int>& active_tires, const std::vector<double>& x, double& force_residual, double& position_residual, int* worst_force_coordinate = nullptr, double* worst_force_value = nullptr, double load_scale = 1.0 );

ConstraintResidualMaxima constraint_residual_maxima( const Model& model, const State& state, const SampleInput* input = nullptr );

double static_contact_tolerance(const Model& model);

int pin_null_pose_directions( std::vector<double>& jacobian, const std::vector<double>& residual, int dimension, int pose_dimension, double tolerance );

std::vector<int> static_gauge_coordinates(const Model& model);

std::vector<unsigned char> static_position_constraint_mask( const Model& model );

std::vector<int> static_position_constraint_rows(const Model& model);

std::vector<double> static_position_constraint_jacobian( const Model& model, const State& state );

bool static_jacobian( const Model& model, const State& base, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, const std::vector<int>& active_tires, const std::vector<double>& x, std::vector<double>& jacobian, double load_scale = 1.0 );

bool normalized_static_constraint_matrix( const Model& model, const State& state, std::vector<double>& matrix, std::vector<double>& row_scale, int& row_count );

bool static_tangent_projection( const Model& model, const State& state, const std::vector<double>& pose_residual, std::vector<double>& projected, double& projected_norm );

bool project_static_pose( const Model& model, const State& base, std::vector<double>& pose_increment, double tolerance, const SampleInput* sample );

bool solve_static_least_squares( const std::vector<double>& matrix, const std::vector<double>& rhs, int dimension, std::vector<double>& solution );

bool static_manifold_relaxation_step( const Model& model, const State& base, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, const std::vector<int>& active_tires, const std::vector<double>& residual, std::vector<double>& x, double tolerance, double load_scale );

bool static_global_contact_pretrim( const Model& model, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, double external_load_scale, State& base );

bool static_trim(const Model& model, const AxleInput& input, State& state, double& force_residual, double& position_residual, int& iterations, std::vector<double>& constraint_multiplier, int& pinned_directions, int& worst_force_coordinate, double& worst_force_value);

} // namespace axle_kernel
