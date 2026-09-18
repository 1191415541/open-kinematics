#pragma once

/// The free functions of the `mb_constraint` module.
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
#include "mb_linalg/factorization_types.hpp"
#include "mb_constraint/types.hpp"
#include "mb_base/prelude.hpp"
#include "mb_constraint/registry.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
double joint_coordinate_value( const Model& model, const State& state, int joint_index, int coordinate, double reference_translation, const Quat& reference_rotation );

Vec3 steering_axis_reference(const SteeringActuator& actuator);

bool prescribed_steering(const SteeringActuator& actuator);

Vec3 perpendicular_reference(const Vec3& axis);

double wrap_to_pi(double angle);

double driven_target_value(const Constraint& c, const SampleInput* input);

double driven_row_target_rate( const Model& model, int row, const SampleInput& input );

std::vector<double> constraint_residual( const Model& model, const State& state, const SampleInput* input = nullptr );

void perturb_pose(State& state, const Model& model, const std::vector<double>& dy, double scale);

std::vector<double> constraint_jacobian_central_difference( const Model& model, const State& state );

std::vector<double> constraint_jacobian(const Model& model, const State& state);

bool analytic_constraint_jacobian_matches_reference( const Model& model, const State& state, double& error );

bool constraint_jacobian_directional( const Model& model, const State& state, const DirectionalState& direction, std::vector<double>& derivative, std::vector<std::size_t>* touched_indices = nullptr );

void apply_acceleration(State& state, const Model& model, const std::vector<double>& a);

bool mass_inverse_of_jt_mu( const Model& model, const State& state, const std::vector<double>& J, const std::vector<double>& mu, int n, int m, std::vector<double>& out );

bool mass_inverse_jt_mu_directional( const Model& model, const State& state, const DirectionalState& direction, const std::vector<double>& dJ, const std::vector<double>& mu, const std::vector<double>& base_mass_inverse, int n, std::vector<double>& derivative );
} // namespace axle_kernel
