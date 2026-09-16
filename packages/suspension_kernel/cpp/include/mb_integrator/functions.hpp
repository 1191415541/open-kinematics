#pragma once

/// The free functions of the `mb_integrator` module.
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
#include "mb_linalg/factorization_types.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include "mb_tire/pac2002/spin.hpp"
#include "mb_integrator/context.hpp"
#include "mb_constraint/types.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_tire/assembly.hpp"
#include "mb_tire/force_context.hpp"
#include "mb_base/functions.hpp"
#include "mb_base/prelude.hpp"
#include "mb_constraint/functions.hpp"
#include "mb_constraint/registry.hpp"
#include "mb_integrator/functions.hpp"
#include "mb_linalg/functions.hpp"
#include "mb_model/enums.hpp"
#include "mb_model/functions.hpp"
#include "mb_suspension/functions.hpp"
#include "mb_tire/brush/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_vehicle/functions.hpp"
#include "axle_kernel.hpp"
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire_state/functions.hpp"
#include "mb_constraint/functions.hpp"
#include "mb_tire/functions.hpp"
#include "mb_tire/common/functions.hpp"
#include "mb_tire/brush/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_vehicle/functions.hpp"
#include "mb_suspension/functions.hpp"
#include "mb_linalg/functions.hpp"

namespace axle_kernel {
void residual( const ResidualContext& ctx, const std::vector<double>& x, std::vector<double>& out, double& pos_res, double& vel_res, double& dyn_res, int& active, ResidualWorkspace& workspace, const PoseJacobians* pose_jacobians = nullptr );

void interpolate_input(const AxleInput& in, double t, SampleInput& out);

void interpolate_input( const Model& model, const AxleInput& in, double t, SampleInput& out );

std::vector<double> generalized_velocity(const Model& model, const State& state);

bool validate_initial_velocity( const Model& model, const AxleInput& input, const State& state, double tolerance, double& residual );

bool initialize_acceleration( const Model& model, const AxleInput& input, State& state, double& residual, std::vector<double>& constraint_multiplier );

double next_prescribed_input_breakpoint( const AxleInput& input, double time, double target );

void pose_candidate( const State& base, const Model& model, const std::vector<double>& dy, State& candidate );

void pose_candidate_pose_only( const State& base, const Model& model, const std::vector<double>& dy, State& candidate );

State pose_candidate(const State& base, const Model& model, const std::vector<double>& dy);

State state_from_unknown( const ResidualContext& ctx, const std::vector<double>& x, std::vector<double>& a_next, std::vector<double>& v_next, std::vector<double>& mu, std::vector<double>& lambda);

void state_from_unknown( const ResidualContext& ctx, const std::vector<double>& x, ResidualWorkspace& workspace );

void residual( const ResidualContext& ctx, const std::vector<double>& x, std::vector<double>& out, double& pos_res, double& vel_res, double& dyn_res, int& active, ResidualWorkspace& workspace, const PoseJacobians* pose_jacobians);

void fill_analytic_jacobian_columns( const ResidualContext& ctx, int dim, std::vector<double>& J, ResidualWorkspace& workspace, const PoseJacobians& pose_jacobians, std::vector<unsigned char>& analytic_columns );

bool analytic_newton_columns_match_runtime_difference( const ResidualContext& ctx, const std::vector<double>& x, const std::vector<double>& base_residual, const std::vector<double>& J, int dim, const std::vector<unsigned char>& analytic_columns, const PoseJacobians& pose_jacobians );

bool newton_step(const ResidualContext& ctx, std::vector<double>& x, double& pos, double& vel, double& dyn, int& active, int& iterations, const AxleInput& in, NewtonLinearizationCache* linearization_cache = nullptr );

bool finite_state(const Model& model, const State& state);

std::vector<double> initial_step_unknown( const Model& model, const AxleInput& input, double time, const State& state, double h, const std::vector<Vec3>* previous_acceleration = nullptr, const std::vector<Vec3>* previous_alpha = nullptr, const std::vector<double>* previous_constraint_multiplier = nullptr );

bool initialize_internal_derivatives( const Model& model, const AxleInput& input, State& state );

bool apply_brush_return_mapping( const Model& model, const AxleInput& input, const State& previous, double time, double h, double gamma_z, State& state );

bool solve_one_step( const Model& model, const AxleInput& input, const State& start, double time, double h, double alpha_m, double alpha_f, double beta, double gamma, double alpha_m_z, double alpha_f_z, double gamma_z, StepResult& result, PerformanceCounters* performance = nullptr, const std::vector<Vec3>* previous_acceleration = nullptr, const std::vector<Vec3>* previous_alpha = nullptr, const std::vector<double>* previous_constraint_multiplier = nullptr, NewtonLinearizationCache* linearization_cache = nullptr );

double scaled_scalar_error( double a, double b, double absolute_tolerance, double relative_tolerance );

double normalized_state_error( const Model& model, const AxleInput& input, const State& start, const State& full_step, const State& two_half_steps );

std::vector<double> contact_penetrations( const Model& model, const AxleInput& input, const State& state, double time );

std::vector<int> contact_modes( const Model& model, const AxleInput& input, const State& state, double time );

bool contact_transition( const Model& model, const AxleInput& input, const State& before, double before_time, const State& after, double after_time );

bool solve_event_localization_step( const Model& model, const AxleInput& input, const State& start, double time, double h, StepResult& result, int depth = 0 );

void append_contact_events( const Model& model, const AxleInput& input, const State& before, double before_time, const State& after, double after_time, std::vector<ContactEventRecord>& events );

bool write_contact_events( const std::vector<ContactEventRecord>& events, AxleOutput& output );

bool localize_contact_event( const Model& model, const AxleInput& input, const State& start, double time, double h, const StepResult& end_step, StepResult& event_step, double& event_h );

bool exceeds_tire_compression_limit( const Model& model, const AxleInput& input, const State& state, double time );
} // namespace axle_kernel
