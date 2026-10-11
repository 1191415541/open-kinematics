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
#include "mb_linearization/types.hpp"
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

/// One coordinate-driven column of the frozen force tangent.
///
/// The question is: with the mechanism held where a dynamic sample left it, what
/// external force must be applied at the *port* coordinates to hold a prescribed
/// displacement there?  That is a different question from the free tangent
/// `assemble_physical_tangent` returns -- the free tangent says how the residual
/// changes when *nothing* is held, while this says what the holding force is.
///
/// It lives here rather than in `mb_linearization` because *which coordinates are
/// held* is a solver policy, not physics: the assembly module owns the force-law
/// derivative and the `d(J^T lambda)` preload term and deliberately knows nothing
/// about gauges, held sets or ports.  The solve also needs `mb_linear`, which the
/// assembly module sits below and must not reach up into.
///
/// ``tangent`` is the assembled `A = dR/dx` of the same working sample, laid out
/// exactly as `assemble_physical_tangent` writes it: `[pose | multipliers | contact
/// compressions]`, `dimension = ndof + rows + active_tires.size()`.  The port
/// coordinates are pose coordinates, so their indices are `0 .. ndof-1`.
///
/// The solve is a genuine partitioned one, not a matrix inverse.  The driven
/// coordinates are held at their prescribed increments (a Dirichlet condition), the
/// remaining pose and multiplier coordinates are solved from the equilibrium of the
/// *free* block, and the reaction is read as the residual the held rows would
/// otherwise carry:
///
///     delta = [ds ; dq_free],   Q_s = (A * delta)[driven rows]
///
/// which is the external force the port must supply.  It is work conjugate by
/// construction: `ds^T Q_s` is exactly the work the driven coordinates do against
/// the internal force, so no stiffness-like rescaling is applied afterwards.  The
/// sign convention is *external force required to hold the displacement*, so a
/// restoring (spring-like) mechanism reports a **negative** `Q_s` for a positive
/// `ds`; a caller wanting a restoring stiffness negates it.  The convention is
/// stated once here and assumed nowhere else, so it cannot be flipped twice.
///
/// `d(J^T lambda*)` is already inside ``tangent`` -- `assemble_physical_tangent`
/// builds it from the sample's multipliers -- so a loaded linkage's tangent correctly
/// depends on where it is loaded.  Nothing here re-derives or freezes it.
///
/// ``prescribed`` holds one increment per driven coordinate: the column of `K` being
/// computed.  ``reaction`` is written as `driven.size()` values in the order given.
/// Returns ``false`` -- writing nothing -- when the inputs are inconsistent (a driven
/// index outside the pose block, a repeated driven index, a length disagreement) or
/// when the free block cannot be factorized.  A refusal is reported rather than
/// folded into a zero: a zero reaction is a legitimate answer for a mechanism with no
/// stiffness and must not double as the failure signal.
bool frozen_force_tangent_column(
    const Model& model,
    const WorkingSample& working,
    const std::vector<double>& tangent,
    const std::vector<int>& driven,
    const std::vector<double>& prescribed,
    std::vector<double>& reaction
);

/// ``B_w = -dR/dw`` for one unit wrench component on one body: a length-``ndof``
/// column of the residual derivative with respect to the load.
///
/// ``component`` indexes the ordered ``[Fx, Fy, Fz, Mx, My, Mz]`` wrench, matching
/// ``body_wrench``'s own layout, so the caller that named the load and the column
/// that is assembled agree on which physical quantity a unit means.
///
/// The column is analytic.  The load enters the mechanical residual once and
/// linearly, so this is the exact derivative rather than a finite difference of the
/// solver's response -- the design refuses the latter (SPEC :439).  A force applied
/// at a declared ``body_wrench_markers`` point also makes a pose-dependent moment
/// ``cross(R*p, F)`` (``force/layout.cpp:172``), so ``pose`` participates: the
/// derivative is taken at the base point being solved for, which is the geometric
/// term the design requires (SPEC :35, :42).
///
/// Returns a **zero** column -- not an error -- when the request cannot move the
/// mechanism: a fixed body has no free coordinate, and an out-of-range component or
/// body is reported the same way.  That is deliberate.  "This load direction does
/// nothing" and "this load direction is impossible" are different facts, and only
/// the caller's compatibility audit can tell them apart from the assembled system;
/// folding them together here would turn a legitimate zero response into a refusal.
std::vector<double> frozen_load_column(
    const Model& model,
    const State& pose,
    int body,
    int component
);

bool normalized_static_constraint_matrix( const Model& model, const State& state, std::vector<double>& matrix, std::vector<double>& row_scale, int& row_count );

bool static_tangent_projection( const Model& model, const State& state, const std::vector<double>& pose_residual, std::vector<double>& projected, double& projected_norm );

bool project_static_pose( const Model& model, const State& base, std::vector<double>& pose_increment, double tolerance, const SampleInput* sample );

bool solve_static_least_squares( const std::vector<double>& matrix, const std::vector<double>& rhs, int dimension, std::vector<double>& solution );

bool static_manifold_relaxation_step( const Model& model, const State& base, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, const std::vector<int>& active_tires, const std::vector<double>& residual, std::vector<double>& x, double tolerance, double load_scale );

bool static_global_contact_pretrim( const Model& model, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, double external_load_scale, State& base );

bool static_trim(const Model& model, const AxleInput& input, State& state, double& force_residual, double& position_residual, int& iterations, std::vector<double>& constraint_multiplier, int& pinned_directions, int& worst_force_coordinate, double& worst_force_value);

} // namespace axle_kernel
