#pragma once

/// The physical tangent assembly of the `mb_linearization` module.
///
/// `mb_linearization` owns *one* thing: the assembly of a physical working
/// sample's derivatives -- the force-law Jacobian, the constraint Jacobian and
/// the `d(J^T lambda)` preload geometry term -- and the solves over that
/// assembly.  It is deliberately below the solver: `mb_solve_static` and
/// `mb_solve_dynamic` call down into it, and it never calls back up.
///
/// What stays in the solver, and must not move here: the Newton increment
/// parameterisation, the gauge/pin policy, the active-set choice, the line-search
/// step coefficients, and the residual scaling.  Those are *solver strategies*:
/// two solvers may legitimately disagree about them while agreeing about physics,
/// so putting them in a shared module would freeze one solver's choice into the
/// other's contract.  The split is therefore "physics here, policy there".
///
/// The declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_input/types.hpp"

namespace axle_kernel {

/// Which coordinates the working sample holds fixed, and by what.
///
/// A gauge row is a *declared* coordinate rather than a physical constraint: the
/// static problem is rank-deficient by the linkage's rigid-body modes, and the
/// gauge removes them by naming coordinates to hold.  Carrying it as data (rather
/// than re-deriving it per call) is what lets a caller replay one assembly with
/// two different gauges and see that the *physics* did not change.
struct WorkingSample {
    /// Per-coordinate flag: non-zero means this pose coordinate is a gauge.
    std::vector<unsigned char> gauge;
    /// The active tire set, in the order the sample's contact columns use.
    std::vector<int> active_tires;
    /// Load scaling applied to the external force bus.
    double load_scale = 1.0;
    /// The physical pose the assembly is taken at.
    ///
    /// It arrives as *data* rather than being derived here, because deriving it
    /// means applying an increment through the pose parameterisation, and that
    /// parameterisation is the solver's own convention (left-SO(3) increments for
    /// static, a different map for the dynamic unknown).  A shared module that
    /// picked one would silently impose it on the other.
    State pose;
    /// `d(position_constraint)/d(pose)`, `rows x ndof`, and the set of rows whose
    /// position part participates.  Both belong to the static solver's notion of
    /// which rows are *position* constraints (a coupler row is, a force row is
    /// not), so the solver supplies them instead of this module re-deriving a
    /// mask that could disagree with the residual the solver is actually driving.
    std::vector<double> position_constraint;
    // World wrench at each material COM, in free-body generalized order. The
    // complete m*a and I_world*alpha + omega x I_world*omega are evaluated once
    // at the original sample. They never rotate/recompute with a pose variation.
    bool frozen_inertial_load = false;
    std::vector<double> inertial_wrench;
};

/// Assemble the physical tangent `K` of one working sample.
///
/// `jacobian` is filled as a dense `dimension x dimension` matrix with
/// `dimension = physical_tangent_dimension(model, working)`:
///
///   `ndof + rows + active_tires.size()` for an ordinary sample, and
///   `ndof + rows` when `working.frozen_inertial_load` is set -- a frozen dynamic
///   sample brings its own contact state, so it has no static compression
///   unknowns to solve for.  Callers must use that function rather than the sum:
///   the two forms differ, and a mismatch is refused by `assemble_physical_tangent`.
///
/// For the ordinary form the block order is
///
///   [ pose coordinates | constraint multipliers | contact compressions ]
///
/// which is the order the solver's increment vector already uses.  The pose block
/// is `-d(force)/d(pose) - sum_r lambda_r d(J_r)/d(pose)`: the first term is the
/// material tangent and the second is the *preload geometry* term, which is
/// non-zero whenever a constraint carries load and the constraint direction
/// changes with pose.  Dropping that second term is the classic way to get a
/// tangent that converges for a four-bar and diverges for a loaded wishbone.
///
/// Returns `false` when any directional evaluation reports a non-smooth force
/// law or the assembled matrix is not finite.  A `false` return is a *reported*
/// refusal, not a silent one: the matrix is still filled, and callers that have a
/// fallback (a finite-difference column, a smaller step) are expected to use it.
bool assemble_physical_tangent(
    const Model& model,
    const SampleInput& sample,
    const WorkingSample& working,
    const std::vector<double>& x,
    std::vector<double>& jacobian
);

/// The pose block of the physical tangent alone, without the multiplier or
/// contact columns, for callers that only need the stiffness they act on.
///
/// This exists so a caller cannot get a *nearly* right stiffness by taking a
/// sub-block of the full tangent and forgetting that the preload term is part of
/// it: the term is assembled here too.
bool assemble_pose_tangent(
    const Model& model,
    const SampleInput& sample,
    const WorkingSample& working,
    const std::vector<double>& x,
    std::vector<double>& pose_block
);

/// Number of rows/columns `assemble_physical_tangent` produces for this sample.
int physical_tangent_dimension(const Model& model, const WorkingSample& working);

/// Assemble the working sample of one **frozen** state.
///
/// A frozen state is a state that is not being solved for: the dynamic integrator
/// already produced it, and the question is what the mechanism would do locally if
/// the load were changed with the state held where it is.  That is a different
/// question from the static solver's, but it needs the same three policy values
/// the static path supplies, and this function supplies them from the *model*
/// rather than from an iteration, so no solver's convention is imported:
///
/// * ``gauge`` is `static_gauge_coordinates(model)` -- a pure function of the
///   model's declared gauge, which the contract installs before the first run and
///   every case shares.  A model that declares no gauge yields an all-zero gauge,
///   which is reported rather than defaulted.
/// * ``position_constraint`` is `constraint_jacobian(model, pose)`, unmasked.  The
///   static path passes the same matrix through `static_position_constraint_jacobian`,
///   which clears the rows `static_position_constraint_mask` selects; that mask is
///   built all-ones and its only write re-sets a row to the 1 it already holds, so no
///   row is ever cleared and the masked matrix *is* the plain one.  Reproducing the
///   mask here would be a second copy of an identity.
/// * ``load_scale`` is 1.0: a frozen response is asked at the unperturbed load.
///
/// ``r/q/v/omega/a/alpha`` must describe the complete original state. The inertial
/// COM wrench is frozen here, including gyroscopic torque. The force bus's live
/// gyroscopic derivative is removed from the frozen tangent, while velocity and
/// all internal State vectors remain those of this original sample.
///
/// Returns ``false`` when the frozen state cannot be described -- a body count that
/// disagrees with the model, a non-finite pose or velocity, or model dimensions that
/// do not admit a constraint Jacobian.  Every such test runs *before* ``working`` is
/// written, so a refusal leaves the caller's sample exactly as it was; a partially
/// filled refusal would leave the previous state's matrix in place with a size that
/// still matches the model, and the assembler would accept it.
bool frozen_working_sample(
    const Model& model,
    const State& pose,
    const std::vector<int>& active_tires,
    WorkingSample& working
);

} // namespace axle_kernel
