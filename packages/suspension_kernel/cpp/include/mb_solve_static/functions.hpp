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

std::vector<double> static_residual( const Model& model, const State& base, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, const std::vector<int>& active_tires, const std::vector<double>& x, double& force_residual, double& position_residual, int* worst_force_coordinate = nullptr, double* worst_force_value = nullptr, double load_scale = 1.0, const std::vector<double>* frozen_inertial_wrench = nullptr );

/// Evaluate the nonlinear mechanical residual with the original COM inertia
/// wrench held fixed. This audits a sample; it never equilibrates or advances it.
/// A malformed checkpoint is refused without changing the output arguments.
bool frozen_inertial_residual(const Model& model, const SampleInput& sample,
    const WorkingSample& working, const std::vector<double>& x,
    double gravity_x, double gravity_y, double gravity_z,
    std::vector<double>& residual, double& force_residual,
    double& position_residual);

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

/// One *named* load mode: the wrench amplitudes of `P[:,j]`.
///
/// A mode is data, not a direction the kernel invents.  ``amplitudes`` holds six
/// entries per body in body order, laid out as `[Fx, Fy, Fz, Mx, My, Mz]` -- the
/// same order ``body_wrench`` itself uses, which is why the caller that named the
/// load and the column ``frozen_load_column`` assembles cannot disagree about what
/// a unit means.  A mixed mode (force and moment, or two bodies at once) is
/// exactly what this representation exists for: the design's ``heave_load`` is two
/// wheels at once and the Adams unit basis lives in the same space.
struct FrozenLoadMode {
    std::string id;
    /// `body_count * 6`, in body order and `[Fx,Fy,Fz,Mx,My,Mz]` order.
    std::vector<double> amplitudes;
};

/// One *named* measurement: a linear functional of the pose increment.
///
/// The gradient is written here rather than derived from a name.  The contract
/// bans recovering a metric or a frame from an entity name
/// (`mb_cases/functions.hpp:542-545`), so the caller declares the blade of the
/// functional -- which body, which world axis, which reference body, and which
/// material point -- and `find_marker` (or the caller's own reading) supplies the
/// entity.  No prefix or suffix is ever parsed.
///
/// The functional is the exact linearisation of the solver's own increment map
/// `pose_candidate` (`solve_dynamic/kernel_integrator_state.cpp:39-55`):
/// `r += dr` with `dr` a **world** translation, and `q = exp(dtheta) * q` with
/// `dtheta` a **world** rotation about the body origin.  A material point at
/// body-local `point` therefore moves by `dr + dtheta x (R(q) point)`, whose
/// projection on a world axis `e` is `dr.e + dtheta.(a x e)` with
/// `a = R(q) point`.  Writing "the wheel-centre vertical displacement relative to
/// the chassis" as this functional is what keeps the answer tied to the
/// parameterisation the solver actually uses, rather than to a second, plausible
/// one invented at the measurement site.
struct FrozenMeasurement {
    std::string id;
    std::string units;
    /// Primary entity: the body whose point or attitude is measured.
    int body = -1;
    /// `0..2` for a translation along world `x/y/z`, `3..5` for a rotation about
    /// world `x/y/z` (the small rotation of the relative attitude, in rad).
    int component = -1;
    /// Reference body whose increment is subtracted, or `-1` for the world.
    int reference = -1;
    /// Body-local point the translation is taken at.  Unused for a rotation, and
    /// also unused when the body declares no point -- the origin is then the
    /// honest reading, and a caller that meant a marker must say so.
    Vec3 point{};
    bool has_point = false;
    /// Body-local point of the *reference* body whose increment is subtracted.
    /// Defaults to the reference body origin, which is the `declared_frame_origin`
    /// reading of `boundary-profiles.json`'s `measurement_reference`.
    Vec3 reference_point{};
    bool has_reference_point = false;
};

/// Assemble the measurement gradient `G = dy/dx`: `measurements.size() x dimension`,
/// row major, one row per measurement in the order given, with
/// `dimension = physical_tangent_dimension(model, working)`.
///
/// The width is the **same** width the unknown `X` and the right-hand side have,
/// not `ndof`.  A measurement reads pose coordinates only, so its multiplier and
/// contact columns are exactly zero -- but they are *present*, because the
/// consumer solves `Y = G X` against the same vector `A X = B_w P` produced.
/// A narrower `G` would refuse there (`results/equilibrium.py` requires
/// `gradient.shape[1] == dimension`), and a caller that padded it itself would be
/// re-deriving this function's layout outside this module.  `working` is taken for
/// that width alone; it is not read for anything else.
///
/// Returns `false` -- writing nothing -- when a measurement names an unknown or
/// fixed body, or a component outside `0..5`.  A refusal is reported rather than
/// folded into a zero row: an all-zero row would make the measurement read as
/// "this mechanism does not move there", which is a legitimate physical answer and
/// must not double as the failure signal.
bool frozen_measurement_gradient(
    const Model& model,
    const WorkingSample& working,
    const std::vector<FrozenMeasurement>& measurements,
    std::vector<double>& gradient
);

/// The per-request validity audit of a load-driven frozen response.
///
/// `frozen-protocols.json` `validity_conditions` names two checks and this
/// structure is where their answers are published, per (mode, measurement)
/// rather than as one sample-level flag:
///
///   * `singular` -- the mode's **actual** right-hand side `B_w P[:,j]` is not in
///     the range of `A`.  The check is on the assembled right-hand side and not
///     on the individual unit columns, which is what makes the design's
///     `combined_mode_rule` hold: a combination can be compatible while its own
///     columns are not, and it then reports normally.
///   * `unidentifiable` -- a measurement is not invariant on the null space that
///     remains, so the load does not determine the motion it is reading.  The
///     null space used is the one that survives the mode's own incompatibility.
///
/// The two are reported separately because they are different facts: an
/// incompatible mode has no solution at all, while an unidentifiable one has a
/// solution whose measured projection is not unique.  Both **refuse** the mode in
/// `mode_ok` -- a number a caller can read but that no load determined is exactly
/// the "plausible wrong number" this project refuses to publish -- so the audit is
/// the only place the distinction survives.
///
/// `matrix_rank` is `A`'s rank, so `nullity = dimension - matrix_rank` is
/// observable too; a full-rank sample reports `nullity == 0` and every mode as
/// compatible and every measurement as identifiable by construction.
struct FrozenResponseAudit {
    int dimension = 0;
    int matrix_rank = 0;
    /// One flag per mode: the mode's right-hand side is in `A`'s range.
    std::vector<unsigned char> mode_compatible;
    /// One flag per measurement: the measurement annihilates the remaining null
    /// space, so its response is determined by the load.  The flag does not vary
    /// by mode -- the null space is a property of `A`, not of the right-hand
    /// side -- so a per-mode copy would only invite the two to disagree.
    std::vector<unsigned char> measurement_identifiable;
};

/// Solve the load-driven frozen response `A X = B_w P` and project it onto the
/// measurements, `Y = G X + D P`.
///
/// This is the *load-driven* half of P4a and it is deliberately not derived from
/// the coordinate-driven one: `frozen_force_tangent_column` prescribes a
/// displacement and reports the holding force, while this prescribes a load and
/// reports the displacement.  Neither boundary can be obtained from the other by
/// inverting a matrix (SPEC :37, design :422).
///
/// ``tangent`` is the assembled `A` of the same working sample, laid out exactly as
/// `assemble_physical_tangent` writes it, `dimension = physical_tangent_dimension`.
/// The load enters only the pose coordinates, so `B_w`'s columns are `ndof` long and
/// are padded with zeros over the multiplier and contact rows; the right-hand side
/// is `dimension` long for the same reason the unknown is (`static_residual` reads
/// the multiplier increment out of the same vector, `kernel_static_trim.cpp:38-50`).
///
/// `X` contains the multiplier increments as well as the pose increments.  Freezing
/// `delta lambda` would be a different physical statement (`frozen-protocols.json`
/// `lambda_rule`: "不冻结 delta lambda"), so nothing here pins it.
///
/// The factorization is built **once** for `A` and reused for every mode, and
/// ``factorizations_used`` reports the number actually spent so the design's
/// "assemble and decompose once per working sample" claim is measured rather than
/// asserted.  It counts **numeric** factorizations and excludes any symbolic or
/// pattern step, which no backend in this tree exposes separately
/// (`mb_linear/batch.hpp:58-67`).
///
/// ``response`` is `modes.size() * measurements.size()`, row major in
/// `[mode][measurement]`, and is initialised to **NaN** rather than zero: a mode that
/// is refused leaves NaN behind, because a zero-filled refusal is numerically
/// indistinguishable from a mechanism whose response genuinely is zero.
///
/// ``mode_ok`` is one flag per mode and **must be read**: it is the only place the
/// refusal is visible, and a caller that reads ``response`` alone will publish NaN as
/// if it were a number.  A mode is refused when it is incompatible with `A` or when
/// the response it produces is not finite.  The modes are solved one at a time so
/// that a mode which cannot be answered does not take its neighbours down with it --
/// `solve_many` abandons the whole batch on the first non-finite column
/// (`linear/kernel_linear_batch.cpp:161-167`), which would be the opposite of
/// "a compatible mode still reports".
///
/// ``audit`` publishes the compatibility and identifiability checks described on
/// ``FrozenResponseAudit``.  A measurement that is *not* identifiable refuses every
/// mode that reaches it: the solve would otherwise return one member of a family and
/// report it as *the* reading.  `D` is zero for every measurement this routine
/// declares -- each is a kinematic functional of the pose -- so a measurement that is
/// itself a function of the load cannot be expressed here and is a named gap rather
/// than a silent zero.
///
/// ``tangent`` must be the assembly at the **base point** (`x = 0`): the rotation
/// block of `assemble_physical_tangent` carries a left-Jacobian factor that is the
/// identity only there (`linearization/physical.cpp:39-53`), so a tangent assembled
/// at a non-zero increment describes a different coordinate map than `G` does.
///
/// Returns `false` -- writing nothing -- when the inputs are inconsistent, when the
/// request is empty, or when `A` cannot be factorized; a singular `A` means no mode
/// can be answered at all, which is a property of the sample rather than of one mode.
bool frozen_local_response(
    const Model& model,
    const WorkingSample& working,
    const std::vector<double>& tangent,
    const std::vector<FrozenLoadMode>& modes,
    const std::vector<FrozenMeasurement>& measurements,
    std::vector<double>& response,
    std::vector<unsigned char>& mode_ok,
    int& factorizations_used,
    FrozenResponseAudit& audit
);

bool normalized_static_constraint_matrix( const Model& model, const State& state, std::vector<double>& matrix, std::vector<double>& row_scale, int& row_count );

bool static_tangent_projection( const Model& model, const State& state, const std::vector<double>& pose_residual, std::vector<double>& projected, double& projected_norm );

bool project_static_pose( const Model& model, const State& base, std::vector<double>& pose_increment, double tolerance, const SampleInput* sample );

bool solve_static_least_squares( const std::vector<double>& matrix, const std::vector<double>& rhs, int dimension, std::vector<double>& solution );

bool static_manifold_relaxation_step( const Model& model, const State& base, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, const std::vector<int>& active_tires, const std::vector<double>& residual, std::vector<double>& x, double tolerance, double load_scale );

bool static_global_contact_pretrim( const Model& model, const SampleInput& sample, double gravity_x, double gravity_y, double gravity_z, double external_load_scale, State& base );

bool static_trim(const Model& model, const AxleInput& input, State& state, double& force_residual, double& position_residual, int& iterations, std::vector<double>& constraint_multiplier, int& pinned_directions, int& worst_force_coordinate, double& worst_force_value);

} // namespace axle_kernel
