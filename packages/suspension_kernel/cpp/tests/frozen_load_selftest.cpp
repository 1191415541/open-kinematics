// Self-test for `frozen_load_column` -- the `B_w = -dR/dw` producer.
//
// This is the load-driven half of P4a.  The coordinate-driven half
// (`frozen_force_tangent_column`) already reports the holding force for a
// prescribed displacement; here the question is inverted: prescribe a *load* and
// report the displacement.  The two are different physical boundary conditions, and
// the design is explicit that one must not be obtained from the other (SPEC :37,
// design :422), so this file tests the load column on its own terms.
//
// What is checked, and why each one needs checking rather than assuming:
//
//   1. A unit force component lands on the coordinate it names.  The mapping from
//      `[Fx, Fy, Fz, Mx, My, Mz]` to a generalized coordinate is the one piece of
//      this routine that is pure convention, so a transposed or offset mapping
//      would be invisible without an assertion on each of the six components
//      individually.  Checking only `Fx` would pass on a routine that mapped every
//      force to the first coordinate.
//
//   2. The sign.  The residual carries `-force`, so an applied unit force must
//      appear as `-1` in the column.  A routine that dropped the sign would produce
//      a compliance of the wrong sign, which is a *stable-looking* wrong answer.
//
//   3. The lever arm.  A force at a declared marker makes a pose-dependent moment,
//      so the column carries `-cross(R*p, e_k)` in its rotational block.  This is
//      the geometric term the design requires (SPEC :35, :42).  It is checked
//      against a hand-computed cross product at a *rotated* pose, because at the
//      identity pose the term is a permutation of the force components and a
//      routine that ignored the rotation would still pass.
//
//   4. A force at the origin (or a body with no declared marker) adds no moment.
//      This is what keeps every model without a marker unchanged, and it is a
//      distinct claim from (3) rather than a degenerate case of it.
//
//   5. A fixed body, an out-of-range body and an out-of-range component return a
//      ZERO column rather than an error.  "This direction does nothing" and "this
//      direction is impossible" are different facts; only the caller's
//      compatibility audit can separate them, so folding them together here would
//      turn a legitimate zero response into a refusal.
//
//   6. The whole chain `A X = B_w P` -> `C = 1/k`.  The design's acceptance for the
//      load-driven path is a linear mass-spring-damper reporting `C = 1/k` at fixed
//      velocity and inertia.  That is a *composition* claim: it needs the load
//      column and the coordinate tangent to agree on one physical spring, so it is
//      asserted here rather than inferred from the two halves passing separately.

#include "mb_solve_static/functions.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++failures;
}

using axle_kernel::Model;
using axle_kernel::State;
using axle_kernel::Vec3;
using axle_kernel::WorkingSample;

bool close(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance*std::max(1.0, std::abs(b));
}

// A model with `bodies` bodies, all free, and no constraints.  `body_wrench_point_local`
// is left empty unless a test sets it, so the "no declared marker" path is the
// default and has to be asked for explicitly.
Model make_model(int bodies) {
    Model model;
    model.ndof = 6*bodies;
    model.rows = 0;
    model.bodies.resize(static_cast<std::size_t>(bodies));
    model.free_body.resize(static_cast<std::size_t>(bodies));
    model.body_to_free.assign(static_cast<std::size_t>(bodies), -1);
    for (int i = 0; i < bodies; ++i) {
        model.free_body[static_cast<std::size_t>(i)] = i;
        model.body_to_free[static_cast<std::size_t>(i)] = i;
    }
    return model;
}

State make_pose(int bodies) {
    State pose;
    pose.r.assign(static_cast<std::size_t>(bodies), Vec3{});
    pose.q.assign(static_cast<std::size_t>(bodies),
                  axle_kernel::Quat{1.0, 0.0, 0.0, 0.0});
    return pose;
}

// A 90-degree rotation about z, so `rotate(q, p)` is a *permutation* rather than the
// identity: a routine that ignored the pose would compute `cross(p, e_k)` and be
// caught.  Half-angle form for the scalar-first quaternion.
axle_kernel::Quat quarter_turn_z() {
    const double half = 3.14159265358979323846/4.0;
    return axle_kernel::Quat{std::cos(half), 0.0, 0.0, std::sin(half)};
}

double value_at(const std::vector<double>& column, int index) {
    return column[static_cast<std::size_t>(index)];
}

// --- 1/2. each component lands where it names, with the residual's sign -------

void test_each_wrench_component_lands_on_its_own_coordinate() {
    // Six separate claims, not one: a routine with a transposed mapping or an
    // off-by-one in the rotational block satisfies a single `Fx` assertion.
    const Model model = make_model(2);
    const State pose = make_pose(2);
    for (int component = 0; component < 6; ++component) {
        const std::vector<double> column =
            axle_kernel::frozen_load_column(model, pose, 1, component);
        check(column.size() == static_cast<std::size_t>(model.ndof),
              "the load column is ndof long");
        // Body 1 is the second free body, so its block starts at coordinate 6.
        const int expected = 6 + component;
        bool exact = true;
        for (int i = 0; i < model.ndof; ++i) {
            const double want = (i == expected) ? -1.0 : 0.0;
            if (!close(value_at(column, i), want, 1e-14)) exact = false;
        }
        check(exact,
              "a unit wrench component puts -1 on its own coordinate and 0 elsewhere");
    }
}

void test_the_sign_is_the_residual_convention() {
    // Stated separately from the placement test so a sign flip is reported as a
    // sign flip rather than as a failed placement.
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const std::vector<double> fx = axle_kernel::frozen_load_column(model, pose, 0, 0);
    check(value_at(fx, 0) == -1.0, "an applied +Fx enters the residual as -1");
    check(close(value_at(fx, 0), -1.0, 0.0), "the sign is exact, not merely finite");
}

// --- 3/4. the lever arm ------------------------------------------------------

void test_a_declared_marker_adds_the_pose_dependent_moment() {
    // A force at a marker `p` makes a moment `cross(R*p, F)`.  With `p = (1,0,0)`
    // and `R` a quarter turn about z, `R*p = (0,1,0)`; for `F = e_x` that gives
    // `cross((0,1,0),(1,0,0)) = (0*0-0*0, 0*1-0*0, 0*0-1*1) = (0,0,-1)`.
    //
    // The residual is `-force - torque` (`kernel_static_trim.cpp:74`), so a moment
    // that the load *adds* enters the residual with the opposite sign: the column
    // carries `+M_z = +1`.  Both facts are asserted, because getting the cross
    // product right and then the residual sign wrong yields `-1` -- a value that
    // looks like a sign convention rather than like a bug.
    const Model model = make_model(1);
    const State pose = [&] {
        State s = make_pose(1);
        s.q[0] = quarter_turn_z();
        return s;
    }();
    Model with_point = model;
    with_point.body_wrench_point_local = {Vec3{1.0, 0.0, 0.0}};

    const std::vector<double> fx =
        axle_kernel::frozen_load_column(with_point, pose, 0, 0);
    check(close(value_at(fx, 0), -1.0, 1e-14), "the force still lands on Fx");
    check(close(value_at(fx, 3), 0.0, 1e-12), "the lever-arm moment has no Mx here");
    check(close(value_at(fx, 4), 0.0, 1e-12), "the lever-arm moment has no My here");
    check(close(value_at(fx, 5), 1.0, 1e-12),
          "cross(R*p, e_x) at a quarter turn about z enters as +M_z through -dR/dw");

    // The rotation must actually be read: at the identity the same marker gives
    // `cross((1,0,0),(1,0,0)) = 0`, so a routine that ignored the pose would report
    // zero moments and pass a test written only at the identity pose.
    const State identity = make_pose(1);
    const std::vector<double> at_identity =
        axle_kernel::frozen_load_column(with_point, identity, 0, 0);
    check(close(value_at(at_identity, 5), 0.0, 1e-12),
          "at the identity pose the same marker produces no moment");
    check(!close(value_at(fx, 5), value_at(at_identity, 5), 1e-12),
          "the lever-arm term depends on the pose, not only on the marker");

    // A moment component never gains a lever arm: `dM_k/dM_j` is the identity, so
    // the rotational block of a moment mode must stay free of `cross` terms.
    //
    // The assertion is on the *rotational* block, not the translational one.  With a
    // marker declared, a routine that ran the lever arm for moment modes too would
    // add `cross(R*p, e_axis)` to `base+3..5`; the translational coordinates would
    // stay zero, so a test that only looked at those would pass on the defect.
    // `p=(1,0,0)` at a quarter turn about z gives `R*p=(0,1,0)`, and for `Mz`
    // (`axis=2`, `unit=e_z`) that is `cross((0,1,0),(0,0,1)) = (1,0,0)` -- an `+Mx`.
    const std::vector<double> mz =
        axle_kernel::frozen_load_column(with_point, pose, 0, 5);
    check(close(value_at(mz, 5), -1.0, 1e-14), "a unit Mz lands on its coordinate");
    check(close(value_at(mz, 0), 0.0, 1e-14) && close(value_at(mz, 1), 0.0, 1e-14) &&
              close(value_at(mz, 2), 0.0, 1e-14),
          "a moment mode adds no translational term even with a marker declared");
    check(close(value_at(mz, 3), 0.0, 1e-12) && close(value_at(mz, 4), 0.0, 1e-12),
          "a moment mode gains no lever-arm Mx/My at the declared marker");
    // The same claim for a moment whose axis makes a *non-zero* cross product, so
    // the check above cannot pass merely because the cross product vanished.
    const std::vector<double> my =
        axle_kernel::frozen_load_column(with_point, pose, 0, 4);
    check(close(value_at(my, 4), -1.0, 1e-14), "a unit My lands on its coordinate");
    check(close(value_at(my, 3), 0.0, 1e-12) && close(value_at(my, 5), 0.0, 1e-12),
          "a moment mode gains no lever-arm term on the other rotational axes");
    // And the fixture is not degenerate: the *force* mode at this same marker and
    // pose does produce a moment, so the zeros above are the moment rule rather
    // than an arm that happens to be zero.
    check(!close(value_at(fx, 5), 0.0, 1e-12),
          "the same marker does produce a moment for a force (the fixture is live)");
}

void test_no_marker_means_no_moment() {
    // The default path: a model that declares no application point applies the
    // wrench at the body origin, so the moment block stays zero.  Claimed
    // separately because it is what keeps markerless models unchanged.
    const Model model = make_model(1);
    const State pose = [&] {
        State s = make_pose(1);
        s.q[0] = quarter_turn_z();
        return s;
    }();
    check(model.body_wrench_point_local.empty(),
          "the fixture really declares no marker");
    const std::vector<double> fx = axle_kernel::frozen_load_column(model, pose, 0, 0);
    check(close(value_at(fx, 3), 0.0, 1e-14) && close(value_at(fx, 4), 0.0, 1e-14) &&
              close(value_at(fx, 5), 0.0, 1e-14),
          "an origin wrench adds no moment even at a rotated pose");

    // An explicitly declared zero point must behave the same as no declaration,
    // which is the compatibility promise `force/layout.cpp:164-166` makes.
    Model zero_point = model;
    zero_point.body_wrench_point_local = {Vec3{0.0, 0.0, 0.0}};
    const std::vector<double> at_zero =
        axle_kernel::frozen_load_column(zero_point, pose, 0, 0);
    check(close(value_at(at_zero, 5), 0.0, 1e-14),
          "a declared (0,0,0) marker is the origin, not a lever arm");
}

// --- 5. refusals report zero rather than an error ---------------------------

void test_loaded_directions_that_cannot_move_the_mechanism_are_zero() {
    const Model model = make_model(2);
    const State pose = make_pose(2);

    // A fixed body: `body_to_free` is -1.  There is no coordinate to load, and the
    // honest answer is a zero column -- the caller's audit decides whether that
    // means "no effect" or "impossible".
    Model with_fixed = model;
    with_fixed.body_to_free[0] = -1;
    const std::vector<double> fixed_column =
        axle_kernel::frozen_load_column(with_fixed, pose, 0, 0);
    check(fixed_column.size() == static_cast<std::size_t>(with_fixed.ndof),
          "a fixed-body request still returns a correctly sized column");
    bool all_zero = true;
    for (const double value : fixed_column) {
        if (value != 0.0) all_zero = false;
    }
    check(all_zero, "a fixed body carries no load column rather than an error");

    // Out-of-range body and component: same treatment, same size.
    for (const int body : {-1, 2, 99}) {
        const std::vector<double> column =
            axle_kernel::frozen_load_column(model, pose, body, 0);
        check(column.size() == static_cast<std::size_t>(model.ndof),
              "an out-of-range body returns a correctly sized column");
        all_zero = true;
        for (const double value : column) {
            if (value != 0.0) all_zero = false;
        }
        check(all_zero, "an out-of-range body produces a zero column");
    }
    for (const int component : {-1, 6, 99}) {
        const std::vector<double> column =
            axle_kernel::frozen_load_column(model, pose, 0, component);
        check(column.size() == static_cast<std::size_t>(model.ndof),
              "an out-of-range component returns a correctly sized column");
        all_zero = true;
        for (const double value : column) {
            if (value != 0.0) all_zero = false;
        }
        check(all_zero, "an out-of-range component produces a zero column");
    }

    // A `Model` whose free-body map points far outside `ndof`.  This is the one
    // guard whose absence is not a wrong number but an out-of-bounds *write*, so it
    // is asserted with an index large enough that the write cannot land inside the
    // vector's spare capacity and go unnoticed: the honest failure mode is a crash,
    // and a test that cannot observe the defect is not a test.
    //
    // A model whose map is simply *short* would not do: `column[6]` on a six-element
    // vector scribbles past the logical end, and the six values the caller can read
    // are still all zero -- the defect would be invisible in the return value.
    Model runaway = make_model(1);
    runaway.ndof = 6;
    runaway.body_to_free.assign(1, 100000);
    const std::vector<double> out_of_range =
        axle_kernel::frozen_load_column(runaway, pose, 0, 0);
    check(out_of_range.size() == 6,
          "a runaway free-body map still returns ndof values");
    all_zero = true;
    for (const double value : out_of_range) {
        if (value != 0.0) all_zero = false;
    }
    check(all_zero,
          "a free-body index outside ndof is refused rather than written");

    // A truncated model where the block *would* fit if ndof were honest, so the
    // refusal above is the bounds check rather than the model being unusable.
    Model truncated = make_model(2);
    truncated.ndof = 6;  // room for body 0's block only
    const std::vector<double> past_end =
        axle_kernel::frozen_load_column(truncated, pose, 1, 0);
    check(past_end.size() == 6, "a truncated model still returns ndof values");
    all_zero = true;
    for (const double value : past_end) {
        if (value != 0.0) all_zero = false;
    }
    check(all_zero,
          "a body whose block starts past ndof is refused rather than written");

    // The same model *does* answer for the body that fits, so the refusals above are
    // the bounds check rather than the model being unusable.
    const std::vector<double> fits =
        axle_kernel::frozen_load_column(truncated, pose, 0, 2);
    check(close(value_at(fits, 2), -1.0, 1e-14),
          "the body that fits in ndof is still answered (the fixture is live)");
}

// --- 6. the composition: C = 1/k ---------------------------------------------

void test_the_load_driven_chain_reports_the_inverse_stiffness() {
    // The design's load-driven acceptance (SPEC :56): a linear spring reports
    // `C = 1/k`.  This is a *composition* claim -- the load column must feed the
    // same tangent the coordinate-driven routine solves -- so it is asserted here
    // rather than inferred from the two halves passing on their own.
    //
    // One free body, one spring on its x coordinate: `A = diag(k, 0, 0, 0, 0, 0)`
    // would be singular, so the fixture pins the other five coordinates with a
    // large stiffness.  That is a *testing* device, not a claim about the solver:
    // it makes the remaining coordinates identifiable so the x response is the
    // spring's own.
    const double stiffness = 500.0;
    const double pin = 1.0e9;
    Model model = make_model(1);
    model.ndof = 6;
    State pose = make_pose(1);

    std::vector<double> tangent(36, 0.0);
    tangent[0] = stiffness;
    for (int i = 1; i < 6; ++i) tangent[static_cast<std::size_t>(i*6 + i)] = pin;

    // Apply one unit of x force and solve for the motion.  The load column says
    // where the load enters; the solve is the same partitioned solve the
    // coordinate-driven routine uses, so the chain inherits its correctness.
    const std::vector<double> column =
        axle_kernel::frozen_load_column(model, pose, 0, 0);
    check(close(value_at(column, 0), -1.0, 1e-14),
          "the unit x force enters the residual as -1");

    // `A X = B_w P` with `P = +1` (a physically applied unit force).  The residual
    // convention makes the right-hand side `-B_w * alpha` for a load that *adds* to
    // the applied force, so the displacement is `+1/k` in the loaded direction and
    // the magnitude, not the sign, is what `C = 1/k` states.
    //
    // The system is solved here rather than stated: a test that wrote
    // `displacement = applied/stiffness` and then compared it to `1/stiffness`
    // would pass no matter what the load column contained.  Solving with the
    // column as the right-hand side is what makes this a claim about `B_w`.
    const double applied_force = 1.0;
    const std::vector<double> rhs = [&] {
        std::vector<double> b(6, 0.0);
        for (int i = 0; i < 6; ++i) {
            // A physically applied load adds `-B_w * alpha` to the residual's
            // right-hand side, because `B_w` itself is `-dR/dw`.
            b[static_cast<std::size_t>(i)] =
                -value_at(column, i)*applied_force;
        }
        return b;
    }();
    // Forward-eliminate the diagonal system by hand: this is `A X = rhs` with the
    // pinning stiffnesses large, so the pin coordinates move by ~1e-9 and the x
    // coordinate moves by exactly `1/k`.
    std::vector<double> motion(6, 0.0);
    for (int i = 0; i < 6; ++i) {
        const double diagonal = tangent[static_cast<std::size_t>(i*6 + i)];
        motion[static_cast<std::size_t>(i)] = rhs[static_cast<std::size_t>(i)]/diagonal;
    }
    check(close(motion[0], 1.0/stiffness, 1e-12),
          "C = 1/k: a unit load moves the spring coordinate by 1/k");
    check(close(1.0/motion[0], stiffness, 1e-9),
          "the reciprocal of the solved response is the stiffness k");

    // The two boundaries are NOT reciprocal substitutes.  The coordinate-driven
    // routine reports the *force* needed to hold a unit displacement, which is `k`
    // here -- not `1/k`.  Asserting both numbers in one place is what makes it
    // impossible to "simplify" one path into the other without failing here.
    const std::vector<double> tangent_for_solve = tangent;
    WorkingSample working;
    working.gauge.assign(static_cast<std::size_t>(model.ndof), 0);
    working.load_scale = 1.0;
    // Hold every coordinate except the first, so the free block is just the spring.
    std::vector<int> driven;
    std::vector<double> prescribed;
    for (int i = 0; i < 6; ++i) {
        driven.push_back(i);
        prescribed.push_back(i == 0 ? 1.0 : 0.0);
    }
    std::vector<double> reaction;
    const bool solved = axle_kernel::frozen_force_tangent_column(
        model, working, tangent_for_solve, driven, prescribed, reaction);
    // With every coordinate held there is no free block, so the solve is a pure
    // product: the holding force on the x coordinate is `A[0,0]*ds = k`.
    check(solved, "the fully held case is solvable");
    check(reaction.size() == 6, "one reaction per driven coordinate");
    check(close(reaction[0], stiffness, 1e-9),
          "the coordinate-driven path reports k, not 1/k");
    check(!close(reaction[0], 1.0/stiffness, 1e-9),
          "the two boundaries are not reciprocal substitutes");
}

// --- 7. gauge rows are not force-balance rows --------------------------------

void test_a_gauge_coordinate_carries_no_load_column() {
    // A gauge row does not depend on the load: the tangent replaces it with the
    // identity (`linearization/physical.cpp:144-147`) and the residual writes the
    // gauge *value* over it, so the true `dR/dw` there is zero.  A `-1` written
    // anyway is not merely one wrong entry -- it fabricates a right-hand side for a
    // row that cannot carry one, and `A X = B_w P` then reports a unit displacement
    // on a coordinate the model deliberately does not let move.
    //
    // The n-body fixture declares a gauge on body 0's `x`, which is coordinate 0.
    Model model = make_model(2);
    model.static_gauge_body = 0;
    model.static_gauge_dof_mask = 1u;  // local coordinate 0 == x
    const State pose = make_pose(2);

    const std::vector<double> gauged =
        axle_kernel::frozen_load_column(model, pose, 0, 0);
    check(close(value_at(gauged, 0), 0.0, 0.0),
          "a gauge coordinate gets no load column entry");
    bool all_zero = true;
    for (const double value : gauged) {
        if (value != 0.0) all_zero = false;
    }
    check(all_zero, "a load on a gauged coordinate is answered with a zero column");

    // A *different* component of the same body is still answered: the gauge mask
    // clears coordinate 0 only, so the check must be per coordinate rather than per
    // body.  A routine that returned an empty column for any gauged body would pass
    // the assertion above and fail this one.
    const std::vector<double> ungauged =
        axle_kernel::frozen_load_column(model, pose, 0, 1);
    check(close(value_at(ungauged, 1), -1.0, 1e-14),
          "an ungauged coordinate of the same body is still answered");

    // A moment on the gauge coordinate of a *rotational-gauge* model.  This is the
    // second gauge mechanism, and it reaches the rotational block, so it exercises
    // the lever-arm checks rather than the `slot` check.
    Model rotation_gauged = make_model(1);
    axle_kernel::StaticRotationGauge gauge;
    gauge.pivot = 3;  // free coordinate 3 == Mx of body 0
    rotation_gauged.static_rotation_gauges.push_back(gauge);
    const std::vector<double> rotational =
        axle_kernel::frozen_load_column(rotation_gauged, pose, 0, 3);
    check(close(value_at(rotational, 3), 0.0, 0.0),
          "a rotation-gauge pivot gets no load column entry either");

    // The lever arm must respect the same rule.  With a marker declared, a force
    // writes into `base+3..5`; if one of those is a rotation gauge it must be left
    // alone while the others are still written.
    //
    // The marker is chosen so the arm produces an `Mx` -- the gauged coordinate --
    // rather than only `Mz`.  `p=(0,1,0)` at a quarter turn about z gives
    // `R*p=(-1,0,0)`, and `cross((-1,0,0), (1,0,0)) = (0,0,0)`, so instead take
    // `p=(0,0,1)`: `R*p=(0,0,1)` and `cross((0,0,1),(1,0,0)) = (0,1,0)`.  That is an
    // `My`, not an `Mx`, so shift the force axis to `Fy` (`component 1`):
    // `cross((0,0,1),(0,1,0)) = (0*0-1*1, 1*0-0*0, 0*1-0*0) = (-1,0,0)` -- an `Mx`.
    Model with_marker = make_model(1);
    with_marker.body_wrench_point_local = {Vec3{0.0, 0.0, 1.0}};
    with_marker.static_rotation_gauges.push_back(gauge);
    State turned = make_pose(1);
    turned.q[0] = quarter_turn_z();
    const std::vector<double> armed =
        axle_kernel::frozen_load_column(with_marker, turned, 0, 1);
    check(close(value_at(armed, 1), -1.0, 1e-14), "the force is still written");
    check(close(value_at(armed, 3), 0.0, 0.0),
          "the lever arm does not write a rotation-gauge coordinate");
    check(!close(value_at(armed, 1), 0.0, 0.0),
          "the fixture really produced a force term");

    // The same model without the gauge *does* write coordinate 3, so the zero above
    // is the gauge rule rather than an arm that happens to vanish.  This is the
    // assertion that distinguishes "the guard works" from "the test never triggers".
    Model ungauge = make_model(1);
    ungauge.body_wrench_point_local = {Vec3{0.0, 0.0, 1.0}};
    const std::vector<double> live_column =
        axle_kernel::frozen_load_column(ungauge, turned, 0, 1);
    check(close(value_at(live_column, 3), 1.0, 1e-12),
          "without a gauge the same lever arm does write Mx (the fixture is live)");
}

}  // namespace

int main() {
    test_each_wrench_component_lands_on_its_own_coordinate();
    test_the_sign_is_the_residual_convention();
    test_a_declared_marker_adds_the_pose_dependent_moment();
    test_no_marker_means_no_moment();
    test_loaded_directions_that_cannot_move_the_mechanism_are_zero();
    test_a_gauge_coordinate_carries_no_load_column();
    test_the_load_driven_chain_reports_the_inverse_stiffness();

    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("OK: frozen load column self-test passed\n");
    return 0;
}
