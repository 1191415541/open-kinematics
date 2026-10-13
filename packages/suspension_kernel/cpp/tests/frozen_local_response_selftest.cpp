// Self-test for the load-driven frozen response, `A X = B_w P` and `Y = G X + D P`.
//
// This is the composition the design's P4a acceptance names: on a linear
// mass-spring-damper at a fixed velocity and inertia, the *coordinate-driven*
// boundary must report `K = k` and the *load-driven* one must report `C = 1/k`.
// The two are asserted side by side because the design forbids deriving either from
// the other (SPEC :37, design :422), and a test that checked only `C = 1/k` would
// pass on a routine that quietly inverted the stiffness matrix.
//
// What is checked, and why each one needs checking rather than assuming:
//
//   1. `G`'s blade for a *translation* is the unit vector on the world axis the
//      measurement names.  This is the one piece of the measurement functional that
//      is pure convention, so all three axes are asserted separately.
//
//   2. `G`'s blade carries the **lever arm** when the measurement is taken at a
//      material point away from the body origin.  At the identity pose the arm is a
//      permutation of the axes and a routine that ignored the pose would still
//      pass, so the assertion uses a quarter-turned body and a hand-computed cross
//      product -- the same trap `frozen_load_selftest.cpp` documents for `B_w`.
//
//   3. A measurement *relative to a reference body* subtracts that body's blade.
//      The reference's contribution is the whole point of
//      `measurement_reference`, and a routine that forgot it would agree with the
//      absolute measurement whenever the reference does not move -- which is
//      exactly the fixture that a careless test would build.
//
//   4. The whole chain `A X = B_w P` with a diagonal spring: `C = 1/k`.  The
//      tangent is assembled by hand so the expected value is known by construction
//      rather than by running the implementation.
//
//   5. A mode is a *combination*: two bodies loaded in phase produce one right-hand
//      side, and the relative measurement of the pair must report zero.  This is the
//      design's `heave_load` shape in miniature, and it is the case a per-column
//      implementation gets wrong.
//
//   6. One factorization for many modes.  The design's cost claim is "assemble and
//      decompose once per working sample", so the count is read from the routine's
//      own out-parameter rather than asserted in prose.
//
//   7. Refusals: a non-finite amplitude, a wrong-length amplitude vector, a
//      measurement on a fixed body, and an out-of-range component each refuse the
//      request instead of contributing a plausible zero.

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

using axle_kernel::FrozenLoadMode;
using axle_kernel::FrozenMeasurement;
using axle_kernel::Model;
using axle_kernel::State;
using axle_kernel::Vec3;
using axle_kernel::WorkingSample;

bool close(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance*std::max(1.0, std::abs(b));
}

// A model with `bodies` free bodies, no constraints and no tires.  `dimension`
// is therefore `ndof`.
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

// A 90-degree rotation about z, so `rotate(q, p)` is a permutation rather than the
// identity and an implementation that ignored the pose is caught.
axle_kernel::Quat quarter_turn_z() {
    const double half = 3.14159265358979323846/4.0;
    return axle_kernel::Quat{std::cos(half), 0.0, 0.0, std::sin(half)};
}

WorkingSample make_working(const Model& model, const State& pose) {
    WorkingSample working;
    working.pose = pose;
    working.load_scale = 1.0;
    working.frozen_inertial_load = false;
    working.active_tires.clear();
    (void)model;
    return working;
}

// `A = diag(k, pin, ..., k, pin, ...)`: one soft spring per body on its x
// coordinate and a stiff pin on the other five, so the answer is `1/k` on x and
// effectively zero elsewhere.  `pin` is large but finite, so the matrix is not
// singular and no least-squares path is involved.
std::vector<double> make_tangent(int ndof, double k, double pin) {
    std::vector<double> tangent(
        static_cast<std::size_t>(ndof)*static_cast<std::size_t>(ndof), 0.0
    );
    for (int coordinate = 0; coordinate < ndof; ++coordinate) {
        const bool is_spring = coordinate % 6 == 0;
        tangent[static_cast<std::size_t>(coordinate)*
                static_cast<std::size_t>(ndof) +
                static_cast<std::size_t>(coordinate)] = is_spring ? k : pin;
    }
    return tangent;
}

FrozenLoadMode mode_x(const std::string& id, int bodies, double per_body) {
    FrozenLoadMode mode;
    mode.id = id;
    mode.amplitudes.assign(static_cast<std::size_t>(bodies)*6, 0.0);
    for (int body = 0; body < bodies; ++body) {
        mode.amplitudes[static_cast<std::size_t>(body)*6] = per_body;
    }
    return mode;
}

FrozenMeasurement measurement(
    const std::string& id, int body, int component,
    int reference = -1
) {
    FrozenMeasurement entry;
    entry.id = id;
    entry.units = component < 3 ? "m" : "rad";
    entry.body = body;
    entry.component = component;
    entry.reference = reference;
    return entry;
}

// --- 1. a translation blade names its own axis --------------------------------

void test_a_translation_blade_names_its_own_axis() {
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    for (int component = 0; component < 3; ++component) {
        const std::vector<FrozenMeasurement> measurements{
            measurement("m", 0, component)
        };
        std::vector<double> gradient;
        check(axle_kernel::frozen_measurement_gradient(
                  model, working, measurements, gradient),
              "a translation measurement resolves");
        check(gradient.size() == static_cast<std::size_t>(model.ndof),
              "G has one row of ndof when the sample has no constraint rows");
        for (int coordinate = 0; coordinate < model.ndof; ++coordinate) {
            const double expected = coordinate == component ? 1.0 : 0.0;
            check(close(gradient[static_cast<std::size_t>(coordinate)],
                        expected, 1e-14),
                  "the translation blade is the unit vector on its axis");
        }
    }
}

// --- 2. a translation at a material point carries its lever arm --------------

void test_a_translation_at_a_point_carries_its_lever_arm() {
    // `point = (1, 0, 0)` on a body turned 90 degrees about z: `R*p = (0, 1, 0)`,
    // so the y measurement picks up a `-1` rotation blade.  At the identity pose
    // the same point would give `(1,0,0)`, whose cross with `e_y` is `-e_z` and
    // whose y component is zero -- a routine that ignored the pose would agree with
    // a wrong answer there and be caught only here.
    Model model = make_model(1);
    State pose = make_pose(1);
    pose.q[0] = quarter_turn_z();
    const WorkingSample working = make_working(model, pose);
    FrozenMeasurement entry = measurement("wheel_center_L.y", 0, 1);
    entry.point = Vec3{1.0, 0.0, 0.0};
    entry.has_point = true;
    std::vector<double> gradient;
    check(axle_kernel::frozen_measurement_gradient(
              model, working, {entry}, gradient),
          "a point measurement resolves");
    // dr.y contributes 1.
    check(close(gradient[1], 1.0, 1e-14), "the translation axis is itself");
    // arm = R*p = (0,1,0); blade = arm x e_y = (0,1,0)x(0,1,0) = 0.
    check(close(gradient[3], 0.0, 1e-14), "arm x e_y x component");
    check(close(gradient[4], 0.0, 1e-14), "arm x e_y y component");
    check(close(gradient[5], 0.0, 1e-14), "arm x e_y z component");
    // Now measure along x: arm x e_x = (0,1,0)x(1,0,0) = (0*0-0*0, 0*1-0*0,
    // 0*0-1*1) = (0,0,-1), so the rz blade is -1.  This is the assertion the
    // identity-pose fixture cannot make.
    FrozenMeasurement along_x = measurement("wheel_center_L.x", 0, 0);
    along_x.point = Vec3{1.0, 0.0, 0.0};
    along_x.has_point = true;
    std::vector<double> gradient_x;
    check(axle_kernel::frozen_measurement_gradient(
              model, working, {along_x}, gradient_x),
          "the x point measurement resolves");
    check(close(gradient_x[0], 1.0, 1e-14), "x translation axis");
    check(close(gradient_x[3], 0.0, 1e-14), "rx blade cancels");
    check(close(gradient_x[4], 0.0, 1e-14), "ry blade cancels");
    check(close(gradient_x[5], -1.0, 1e-14),
          "the lever arm turns e_x into a -rz blade at a quarter turn");
}

// --- 3. a relative measurement subtracts the reference blade -----------------

void test_a_relative_measurement_subtracts_the_reference() {
    const Model model = make_model(2);
    const State pose = make_pose(2);
    const WorkingSample working = make_working(model, pose);
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0),
        measurement("body1.x_relative", 1, 0, 0),
    };
    std::vector<double> gradient;
    check(axle_kernel::frozen_measurement_gradient(
              model, working, measurements, gradient),
          "the relative measurement resolves");
    check(gradient.size() == 2*static_cast<std::size_t>(model.ndof),
          "G has two rows");
    check(close(gradient[0], 1.0, 1e-14), "absolute row reads body 0");
    check(close(gradient[6], 0.0, 1e-14), "absolute row ignores body 1");
    // Row 1 = e_{body1.x} - e_{body0.x}.
    check(close(gradient[static_cast<std::size_t>(model.ndof) + 0], -1.0, 1e-14),
          "relative row subtracts the reference translation");
    check(close(gradient[static_cast<std::size_t>(model.ndof) + 6], 1.0, 1e-14),
          "relative row keeps the measured body translation");
}

// --- 4/6. the whole chain, and one factorization for many modes --------------

void test_the_load_driven_chain_reports_inverse_stiffness() {
    const double k = 500.0;
    const double pin = 1.0e6;
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const std::vector<double> tangent = make_tangent(model.ndof, k, pin);

    const std::vector<FrozenLoadMode> modes{mode_x("single_x", 1, 1.0)};
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0)
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = 0;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, audit),
          "the load-driven chain solves");
    check(response.size() == 1, "one mode x one measurement");
    check(close(response[0], 1.0/k, 1e-12),
          "C = 1/k for a unit load on the spring");
    // The audit agrees: this tangent is diagonal and full rank, so nothing is
    // refused for compatibility or identifiability.  A full-rank sample is the
    // case that makes every measurement identifiable *by construction*, and the
    // audit has to say so rather than staying silent.
    check(audit.dimension == model.ndof && audit.matrix_rank == model.ndof,
          "the audit reports the rank of a full-rank tangent");
    check(audit.mode_compatible.size() == 1 && audit.mode_compatible[0] == 1,
          "a full-rank tangent leaves no mode incompatible");
    check(audit.measurement_identifiable.size() == 1 &&
              audit.measurement_identifiable[0] == 1,
          "a full-rank tangent leaves every measurement identifiable");

    // The coordinate-driven sibling on the *same* tangent must report `K = k`.
    // Asserting them side by side is what makes it impossible for one boundary to
    // be silently standing in for the other.
    const std::vector<int> driven{0};
    const std::vector<double> prescribed{0.01};
    std::vector<double> reaction;
    check(axle_kernel::frozen_force_tangent_column(
              model, working, tangent, driven, prescribed, reaction),
          "the coordinate-driven chain solves");
    check(reaction.size() == 1, "one reaction");
    check(close(reaction[0], k*0.01, 1e-12),
          "K = k for the same spring, from the coordinate-driven boundary");
    // And the two numbers are not reciprocals of each other by accident: 1/500 is
    // 2e-3 while the chord solve reports 5.0.
    check(std::abs(reaction[0] - response[0]) > 1.0,
          "the two boundaries give genuinely different quantities");
}

void test_one_factorization_serves_every_mode() {
    const double k = 250.0;
    const double pin = 1.0e6;
    const Model model = make_model(2);
    const State pose = make_pose(2);
    const WorkingSample working = make_working(model, pose);
    const std::vector<double> tangent = make_tangent(model.ndof, k, pin);

    std::vector<FrozenLoadMode> modes{
        mode_x("in_phase", 2, 1.0),
        mode_x("anti_phase", 2, 1.0),
    };
    std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0),
        measurement("body1.x_relative", 1, 0, 0),
    };
    // The anti-phase mode is the second column with body 1 negated, which
    // `mode_x` cannot express; build it explicitly so the two modes really differ.
    modes[1].amplitudes[6] = -1.0;
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = 0;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, audit),
          "the batch solves");
    check(factorizations == 1,
          "the whole batch costs exactly one numeric factorization");
    check(mode_ok.size() == 2 && mode_ok[0] != 0 && mode_ok[1] != 0,
          "both modes report");

    // In phase: body0.x = 1/k, and body1.x - body0.x = 0.  The pair moving
    // together is the miniature of the design's `heave_load`.
    check(close(response[0], 1.0/k, 1e-12), "in-phase body0 displacement");
    check(close(response[1], 0.0, 1e-12),
          "in-phase relative displacement vanishes, not because it was never computed");
    // Anti phase: body0.x = 1/k, body1.x = -1/k, so the relative measurement is
    // -2/k.  A per-column implementation that ignored the sign would report zero.
    check(close(response[2], 1.0/k, 1e-12), "anti-phase body0 displacement");
    check(close(response[3], -2.0/k, 1e-12),
          "anti-phase relative displacement is twice the single one, with the load's sign");
}

// --- 5. a mode that moves nothing is a zero, not a failure -------------------

void test_a_mode_on_a_fixed_body_is_a_zero_response() {
    const double k = 500.0;
    Model model = make_model(2);
    // Body 1 is fixed: it has no free coordinate, so a load on it moves nothing.
    model.body_to_free[1] = -1;
    model.free_body.resize(1);
    const State pose = make_pose(2);
    WorkingSample working = make_working(model, pose);
    const std::vector<double> tangent = make_tangent(model.ndof, k, 1.0e6);
    const std::vector<FrozenLoadMode> modes{mode_x("fixed_body_x", 2, 1.0)};
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0)
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = 0;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, audit),
          "a load on a fixed body still produces a well-formed response");
    // Body 0 is loaded too (the mode loads both bodies), so the answer is 1/k for
    // body 0; body 1's share contributes nothing.  The point of the test is that the
    // routine did not refuse and did not crash on the unreachable coordinate.
    check(close(response[0], 1.0/k, 1e-12),
          "the free body answers while the fixed body's amplitude is inert");
}

// --- 3b. a rotation measurement, and the reference body's own lever arm -------

void test_a_rotation_blade_and_the_reference_lever_arm() {
    // A rotation measurement reads the *relative* world angular increment.  Nothing
    // above this point exercises component 3..5, so a routine that mapped a rotation
    // onto a translation coordinate would pass every earlier assertion.
    const Model model = make_model(2);
    const State pose = make_pose(2);
    const WorkingSample working = make_working(model, pose);
    {
        const std::vector<FrozenMeasurement> measurements{
            measurement("body0.rx", 0, 3),
            measurement("body1.rz_relative", 1, 5, 0),
        };
        std::vector<double> gradient;
        check(axle_kernel::frozen_measurement_gradient(
                  model, working, measurements, gradient),
              "rotation measurements resolve");
        check(close(gradient[3], 1.0, 1e-14), "rx blade is the rx coordinate");
        check(close(gradient[0], 0.0, 1e-14),
              "a rotation blade does not touch a translation coordinate");
        const std::size_t row = static_cast<std::size_t>(model.ndof);
        // body 0 is free body 0 (coordinates 0..5) and body 1 is free body 1
        // (coordinates 6..11), so the measured rz is column 11 and the subtracted
        // reference rz is column 5.  Writing `row + 5` for the measured body -- the
        // first version of this assertion did -- tests nothing but the author's
        // arithmetic: the fixture's two bodies and the row offset are easy to
        // conflate, and only running it shows which coordinate the blade really
        // lands on.
        check(close(gradient[row + 3], 0.0, 1e-14),
              "the relative rz blade leaves the reference rx alone");
        check(close(gradient[row + 5], -1.0, 1e-14),
              "relative rotation subtracts the reference rz (column 5)");
        check(close(gradient[row + 11], 1.0, 1e-14),
              "relative rotation keeps the measured rz (column 11)");
    }
    // The reference body's own material point contributes a lever arm to the
    // *subtracted* blade.  `has_reference_point` was never set before this case, so
    // the branch that adds `-cross(R*p_ref, e)` had no coverage at all.
    //
    // Both sub-cases below use **one** measurement, so `G` is `1 x ndof` = 12 doubles.
    // Indexing it as `row + column` -- where `row` is the measurement's own offset --
    // would read past the end: that is how the first version of this assertion went
    // wrong.  The columns are written absolutely and the size is asserted first.
    {
        FrozenMeasurement relative = measurement("body1.x_rel_at_point", 1, 0, 0);
        relative.reference_point = Vec3{0.0, 1.0, 0.0};
        relative.has_reference_point = true;
        std::vector<double> gradient;
        check(axle_kernel::frozen_measurement_gradient(
                  model, working, {relative}, gradient),
              "the reference point measurement resolves");
        check(gradient.size() == static_cast<std::size_t>(model.ndof),
              "one measurement is one row of ndof");
        // body 1 is free body 1, so its x translation is column 6; the reference is
        // body 0, whose block is columns 0..5 and whose rz blade is therefore column 5.
        check(close(gradient[6], 1.0, 1e-14),
              "the measured body's own translation is unaffected by the reference point");
        // arm = R*(0,1,0) = (0,1,0); arm x e_x = (0,0,-1); the reference blade is
        // `-=` that, so column 5 reads +1.0.
        check(close(gradient[5], 1.0, 1e-14),
              "the reference lever arm enters with the opposite sign at the reference rz");
        // Without the point the same measurement has no rz blade on the reference at
        // all: the two cases must differ, or the assertion above proves nothing about
        // the point.
        const std::vector<FrozenMeasurement> plain{
            measurement("body1.x_rel", 1, 0, 0)
        };
        std::vector<double> plain_gradient;
        check(axle_kernel::frozen_measurement_gradient(
                  model, working, plain, plain_gradient),
              "the plain relative measurement resolves");
        check(plain_gradient.size() == static_cast<std::size_t>(model.ndof),
              "the plain measurement is also one row");
        check(close(plain_gradient[6], 1.0, 1e-14),
              "the plain measurement still reads the measured body");
        check(close(plain_gradient[5], 0.0, 1e-14),
              "without a declared point the reference lever arm is absent");
    }
}

// --- 5b. a mode that loads only a fixed body is a *valid* zero ----------------

void test_a_pure_fixed_body_mode_is_a_valid_zero() {
    // The earlier fixed-body case loaded both bodies, so its answer was 1/k and it
    // never exercised the "the load cannot move anything" reading.  Here the model's
    // only free coordinate set is body 0, and the mode loads *only* body 1, which has
    // no free coordinate: `frozen_load_column` returns a zero column for it, the
    // right-hand side is zero, and the honest answer is 0 with `mode_ok == 1`.
    //
    // This is the distinction the project keeps re-learning: "this load direction does
    // nothing" is a legitimate zero and must be reported as valid, not refused and not
    // confused with a failure.
    const double k = 500.0;
    Model model = make_model(2);
    model.body_to_free[1] = -1;
    model.free_body.resize(1);
    // Keep the model invariant `ndof == 6 * free_body.size()`: a model whose ndof
    // disagrees with its free set is not a real model and would make the fixture
    // unrepresentative of the case it claims to cover.
    model.ndof = 6*static_cast<int>(model.free_body.size());
    const State pose = make_pose(2);
    const WorkingSample working = make_working(model, pose);
    const std::vector<double> tangent = make_tangent(model.ndof, k, 1.0e6);
    FrozenLoadMode fixed_only;
    fixed_only.id = "fixed_body_only";
    fixed_only.amplitudes.assign(12, 0.0);
    fixed_only.amplitudes[6] = 1.0;  // body 1's Fx, which has no free coordinate
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0)
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = 0;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, {fixed_only}, measurements, response,
              mode_ok, factorizations, audit),
          "a mode on a fixed body still answers");
    check(mode_ok.size() == 1 && mode_ok[0] == 1,
          "a load that moves nothing is a valid zero, not a refusal");
    check(response.size() == 1 && close(response[0], 0.0, 1e-14),
          "the response of an inert load is exactly zero");
}

// --- 7. refusals -------------------------------------------------------------

void test_refusals_are_reported_and_not_written() {
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const std::vector<double> tangent = make_tangent(model.ndof, 500.0, 1.0e6);
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0)
    };
    // One audit instance for the whole test: every refusal below must leave it
    // untouched, and passing the same object is what makes that observable.
    axle_kernel::FrozenResponseAudit audit;

    // A non-finite amplitude: the request is malformed, not a legitimate zero.
    {
        FrozenLoadMode bad = mode_x("bad", 1, 1.0);
        bad.amplitudes[0] = std::nan("");
        std::vector<double> response{7.0};
        std::vector<unsigned char> mode_ok{9};
        int factorizations = -1;
        check(!axle_kernel::frozen_local_response(
                  model, working, tangent, {bad}, measurements, response, mode_ok,
                  factorizations, audit),
              "a non-finite amplitude is refused");
        check(response.size() == 1 && response[0] == 7.0,
              "a refusal leaves the caller's response alone");
        check(mode_ok.size() == 1 && mode_ok[0] == 9,
              "a refusal leaves mode_ok alone");
        check(factorizations == -1,
              "a refusal does not write factorizations_used");
    }
    // A wrong-length amplitude vector: silently padding it would change which
    // bodies a mode loads.
    {
        FrozenLoadMode bad = mode_x("short", 1, 1.0);
        bad.amplitudes.resize(3);
        std::vector<double> response{7.0};
        std::vector<unsigned char> mode_ok{9};
        int factorizations = -1;
        check(!axle_kernel::frozen_local_response(
                  model, working, tangent, {bad}, measurements, response, mode_ok,
                  factorizations, audit),
              "a short amplitude vector is refused");
        check(response.size() == 1 && response[0] == 7.0 &&
                  mode_ok.size() == 1 && mode_ok[0] == 9 && factorizations == -1,
              "a malformed mode leaves every output untouched");
    }
    // A tangent that is not the dimension it claims.
    {
        std::vector<double> short_tangent(4, 0.0);
        std::vector<double> response{7.0};
        std::vector<unsigned char> mode_ok{9};
        int factorizations = -1;
        check(!axle_kernel::frozen_local_response(
                  model, working, short_tangent, {mode_x("m", 1, 1.0)},
                  measurements, response, mode_ok, factorizations, audit),
              "a tangent of the wrong size is refused");
        check(response.size() == 1 && response[0] == 7.0 &&
                  mode_ok.size() == 1 && mode_ok[0] == 9 && factorizations == -1,
              "a bad tangent leaves every output untouched");
    }
    // An empty request: answering it with an empty block would be
    // indistinguishable from having answered nothing at all.
    {
        std::vector<double> response{7.0};
        std::vector<unsigned char> mode_ok{9};
        int factorizations = -1;
        check(!axle_kernel::frozen_local_response(
                  model, working, tangent, {}, measurements, response, mode_ok,
                  factorizations, audit),
              "an empty mode list is refused");
        check(!axle_kernel::frozen_local_response(
                  model, working, tangent, {mode_x("m", 1, 1.0)}, {}, response,
                  mode_ok, factorizations, audit),
              "an empty measurement list is refused");
        check(response.size() == 1 && response[0] == 7.0 &&
                  mode_ok.size() == 1 && mode_ok[0] == 9 && factorizations == -1,
              "an empty request leaves every output untouched");
    }
    // A measurement on a body that does not exist, and one with an out-of-range
    // component.
    {
        std::vector<double> gradient;
        check(!axle_kernel::frozen_measurement_gradient(
                  model, working, {measurement("m", 5, 0)}, gradient),
              "a measurement on an unknown body is refused");
        check(!axle_kernel::frozen_measurement_gradient(
                  model, working, {measurement("m", 0, 6)}, gradient),
              "a measurement with an out-of-range component is refused");
        check(gradient.empty(), "a refused gradient is not written");
    }
    // A measurement on a fixed body.
    {
        Model fixed = make_model(1);
        fixed.body_to_free[0] = -1;
        fixed.free_body.clear();
        std::vector<double> gradient;
        check(!axle_kernel::frozen_measurement_gradient(
                  fixed, working, {measurement("m", 0, 0)}, gradient),
              "a measurement on a fixed body is refused");
    }
}

// --- 8. `G` spans the same vector the solve produces --------------------------

void test_the_gradient_spans_the_solved_dimension() {
    // A sample with constraint rows makes `dimension > ndof`, which is the case
    // that matters: the consumer solves `Y = G X` against the `X` of
    // `A X = B_w P`, so a `G` of `ndof` columns would be refused there
    // (`results/equilibrium.py` requires `gradient.shape[1] == dimension`).
    // Asserting the width on a `rows == 0` fixture would pass for both layouts,
    // which is exactly why the earlier version of this self-test could not see it.
    Model model = make_model(1);
    model.rows = 2;
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const int dimension = axle_kernel::physical_tangent_dimension(model, working);
    check(dimension == model.ndof + 2,
          "the fixture really has a wider tangent than its pose block");
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0),
    };
    std::vector<double> gradient;
    check(axle_kernel::frozen_measurement_gradient(
              model, working, measurements, gradient),
          "a measurement resolves on a constrained sample");
    check(gradient.size() == static_cast<std::size_t>(dimension),
          "one measurement is one row of `dimension`, not of `ndof`");
    // The pose blade is unchanged and the extra columns are exactly zero: the
    // widening must add columns, not shift the ones that were already right.
    check(close(gradient[0], 1.0, 1e-14), "the x blade still lands on its own column");
    for (int column = model.ndof; column < dimension; ++column) {
        check(close(gradient[static_cast<std::size_t>(column)], 0.0, 1e-14),
              "a multiplier column of `G` is exactly zero, not absent");
    }
    // And the response consumes it: the widened `G` is what the solve projects
    // with, so the chain stays consistent end to end rather than only in `G`.
    const std::vector<double> tangent =
        make_tangent(dimension, 500.0, 1.0e6);
    const std::vector<FrozenLoadMode> modes{mode_x("m", 1, 1.0)};
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = 0;
    axle_kernel::FrozenResponseAudit widen_audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, widen_audit),
          "the widened chain solves");
    check(mode_ok.size() == 1 && mode_ok[0] == 1 && response.size() == 1,
          "the widened chain answers one mode with one measurement");
    check(close(response[0], 1.0/500.0, 1e-12),
          "the response is the same physical compliance the narrow layout gave");
}

// --- 9. the compatibility and identifiability audit ---------------------------

void test_a_singular_tangent_is_reported_rather_than_aborted() {
    // A degenerate `A`: coordinate 5 is unconstrained, so `dimension - rank == 1`.
    // The request is still answered -- with the audit -- because "this sample is
    // singular" and "this request is malformed" are different facts, and a bare
    // `false` would erase the difference.
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const int dimension = model.ndof;
    std::vector<double> tangent(
        static_cast<std::size_t>(dimension)*static_cast<std::size_t>(dimension),
        0.0);
    for (int coordinate = 0; coordinate < dimension - 1; ++coordinate) {
        tangent[static_cast<std::size_t>(coordinate)*
                    static_cast<std::size_t>(dimension) +
                static_cast<std::size_t>(coordinate)] = 1.0;
    }
    const std::vector<FrozenLoadMode> modes{mode_x("in_the_rank", 1, 1.0)};
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0)
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = -1;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, audit),
          "a singular sample is reported rather than aborted");
    check(audit.dimension == dimension && audit.matrix_rank == dimension - 1,
          "the audit publishes the rank deficiency as the reason");
    // The load is `Fx`, which lands on coordinate 0 and is therefore inside the
    // range; the unconstrained direction is coordinate 5, which this request never
    // touches.  The measurement reads coordinate 0, which the load determines, so
    // the answer is published even though the sample is not full rank.
    check(audit.mode_compatible.size() == 1 && audit.mode_compatible[0] == 1,
          "a mode whose load is in the range is compatible");
    check(audit.measurement_identifiable.size() == 1 &&
              audit.measurement_identifiable[0] == 1,
          "a measurement inside the rank space is identifiable");
    check(mode_ok.size() == 1 && mode_ok[0] == 1 && response.size() == 1,
          "a compatible mode of a singular sample still answers");
    check(close(response[0], 1.0, 1e-12),
          "the published value is the solved response, not a family member");
}

void test_an_unidentifiable_measurement_is_not_published() {
    // `A` constrains only coordinate 0, so the null space is everything else.  The
    // load is in the range and the mode is answered; the two measurements differ
    // exactly in whether the load determines them.  Publishing the solver's member
    // for `body0.z` would be a plausible wrong number -- the failure this audit
    // exists to prevent -- and refusing the whole mode would throw away the `x`
    // reading that *is* determined.
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const int dimension = model.ndof;
    std::vector<double> tangent(
        static_cast<std::size_t>(dimension)*static_cast<std::size_t>(dimension),
        0.0);
    tangent[0] = 1.0;
    const std::vector<FrozenLoadMode> modes{mode_x("in_range", 1, 1.0)};
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0),
        measurement("body0.z", 0, 2),
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = -1;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, audit),
          "an unidentifiable measurement does not abort the sample");
    check(audit.mode_compatible.size() == 1 && audit.mode_compatible[0] == 1,
          "the load lands in the range and the mode is compatible");
    check(audit.measurement_identifiable.size() == 2 &&
              audit.measurement_identifiable[0] == 1 &&
              audit.measurement_identifiable[1] == 0,
          "the audit separates an in-range measurement from a null-space one");
    check(mode_ok.size() == 1 && mode_ok[0] == 1,
          "the mode is answered, because one of its measurements is determined");
    check(close(response[0], 1.0, 1e-12),
          "the determined measurement is published");
    check(std::isnan(response[1]),
          "the undetermined measurement stays NaN rather than reading as zero");
}

void test_an_incompatible_mode_is_refused_by_name() {
    // `A` again constrains only coordinate 0, and this load lands on coordinate 1:
    // the right-hand side is *not* in the range, so no solution exists.  A solver
    // would still return a vector (or a non-finite one), and `G X` would project it
    // as if the load had picked it -- the mode must be refused instead.
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const int dimension = model.ndof;
    std::vector<double> tangent(
        static_cast<std::size_t>(dimension)*static_cast<std::size_t>(dimension),
        0.0);
    tangent[0] = 1.0;
    FrozenLoadMode lateral;
    lateral.id = "lateral";
    lateral.amplitudes.assign(
        static_cast<std::size_t>(model.body_to_free.size())*6, 0.0);
    lateral.amplitudes[1] = 1.0;  // body 0 Fy, whose coordinate `A` does not reach
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0)
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = -1;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, {lateral}, measurements, response,
              mode_ok, factorizations, audit),
          "an incompatible mode does not abort the sample");
    check(audit.mode_compatible.size() == 1 && audit.mode_compatible[0] == 0,
          "a load outside the range makes the mode incompatible");
    check(mode_ok.size() == 1 && mode_ok[0] == 0,
          "an incompatible mode is refused by name");
    check(std::isnan(response[0]),
          "a refused mode publishes NaN, not a member of the family");
}

void test_a_compatible_combination_reports_when_its_columns_do_not() {
    // The design's `combined_mode_rule`: a combination can be compatible while its
    // own single columns are not, and the audit is therefore on the *actual* mode
    // right-hand side.  Here only the direction `e_x + e_y` is in `A`'s range: `Fx`
    // alone is incompatible, `Fx + Fy` together is compatible.  An audit that
    // rejected the combination because a unit column inside it was incompatible
    // would refuse a request the physics answers -- and one that only checked unit
    // columns would answer the request the physics refuses.
    const Model model = make_model(1);
    const State pose = make_pose(1);
    const WorkingSample working = make_working(model, pose);
    const int dimension = model.ndof;
    std::vector<double> tangent(
        static_cast<std::size_t>(dimension)*static_cast<std::size_t>(dimension),
        0.0);
    for (int coordinate = 2; coordinate < dimension; ++coordinate) {
        tangent[static_cast<std::size_t>(coordinate)*
                    static_cast<std::size_t>(dimension) +
                static_cast<std::size_t>(coordinate)] = 1.0;
    }
    // The first two rows are both `e_x + e_y`, so the range contains that sum and
    // not either unit vector alone.
    for (int row = 0; row < 2; ++row) {
        tangent[static_cast<std::size_t>(row)*
                    static_cast<std::size_t>(dimension) + 0] = 1.0;
        tangent[static_cast<std::size_t>(row)*
                    static_cast<std::size_t>(dimension) + 1] = 1.0;
    }
    FrozenLoadMode single_x;
    single_x.id = "single_x";
    single_x.amplitudes.assign(
        static_cast<std::size_t>(model.body_to_free.size())*6, 0.0);
    single_x.amplitudes[0] = 1.0;
    FrozenLoadMode pair_xy;
    pair_xy.id = "pair_xy";
    pair_xy.amplitudes.assign(
        static_cast<std::size_t>(model.body_to_free.size())*6, 0.0);
    pair_xy.amplitudes[0] = 1.0;
    pair_xy.amplitudes[1] = 1.0;
    const std::vector<FrozenLoadMode> modes{single_x, pair_xy};
    // `body0.z` reads coordinate 2, which the mode fixes exactly; the measurements
    // on x and y sit inside the null space and stay NaN.
    const std::vector<FrozenMeasurement> measurements{
        measurement("body0.x", 0, 0),
        measurement("body0.z", 0, 2),
    };
    std::vector<double> response;
    std::vector<unsigned char> mode_ok;
    int factorizations = -1;
    axle_kernel::FrozenResponseAudit audit;
    check(axle_kernel::frozen_local_response(
              model, working, tangent, modes, measurements, response, mode_ok,
              factorizations, audit),
          "a mixed request is answered per mode");
    check(audit.mode_compatible.size() == 2 && audit.mode_compatible[0] == 0 &&
              audit.mode_compatible[1] == 1,
          "the single column is incompatible while the combination is not");
    check(mode_ok.size() == 2 && mode_ok[0] == 0 && mode_ok[1] == 1,
          "the combination reports while the single column is refused");
    check(std::isnan(response[0]) && std::isnan(response[1]),
          "the refused mode's slots stay NaN");
    check(std::isnan(response[2]),
          "the in-null-space measurement of the answered mode stays NaN");
    check(close(response[3], 0.0, 1e-12),
          "the determined measurement of the answered mode is published");
}

}  // namespace

int main() {
    test_a_translation_blade_names_its_own_axis();
    test_a_translation_at_a_point_carries_its_lever_arm();
    test_a_relative_measurement_subtracts_the_reference();
    test_a_rotation_blade_and_the_reference_lever_arm();
    test_the_load_driven_chain_reports_inverse_stiffness();
    test_one_factorization_serves_every_mode();
    test_a_mode_on_a_fixed_body_is_a_zero_response();
    test_a_pure_fixed_body_mode_is_a_valid_zero();
    test_refusals_are_reported_and_not_written();
    test_the_gradient_spans_the_solved_dimension();
    test_a_singular_tangent_is_reported_rather_than_aborted();
    test_an_unidentifiable_measurement_is_not_published();
    test_an_incompatible_mode_is_refused_by_name();
    test_a_compatible_combination_reports_when_its_columns_do_not();
    if (failures != 0) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("frozen local response self-test passed\n");
    return 0;
}
