// Self-test for the frozen working sample of `mb_linearization`.
//
// Verified by a standalone executable rather than through the product C ABI: the
// frozen sample builder is an internal capability, and exercising it must not
// require bumping an ABI version.  (Same reasoning as `linear_batch_selftest`.)
//
// What this checks, and why each one needs checking rather than assuming:
//
//   1. `frozen_working_sample` describes the state it was GIVEN.  The pose it
//      reports must be the pose handed in, not a re-derived or zeroed one -- a
//      builder that silently solved a nearby state would still return `true` and
//      would make every downstream tangent answer a question nobody asked.
//
//   2. The gauge comes from the MODEL, and a model with no declared gauge yields
//      an all-zero gauge.  This is the honest answer for such a model, so the test
//      builds one and asserts zeros rather than asserting a default.
//
//   3. The gauge is the model's, not invented: a model that DOES declare a gauge
//      must produce exactly the coordinates that declaration names -- and exactly
//      those, no more (a builder that gauged all six pose bits of the body, or
//      gauged every free body, would be wrong in a way that still looks plausible).
//
//   4. `position_constraint` is the plain constraint Jacobian and its shape is
//      `rows x ndof`.  The static path masks this matrix, but that mask is
//      identically one, so the frozen path must NOT produce a differently-masked
//      matrix -- and the size check is what catches a transposed or ragged one.
//
//   5. `load_scale` is 1.0 and the active tire set is passed through verbatim.  A
//      frozen response is asked at the unperturbed load, so a builder that applied
//      a scale would answer a different question.
//
//   6. A state that does not match the model is REFUSED.  A short body count is a
//      caller handing over another model's state, not a physics verdict; accepting
//      it would assemble a plausible-looking tangent from mismatched data.
//
//   7. A non-finite pose is refused rather than assembled.

#include "mb_linearization/types.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++failures;
}

using axle_kernel::Model;
using axle_kernel::Quat;
using axle_kernel::State;
using axle_kernel::Vec3;
using axle_kernel::WorkingSample;

// A three-body model: one fixed "fixture" and two free sliders.  `ndof` is
// therefore 12 and `body_to_free` maps body 1 -> free 0, body 2 -> free 1.
//
// The bodies are deliberately few: the properties under test (which coordinates a
// gauge names, how many rows the constraint Jacobian has) are counted, not
// numerically integrated, so a smaller model makes a wrong count easier to see.
Model make_model(bool with_gauge, std::uint32_t mask, int gauge_body) {
    Model model;
    axle_kernel::Body fixed_body;
    fixed_body.fixed = true;
    axle_kernel::Body first;
    axle_kernel::Body second;
    model.bodies = {fixed_body, first, second};
    model.free_body = {1, 2};
    model.body_to_free = {-1, 0, 1};
    model.ndof = 12;
    model.rows = 0;
    if (with_gauge) {
        model.static_gauge_body = gauge_body;
        model.static_gauge_dof_mask = mask;
    }
    return model;
}

// A pose that is distinguishable from zero in every component, so a builder that
// dropped or re-derived the pose cannot coincidentally look correct.
State make_state(const Model& model, double scale) {
    State state;
    state.r.resize(model.bodies.size());
    state.q.resize(model.bodies.size());
    state.v.resize(model.bodies.size());
    state.omega.resize(model.bodies.size());
    for (std::size_t i = 0; i < model.bodies.size(); ++i) {
        const double k = scale * static_cast<double>(i + 1);
        state.r[i] = Vec3{k + 1.0, k + 2.0, k + 3.0};
        state.q[i] = Quat{1.0, 0.0, 0.0, 0.0};
        state.v[i] = Vec3{k + 4.0, k + 5.0, k + 6.0};
        state.omega[i] = Vec3{k + 7.0, k + 8.0, k + 9.0};
    }
    return state;
}

// --- 1. the reported pose is the pose that was given -------------------------

void test_pose_is_the_given_pose() {
    const Model model = make_model(false, 0, -1);
    const State state = make_state(model, 1.0);
    const std::vector<int> tires{};
    WorkingSample working;
    check(axle_kernel::frozen_working_sample(model, state, tires, working),
          "a well-formed frozen state is described");

    bool same = working.pose.r.size() == state.r.size();
    for (std::size_t i = 0; same && i < state.r.size(); ++i) {
        same = working.pose.r[i].x == state.r[i].x &&
               working.pose.r[i].y == state.r[i].y &&
               working.pose.r[i].z == state.r[i].z;
    }
    check(same, "the frozen sample reports the pose it was given, bit-for-bit");

    bool velocity_same = working.pose.v.size() == state.v.size();
    for (std::size_t i = 0; velocity_same && i < state.v.size(); ++i) {
        velocity_same = working.pose.v[i].x == state.v[i].x &&
                        working.pose.v[i].y == state.v[i].y &&
                        working.pose.v[i].z == state.v[i].z;
    }
    check(velocity_same, "the frozen sample reports the world velocity it was given");
}

// --- 2. a model with no declared gauge yields an all-zero gauge --------------

void test_no_gauge_means_no_gauge_rows() {
    const Model model = make_model(false, 0, -1);
    const State state = make_state(model, 1.0);
    WorkingSample working;
    check(axle_kernel::frozen_working_sample(model, state, {}, working),
          "a gauge-less model still describes a frozen sample");
    check(working.gauge.size() == static_cast<std::size_t>(model.ndof),
          "the gauge flag vector spans every pose coordinate");
    bool all_zero = true;
    for (const unsigned char flag : working.gauge) all_zero = all_zero && flag == 0;
    check(all_zero,
          "a model that declares no gauge yields an all-zero gauge, not a default");
}

// --- 3. the gauge is exactly the model's declaration ------------------------

void test_gauge_is_exactly_the_model_declaration() {
    // Body 1 is free-body 0, so its pose coordinates are 0..5.  Declaring bits 0
    // and 3 must gauge coordinates 0 and 3 -- and nothing else.
    const Model model = make_model(true, 0x9u, 1);
    const State state = make_state(model, 1.0);
    WorkingSample working;
    check(axle_kernel::frozen_working_sample(model, state, {}, working),
          "a gauged model describes a frozen sample");
    check(working.gauge.size() == static_cast<std::size_t>(model.ndof),
          "the gauged model's flag vector spans every pose coordinate");
    check(working.gauge[0] == 1 && working.gauge[3] == 1,
          "the gauge names the coordinates the model's mask selects");
    check(working.gauge[1] == 0 && working.gauge[2] == 0 &&
          working.gauge[4] == 0 && working.gauge[5] == 0,
          "the gauge does not gauge coordinates the mask leaves clear");
    // Free-body 1 (body 2) owns coordinates 6..11 and must be untouched: a builder
    // that gauged every free body would pass the two checks above.
    bool others_clear = true;
    for (int coordinate = 6; coordinate < model.ndof; ++coordinate) {
        others_clear = others_clear && working.gauge[coordinate] == 0;
    }
    check(others_clear, "the gauge is confined to the body the model names");
}

// --- 3b. the gauge distinguishes bit positions, not just a bit count ---------

void test_gauge_reads_bit_positions() {
    // Mask 0x21 selects bits 0 and 5 (coordinates 0 and 5).  This is deliberately
    // *not* the mask of the previous test: a builder that interpreted the mask as "a
    // count of coordinates" or as "a contiguous range from zero" would coincide with
    // the correct answer on 0x9, where the selected coordinates happen to be 0 and 3.
    // With bit 5 set, "the first two coordinates" and "coordinates 0 and 5" are
    // different answers and only one of them is right.
    const Model model = make_model(true, 0x21u, 1);
    const State state = make_state(model, 1.0);
    WorkingSample working;
    check(axle_kernel::frozen_working_sample(model, state, {}, working),
          "a sparsely masked gauge describes a frozen sample");
    check(working.gauge[0] == 1 && working.gauge[5] == 1,
          "the gauge selects the coordinate of every set bit");
    check(working.gauge[1] == 0 && working.gauge[3] == 0,
          "the gauge does not select coordinates between set bits");
}

// --- 3c. a refusal leaves the caller's sample untouched ---------------------

void test_refusal_does_not_write_the_sample() {
    // This is the property the return value's contract rests on.  A refusal that had
    // already written `gauge`/`pose`/`position_constraint` would leave the caller
    // holding the *previous* state's matrix, with a size that still matches the
    // model -- and the assembler checks only the size, so it would accept stale data.
    const Model model = make_model(true, 0x9u, 1);
    const State good = make_state(model, 1.0);
    WorkingSample working;
    check(axle_kernel::frozen_working_sample(model, good, {}, working),
          "a valid state populates the sample");

    // Snapshot the populated sample, then attempt a refusal.
    const std::vector<unsigned char> gauge_before = working.gauge;
    const std::vector<double> matrix_before = working.position_constraint;
    const double scale_before = working.load_scale;
    const double pose_before = working.pose.r[1].x;

    State bad = good;
    bad.r.pop_back();
    check(!axle_kernel::frozen_working_sample(model, bad, {}, working),
          "a mismatched state is refused");

    check(working.gauge == gauge_before,
          "a refusal leaves the sample's gauge unchanged");
    check(working.position_constraint == matrix_before,
          "a refusal leaves the sample's position_constraint unchanged");
    check(working.load_scale == scale_before,
          "a refusal leaves the sample's load_scale unchanged");
    check(working.pose.r.size() == good.r.size() &&
          working.pose.r[1].x == pose_before,
          "a refusal leaves the sample's pose unchanged");
}

// --- 4/5. position_constraint shape, load scale and tire pass-through -------

void test_shape_scale_and_tires() {
    // A model with real constraint rows, so `position_constraint` has a size that
    // a broken builder cannot coincidentally reproduce by returning nothing: with
    // `rows == 0` an emptied matrix and a correctly-computed one are both empty and
    // the shape check would pass on the defect.
    //
    // The rows are prismatic joints.  They are chosen because each contributes
    // exactly one row and needs only an axis, so the matrix is non-empty without
    // the test having to build a fully consistent linkage -- this test is about the
    // *shape and presence* of the frozen description, not about its physics.
    Model model = make_model(false, 0, -1);
    model.rows = 2;
    axle_kernel::Constraint first;
    first.type = AXLE_PRISMATIC;
    first.a = 0;
    first.b = 1;
    first.axis_a = Vec3{0.0, 0.0, 1.0};
    first.axis_b = Vec3{0.0, 0.0, 1.0};
    first.row = 0;
    axle_kernel::Constraint second;
    second.type = AXLE_PRISMATIC;
    second.a = 0;
    second.b = 2;
    second.axis_a = Vec3{0.0, 0.0, 1.0};
    second.axis_b = Vec3{0.0, 0.0, 1.0};
    second.row = 1;
    model.constraints = {first, second};

    const State state = make_state(model, 1.0);
    const std::vector<int> tires{1, 0};
    WorkingSample working;
    check(axle_kernel::frozen_working_sample(model, state, tires, working),
          "a frozen sample with an active tire set is described");
    check(working.position_constraint.size() ==
              static_cast<std::size_t>(model.rows)*
                  static_cast<std::size_t>(model.ndof),
          "position_constraint is rows x ndof");
    // Distinct from the size check above only in that it names the failure mode: a
    // builder that returned an empty matrix would satisfy `size() == 0 == rows*ndof`
    // on a model with no constraint rows, so this model deliberately has rows.
    check(!working.position_constraint.empty(),
          "position_constraint is actually filled for a model with constraint rows");
    check(working.load_scale == 1.0,
          "a frozen response is assembled at the unperturbed load");
    // Order is preserved, not sorted: the tire order is the sample's contact column
    // order, so re-ordering it would silently relabel the contact columns.
    check(working.active_tires.size() == tires.size() &&
          working.active_tires[0] == 1 && working.active_tires[1] == 0,
          "the active tire set is passed through in the order given");
}

// --- 6. a mismatched state is refused ---------------------------------------

void test_mismatched_state_is_refused() {
    const Model model = make_model(false, 0, -1);
    State short_state = make_state(model, 1.0);
    short_state.r.pop_back();  // one body short: another model's state
    WorkingSample working;
    check(!axle_kernel::frozen_working_sample(model, short_state, {}, working),
          "a state with the wrong body count is refused, not assembled");

    State long_state = make_state(model, 1.0);
    long_state.r.push_back(Vec3{0.0, 0.0, 0.0});
    check(!axle_kernel::frozen_working_sample(model, long_state, {}, working),
          "a state with too many bodies is refused, not assembled");
}

// --- 7. a non-finite pose is refused ---------------------------------------

void test_non_finite_pose_is_refused() {
    const Model model = make_model(false, 0, -1);
    State state = make_state(model, 1.0);
    state.r[1].y = std::numeric_limits<double>::quiet_NaN();
    WorkingSample working;
    check(!axle_kernel::frozen_working_sample(model, state, {}, working),
          "a non-finite position is refused, not assembled");

    State bad_quaternion = make_state(model, 1.0);
    bad_quaternion.q[2].x = std::numeric_limits<double>::infinity();
    check(!axle_kernel::frozen_working_sample(model, bad_quaternion, {}, working),
          "a non-finite quaternion is refused, not assembled");

    // Velocity is checked as well as pose: the force laws the tangent differentiates
    // read it, so a NaN there yields a NaN tangent rather than a refusal.
    State bad_velocity = make_state(model, 1.0);
    bad_velocity.v[1].z = std::numeric_limits<double>::quiet_NaN();
    check(!axle_kernel::frozen_working_sample(model, bad_velocity, {}, working),
          "a non-finite world velocity is refused, not assembled");

    State bad_omega = make_state(model, 1.0);
    bad_omega.omega[2].y = std::numeric_limits<double>::infinity();
    check(!axle_kernel::frozen_working_sample(model, bad_omega, {}, working),
          "a non-finite angular velocity is refused, not assembled");
}

}  // namespace

int main() {
    test_pose_is_the_given_pose();
    test_no_gauge_means_no_gauge_rows();
    test_gauge_is_exactly_the_model_declaration();
    test_gauge_reads_bit_positions();
    test_refusal_does_not_write_the_sample();
    test_shape_scale_and_tires();
    test_mismatched_state_is_refused();
    test_non_finite_pose_is_refused();
    if (failures == 0) {
        std::printf("OK: frozen working sample self-test passed\n");
        return 0;
    }
    std::fprintf(stderr, "%d frozen-sample check(s) failed\n", failures);
    return 1;
}
