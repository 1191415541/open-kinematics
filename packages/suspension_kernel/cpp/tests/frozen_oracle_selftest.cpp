// Self-test for the *independent* frozen-state oracle: does the analytic tangent
// actually predict what the nonlinear residual does?
//
// Steps 1-3 built the frozen protocol, the coordinate-driven tangent and the load
// column, and each was tested against numbers whose answer was known by
// construction.  That is the right way to test an operator in isolation, but it
// leaves one question completely open: the three pieces could be *mutually
// consistent and jointly wrong*.  A tangent with a systematically wrong scale, a
// load column that disagrees with the residual's sign convention, or an assembly
// that silently drops the preload term would all pass an operator-level assertion
// and still misdescribe the mechanism.
//
// So this file takes a different route.  It never trusts the analytic tangent as
// the answer; it uses it only as a *prediction*, and it checks that prediction
// against a finite perturbation of an unrelated computation -- the same
// `static_residual` the Newton solver iterates on.  The residual is the ground
// truth here because it is *not* the thing under test: it is assembled from the
// force laws and constraint Jacobian at an explicitly constructed pose, with no
// reference to the tangent assembly at all.
//
// Concretely, for a perturbation `dx`:
//
//     R(x0 + dx) - R(x0)  ~=  A dx
//
// is checked for several `dx`, with a step ladder, and the error is required to
// *shrink at the second-order rate* as the step shrinks.  That rate is what makes
// this a real oracle rather than a coincidence.  Expand the residual about `x0`:
//
//     R(x0+dx) - R(x0) = A dx + 0.5 dx^T H dx + ...,      dx = h*v
//
// so for a correct tangent the discrepancy is `O(h^2)` and falls by ~100 per decade
// of step, while a tangent wrong by an additive `dA` contributes an additional
// `h*(dA v)` term that falls by only ~10.  The rate therefore separates "correct
// first derivative" from "plausible nearly-right tangent" in a way a single
// tolerance cannot.
//
// The resolution is finite and has to be stated honestly.  The discrepancy is
// `|0.5 h^2 v^T H v + h (dA v)|`, so an error only becomes visible once `h|dA|` is
// comparable to the truncation -- and since the truncation itself shrinks with `h`,
// the binding comparison is at the *smallest* step.  Measured on this fixture
// (tangent entries ~5000, `h` down to 1e-6): additive errors of 1e-3, 1e-2 and 1e-1
// leave the step ratio at 102.2, 128.7 and 53.5 against a threshold of 50, so only
// the last is even near detection; 5e-1 collapses the ratio to 5.8 and is caught.
// The oracle is thus bounded at roughly a 1e-4 *relative* tangent error, and the
// numbers are printed at run time so a drifting bound is visible rather than
// assumed.  It is a sanity oracle over the whole chain, not a substitute for the
// operator-level assertions in the three sibling self-tests.
//
// What is checked, and why each one needs checking rather than assuming:
//
//   1. First-order consistency of the *pose* block.  This is the claim the whole
//      frozen protocol rests on: the assembled `A` is the derivative of the
//      residual the solver would have seen.
//
//   2. Frame equivalence.  The SPEC asks for "non-symmetric / units / frame
//      equivalence" (SPEC :51).  Rotating the *whole* system -- every body's
//      position and orientation -- must rotate the tangent by congruence,
//      `A' = T A T^T` with `T` the block rotation, because the residual and the pose
//      increment are both vectors that rotate.  This is a genuinely independent
//      claim from (1): a tangent with a transposed or block-mismatched layout passes
//      a scalar comparison and fails this one.  The SPEC's clause also names *unit*
//      equivalence, which item 4b covers by rescaling the coordinate units.
//
//   3. An ASYMMETRIC tangent transposes correctly.  The design (SPEC :67) requires
//      direct and true-transpose paths to agree on the same frozen oracle, and
//      `mb_linear/batch.hpp` documents that no backend solves `A^T` natively.  A
//      symmetric fixture would make "assume symmetry" pass, so the fixture here is
//      deliberately non-symmetric and the test asserts that it really is.
//
//   4. Many-modes/few-measurements via the adjoint `A^T Z = G^T`.  This is the
//      cheap route the design describes (design :428).  It is checked to give the
//      *same* answer as the direct solve, which is the only claim that matters: the
//      adjoint is a performance strategy, not a different physics.
//
//      Its coverage is stated honestly rather than overstated: the comparison is an
//      *identity* (`Z^T B_w P` equals `G A^{-1} B_w P` algebraically by construction
//      of `Z`), so it does not by itself catch a transpose that was silently solved
//      forward.  What catches that is test 3, which asserts the forward and transpose
//      solutions genuinely differ on the asymmetric fixture and then verifies
//      `A^T x = b` against the original matrix.  Test 4's job is narrower and real:
//      it pins the *route* -- that solving one adjoint column reproduces the
//      many-mode answer, which is the shape the design's performance argument rests
//      on -- so a change that made the adjoint path answer a different system would
//      fail here.
//
//   4b. Direct/transpose equivalence on the *real* frozen tangent.  Tests 3 and 4
//      above run on a hand-built matrix, which pins the arithmetic but not the
//      physics: the spring fixture's assembled tangent (rows == 0) is exactly
//      symmetric, so a transpose check there would be vacuous.  The design (:439)
//      asks for the equivalence on "同一物理问题 ... 含非对称切线", so a fixture with a
//      genuinely asymmetric frozen tangent is assembled and the direct route
//      (`A X = B_w P`) and the adjoint route (`A^T Z = G^T`) are required to agree,
//      with the forward/transpose solutions asserted to differ so the agreement is
//      not the symmetric-matrix coincidence.  The fixture is a **supported** model
//      -- a symmetric bushing plus one `AXLE_INPLANE` row -- because an asymmetric
//      *bushing stiffness* is rejected by both model builders, so a fixture built
//      that way would exercise the assembler on a matrix no supported model can
//      produce.  The same fixture additionally requires the assembled `A` (and not
//      `A^T`) to predict the nonlinear residual's change, which is what rules out a
//      tangent that is internally consistent but the derivative of the wrong system.
//
//   5. The original frozen state is not modified.  Every routine here takes the
//      base pose by const reference, but "takes it by const reference" is a
//      statement about the signature, not about the three library calls it makes.
//      The state is copied before and compared bitwise after, because a routine
//      that quietly advances the pose would make the next question be asked about a
//      different mechanism -- and every number would still look plausible.

#include "mb_linearization/types.hpp"
#include "mb_linear/batch.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_solve_static/functions.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
using axle_kernel::SampleInput;
using axle_kernel::State;
using axle_kernel::WorkingSample;

// --- fixtures ------------------------------------------------------------------

// A two-body model with one free body and a real spring to a grounded body.
//
// The spring is not decoration.  A fixture with no stiffness has an identically
// zero residual and an identically zero tangent, and then "the tangent predicts the
// residual change" is satisfied by `0 == 0` no matter what the assembly does.  The
// oracle is only worth running when the residual it perturbs has curvature in it,
// so the geometry is deliberately off-axis: the spring direction, the lever arm and
// the magnitude all change with the pose, which is exactly what makes the second
// derivative non-zero and the first-order comparison meaningful.
//
// Body 1 is grounded (`body_to_free == -1`) and carries the spring's far end, so the
// spring has a fixed anchor.  `ndof` counts the free block only.
Model make_model() {
    Model model;
    model.ndof = 6;
    model.rows = 0;
    model.bodies.resize(2);
    model.free_body.assign(1, 0);
    model.body_to_free.assign(2, -1);
    model.body_to_free[0] = 0;

    axle_kernel::Spring spring;
    spring.a = 0;      // the free body
    spring.b = 1;      // the grounded anchor
    spring.pa = axle_kernel::Vec3{0.0, 0.0, 0.0};
    spring.pb = axle_kernel::Vec3{0.0, 0.0, 0.0};
    spring.k = 5000.0;
    // Chosen so the spring is under real tension at the base pose: a free length
    // equal to the current length would put the elastic force at zero and hide a
    // dropped stiffness behind a zero right-hand side.
    spring.free_length = 0.5;
    model.springs.push_back(spring);
    return model;
}

State make_pose() {
    // Every per-body array the force assembly reads is sized: `r`/`q` for the pose,
    // `v`/`omega` because the force laws take velocity-dependent terms even at a
    // static point.  A fixture that left one of them empty would index out of bounds
    // inside the residual rather than fail an assertion, which is why this builder
    // fills them all instead of only the two the perturbation actually changes.
    //
    // The grounded body sits off-axis so the spring direction is not a coordinate
    // axis; a perturbation then changes both the length and the direction.
    State pose;
    pose.r.assign(2, axle_kernel::Vec3{});
    pose.r[1] = axle_kernel::Vec3{0.1, -0.2, -0.5};
    pose.q.assign(2, axle_kernel::Quat{1.0, 0.0, 0.0, 0.0});
    pose.v.assign(2, axle_kernel::Vec3{});
    pose.omega.assign(2, axle_kernel::Vec3{});
    return pose;
}

// Build the working sample the way the frozen protocol does.
//
// This step is not optional bookkeeping: `assemble_physical_tangent` reads
// `working.pose` and `working.position_constraint`, and it refuses when they are
// empty rather than deriving them (that derivation would call *up* into the
// solvers -- see `WorkingSample`'s layering note).  So a fixture that skipped this
// call would test nothing but the refusal.
bool make_sample(const Model& model, const State& pose, WorkingSample& working) {
    const std::vector<int> no_tires;
    if (!axle_kernel::frozen_working_sample(model, pose, no_tires, working)) {
        return false;
    }
    return true;
}

double value_at(const std::vector<double>& v, int index) {
    return v[static_cast<std::size_t>(index)];
}

// Evaluate the residual at `perturbed` increments, with the caller's multiplier.
//
// `load_scale` is pinned to 1.0 by the frozen protocol itself
// (`physical.cpp:350-353`), so it is not a knob here either.
std::vector<double> residual_at(
    const Model& model,
    const State& base,
    const SampleInput& sample,
    const std::vector<double>& x,
    double& force_residual,
    double& position_residual
) {
    const std::vector<int> no_tires;
    return axle_kernel::static_residual(
        model, base, sample, 0.0, 0.0, 0.0, no_tires, x,
        force_residual, position_residual, nullptr, nullptr, 1.0
    );
}

// The residual has `ndof + rows + active_tires` entries; the tangent has the same
// dimension, so the comparison is over the mechanical block only (`ndof`), which is
// what the frozen protocol's `A` describes.
std::vector<double> mechanical_block(
    const std::vector<double>& residual, int ndof
) {
    return std::vector<double>(residual.begin(), residual.begin() + ndof);
}

// Multiply a row-major `dim x dim` matrix by a vector.
//
// An empty or short matrix here means an earlier assembly was refused and the test
// is about to index nothing.  That case is made loud rather than silently padded:
// an empty matrix would otherwise produce an all-zero prediction, and a comparison
// against an all-zero residual change would pass while testing nothing.
std::vector<double> apply(
    const std::vector<double>& matrix, const std::vector<double>& x, int dim
) {
    if (static_cast<int>(matrix.size()) != dim*dim ||
        static_cast<int>(x.size()) != dim) {
        std::fprintf(
            stderr,
            "FAIL: apply() got a %zu-entry matrix and %zu-entry vector for dim %d\n",
            matrix.size(), x.size(), dim
        );
        ++failures;
        return std::vector<double>(static_cast<std::size_t>(dim), 0.0);
    }
    std::vector<double> out(static_cast<std::size_t>(dim), 0.0);
    for (int row = 0; row < dim; ++row) {
        double sum = 0.0;
        for (int col = 0; col < dim; ++col) {
            sum += matrix[static_cast<std::size_t>(row*dim + col)]*
                   value_at(x, col);
        }
        out[static_cast<std::size_t>(row)] = sum;
    }
    return out;
}

double max_abs_difference(
    const std::vector<double>& a, const std::vector<double>& b
) {
    // A length mismatch is reported rather than truncated: comparing only the common
    // prefix would let a shortened result pass against a full one.
    if (a.size() != b.size()) {
        std::fprintf(
            stderr,
            "FAIL: max_abs_difference() got %zu vs %zu entries\n",
            a.size(), b.size()
        );
        ++failures;
        return std::numeric_limits<double>::infinity();
    }
    double worst = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        worst = std::max(worst, std::abs(a[i] - b[i]));
    }
    return worst;
}

// One entry of the rotation matrix `R(q)`, as a plain number.
//
// Built by rotating each basis vector rather than by reading a matrix accessor, so
// the test states the rotation in the same terms the kernel's `rotate` uses.  A
// helper that re-derived `R` from the quaternion by hand would be a second
// implementation to get wrong.
double rotation_entry(const axle_kernel::Quat& q, int row, int column) {
    axle_kernel::Vec3 basis{};
    if (column == 0) basis.x = 1.0;
    else if (column == 1) basis.y = 1.0;
    else basis.z = 1.0;
    const axle_kernel::Vec3 rotated = axle_kernel::rotate(q, basis);
    if (row == 0) return rotated.x;
    if (row == 1) return rotated.y;
    return rotated.z;
}

// --- 1. first-order consistency of the pose block -------------------------------

void test_the_analytic_tangent_predicts_the_nonlinear_residual() {
    // The central claim.  `A dx` must reproduce `R(x0+dx) - R(x0)` to first order,
    // and the discrepancy must fall off roughly linearly with the step.  A tangent
    // that is off by a constant factor passes at no step; one that is off by an
    // additive constant passes only at large steps; one with the right first
    // derivative passes at every step with a shrinking error.
    const Model model = make_model();
    const State base = make_pose();
    const SampleInput sample;
    WorkingSample working;
    check(make_sample(model, base, working),
          "the frozen working sample is built for the oracle fixture");

    std::vector<double> x(6, 0.0);
    const std::vector<double> identity_pose =
        [&] { std::vector<double> v(6, 0.0); return v; }();

    double force_before = 0.0, position_before = 0.0;
    const std::vector<double> residual_before = mechanical_block(
        residual_at(model, base, sample, x, force_before, position_before), 6
    );

    std::vector<double> tangent;
    const bool assembled = axle_kernel::assemble_physical_tangent(
        model, sample, working, identity_pose, tangent
    );
    check(assembled, "the physical tangent assembles for the oracle fixture");
    check(tangent.size() == 36, "the assembled tangent is ndof x ndof");

    // Three steps.  The discrepancy is `|0.5*h^2*v^T H v + h*(dA v)|`: the honest
    // truncation plus whatever the tangent is wrong by.  With no error the first term
    // dominates and the ratio between successive steps is ~100; if the linear term
    // dominated it would be ~10.  The assertions below threshold that *ratio* at 50,
    // which is what sits between the two regimes.
    //
    // This makes the oracle's resolution a measured quantity rather than a guessed
    // one.  An additive error `dA` is only resolved once `h*|dA|` becomes comparable
    // to the truncation at the smallest step, so for this fixture (tangent entries
    // ~5000, `h` down to 1e-6) the crossover is around `0.1`-`0.5`: measured, `dA=1e-3`
    // leaves the ratio at 102 -- indistinguishable from the error-free 100 -- and
    // `dA=0.1` reaches 53.5, which clears the threshold by only 7%.  The bound is
    // stated here so that a reader does not mistake "the oracle passes" for "the
    // tangent is exactly right to machine precision".
    const double steps[3] = {1e-4, 1e-5, 1e-6};
    double errors[3] = {0.0, 0.0, 0.0};
    for (int index = 0; index < 3; ++index) {
        const double step = steps[index];
        // A direction with no symmetry: different magnitude on every coordinate, so
        // a tangent that mixed up two rows cannot pass by cancellation.
        std::vector<double> dx(6, 0.0);
        for (int i = 0; i < 6; ++i) {
            dx[static_cast<std::size_t>(i)] = step*(1.0 + i);
        }
        std::vector<double> perturbed(6, 0.0);
        for (int i = 0; i < 6; ++i) {
            perturbed[static_cast<std::size_t>(i)] = value_at(dx, i);
        }
        double force_after = 0.0, position_after = 0.0;
        const std::vector<double> residual_after = mechanical_block(
            residual_at(
                model, base, sample, perturbed, force_after, position_after
            ),
            6
        );

        std::vector<double> predicted = apply(tangent, dx, 6);
        std::vector<double> observed(6, 0.0);
        for (int i = 0; i < 6; ++i) {
            observed[static_cast<std::size_t>(i)] =
                value_at(residual_after, i) - value_at(residual_before, i);
        }
        errors[index] = max_abs_difference(predicted, observed);
    }

    std::printf(
        "  pose-block oracle error: %.3e (h=1e-4), %.3e (1e-5), %.3e (1e-6)\n",
        errors[0], errors[1], errors[2]
    );

    // The residual is non-trivial: if both were identically zero every ratio below
    // would be `0/0` or pass trivially, so the fixture's own liveness is asserted
    // first.  A spring at free length, or a missing element, would show up here.
    check(errors[0] > 0.0, "the oracle fixture has a non-zero residual change");
    check(errors[1] < errors[0]/50.0,
          "the pose-block discrepancy falls at the second-order rate (decade 1)");
    check(errors[2] < errors[1]/50.0,
          "the pose-block discrepancy falls at the second-order rate (decade 2)");
    // And the errors are *small* in absolute terms against the perturbation.
    check(errors[2] < 1e-6,
          "the analytic tangent reproduces the residual change to O(h^2)");
}

// --- 2. frame equivalence -------------------------------------------------------

void test_the_tangent_is_frame_equivalent() {
    // Rotate the entire system -- both bodies' positions and orientations -- and the
    // tangent must transform by congruence: `A' = T A T^T`, where `T` is the block
    // rotation acting on the 6-vector of a body's increment (three translations,
    // three rotations).
    //
    // This is stronger than repeating the residual comparison in a second frame: it
    // pins the *layout* of the matrix.  A tangent whose translational and rotational
    // blocks were swapped, or which rotated one and not the other, would still
    // predict the residual in the base frame and fail here.
    //
    // The world is rotated by a quarter turn about z, so the transformation is a
    // permutation with signs rather than the identity: a routine that ignored the
    // frame entirely would leave the tangent unchanged and be caught.
    const Model model = make_model();
    const State base = make_pose();
    const SampleInput sample;

    WorkingSample working;
    check(make_sample(model, base, working), "the base frame sample is built");
    std::vector<double> tangent;
    const std::vector<double> zero_increment(6, 0.0);
    check(axle_kernel::assemble_physical_tangent(
              model, sample, working, zero_increment, tangent),
          "the base frame tangent assembles");
    check(tangent.size() == 36, "the base frame tangent is 6x6");

    // The rotated system: every body's position and orientation is turned.
    const double half = 3.14159265358979323846/4.0;
    const axle_kernel::Quat turn{
        std::cos(half), 0.0, 0.0, std::sin(half)
    };
    State turned = base;
    for (std::size_t body = 0; body < turned.r.size(); ++body) {
        turned.r[body] = axle_kernel::rotate(turn, turned.r[body]);
        turned.q[body] = axle_kernel::qmul(turn, turned.q[body]);
    }

    WorkingSample turned_working;
    check(make_sample(model, turned, turned_working),
          "the rotated frame sample is built");
    std::vector<double> rotated_tangent;
    check(axle_kernel::assemble_physical_tangent(
              model, sample, turned_working, zero_increment, rotated_tangent),
          "the rotated frame tangent assembles");
    check(rotated_tangent.size() == 36, "the rotated frame tangent is 6x6");

    // `T` acts on the 6-vector `(dx, dtheta)` as `(R dx, R dtheta)`: the rotational
    // part is a world-frame pseudovector, so it rotates by the same `R` (no extra
    // sign), which is the convention `pose_candidate` already uses.
    double worst = 0.0;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            // `(T A T^T)[row][col] = sum_ij T[row][i] A[i][j] T[col][j]`.
            double expected = 0.0;
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    // `T` is block-diagonal: index within the body block is
                    // `component`, and the block is the same `R` for both halves.
                    const int component_row = row % 3;
                    const int component_i = i % 3;
                    const int component_col = col % 3;
                    const int component_j = j % 3;
                    // Only same-half entries are non-zero (`R` is applied within the
                    // three translational slots and again within the rotational).
                    if ((row / 3) != (i / 3)) continue;
                    if ((col / 3) != (j / 3)) continue;
                    const double t_row =
                        rotation_entry(turn, component_row, component_i);
                    const double t_col =
                        rotation_entry(turn, component_col, component_j);
                    expected += t_row*
                        tangent[static_cast<std::size_t>(i*6 + j)]*t_col;
                }
            }
            const double actual =
                rotated_tangent[static_cast<std::size_t>(row*6 + col)];
            worst = std::max(worst, std::abs(actual - expected));
        }
    }
    std::printf("  frame-equivalence worst deviation: %.3e\n", worst);
    check(worst < 1e-8,
          "rotating the whole system rotates the tangent by congruence");

    // The fixture is live only if the tangent is not rotation-invariant, otherwise
    // the congruence check above would hold for any matrix that happens to be
    // unchanged.  The threshold is a *relative* one -- the tangent entries are ~5000
    // here, so an absolute `1e-6` would be a 1e-10 relative change and would pass on
    // a tangent that barely moved.  Comparing against the matrix's own magnitude
    // keeps the assertion meaningful if the fixture's stiffness is ever retuned.
    double scale = 0.0;
    for (int i = 0; i < 36; ++i) {
        scale = std::max(scale, std::abs(tangent[static_cast<std::size_t>(i)]));
    }
    check(scale > 1.0, "the fixture's tangent has a non-trivial magnitude");
    double difference = 0.0;
    for (int i = 0; i < 36; ++i) {
        difference = std::max(
            difference,
            std::abs(tangent[static_cast<std::size_t>(i)] -
                     rotated_tangent[static_cast<std::size_t>(i)])
        );
    }
    std::printf(
        "  frame-equivalence change: %.3e (scale %.3e, relative %.3e)\n",
        difference, scale, difference/scale
    );
    check(difference > 1e-3*scale,
          "the tangent really changed under the rotation (the fixture is live)");
}

// --- 3. the transpose system, on an asymmetric tangent --------------------------

void test_an_asymmetric_tangent_transposes_exactly() {
    // `mb_linear/batch.hpp` states that no backend in the tree solves `A^T`
    // natively and that `solve_transpose_many` forms the transpose exactly rather
    // than assuming symmetry.  This is the assertion that the *implementation*
    // matches that documentation.
    //
    // The fixture is deliberately non-symmetric, and the test first *proves* it is,
    // because "assume symmetry" is exactly the implementation this catches and a
    // symmetric fixture would let it pass.
    const int dim = 4;
    std::vector<double> matrix(static_cast<std::size_t>(dim*dim), 0.0);
    // A well-conditioned, clearly asymmetric matrix.
    const double values[16] = {
        4.0, 1.0, 0.0, 0.5,
        0.0, 3.0, 1.0, 0.0,
        0.5, 0.0, 5.0, 1.0,
        0.0, 1.0, 0.0, 2.0
    };
    for (int i = 0; i < dim*dim; ++i) {
        matrix[static_cast<std::size_t>(i)] = values[i];
    }
    bool asymmetric = false;
    for (int row = 0; row < dim; ++row) {
        for (int col = 0; col < dim; ++col) {
            const double a = matrix[static_cast<std::size_t>(row*dim + col)];
            const double b = matrix[static_cast<std::size_t>(col*dim + row)];
            if (std::abs(a - b) > 1e-12) asymmetric = true;
        }
    }
    check(asymmetric, "the transpose fixture is really asymmetric");

    axle_kernel::LinearFactorHandle handle;
    check(handle.factorize_numeric(matrix, dim), "the asymmetric fixture factors");

    // One right-hand side, solved forward and transposed.
    std::vector<double> rhs(static_cast<std::size_t>(dim), 0.0);
    rhs[0] = 1.0; rhs[1] = -2.0; rhs[2] = 0.5; rhs[3] = 3.0;

    std::vector<double> forward;
    int forward_factorizations = 0;
    check(axle_kernel::solve_many(handle, rhs, 1, forward, forward_factorizations),
          "the forward solve succeeds");
    check(forward_factorizations == 1, "the forward solve reuses one factorization");
    check(forward.size() == static_cast<std::size_t>(dim), "forward is dim long");

    std::vector<double> transposed;
    int transpose_factorizations = 0;
    check(axle_kernel::solve_transpose_many(
              handle, rhs, 1, transposed, transpose_factorizations),
          "the transpose solve succeeds");
    // The honest cost: no backend solves `A^T` natively, so a second factorization
    // is built.  Asserting the number keeps the "we did not secretly assume
    // symmetry" claim pinned to an observable.
    check(transpose_factorizations == 2,
          "the transpose reports the second factorization it really built");

    // The two are NOT the same answer -- on an asymmetric matrix they must not be.
    // If a routine returned the forward solution for both, this catches it.
    double difference = 0.0;
    for (int i = 0; i < dim; ++i) {
        difference = std::max(
            difference,
            std::abs(value_at(forward, i) - value_at(transposed, i))
        );
    }
    check(difference > 1e-3,
          "the forward and transpose solutions genuinely differ (fixture is live)");

    // And the transpose solution really solves `A^T x = b`: verified by applying
    // `A^T` to it, which uses the *original* matrix rather than the factorization.
    std::vector<double> transposed_matrix(static_cast<std::size_t>(dim*dim), 0.0);
    for (int row = 0; row < dim; ++row) {
        for (int col = 0; col < dim; ++col) {
            transposed_matrix[static_cast<std::size_t>(row*dim + col)] =
                matrix[static_cast<std::size_t>(col*dim + row)];
        }
    }
    const std::vector<double> applied = apply(transposed_matrix, transposed, dim);
    check(max_abs_difference(applied, rhs) < 1e-9,
          "A^T x really equals the right-hand side the caller passed");
}

// --- 4. the adjoint route agrees with the direct solve --------------------------

void test_the_adjoint_route_agrees_with_the_direct_solve() {
    // For few measurements against many load modes the design suggests
    // `A^T Z = G^T` then `Y = Z^T B_w P + D P` (design :428).  That is a
    // performance strategy, so the only thing worth asserting is that it produces
    // the *same* number as the direct route on the same system.
    //
    // `dim = 3`, two modes, one measurement: the adjoint is a 1-column solve while
    // the direct route is 2 columns, which is the shape the strategy exists for.
    const int dim = 3;
    const int modes = 2;
    std::vector<double> matrix(static_cast<std::size_t>(dim*dim), 0.0);
    const double values[9] = {
        3.0, 1.0, 0.0,
        0.5, 4.0, 1.0,
        0.0, 2.0, 5.0
    };
    for (int i = 0; i < dim*dim; ++i) {
        matrix[static_cast<std::size_t>(i)] = values[i];
    }
    axle_kernel::LinearFactorHandle handle;
    check(handle.factorize_numeric(matrix, dim), "the adjoint fixture factors");

    // `B_w P` as (dim, modes), column-major for the batch API.
    std::vector<double> rhs(static_cast<std::size_t>(dim*modes), 0.0);
    rhs[0] = 1.0; rhs[1] = 0.0; rhs[2] = 0.0;              // mode 0
    rhs[dim + 0] = 0.0; rhs[dim + 1] = 1.0; rhs[dim + 2] = 0.0;  // mode 1

    std::vector<double> direct;
    int direct_factorizations = 0;
    check(axle_kernel::solve_many(handle, rhs, modes, direct, direct_factorizations),
          "the direct many-mode solve succeeds");
    check(direct_factorizations == 1, "the direct solve factors once for both modes");

    // The measurement row `G`, applied afterwards.
    const double g[3] = {2.0, -1.0, 0.5};
    std::vector<double> direct_measurement(static_cast<std::size_t>(modes), 0.0);
    for (int mode = 0; mode < modes; ++mode) {
        double sum = 0.0;
        for (int row = 0; row < dim; ++row) {
            sum += g[row]*direct[static_cast<std::size_t>(mode*dim + row)];
        }
        direct_measurement[static_cast<std::size_t>(mode)] = sum;
    }

    // The adjoint: solve `A^T Z = G^T` (one column) ...
    std::vector<double> g_column(static_cast<std::size_t>(dim), 0.0);
    for (int i = 0; i < dim; ++i) g_column[static_cast<std::size_t>(i)] = g[i];
    std::vector<double> z;
    int adjoint_factorizations = 0;
    check(axle_kernel::solve_transpose_many(
              handle, g_column, 1, z, adjoint_factorizations),
          "the adjoint solve succeeds");

    // ... then form `Z^T (B_w P)` by dotting the solved `z` against each mode
    // column.  This is exactly `Y = Z^T B_w P`, and it uses the *raw* right-hand
    // sides rather than the direct solution.
    std::vector<double> adjoint_measurement(static_cast<std::size_t>(modes), 0.0);
    for (int mode = 0; mode < modes; ++mode) {
        double sum = 0.0;
        for (int row = 0; row < dim; ++row) {
            sum += value_at(z, row)*rhs[static_cast<std::size_t>(mode*dim + row)];
        }
        adjoint_measurement[static_cast<std::size_t>(mode)] = sum;
    }

    std::printf(
        "  adjoint vs direct: %.12e vs %.12e  (max diff %.3e)\n",
        value_at(adjoint_measurement, 0), value_at(direct_measurement, 0),
        max_abs_difference(adjoint_measurement, direct_measurement)
    );
    check(max_abs_difference(adjoint_measurement, direct_measurement) < 1e-9,
          "the adjoint route reports the same measurements as the direct solve");
}

// --- 4b. direct/transpose equivalence on the REAL asymmetric frozen tangent ----

// A bushing with an asymmetric stiffness, driving the free body against a grounded
// one.  This fixture exists because the tests above check the transpose on a
// *hand-built* matrix, which pins the arithmetic but not the physics: a frozen
// tangent can be asymmetric for reasons the synthetic fixture has no way to
// represent.  Measured on the spring fixture above (rows == 0), the assembled
// tangent is exactly symmetric, so a transpose check there would be vacuous -- the
// asymmetry has to be put into the model.  A bushing is the cheapest way to do it:
// it is a pure force element (no constraint rows), so its stiffness enters the pose
// block directly and a nonsymmetric K makes the tangent nonsymmetric.
//
// Design :439 asks for exactly this -- "同一物理问题中 ... 直接模式响应、转置求解和
// 矩阵乘模式的数值等价，含非对称切线" -- so the claim being pinned is the design's,
// not a synthetic one.
Model make_asymmetric_model() {
    // The model is a SYMMETRIC bushing plus one constraint row.  That combination is
    // not arbitrary: an asymmetric *bushing stiffness* is rejected outright by both
    // model builders (`build_model.cpp:326`, `element_reader.cpp:311` both require
    // ``bushing matrices must be symmetric``), so a fixture built that way would
    // exercise the assembler on a matrix no supported model can produce.  The
    // supported source of asymmetry is the **constraint row**: `assemble_physical_tangent`
    // writes `A[pose][mult] = -J` and `A[mult][pose] = +J` (physical.cpp:196-208), so a
    // non-zero `J` makes `A` asymmetric exactly as a loaded linkage does.
    //
    // `AXLE_INPLANE` is chosen because it contributes exactly one row
    // (`joint/types.hpp` row table), so `model.rows = 1` and the tangent is 7x7.  The
    // bushing is full 6-DOF and symmetric, so the pose block is non-singular and the
    // whole tangent factorizes without a gauge.
    Model model;
    model.ndof = 6;
    model.rows = 1;
    model.bodies.resize(2);
    model.free_body.assign(1, 0);
    model.body_to_free.assign(2, -1);
    model.body_to_free[0] = 0;

    axle_kernel::Bushing bushing;
    bushing.a = 1;   // the grounded body
    bushing.b = 0;   // the free body
    bushing.pa = axle_kernel::Vec3{0.0, 0.0, 0.0};
    bushing.pb = axle_kernel::Vec3{0.0, 0.0, 0.0};
    std::array<double, 36> stiffness{};
    for (int i = 0; i < 6; ++i) {
        stiffness[static_cast<std::size_t>(i*6 + i)] = 1000.0*(1.0 + i);
    }
    // Symmetric off-diagonal pairs: legal at every model entry, and enough to make the
    // pose block a general (non-diagonal) matrix.
    stiffness[0*6 + 1] = 250.0; stiffness[1*6 + 0] = 250.0;
    stiffness[2*6 + 5] = 120.0; stiffness[5*6 + 2] = 120.0;
    bushing.stiffness = stiffness;
    model.bushings.push_back(bushing);

    axle_kernel::Constraint joint;
    joint.type = AXLE_INPLANE;      // exactly one row
    joint.a = 1;                    // the grounded body
    joint.b = 0;                    // the free body
    // The normal is not a coordinate axis, so `J` couples several pose coordinates and
    // the KKT block is genuinely non-symmetric rather than a near-diagonal artefact.
    joint.axis_a = axle_kernel::Vec3{0.3, -0.7, 0.6};
    joint.axis_b = axle_kernel::Vec3{0.3, -0.7, 0.6};
    joint.pa = axle_kernel::Vec3{0.0, 0.0, 0.0};
    joint.pb = axle_kernel::Vec3{0.0, 0.0, 0.0};
    joint.row = 0;
    model.constraints.push_back(joint);
    return model;
}

void test_the_direct_and_transpose_routes_agree_on_the_frozen_tangent() {
    // The load column as the right-hand side, in the two routes the design names.
    // Direct:  `A X = B_w P`, `Y = G X`.  Adjoint: `A^T Z = G^T`, `Y = Z^T B_w P`.
    const Model model = make_asymmetric_model();
    const State base = make_pose();
    const SampleInput sample;
    WorkingSample working;
    check(make_sample(model, base, working),
          "the asymmetric frozen sample is built");

    const int dim = axle_kernel::physical_tangent_dimension(model, working);
    check(dim == model.ndof + model.rows,
          "the asymmetric fixture has no active tires, so dim == ndof + rows");
    std::vector<double> A;
    std::vector<double> zero(static_cast<std::size_t>(dim), 0.0);
    const bool assembled =
        axle_kernel::assemble_physical_tangent(model, sample, working, zero, A);
    check(assembled, "the asymmetric frozen tangent assembles");
    check(A.size() == static_cast<std::size_t>(dim)*static_cast<std::size_t>(dim),
          "the asymmetric frozen tangent is dim x dim");
    if (A.size() != static_cast<std::size_t>(dim)*static_cast<std::size_t>(dim)) {
        return;
    }

    // Liveness of the fixture itself: the tangent must really be asymmetric, or the
    // transpose route below is satisfied by `A == A^T` no matter what it does.
    double asymmetry = 0.0;
    for (int row = 0; row < dim; ++row) {
        for (int col = 0; col < dim; ++col) {
            const double a = A[static_cast<std::size_t>(row*dim + col)];
            const double b = A[static_cast<std::size_t>(col*dim + row)];
            asymmetry = std::max(asymmetry, std::abs(a - b));
        }
    }
    check(asymmetry > 1.0,
          "the frozen fixture's tangent is really asymmetric (the route is live)");

    // --- the tangent must also be the *residual's* derivative, not only self-consistent
    //
    // Reviewer finding, and the reason this block exists: the direct/transpose checks
    // below compare two algebraic routes over the *same* assembled matrix, so a tangent
    // whose derivative was written as `K^T` -- while the residual still uses `K` --
    // would satisfy every one of them.  So the assembled `A` is required to predict the
    // nonlinear residual's change, and the transposed matrix is required to *fail* that
    // same check, which is what makes "the asymmetric tangent is the right one"
    // observable rather than assumed.
    const std::vector<int> no_tires;
    auto residual_at = [&](const std::vector<double>& increment) {
        double force = 0.0, position = 0.0;
        return axle_kernel::static_residual(
            model, base, sample, 0.0, 0.0, 0.0, no_tires, increment,
            force, position, nullptr, nullptr, 1.0
        );
    };
    const std::vector<double> residual_base =
        residual_at(std::vector<double>(static_cast<std::size_t>(dim), 0.0));
    check(residual_base.size() == static_cast<std::size_t>(dim),
          "the asymmetric fixture's residual spans the tangent dimension");
    // Two steps.  The leading error is the honest `O(h^2)` term, and the absolute floor
    // is set by the residual's own magnitude (~1.5e3 here), so the assertions below are
    // on *predicts vs does not predict*, not on a rate (that rate is test 1's job).
    const double steps[2] = {1e-4, 1e-5};
    for (const double step : steps) {
        std::vector<double> increment(static_cast<std::size_t>(dim), 0.0);
        for (int index = 0; index < dim; ++index) {
            increment[static_cast<std::size_t>(index)] = step*(1.0 + index);
        }
        const std::vector<double> residual_moved = residual_at(increment);
        double error_with_A = 0.0, error_with_at = 0.0;
        for (int row = 0; row < dim; ++row) {
            double predicted = 0.0, predicted_transposed = 0.0;
            for (int col = 0; col < dim; ++col) {
                predicted += A[static_cast<std::size_t>(row*dim + col)]*
                    value_at(increment, col);
                predicted_transposed += A[static_cast<std::size_t>(col*dim + row)]*
                    value_at(increment, col);
            }
            const double observed =
                value_at(residual_moved, row) - value_at(residual_base, row);
            error_with_A = std::max(error_with_A, std::abs(predicted - observed));
            error_with_at = std::max(
                error_with_at, std::abs(predicted_transposed - observed)
            );
        }
        std::printf(
            "  frozen asymmetric residual oracle (h=%.0e): "
            "|A dx - dR| = %.3e, |A^T dx - dR| = %.3e\n",
            step, error_with_A, error_with_at
        );
        check(error_with_A < 1e-9,
              "the assembled asymmetric tangent predicts the residual's change");
        check(error_with_at > 100.0*std::max(error_with_A, 1e-16),
              "the transposed matrix does NOT predict it (the check discriminates)");
    }

    // The load column: a unit x-force on the free body, `B_w = -dR/dw`.
    //
    // `frozen_load_column` spans the **pose** coordinates (`ndof` entries); the tangent
    // also carries the multiplier rows, so the right-hand side is the column padded with
    // zeros on the multiplier block -- a load produces no direct multiplier residual.
    const std::vector<double> load_column =
        axle_kernel::frozen_load_column(model, base, 0, 0);
    check(load_column.size() == static_cast<std::size_t>(model.ndof),
          "the load column spans the pose coordinates");
    std::vector<double> rhs(static_cast<std::size_t>(dim), 0.0);
    for (std::size_t index = 0; index < load_column.size(); ++index) {
        rhs[index] = load_column[index];
    }

    axle_kernel::LinearFactorHandle handle;
    check(handle.factorize_numeric(A, dim), "the asymmetric frozen tangent factors");

    // Route 1: the direct solve with the load column as its single right-hand side.
    std::vector<double> forward;
    int forward_factorizations = 0;
    check(axle_kernel::solve_many(
              handle, rhs, 1, forward, forward_factorizations),
          "the direct load-column solve succeeds");

    // The measurement is a general linear combination of the coordinates, and *which*
    // coordinates it touches is load-bearing rather than cosmetic.  Two facts, both
    // measured rather than assumed:
    //
    //   * With `G = e_0` and this `B_w = -e_0` both routes would read the same single
    //     entry `(A^{-1})[0][0]`, and swapping the adjoint solve for a forward one
    //     would be invisible -- that mutation survived.
    //   * Even a *general* pose-only `G` does not distinguish the two routes on this
    //     fixture: with a pose-only right-hand side and a pose-only measurement, the
    //     pose block of `A^{-1}` is symmetric (a Schur complement of a symmetric
    //     factor), so `G^T A^{-1} b == (A^{-T} G)^T b` identically.  Measured: the
    //     forward and forward-wrong adjoint gave the same -1.196e-03.
    //
    // What makes the route observable is that the measurement also reads the
    // **multiplier** (constraint reaction) coordinate -- a real physical output, and
    // exactly where `A`'s KKT block is asymmetric.  Measured with the component below,
    // the correct adjoint agrees to 1.1e-16 while a forward solve in its place is
    // wrong by 1.34.
    const double g_values[6] = {2.0, -1.0, 0.5, 1.0, -0.25, 0.75};
    const double g_multiplier = 0.9;   // the constraint reaction is part of the reading
    std::vector<double> g(static_cast<std::size_t>(dim), 0.0);
    for (int index = 0; index < dim && index < 6; ++index) {
        g[static_cast<std::size_t>(index)] = g_values[index];
    }
    if (dim > model.ndof) {
        g[static_cast<std::size_t>(model.ndof)] = g_multiplier;
    }
    double y_direct = 0.0;
    for (int index = 0; index < dim; ++index) {
        y_direct += value_at(g, index)*value_at(forward, index);
    }

    // Route 2: the adjoint.  `A^T Z = G^T`, then `Y = Z^T B_w`.
    std::vector<double> z;
    int adjoint_factorizations = 0;
    check(axle_kernel::solve_transpose_many(
              handle, g, 1, z, adjoint_factorizations),
          "the adjoint solve on the frozen tangent succeeds");
    double y_adjoint = 0.0;
    for (int index = 0; index < dim; ++index) {
        y_adjoint += value_at(z, index)*value_at(rhs, index);
    }

    // The two must agree.  Both sides are O(1e-3) here, so an absolute 1e-12 is a
    // relative 1e-9 -- tight enough to catch a wrong route and loose enough for a
    // scale-free double round-off through two different factorizations.
    std::printf(
        "  frozen direct vs transpose: %.12e vs %.12e (diff %.3e)\n",
        y_direct, y_adjoint, std::abs(y_direct - y_adjoint)
    );
    check(std::abs(y_direct - y_adjoint) < 1e-12,
          "the direct and transpose routes agree on the frozen physical tangent");

    // And the routes are *not* the same object: solving the same right-hand side
    // transposed gives a genuinely different vector on this asymmetric matrix.  On a
    // symmetric one the two would coincide and the agreement above would prove
    // nothing about the transpose path -- which is why the fixture is asymmetric and
    // this is asserted rather than assumed.
    std::vector<double> transposed_solution;
    check(axle_kernel::solve_transpose_many(
              handle, rhs, 1, transposed_solution, adjoint_factorizations),
          "the transposed load-column solve succeeds");
    double route_difference = 0.0;
    for (int index = 0; index < dim; ++index) {
        route_difference = std::max(
            route_difference,
            std::abs(value_at(forward, index) - value_at(transposed_solution, index))
        );
    }
    check(route_difference > 1e-6,
          "the forward and transposed solves genuinely differ (the route is live)");

    // Unit equivalence.  The SPEC's clause (SPEC :51) names "非对称 / 单位 / frame"
    // together, and this is the *unit* half.  A change of coordinate units is a
    // diagonal rescaling `x = S x'`, under which the residual invariant gives
    // `A' = A S`, `B_w' = B_w`, and `G' = G S` (so the physical `Y = G X` is
    // unchanged).  Design :342 requires exactly that the solver's internal row/column
    // scaling be restored before output, so a backend that leaked its equilibration
    // into the answer would break this invariance.  The scales are deliberately
    // extreme (1e-3 .. 1e3) -- a mild one would sit inside round-off.
    std::vector<double> scale(static_cast<std::size_t>(dim), 1.0);
    scale[0] = 1e-3;
    scale[3] = 1e3;
    scale[4] = 1e-2;
    std::vector<double> scaled_A(static_cast<std::size_t>(dim)*static_cast<std::size_t>(dim), 0.0);
    for (int row = 0; row < dim; ++row) {
        for (int col = 0; col < dim; ++col) {
            scaled_A[static_cast<std::size_t>(row*dim + col)] =
                A[static_cast<std::size_t>(row*dim + col)]*
                scale[static_cast<std::size_t>(col)];
        }
    }
    std::vector<double> scaled_g(static_cast<std::size_t>(dim), 0.0);
    for (int index = 0; index < dim; ++index) {
        scaled_g[static_cast<std::size_t>(index)] =
            value_at(g, index)*scale[static_cast<std::size_t>(index)];
    }

    axle_kernel::LinearFactorHandle scaled_handle;
    check(scaled_handle.factorize_numeric(scaled_A, dim),
          "the unit-rescaled frozen tangent factors");
    std::vector<double> scaled_forward;
    int scaled_factorizations = 0;
    check(axle_kernel::solve_many(
              scaled_handle, rhs, 1, scaled_forward, scaled_factorizations),
          "the unit-rescaled frozen solve succeeds");
    double y_scaled = 0.0;
    for (int index = 0; index < dim; ++index) {
        y_scaled += value_at(scaled_g, index)*value_at(scaled_forward, index);
    }
    const double unit_scale = std::max(1.0, std::abs(y_direct));
    std::printf(
        "  frozen unit rescaled: %.12e vs %.12e (relative %.3e)\n",
        y_direct, y_scaled, std::abs(y_direct - y_scaled)/unit_scale
    );
    check(std::abs(y_direct - y_scaled) <= 1e-9*unit_scale,
          "the physical response is invariant under a change of coordinate units");

    // Negative control: dropping the compensating `G' = G S` must change the answer,
    // or the invariance above would be satisfied by a rescaled system that happens to
    // give the same number for an unrelated reason.
    double y_uncompensated = 0.0;
    for (int index = 0; index < dim; ++index) {
        y_uncompensated += value_at(g, index)*value_at(scaled_forward, index);
    }
    check(std::abs(y_direct - y_uncompensated) > 1e-3*unit_scale,
          "the unit-equivalence assertion is live (an uncompensated G changes Y)");
}



// --- 5. the frozen state is not modified ----------------------------------------

void test_the_oracle_leaves_the_frozen_state_untouched() {
    // The design requires the local response to be taken at the *original* sample
    // with the state held (`SPEC :59`, design :402 "响应不随时间传播").  Every
    // routine here takes the base pose by const reference, but that says nothing
    // about what they do internally -- so the state is copied and compared.
    //
    // A bitwise comparison of every field the routines are handed, not just the two
    // the perturbation changes: `v`/`omega` participate in the force laws and a
    // routine that advanced them would change the *next* sample's physics while
    // leaving this answer intact.
    //
    // Its honest strength is stated rather than implied: `static_residual` and
    // `assemble_physical_tangent` both take the base pose by `const&`, so a mutation
    // of `base` would not compile.  What this test can actually catch is therefore
    // narrow -- a `const_cast`, or an aliasing bug where the callee writes through a
    // pointer it was handed.  It is kept because that class of bug is real (the
    // residual receives `const std::vector<double>& active_tires`, and `Model` is
    // full of raw pointers), and because the *observable* half of the same claim is
    // asserted below, which is the half that can genuinely fail.
    //
    // The comparison is bitwise, not `close`: a routine that advanced the pose by a
    // tiny amount would pass a tolerance check while asking the next question about
    // a subtly different mechanism.
    const Model model = make_model();
    const State base = make_pose();
    const SampleInput sample;
    WorkingSample working;
    check(make_sample(model, base, working),
          "the frozen working sample is built for the invariance fixture");

    const State before = base;

    auto state_matches = [](const State& a, const State& b) {
        if (a.r.size() != b.r.size() || a.q.size() != b.q.size() ||
            a.v.size() != b.v.size() || a.omega.size() != b.omega.size()) {
            return false;
        }
        for (std::size_t i = 0; i < a.r.size(); ++i) {
            if (a.r[i].x != b.r[i].x || a.r[i].y != b.r[i].y ||
                a.r[i].z != b.r[i].z) return false;
        }
        for (std::size_t i = 0; i < a.q.size(); ++i) {
            if (a.q[i].w != b.q[i].w || a.q[i].x != b.q[i].x ||
                a.q[i].y != b.q[i].y || a.q[i].z != b.q[i].z) return false;
        }
        for (std::size_t i = 0; i < a.v.size(); ++i) {
            if (a.v[i].x != b.v[i].x || a.v[i].y != b.v[i].y ||
                a.v[i].z != b.v[i].z) return false;
            if (a.omega[i].x != b.omega[i].x || a.omega[i].y != b.omega[i].y ||
                a.omega[i].z != b.omega[i].z) return false;
        }
        return true;
    };

    std::vector<double> x(6, 0.0);
    for (int i = 0; i < 6; ++i) x[static_cast<std::size_t>(i)] = 1e-3*(1 + i);

    double force = 0.0, position = 0.0;
    residual_at(model, base, sample, x, force, position);

    std::vector<double> tangent;
    const std::vector<double> pose_increment(6, 0.0);
    axle_kernel::assemble_physical_tangent(
        model, sample, working, pose_increment, tangent
    );

    check(state_matches(base, before),
          "the frozen base state is bitwise unchanged after the oracle runs");

    // --- the observable half of the same claim -----------------------------------
    //
    // The const-reference argument above is structural; this is the part a mutation
    // can genuinely break, and it is the reason the requirement exists at all.
    // "The local response is taken at the original sample" (design :402) means the
    // *answer* must not depend on how many questions were asked or in what order.
    // A routine that advanced the pose between questions would still return a
    // plausible tangent for the first question and a different one for the second,
    // so the two must be bitwise equal.
    //
    // Asking twice from the same base, with the first call's result retained, is the
    // cheapest way to see that: a shared-state implementation gives two different
    // tangents, and every number still looks reasonable.
    std::vector<double> first_tangent;
    WorkingSample first_sample;
    check(make_sample(model, base, first_sample),
          "the first repeated-query sample is built");
    check(axle_kernel::assemble_physical_tangent(
              model, sample, first_sample, pose_increment, first_tangent),
          "the first repeated query assembles");

    std::vector<double> second_tangent;
    WorkingSample second_sample;
    check(make_sample(model, base, second_sample),
          "the second repeated-query sample is built");
    check(axle_kernel::assemble_physical_tangent(
              model, sample, second_sample, pose_increment, second_tangent),
          "the second repeated query assembles");

    check(first_tangent.size() == second_tangent.size() &&
              !first_tangent.empty(),
          "both repeated queries produced a full tangent");
    bool identical = !first_tangent.empty();
    for (std::size_t i = 0; i < first_tangent.size() && identical; ++i) {
        // Bitwise, so an accumulation through shared state cannot hide in a
        // tolerance.  (The frozen protocol's whole value is reproducibility.)
        if (first_tangent[i] != second_tangent[i]) identical = false;
    }
    check(identical,
          "asking the same question twice returns bitwise the same tangent");
}

}  // namespace

int main(int argc, char** argv) {
    // A selector so a crash can be attributed to one case without editing the
    // source.  Commenting calls out instead would leave unused functions behind,
    // and `-Werror=unused-function` would then fail the build -- which is exactly
    // how a "which test crashes?" run silently reports the old binary's exit code.
    int only = -1;
    if (argc > 1) only = std::atoi(argv[1]);

    void (*tests[])() = {
        test_the_analytic_tangent_predicts_the_nonlinear_residual,
        test_the_tangent_is_frame_equivalent,
        test_an_asymmetric_tangent_transposes_exactly,
        test_the_adjoint_route_agrees_with_the_direct_solve,
        test_the_direct_and_transpose_routes_agree_on_the_frozen_tangent,
        test_the_oracle_leaves_the_frozen_state_untouched,
    };
    const int count = static_cast<int>(sizeof(tests)/sizeof(tests[0]));

    for (int index = 0; index < count; ++index) {
        if (only >= 0 && index != only) continue;
        std::printf("[%d] running\n", index);
        std::fflush(stdout);
        tests[index]();
        std::printf("[%d] done\n", index);
        std::fflush(stdout);
    }

    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("OK: frozen oracle self-test passed\n");
    return 0;
}
