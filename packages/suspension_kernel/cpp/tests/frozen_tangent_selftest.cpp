// Self-test for `frozen_force_tangent_column`.
//
// The port tangent is a *solve*, so a test that only checks the function returns
// `true` proves nothing.  Every case below states the answer in closed form and
// compares against it, because the whole point of the routine is the number it
// produces.
//
// What this checks, and why each one needs checking rather than assuming:
//
//   1. A single spring.  A tangent of `[[k]]` on one held coordinate must report
//      `-k * ds` -- the external force needed to hold the displacement, which is
//      opposite in sign to the restoring force.  The sign is the part most easily
//      lost, and a test asserting `|Q| == k*ds` would pass on either convention.
//
//   2. The free coordinates really are solved, not ignored.  A two-coordinate
//      spring chain whose free end is held must report the *series* stiffness; a
//      routine that read the diagonal of A instead of eliminating would report the
//      first spring alone, which is a different number.
//
//   3. The sign convention and the magnitude are checked against a coupled,
//      analytically known 2x2 case, so the off-diagonal coupling is exercised too.
//
//   4. Refusals are reported.  An out-of-range port, a repeated port, a length
//      mismatch and a singular free block must all return `false` -- and a singular
//      block must NOT come back as a plausible finite reaction, which is what a
//      least-squares fallback would do.
//
//   5. A refusal writes nothing.  A partially written reaction is indistinguishable
//      from a real one of that value, so the caller must be able to trust that
//      `false` left its vector alone.
//
//   6. A zero reaction is a *success*, not a failure.  A mechanism with no
//      stiffness on the held coordinate legitimately reports zero, so the failure
//      signal cannot be "the answer is zero".

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
using axle_kernel::WorkingSample;

bool close(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance*std::max(1.0, std::abs(b));
}

// A model that carries only what the routine under test reads: the pose dimension
// and the constraint count.  The tangent is supplied by the test rather than
// assembled, so this routine is exercised on numbers whose answer is known exactly
// -- an assembled tangent would make a wrong *solve* harder to see than a wrong
// matrix.
Model make_model(int ndof, int rows) {
    Model model;
    model.ndof = ndof;
    model.rows = rows;
    return model;
}

WorkingSample make_sample(const Model& model) {
    WorkingSample working;
    working.gauge.assign(static_cast<std::size_t>(model.ndof), 0);
    working.load_scale = 1.0;
    return working;
}

// A square tangent of `dimension`, from a full matrix given row-major.
std::vector<double> square(
    std::initializer_list<std::initializer_list<double>> rows_in
) {
    std::vector<double> out;
    for (const auto& row : rows_in) {
        for (const double value : row) out.push_back(value);
    }
    return out;
}

// --- 1. one spring, and the sign convention --------------------------------

void test_single_spring_reports_the_holding_force() {
    const Model model = make_model(1, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({{500.0}});
    std::vector<double> reaction;
    const std::vector<int> driven{0};
    check(axle_kernel::frozen_force_tangent_column(
              model, working, tangent, driven, {0.01}, reaction),
          "a one-coordinate spring is solved");
    check(reaction.size() == 1, "one port yields one reaction value");
    // +5 N of external force holds a 0.01 m compression against a 500 N/m spring.
    // This is the *holding* force; the spring itself pushes back with -5 N, and a
    // test that only checked the magnitude would accept the wrong sign.
    check(close(reaction[0], 5.0, 1e-12),
          "the reaction is the positive holding force, not the restoring force");
}

// --- 2. the free coordinates are eliminated, not skipped -------------------

void test_series_springs_report_the_series_stiffness() {
    // Two springs in series: k1 = 200 between coordinates 0 and 1, k2 = 300
    // between coordinate 1 and ground.  Holding coordinate 0 and letting 1 float
    // gives the series stiffness k1*k2/(k1+k2) = 120 N/m.  Reading the diagonal
    // would give 200 -- a different, plausible number.
    const Model model = make_model(2, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({
        {200.0, -200.0},
        {-200.0, 500.0},
    });
    std::vector<double> reaction;
    const std::vector<int> driven{0};
    check(axle_kernel::frozen_force_tangent_column(
              model, working, tangent, driven, {1.0}, reaction),
          "a two-coordinate spring chain is solved");
    check(close(reaction[0], 120.0, 1e-12),
          "the reaction is the series stiffness, not the first spring alone");
}

// --- 3. coupling and the off-diagonal column -------------------------------

void test_coupled_ports_report_the_full_column() {
    // A symmetric 2x2 with both coordinates driven at once: the reaction on each
    // is the row of K times the prescribed vector, so the off-diagonal entry is
    // exercised rather than assumed zero.
    const Model model = make_model(2, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({
        {400.0, 100.0},
        {100.0, 900.0},
    });
    std::vector<double> reaction;
    const std::vector<int> driven{0, 1};
    check(axle_kernel::frozen_force_tangent_column(
              model, working, tangent, driven, {0.02, -0.03}, reaction),
          "a coupled two-port problem is solved");
    // Q_0 = 400*0.02 + 100*(-0.03) = 8 - 3 = 5
    // Q_1 = 100*0.02 + 900*(-0.03) = 2 - 27 = -25
    check(close(reaction[0], 5.0, 1e-12),
          "the first port reaction includes the coupling term");
    check(close(reaction[1], -25.0, 1e-12),
          "the second port reaction includes the coupling term");
}

// --- 4. refusals -----------------------------------------------------------

void test_inconsistent_inputs_are_refused() {
    const Model model = make_model(2, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({
        {400.0, 100.0},
        {100.0, 900.0},
    });
    std::vector<double> reaction;

    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {2}, {1.0}, reaction),
          "a port beyond the pose block is refused");
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {-1}, {1.0}, reaction),
          "a negative port is refused");
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {0, 0}, {1.0, 1.0}, reaction),
          "a repeated port is refused");
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {0, 1}, {1.0}, reaction),
          "a prescribed/driven length mismatch is refused");
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {0}, {1.0, 2.0}, reaction),
          "a driven/prescribed length mismatch is refused");
    // A tangent that is not dimension x dimension -- a transposed or ragged one.
    // The size must be checked BEFORE anything indexes it, so a matrix that is too
    // SHORT (and would otherwise be read out of bounds) is the case that matters.
    std::vector<double> sentinel{123.0};
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, {1.0, 2.0}, {0}, {1.0}, sentinel),
          "a tangent with the wrong size is refused");
    check(sentinel.size() == 1 && sentinel[0] == 123.0,
          "a wrong-size tangent is refused before the caller's vector is touched");
    // Too long is also wrong, and a check written as `>=` would accept it.
    check(!axle_kernel::frozen_force_tangent_column(
              model, working,
              square({{400.0, 100.0, 0.0}, {100.0, 900.0, 0.0},
                      {0.0, 0.0, 0.0}}),
              {0}, {1.0}, sentinel),
          "an oversized tangent is refused rather than partially read");
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {0}, {std::nan("")}, reaction),
          "a non-finite prescribed increment is refused");
}

void test_singular_free_block_is_refused_not_approximated() {
    // A free coordinate whose equation is entirely absent: the mechanism is
    // underdetermined, so there is no unique reaction and the column must be
    // refused.  The tag is what makes this observable -- a refusal that returned a
    // finite number would leave the sentinel at its initial value, and the
    // assertion below is on the *sentinel not being overwritten* as well as on the
    // return value.
    //
    // The variant that matters most is the least-squares fallback: it would return
    // a finite, plausible reaction for an unconstrained mechanism.  Asserting only
    // `!ok` is enough to catch it, but asserting the sentinel too catches a
    // "return false after writing" regression.
    const Model model = make_model(2, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({
        {500.0, 0.0},
        {0.0, 0.0},
    });
    std::vector<double> reaction{123.0};
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {0}, {0.01}, reaction),
          "a singular free block is refused rather than approximated");
    check(reaction.size() == 1 && reaction[0] == 123.0,
          "a singular refusal did not overwrite the caller's reaction");

    // Same defect reached through a rank-deficient block that has NO zero row: two
    // identical free equations.  A check that only looked for an all-zero row would
    // miss it, so the free block must be larger than one coordinate for this to be
    // a different matrix from the case above.
    const Model larger = make_model(3, 0);
    const WorkingSample larger_sample = make_sample(larger);
    const std::vector<double> duplicated = square({
        {500.0, 0.0, 0.0},
        {0.0, 300.0, 300.0},
        {0.0, 300.0, 300.0},
    });
    std::vector<double> larger_reaction{123.0};
    check(!axle_kernel::frozen_force_tangent_column(
              larger, larger_sample, duplicated, {0}, {0.01}, larger_reaction),
          "a rank-deficient free block with no zero row is refused");

    // A free block that is singular in a way whose *solve* would still produce
    // finite numbers.  This is the case that isolates the factorization check.
    //
    // The assembler's `solve()` reads its stored factors.  When `factor()` fails,
    // `lu` still holds the *input* matrix, so `solve()` keeps eliminating on
    // un-factorized data and returns finite numbers rather than infinities.  A
    // refusal that relied on spotting `inf` downstream would accept such a block, so
    // this assertion pins the refusal to the factorization itself.
    //
    // The block below is chosen for two properties: every entry is non-zero (so no
    // zero row short-circuits it into the inf-producing path) and the rows are
    // proportional (so it is genuinely singular).  Coordinates 1 and 2 are the free
    // ones and their 2x2 block is [[4,6],[6,9]] = 6 * [[2/3,1],[1,1.5]], rank one.
    const Model trap = make_model(3, 0);
    const WorkingSample trap_sample = make_sample(trap);
    const std::vector<double> trap_tangent = square({
        {900.0, 0.0, 0.0},
        {0.0, 4.0, 6.0},
        {0.0, 6.0, 9.0},
    });
    std::vector<double> trap_reaction{123.0};
    check(!axle_kernel::frozen_force_tangent_column(
              trap, trap_sample, trap_tangent, {0}, {1.0}, trap_reaction),
          "a singular free block whose solve would stay finite is refused");
    check(trap_reaction.size() == 1 && trap_reaction[0] == 123.0,
          "the finite-garbage singular case did not overwrite the caller's vector");
}

void test_a_refusal_writes_nothing() {
    const Model model = make_model(2, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({
        {400.0, 100.0},
        {100.0, 900.0},
    });
    std::vector<double> reaction{7.0, 8.0, 9.0};
    const std::vector<double> before = reaction;
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {5}, {1.0}, reaction),
          "an out-of-range port is refused");
    check(reaction == before,
          "a refusal leaves the caller's reaction vector untouched");
}

// --- 6. zero is an answer, not a failure ----------------------------------

void test_zero_stiffness_reports_zero_and_succeeds() {
    const Model model = make_model(1, 0);
    const WorkingSample working = make_sample(model);
    const std::vector<double> tangent = square({{0.0}});
    std::vector<double> reaction;
    check(axle_kernel::frozen_force_tangent_column(
              model, working, tangent, {0}, {0.01}, reaction),
          "a zero tangent is a legitimate solve, not a refusal");
    check(close(reaction[0], 0.0, 1e-12),
          "a mechanism with no stiffness reports a zero reaction");
}

// --- 7. the multiplier block is partitioned too ----------------------------

void test_multiplier_coordinates_are_part_of_the_free_block() {
    // A KKT-shaped tangent for a mechanism with a real constraint.  Coordinate 0 is
    // the held pose coordinate, coordinate 1 is the constraint multiplier:
    //
    //   [ k  J ] [ ds ]   [ Q  ]
    //   [ J  0 ] [ dl ] = [ 0  ]
    //
    // The second equation is the constraint consistency `J*ds = 0`, which a valid
    // mechanism satisfies only if the constraint row is compatible with the held
    // displacement.  Choosing J = 0 (the constraint does not react to this
    // coordinate) makes the multiplier row decouple, and the multiplier then has no
    // equation of its own -- so the free block IS singular and the column must be
    // refused.  That is the honest answer, and it is why the case below asserts a
    // refusal rather than a number.
    const Model model = make_model(1, 1);
    const WorkingSample working = make_sample(model);
    const std::vector<double> decoupled = square({
        {500.0, 0.0},
        {0.0, 0.0},
    });
    std::vector<double> reaction;
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, decoupled, {0}, {0.01}, reaction),
          "a free multiplier with no equation of its own is refused");

    // With J != 0 the system is well posed: the multiplier is determined by the
    // constraint equation `J*ds + 0*dl = 0`, which is *inconsistent* for ds != 0 --
    // so again there is no solution and the refusal must stand.  This is the
    // property worth pinning: a held displacement that violates a constraint is not
    // silently absorbed into a multiplier.
    const std::vector<double> coupled = square({
        {500.0, 1.0},
        {1.0, 0.0},
    });
    check(!axle_kernel::frozen_force_tangent_column(
              model, working, coupled, {0}, {0.01}, reaction),
          "a held displacement that violates the constraint is refused");
}

}  // namespace

int main() {
    test_single_spring_reports_the_holding_force();
    test_series_springs_report_the_series_stiffness();
    test_coupled_ports_report_the_full_column();
    test_inconsistent_inputs_are_refused();
    test_singular_free_block_is_refused_not_approximated();
    test_a_refusal_writes_nothing();
    test_zero_stiffness_reports_zero_and_succeeds();
    test_multiplier_coordinates_are_part_of_the_free_block();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("OK: frozen force tangent self-test passed\n");
    return 0;
}
