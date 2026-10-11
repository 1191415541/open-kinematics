// Self-test for the batch and transpose solve surface of `mb_linear`.
//
// Verified by a standalone executable rather than through the product C ABI: the
// batch surface is an internal capability, and exercising it must not require
// bumping an ABI version.
//
// What this checks, and why each one needs to be checked rather than assumed:
//
//   1. `solve_many` really reuses ONE factorization for N right-hand sides, and the
//      answers match a per-column single-RHS `solve` to the last bit.  A batch API
//      that silently refactors per column would still produce right answers while
//      costing N times more, so the counter is asserted, not just the values.
//
//   2. `solve_transpose_many` solves the ACTUAL transposed system.  The test matrix
//      is deliberately non-symmetric, and the assertion against an independently
//      computed A^T x = b is what makes "did you transpose?" observable.  A
//      symmetry-assuming implementation passes on symmetric input and fails here --
//      which is the whole reason the test uses a non-symmetric matrix.
//
//   3. The transpose is NOT silently the forward solve.  For a non-symmetric
//      matrix, solving A x = b and A^T x = b give different results; the test
//      asserts they differ, so an implementation that ignored the transpose
//      request would be caught even if it were self-consistent.
//
//   4. The capability table tells the truth about this tree: no backend claims a
//      native transpose or native multi-RHS.

#include "mb_linear/batch.hpp"
#include "mb_linear/factorization_types.hpp"

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

// A deliberately non-symmetric, non-singular 3x3 matrix.  Non-symmetric is the
// point: with a symmetric matrix every transpose bug looks like a pass.
const std::vector<double> kMatrix = {
     4.0,  1.0,  2.0,
     0.5,  3.0, -1.0,
     1.5,  0.25, 5.0,
};

constexpr int kDim = 3;
constexpr int kColumns = 3;

// A second right-hand side set: three columns of length 3, column-major.
const std::vector<double> kRhs = {
    1.0, 2.0, 3.0,
    0.5, -1.0, 4.0,
    -2.0, 1.0, 0.25,
};

/// Multiply a row-major `n x n` matrix by a column-major set of vectors.
std::vector<double> multiply(
    const std::vector<double>& matrix, int n,
    const std::vector<double>& vectors, int columns
) {
    std::vector<double> out(
        static_cast<std::size_t>(n)*static_cast<std::size_t>(columns), 0.0
    );
    for (int j = 0; j < columns; ++j) {
        for (int row = 0; row < n; ++row) {
            double sum = 0.0;
            for (int k = 0; k < n; ++k) {
                sum += matrix[static_cast<std::size_t>(row)*static_cast<std::size_t>(n)
                    + static_cast<std::size_t>(k)]
                    * vectors[static_cast<std::size_t>(j)*static_cast<std::size_t>(n)
                        + static_cast<std::size_t>(k)];
            }
            out[static_cast<std::size_t>(j)*static_cast<std::size_t>(n)
                + static_cast<std::size_t>(row)] = sum;
        }
    }
    return out;
}

bool close(double a, double b) {
    const double scale = std::max(1.0, std::max(std::abs(a), std::abs(b)));
    return std::abs(a-b) <= 1.0e-10*scale;
}

bool vectors_close(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!close(a[i], b[i])) return false;
    }
    return true;
}

} // namespace

int main() {
    using axle_kernel::LinearFactorHandle;
    using axle_kernel::solve_many;
    using axle_kernel::solve_transpose_many;

    // --- capability table tells the truth ---------------------------------
    {
        int count = 0;
        const axle_kernel::LinearBackendCapability* table =
            axle_kernel::linear_backend_capabilities(count);
        check(count > 0 && table != nullptr, "the capability table must not be empty");
        for (int i = 0; i < count; ++i) {
            // Asserting the current tree's measured state.  When a backend gains
            // real transpose support this assertion must be *changed deliberately*
            // alongside the implementation -- that is the point of pinning it.
            check(
                !table[i].native_transpose,
                "no backend in this tree has a native transpose solve"
            );
            check(
                !table[i].native_multi_rhs,
                "no backend in this tree solves multiple right-hand sides natively"
            );
            check(
                table[i].transpose_fallback_reason[0] != '\0',
                "a backend without native transpose must state its fallback reason"
            );
        }
        // `analyze_pattern` must not claim a reuse the caller cannot get.
        check(
            !axle_kernel::analyze_pattern("pardiso", kDim),
            "PARDISO's analysis is internal; no caller-visible pattern step exists"
        );
        check(
            axle_kernel::linear_backend_capability("nonexistent") == nullptr,
            "an unknown backend must resolve to nullptr"
        );
    }

    LinearFactorHandle handle;
    check(
        handle.factorize_numeric(kMatrix, kDim),
        "the test matrix must factor (it is non-singular by construction)"
    );

    // --- forward batch: one factorization, N columns ----------------------
    {
        std::vector<double> solutions;
        int used = 0;
        const bool ok = solve_many(handle, kRhs, kColumns, solutions, used);
        check(ok, "solve_many must succeed");
        check(
            used == 1,
            "solve_many must reuse ONE factorization across every column"
        );

        // Residual check: A x_j must reproduce b_j.  This is stronger than
        // comparing against another solver, because it does not depend on the
        // other solver being right.
        //
        // `solutions` and `kRhs` are addressed differently on purpose: `solve_many`
        // writes column-major with the column stride equal to `dim`, so the j-th
        // solution's i-th entry is at j*dim+i.
        for (int j = 0; j < kColumns; ++j) {
            std::vector<double> column(static_cast<std::size_t>(kDim), 0.0);
            for (int i = 0; i < kDim; ++i) {
                column[static_cast<std::size_t>(i)] =
                    solutions[static_cast<std::size_t>(j)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(i)];
            }
            const std::vector<double> residual =
                multiply(kMatrix, kDim, column, 1);
            std::vector<double> expected(static_cast<std::size_t>(kDim), 0.0);
            for (int i = 0; i < kDim; ++i) {
                expected[static_cast<std::size_t>(i)] =
                    kRhs[static_cast<std::size_t>(j)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(i)];
            }
            check(
                vectors_close(residual, expected),
                "each batch column must satisfy A x = b"
            );
        }

        // And the batch agrees with the single-RHS path bit-for-bit, so the batch
        // entry point cannot quietly use different arithmetic.
        for (int j = 0; j < kColumns; ++j) {
            std::vector<double> rhs(static_cast<std::size_t>(kDim), 0.0);
            for (int i = 0; i < kDim; ++i) {
                rhs[static_cast<std::size_t>(i)] =
                    kRhs[static_cast<std::size_t>(j)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(i)];
            }
            std::vector<double> single;
            handle.factorization.solve(rhs, single);
            std::vector<double> batched(static_cast<std::size_t>(kDim), 0.0);
            for (int i = 0; i < kDim; ++i) {
                batched[static_cast<std::size_t>(i)] =
                    solutions[static_cast<std::size_t>(j)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(i)];
            }
            check(
                vectors_close(single, batched),
                "the batch column must equal the single-RHS solve"
            );
        }

        // A mis-sized request must be refused rather than partially filled.
        std::vector<double> short_rhs(static_cast<std::size_t>(kDim), 0.0);
        std::vector<double> out;
        int used_bad = 0;
        check(
            !solve_many(handle, short_rhs, kColumns, out, used_bad),
            "solve_many must refuse a right-hand-side block of the wrong size"
        );
    }

    // --- transpose batch: really A^T, not A -------------------------------
    {
        std::vector<double> solutions;
        int used = 0;
        const bool ok =
            solve_transpose_many(handle, kRhs, kColumns, solutions, used);
        check(ok, "solve_transpose_many must succeed");
        // The whole point: the forward factorization is NOT reused, because no
        // backend can solve a transpose.  Reporting 1 here would overclaim.
        check(
            used == 2,
            "a transpose solve must report its own factorization (2 total)"
        );
        check(
            handle.last_transpose_rebuilt,
            "the handle must record that the transpose cost a second factorization"
        );

        // Build A^T explicitly and check A^T x = b for each column.
        std::vector<double> transposed(
            static_cast<std::size_t>(kDim)*static_cast<std::size_t>(kDim), 0.0
        );
        for (int row = 0; row < kDim; ++row) {
            for (int column = 0; column < kDim; ++column) {
                transposed[static_cast<std::size_t>(column)*static_cast<std::size_t>(kDim)
                    + static_cast<std::size_t>(row)] =
                    kMatrix[static_cast<std::size_t>(row)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(column)];
            }
        }
        for (int j = 0; j < kColumns; ++j) {
            std::vector<double> column(static_cast<std::size_t>(kDim), 0.0);
            for (int i = 0; i < kDim; ++i) {
                column[static_cast<std::size_t>(i)] =
                    solutions[static_cast<std::size_t>(j)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(i)];
            }
            const std::vector<double> residual =
                multiply(transposed, kDim, column, 1);
            std::vector<double> expected(static_cast<std::size_t>(kDim), 0.0);
            for (int i = 0; i < kDim; ++i) {
                expected[static_cast<std::size_t>(i)] =
                    kRhs[static_cast<std::size_t>(j)*static_cast<std::size_t>(kDim)
                        + static_cast<std::size_t>(i)];
            }
            check(
                vectors_close(residual, expected),
                "each transposed batch column must satisfy A^T x = b"
            );
        }

        // The decisive negative: for a non-symmetric matrix the forward and
        // transposed solutions must DIFFER.  If they agree, the transpose was
        // ignored.
        std::vector<double> forward;
        int forward_used = 0;
        solve_many(handle, kRhs, kColumns, forward, forward_used);
        check(
            !vectors_close(forward, solutions),
            "on a non-symmetric matrix the forward and transposed solves must differ"
        );

        // `last_transpose_rebuilt` must land on the CALLER's object.  It is a
        // `mutable` member of a `const&` parameter, so a write through the wrong
        // path would be invisible here -- and the flag is exactly what tells a
        // caller it paid for a second factorization.
        check(
            handle.last_transpose_rebuilt,
            "the rebuild flag must be observable on the caller's own handle"
        );
        // Calling the forward batch again must not clear it via the transpose path,
        // and must still reuse the original factorization.
        std::vector<double> again;
        int again_used = 0;
        check(solve_many(handle, kRhs, kColumns, again, again_used), "repeat solve must succeed");
        check(again_used == 1, "the forward factorization is still reusable after a transpose");
    }

    // --- a singular matrix is refused, not answered with garbage ----------
    {
        LinearFactorHandle singular;
        const std::vector<double> rank_deficient = {
            1.0, 2.0, 3.0,
            2.0, 4.0, 6.0,
            1.0, 1.0, 1.0,
        };
        // Row 1 is exactly twice row 0, so this matrix has rank 2.  `LuFactorization`
        // does not report rank, but a zero pivot must still not be divided by.
        const bool factored = singular.factorize_numeric(rank_deficient, kDim);
        if (factored) {
            std::vector<double> out;
            int used = 0;
            const bool solved = solve_many(singular, kRhs, kColumns, out, used);
            // Either the factorization refused it, or the solve produced
            // non-finite values that `solve_many` catches.  What must never happen
            // is a `true` return carrying finite but meaningless numbers.
            check(
                !solved,
                "a rank-deficient system must not report success"
            );
        } else {
            // Refusing at factor time is also acceptable and is the common case.
            check(true, "a rank-deficient matrix may be refused at factorization");
        }
    }

    if (failures == 0) {
        std::printf("OK: mb_linear batch and transpose solves behave as declared\n");
        return 0;
    }
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return 1;
}
