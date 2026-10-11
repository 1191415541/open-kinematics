// The batch and transpose solve surface of `mb_linear`.
//
// See `mb_linear/batch.hpp` for why the transpose is implemented exactly rather
// than by assuming symmetry.  This file is the arithmetic: the capability table,
// the reusable handle and the two batch entry points.

#include "mb_linear/batch.hpp"

#include <cmath>
#include <limits>

namespace axle_kernel {

namespace {

// The capability table.  Every entry states what the backend in
// `factorization_types.hpp` is *measured* to do today, with the line numbers the
// survey found:
//
//   * All of them are single-RHS.  `native_multi_rhs` is false everywhere:
//     `MklDenseFactorization` passes `nrhs = 1` literally (:443) and
//     `MklPardisoFactorization`'s `nrhs` member (:466) is initialised `{1}` and
//     never written again.
//   * None exposes a transpose solve.  The `kNoTranspose = 'N'` constant (:339) is
//     hard-coded into the `getrs` call (:443) to request the *forward* system -- it
//     is evidence of the absence of transpose support, not of its presence.
//   * `caller_visible_pattern_step` is false everywhere.  PARDISO does keep
//     `analyzed` (:476) and switches phase 12/22 (:638/:652) with a `same_pattern`
//     test (:597), but that analysis is embedded in the first `factor` call; there
//     is no public `analyze()` a caller could invoke to plan ahead.  Claiming true
//     would let a caller report a planning step it cannot take.
//   * `reusable_factorization` is true everywhere `factor` exists: the solve methods
//     are `const` (or take the rhs by const-ref), so one factorization genuinely
//     serves many right-hand sides.
const LinearBackendCapability kCapabilities[] = {
    {
        "lu", /*reusable=*/true, /*pattern=*/false,
        /*multi_rhs=*/false, /*transpose=*/false,
        "dense LU with row/column equilibration; no transpose solve exists, so the "
        "transpose is formed and factored again",
    },
    {
        "mkl_dense", /*reusable=*/true, /*pattern=*/false,
        /*multi_rhs=*/false, /*transpose=*/false,
        "the getrf/getrs wrapper passes nrhs=1 and kNoTranspose literally; the "
        "transpose is formed and factored again",
    },
    {
        "pardiso", /*reusable=*/true, /*pattern=*/false,
        /*multi_rhs=*/false, /*transpose=*/false,
        "PARDISO reuses its symbolic analysis internally across same-pattern "
        "refactors, but exposes no separate analyze() to callers and solves one "
        "forward right-hand side at a time; the transpose is formed and factored "
        "again",
    },
    {
        "sparse_gmres", /*reusable=*/true, /*pattern=*/false,
        /*multi_rhs=*/false, /*transpose=*/false,
        "iterative; a transpose system would need its own Krylov solve and is not "
        "implemented, so the transpose is formed and factored again",
    },
    {
        "sparse_lu", /*reusable=*/true, /*pattern=*/false,
        /*multi_rhs=*/false, /*transpose=*/false,
        "sparse LU wrapper solves one right-hand side; transpose not exposed",
    },
    {
        "block_diagonal_mass", /*reusable=*/true, /*pattern=*/false,
        /*multi_rhs=*/false, /*transpose=*/false,
        "block-diagonal by construction and its solve returns void; a transpose is "
        "block-transposed rather than solved through this handle",
    },
};

constexpr int kCapabilityCount =
    static_cast<int>(sizeof(kCapabilities)/sizeof(kCapabilities[0]));

} // namespace

const LinearBackendCapability* linear_backend_capabilities(int& count) {
    count = kCapabilityCount;
    return kCapabilities;
}

const LinearBackendCapability* linear_backend_capability(const char* name) {
    if (name == nullptr) return nullptr;
    for (int index = 0; index < kCapabilityCount; ++index) {
        const char* candidate = kCapabilities[index].name;
        // Plain comparison rather than std::string: this runs on the hot path of
        // a diagnostic and must not allocate.
        int i = 0;
        while (candidate[i] != '\0' && candidate[i] == name[i]) ++i;
        if (candidate[i] == '\0' && name[i] == '\0') return &kCapabilities[index];
    }
    return nullptr;
}

bool LinearFactorHandle::factorize_numeric(std::vector<double> a, int n) {
    last_transpose_rebuilt = false;
    if (n <= 0) return false;
    if (static_cast<int>(a.size()) != n*n) return false;
    // Keep the original matrix: it is the only way a transpose solve can be exact
    // for a backend that has no transpose support.  `factor` takes the matrix by
    // value and scales it in place, so the copy is taken first.
    matrix = a;
    dim = n;
    // `backend` is intentionally NOT reset here.  This handle always factors with
    // `LuFactorization`, but the field records which backend the *caller* believes
    // it is using, and resetting it would silently change the answer to
    // `has_pattern_analysis()` for a handle that was constructed under another
    // name.  A newly default-constructed handle already reads `"lu"`, which is the
    // correct default because that is what this function builds.
    if (!factorization.factor(std::move(a), n)) {
        // Leave the handle visibly unusable rather than holding a partial factor.
        dim = 0;
        matrix.clear();
        return false;
    }
    return true;
}

bool LinearFactorHandle::has_pattern_analysis() const {
    return analyze_pattern(backend, dim);
}

bool analyze_pattern(const char* backend, int dimension) {
    if (dimension <= 0) return false;
    const LinearBackendCapability* capability = linear_backend_capability(backend);
    if (capability == nullptr) return false;
    return capability->caller_visible_pattern_step;
}

bool solve_many(
    const LinearFactorHandle& handle,
    const std::vector<double>& right_hand_sides,
    int column_count,
    std::vector<double>& solutions,
    int& factorizations_used
) {
    factorizations_used = 0;
    const int n = handle.dim;
    if (n <= 0) return false;
    if (column_count <= 0) return false;
    if (static_cast<int>(right_hand_sides.size()) != n*column_count) return false;

    solutions.assign(static_cast<std::size_t>(n)*static_cast<std::size_t>(column_count), 0.0);
    std::vector<double> column(static_cast<std::size_t>(n));
    std::vector<double> solved(static_cast<std::size_t>(n));
    // One factorization serves every column: it was built by `factorize_numeric`
    // and `LuFactorization::solve` is const, so the reuse is real and the counter
    // can say 1.  A backend whose solve mutated the factorization would have to
    // report its own count; that is why the number is an out-parameter and not a
    // constant.
    factorizations_used = 1;
    for (int j = 0; j < column_count; ++j) {
        for (int i = 0; i < n; ++i) {
            column[static_cast<std::size_t>(i)] =
                right_hand_sides[static_cast<std::size_t>(j)*static_cast<std::size_t>(n)
                    + static_cast<std::size_t>(i)];
        }
        handle.factorization.solve(column, solved);
        for (int i = 0; i < n; ++i) {
            const double value = solved[static_cast<std::size_t>(i)];
            if (!std::isfinite(value)) return false;
            solutions[static_cast<std::size_t>(j)*static_cast<std::size_t>(n)
                + static_cast<std::size_t>(i)] = value;
        }
    }
    return true;
}

bool solve_transpose_many(
    const LinearFactorHandle& handle,
    const std::vector<double>& right_hand_sides,
    int column_count,
    std::vector<double>& solutions,
    int& factorizations_used
) {
    factorizations_used = 0;
    const int n = handle.dim;
    if (n <= 0) return false;
    if (column_count <= 0) return false;
    if (static_cast<int>(right_hand_sides.size()) != n*column_count) return false;
    if (static_cast<int>(handle.matrix.size()) != n*n) return false;

    // `solve_transpose_many` deliberately does NOT branch on the capability table.
    // Forming and factoring A^T unconditionally is the one code path that cannot
    // be wrong: the alternative -- reusing the handle when a backend claims a
    // native transpose -- would copy the *forward* factorization and silently
    // solve `A X = B` while reporting `A^T X = B`.  No backend here claims one, so
    // such a branch would be both wrong and untested.  The selftest asserts the
    // whole table reports no native transpose, so a future backend that gains one
    // trips that assertion and forces this decision to be revisited.

    // Form A^T exactly.  This is the step that must not be skipped by "A looks
    // symmetric": every non-conservative tangent would then be solved as its own
    // symmeterisation, which is a different physical system.
    //
    // It is formed unconditionally, even for a backend that claims a native
    // transpose.  The alternative -- copying the handle in that case -- would
    // copy the *forward* factorization and silently solve `A X = B` while
    // reporting `A^T X = B`.  No backend in this tree claims one, so the branch
    // would be untested as well as wrong; keeping the formation unconditional
    // means there is exactly one code path to reason about, and a future backend
    // that gains transpose support changes this function deliberately.
    std::vector<double> transposed(
        static_cast<std::size_t>(n)*static_cast<std::size_t>(n), 0.0
    );
    for (int row = 0; row < n; ++row) {
        for (int column = 0; column < n; ++column) {
            transposed[static_cast<std::size_t>(column)*static_cast<std::size_t>(n)
                + static_cast<std::size_t>(row)] =
                handle.matrix[static_cast<std::size_t>(row)*static_cast<std::size_t>(n)
                    + static_cast<std::size_t>(column)];
        }
    }

    LinearFactorHandle transposed_handle;
    if (!transposed_handle.factorize_numeric(transposed, n)) return false;
    // `factorize_numeric` stamps `"lu"` because that is the factorization it
    // builds.  The *capability* being reported, however, belongs to the backend
    // the caller named, so the name is carried over explicitly rather than left
    // to be silently rewritten.
    transposed_handle.backend = handle.backend;
    // The factorization could not be reused: no backend here can solve a
    // transpose, and the transposed matrix is a different matrix regardless.
    factorizations_used = 2;
    handle.last_transpose_rebuilt = true;

    int inner_reuse = 0;
    const bool solved = solve_many(
        transposed_handle, right_hand_sides, column_count, solutions, inner_reuse
    );
    if (!solved) {
        factorizations_used = 0;
        return false;
    }
    return true;
}

} // namespace axle_kernel
