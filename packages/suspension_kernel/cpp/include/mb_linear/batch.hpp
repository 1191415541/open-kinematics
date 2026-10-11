#pragma once

/// The batch and transpose solve surface of `mb_linear`.
///
/// The backends in `factorization_types.hpp` are all *single*-right-hand-side and
/// *forward*-only: every `solve` takes one vector and none of them can solve
/// `A^T x = b`.  That is a fact about the current tree, not a limitation this
/// header hides -- and it matters, because a characteristic request that wants
/// many load modes on few measurements is answered most cheaply by the transpose
/// system, and assuming `A` can stand in for `A^T` gives silently wrong numbers
/// on every non-symmetric tangent.
///
/// So this header does two things and nothing else:
///
///   1. It states each backend's *real* capability, so a caller cannot discover it
///      by believing a comment.
///   2. It provides `solve_many` and `solve_transpose_many` over a factorization
///      handle, implemented by the honest route: reuse the factorization, run one
///      back-substitution per right-hand side, and -- for the transpose -- solve
///      the actual transposed system rather than pretending the matrix is
///      symmetric.
///
/// The transpose implementation is deliberately *not* "assume symmetry".  A
/// non-conservative tangent (bushings with velocity-dependent terms, friction)
/// has no reason to be symmetric, and a symmetry assumption would pass every
/// symmetric test anyone bothers to write while being wrong exactly where it
/// matters.  Instead the handle carries the original matrix so the transpose can
/// be formed exactly, and the capability record says whether that cost the caller
/// anything.
///
/// Nothing here changes an existing solver path: the batch entry points are new
/// callers of the same arithmetic, and the single-RHS `solve` the solvers already
/// use is untouched.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_linear/factorization_types.hpp"

namespace axle_kernel {

/// What one backend can actually do.
///
/// The fields are separate rather than one "supported" flag because the answers
/// genuinely differ per backend and per operation.  A caller that needs to plan
/// (few measurements, many load modes -> transpose) has to read the transpose
/// field, not a summary.
///
/// Every field below was read off `factorization_types.hpp`, not assumed from the
/// backend's name.  The layer is duck-typed -- there is no base class, no concept
/// and no enum, and the `solve` signatures are not even uniform (`LuFactorization`
/// and `BlockDiagonalMassFactorization` return `void`, the rest `bool`), so a
/// capability table is the only place the differences can be stated once.
struct LinearBackendCapability {
    /// Stable identifier, e.g. `"lu"`, `"mkl_dense"`, `"pardiso"`.
    const char* name = "";
    /// The backend can factor once and be solved repeatedly.
    bool reusable_factorization = false;
    /// A symbolic/pattern step can be reused for a same-pattern refactor.
    ///
    /// This is deliberately *not* "the backend has internal analysis state".
    /// PARDISO does (`analyzed` at `factorization_types.hpp:476`, phase 12 then
    /// phase 22 at :638/:652, `same_pattern` at :597) but the analysis is embedded
    /// in the first `factor` call and there is no public `analyze()` the caller can
    /// invoke separately.  Reporting true here would let a caller claim a planning
    /// step it cannot actually take, so the honest value is false for every
    /// backend in the tree today.
    bool caller_visible_pattern_step = false;
    /// The backend solves several right-hand sides natively in one call.  None do:
    /// the MKL wrappers pass `nrhs = 1` explicitly and PARDISO's `nrhs` member
    /// (`:466`) is never written from its `{1}` initialiser.  When this is false,
    /// `solve_many` still works -- it loops -- but the loop is what the caller is
    /// paying for, and the measurement should say so.
    bool native_multi_rhs = false;
    /// The backend can solve the transposed system itself.  False for every
    /// backend: the only transpose-related token in the whole header is
    /// `kNoTranspose = 'N'` (`:339`), which is passed *hard-coded* to `getrs`
    /// (`:443`) to request the forward system.  When this is false,
    /// `solve_transpose_many` forms the transpose and factors that.
    bool native_transpose = false;
    /// Human-readable statement of the fallback actually taken, empty when the
    /// backend supports the operation natively.  A caller that must justify its
    /// cost model reads this instead of guessing.
    const char* transpose_fallback_reason = "";
};

/// Every backend this module knows, in a stable order.
const LinearBackendCapability* linear_backend_capabilities(int& count);

/// The capability of one backend by name, or `nullptr` when unknown.
const LinearBackendCapability* linear_backend_capability(const char* name);

/// A reusable factorization plus the matrix it came from.
///
/// It keeps a copy of the matrix *on purpose*.  Without it a transpose solve could
/// only be offered by the backends that support one natively -- and none of the
/// current ones do -- so the honest choice would be to refuse transposes outright.
/// With it, the transpose is exact for every backend, and `transpose_reused` on
/// the result reports truthfully whether the original factorization was reused or
/// a second one was built.  Silently reusing a symmetric-looking factorization for
/// a transpose would be the failure mode this design exists to prevent.
struct LinearFactorHandle {
    /// The matrix as given (before equilibration scaling), row-major `dim x dim`.
    std::vector<double> matrix;
    int dim = 0;
    /// The backend that produced `factorization`, for capability reporting.
    const char* backend = "lu";
    /// The forward factorization.
    LuFactorization factorization;
    /// Set when the last transpose solve had to build its own factorization.
    mutable bool last_transpose_rebuilt = false;

    /// Factor `a` (row-major `n x n`).  Returns false on a non-finite or
    /// singular matrix; the handle is left unusable rather than half-filled.
    bool factorize_numeric(std::vector<double> a, int n);

    /// True when a same-pattern refactor can skip a caller-visible analysis step.
    /// No current backend can, so this reports false and `analyze_pattern` is a
    /// documented no-op rather than a silent "cache hit" that never happened.
    bool has_pattern_analysis() const;
};

/// Report whether a same-pattern refactor can skip analysis for this backend.
///
/// Present as a free function (not only a handle method) so a caller can consult
/// the policy before it has a matrix, which is when the decision is actually made.
/// Returns false when the backend exposes no pattern step: the caller must not
/// claim a reuse it did not get.
bool analyze_pattern(const char* backend, int dimension);

/// Solve `A X = B` for `column_count` right-hand sides.
///
/// `right_hand_sides` holds `dim * column_count` values in column-major order
/// (column `j` occupies `[j*dim, (j+1)*dim)`), and `solutions` is written the same
/// way.  Returns false when any single solve fails, rather than leaving a
/// partially correct block: a caller that receives `true` can rely on every
/// column.
///
/// `factorizations_used` receives 1 (the shared one) so the caller can report how
/// many numeric factorizations the batch actually cost.  It is an out-parameter
/// rather than a guess because the point of the batch API is to make that number
/// visible.
bool solve_many(
    const LinearFactorHandle& handle,
    const std::vector<double>& right_hand_sides,
    int column_count,
    std::vector<double>& solutions,
    int& factorizations_used
);

/// Solve `A^T X = B` for `column_count` right-hand sides.
///
/// The transpose is formed and solved exactly.  When the backend has no native
/// transpose support (all of the current ones), a second factorization of `A^T` is
/// built and `factorizations_used` reports 2 -- the honest cost.  A symmetric
/// matrix is *not* special-cased: detecting symmetry to save a factorization would
/// make the fast and slow paths disagree numerically, and the caller can compare
/// the two itself if it wants.
bool solve_transpose_many(
    const LinearFactorHandle& handle,
    const std::vector<double>& right_hand_sides,
    int column_count,
    std::vector<double>& solutions,
    int& factorizations_used
);

} // namespace axle_kernel
