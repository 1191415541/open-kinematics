#pragma once

/// The linear-algebra backends (MB_LINALG).
///
/// Eight interchangeable factorizations behind one call site: dense LU, two MKL
/// backends, two sparse backends, a block-diagonal mass solver and the reduced
/// Newton system.  They depend on `mb_base` and on nothing else -- no `Model`, no
/// `ResidualContext` definition -- which is the property the epic requires of this
/// layer and the reason the header can be included first.
///
/// The types are defined together rather than one file each because several of
/// them hold each other by value (ten value members in total across
/// `ReducedNewtonFactorization` and `NewtonSystemFactorization`), so a forward
/// declaration cannot separate them.  This is the one header the epic puts under
/// `src/` for that reason.

#include "mb_config/env.hpp"
#include "mb_numeric/util.hpp"
#include "mb_numeric/vector.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>

namespace axle_kernel {

// The cache key's comparison members take a `ResidualContext`; its
// definition belongs to the integrator layer, so the model reference is a
// forward declaration here.  That, plus the base-layer headers above, is what
// makes this header self-contained: the backends call the base layer's solver
// switches and `finite_vec`, never anything above them.
struct ResidualContext;

struct LuFactorization {
    std::vector<double> lu;
    std::vector<int> pivot;
    std::vector<int> column_pivot;
    std::vector<double> row_scale;
    std::vector<double> column_scale;
    int dim{0};
    bool identity{false};

    bool factor(std::vector<double> a, int n) {
        lu = std::move(a);
        dim = n;
        identity = false;
        pivot.resize(static_cast<std::size_t>(n));
        column_pivot.resize(static_cast<std::size_t>(n));
        std::iota(column_pivot.begin(), column_pivot.end(), 0);
        row_scale.assign(static_cast<std::size_t>(n), 1.0);
        column_scale.assign(static_cast<std::size_t>(n), 1.0);
        // Newton rows mix metres, radians, velocities, forces and moments.
        // Equilibrate rows and columns before LU so a small SI coordinate row
        // cannot be discarded by a pivot test against a large tire stiffness.
        if (lu_equilibration_enabled()) {
            for (int row = 0; row < n; ++row) {
                double maximum = 0.0;
                for (int column = 0; column < n; ++column) {
                    const double value = lu[static_cast<std::size_t>(row*n+column)];
                    if (!std::isfinite(value)) return false;
                    maximum = std::max(maximum, std::abs(value));
                }
                if (!(maximum > 0.0) || !std::isfinite(maximum)) return false;
                row_scale[static_cast<std::size_t>(row)] = 1.0/maximum;
                for (int column = 0; column < n; ++column) {
                    lu[static_cast<std::size_t>(row*n+column)] *=
                        row_scale[static_cast<std::size_t>(row)];
                }
            }
            for (int column = 0; column < n; ++column) {
                double maximum = 0.0;
                for (int row = 0; row < n; ++row) {
                    maximum = std::max(
                        maximum,
                        std::abs(lu[static_cast<std::size_t>(row*n+column)])
                    );
                }
                if (!(maximum > 0.0) || !std::isfinite(maximum)) return false;
                column_scale[static_cast<std::size_t>(column)] = 1.0/maximum;
                for (int row = 0; row < n; ++row) {
                    lu[static_cast<std::size_t>(row*n+column)] *=
                        column_scale[static_cast<std::size_t>(column)];
                }
            }
        } else {
            for (int row = 0; row < n; ++row) {
                double maximum = 0.0;
                for (int column = 0; column < n; ++column) {
                    const double value = lu[static_cast<std::size_t>(row*n+column)];
                    if (!std::isfinite(value)) return false;
                    maximum = std::max(maximum, std::abs(value));
                }
                if (!(maximum > 0.0) || !std::isfinite(maximum)) return false;
            }
        }
        if (n >= 128 && blocked_lu_enabled()) {
            const int block_size = blocked_lu_block_size();
            for (int block = 0; block < n; block += block_size) {
                const int block_end = std::min(n, block+block_size);
                // 先完成当前面板的部分主元消元；当前面板内的更新保持
                // 原有顺序，面板外的更新集中为缓存友好的乘加块。
                for (int k = block; k < block_end; ++k) {
                    int best_row = k;
                    double best = std::abs(
                        lu[static_cast<std::size_t>(k*n+k)]
                    );
                    for (int row = k+1; row < n; ++row) {
                        const double value = std::abs(
                            lu[static_cast<std::size_t>(row*n+k)]
                        );
                        if (value > best) {
                            best = value;
                            best_row = row;
                        }
                    }
                    if (best < 1e-14) return false;
                    pivot[static_cast<std::size_t>(k)] = best_row;
                    if (best_row != k) {
                        double* pivot_row =
                            lu.data()+static_cast<std::size_t>(k*n);
                        double* swap_row =
                            lu.data()+static_cast<std::size_t>(best_row*n);
                        for (int column = 0; column < n; ++column) {
                            std::swap(pivot_row[column], swap_row[column]);
                        }
                    }
                    const double diagonal = lu[
                        static_cast<std::size_t>(k*n+k)
                    ];
                    const double* pivot_row =
                        lu.data()+static_cast<std::size_t>(k*n);
                    for (int row = k+1; row < n; ++row) {
                        double* target =
                            lu.data()+static_cast<std::size_t>(row*n);
                        const double factor = target[k]/diagonal;
                        target[k] = factor;
                        if (std::abs(factor) < 1e-30) continue;
                        for (int column = k+1; column < block_end; ++column) {
                            target[column] -= factor*pivot_row[column];
                        }
                    }
                }
                // 面板行的尾部列需要完成 L11*U12=A12 的三角回代；
                // 面板外的行则在下面统一执行一次块更新。
                for (int row = block; row < block_end; ++row) {
                    double* target =
                        lu.data()+static_cast<std::size_t>(row*n);
                    for (int k = block; k < row; ++k) {
                        const double factor = target[k];
                        if (std::abs(factor) < 1e-30) continue;
                        const double* pivot_row =
                            lu.data()+static_cast<std::size_t>(k*n);
                        int column = block_end;
                        for (; column+3 < n; column += 4) {
                            target[column] -= factor*pivot_row[column];
                            target[column+1] -= factor*pivot_row[column+1];
                            target[column+2] -= factor*pivot_row[column+2];
                            target[column+3] -= factor*pivot_row[column+3];
                        }
                        for (; column < n; ++column) {
                            target[column] -= factor*pivot_row[column];
                        }
                    }
                }
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(linear_solver_threads()) \
    if(blocked_lu_parallel_enabled() && n >= 512 && (n-block_end) >= 96)
#endif
                for (int row = block_end; row < n; ++row) {
                    double* __restrict target =
                        lu.data()+static_cast<std::size_t>(row*n);
                    for (int column_block = block_end;
                         column_block < n; column_block += 64) {
                        const int column_end = std::min(n, column_block+64);
                        for (int k = block; k < block_end; ++k) {
                            const double factor = target[k];
                            if (std::abs(factor) < 1e-30) continue;
                            const double* __restrict pivot_row =
                                lu.data()+static_cast<std::size_t>(k*n);
                            int column = column_block;
                            for (; column+3 < column_end; column += 4) {
                                target[column] -= factor*pivot_row[column];
                                target[column+1] -= factor*pivot_row[column+1];
                                target[column+2] -= factor*pivot_row[column+2];
                                target[column+3] -= factor*pivot_row[column+3];
                            }
                            for (; column < column_end; ++column) {
                                target[column] -= factor*pivot_row[column];
                            }
                        }
                    }
                }
            }
            return true;
        }
        for (int k = 0; k < n; ++k) {
            int best_row = k;
            double best = std::abs(lu[static_cast<std::size_t>(k*n+k)]);
            for (int i = k+1; i < n; ++i) {
                const double value =
                    std::abs(lu[static_cast<std::size_t>(i*n+k)]);
                if (value > best) { best = value; best_row = i; }
            }
            if (best < 1e-14) return false;
            pivot[static_cast<std::size_t>(k)] = best_row;
            if (best_row != k) {
                double* pivot_row = lu.data()+static_cast<std::size_t>(k*n);
                double* swap_row =
                    lu.data()+static_cast<std::size_t>(best_row*n);
                for (int j = 0; j < n; ++j) {
                    std::swap(pivot_row[j], swap_row[j]);
                }
            }
            const double diagonal = lu[static_cast<std::size_t>(k*n+k)];
            const double* __restrict pivot_row =
                lu.data()+static_cast<std::size_t>(k*n);
            for (int i = k+1; i < n; ++i) {
                double* __restrict row =
                    lu.data()+static_cast<std::size_t>(i*n);
                const double factor = row[k] / diagonal;
                row[k] = factor;
                if (std::abs(factor) < 1e-30) continue;
                int j = k+1;
                for (; j+3 < n; j += 4) {
                    row[j] -= factor*pivot_row[j];
                    row[j+1] -= factor*pivot_row[j+1];
                    row[j+2] -= factor*pivot_row[j+2];
                    row[j+3] -= factor*pivot_row[j+3];
                }
                for (; j < n; ++j) {
                    row[j] -= factor*pivot_row[j];
                }
            }
        }
        return true;
    }

    void set_identity(int n) {
        dim = n;
        identity = true;
        lu.clear();
        pivot.clear();
        column_pivot.clear();
        row_scale.clear();
        column_scale.clear();
    }

    void solve(const std::vector<double>& b, std::vector<double>& x) const {
        const int n = dim;
        if (identity) {
            x = b;
            return;
        }
        x.resize(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            x[static_cast<std::size_t>(i)] =
                b[static_cast<std::size_t>(i)]
                *row_scale[static_cast<std::size_t>(i)];
        }
        // Apply the whole row permutation before eliminating.  Interleaving the
        // swaps with forward substitution is wrong: a swap at step k can move a
        // right-hand-side entry that earlier steps already eliminated into, so
        // the elimination and the permutation must not be interleaved.
        for (int k = 0; k < n; ++k) {
            const int row = pivot[static_cast<std::size_t>(k)];
            if (row != k) std::swap(x[static_cast<std::size_t>(k)],
                                    x[static_cast<std::size_t>(row)]);
        }
        for (int i = 0; i < n; ++i) {
            const double* __restrict row =
                lu.data()+static_cast<std::size_t>(i*n);
            double sum0 = 0.0;
            double sum1 = 0.0;
            double sum2 = 0.0;
            double sum3 = 0.0;
            int j = 0;
            for (; j+3 < i; j += 4) {
                sum0 += row[j]*x[static_cast<std::size_t>(j)];
                sum1 += row[j+1]*x[static_cast<std::size_t>(j+1)];
                sum2 += row[j+2]*x[static_cast<std::size_t>(j+2)];
                sum3 += row[j+3]*x[static_cast<std::size_t>(j+3)];
            }
            double value = x[static_cast<std::size_t>(i)]
                -(sum0+sum1+sum2+sum3);
            for (; j < i; ++j) {
                value -= row[j]*x[static_cast<std::size_t>(j)];
            }
            x[static_cast<std::size_t>(i)] = value;
        }
        for (int i = n-1; i >= 0; --i) {
            const double* __restrict row =
                lu.data()+static_cast<std::size_t>(i*n);
            double sum0 = 0.0;
            double sum1 = 0.0;
            double sum2 = 0.0;
            double sum3 = 0.0;
            int j = i+1;
            for (; j+3 < n; j += 4) {
                sum0 += row[j]*x[static_cast<std::size_t>(j)];
                sum1 += row[j+1]*x[static_cast<std::size_t>(j+1)];
                sum2 += row[j+2]*x[static_cast<std::size_t>(j+2)];
                sum3 += row[j+3]*x[static_cast<std::size_t>(j+3)];
            }
            double value = x[static_cast<std::size_t>(i)]
                -(sum0+sum1+sum2+sum3);
            for (; j < n; ++j) {
                value -= row[j]*x[static_cast<std::size_t>(j)];
            }
            x[static_cast<std::size_t>(i)] = value/row[i];
        }
        for (int k = n-1; k >= 0; --k) {
            const int column = column_pivot[static_cast<std::size_t>(k)];
            if (column != k) {
                std::swap(
                    x[static_cast<std::size_t>(k)],
                    x[static_cast<std::size_t>(column)]
                );
            }
        }
        for (int i = 0; i < n; ++i) {
            x[static_cast<std::size_t>(i)] *=
                column_scale[static_cast<std::size_t>(i)];
        }
    }

};

struct MklDenseFactorization {
    using Getrf = int (*)(int, int, int, double*, int, int*);
    using Getrs = int (*)(int, char, int, int, const double*, int,
                          const int*, double*, int);
    using SetThreads = void (*)(int);

    static constexpr int kColumnMajor = 102;
    static constexpr char kNoTranspose = 'N';

    int dim{0};
    std::vector<double> lu;
    std::vector<int> pivot;
    std::vector<double> row_scale;
    std::vector<double> column_scale;
    Getrs getrs{nullptr};
    HMODULE library{nullptr};

    static HMODULE load_library() {
        const char* path = std::getenv("SUSPENSION_AXLE_MKL_DLL");
        if (path == nullptr || path[0] == '\0') return nullptr;
        return LoadLibraryA(path);
    }

    bool factor(std::vector<double> matrix, int n) {
        if (n < 128 || matrix.size() != static_cast<std::size_t>(n*n)) {
            return false;
        }
        if (library == nullptr) library = load_library();
        if (library == nullptr) return false;
        Getrf getrf = nullptr;
        const FARPROC getrf_address = GetProcAddress(
            library, "LAPACKE_dgetrf"
        );
        std::memcpy(&getrf, &getrf_address, sizeof(getrf));
        const FARPROC getrs_address = GetProcAddress(
            library, "LAPACKE_dgetrs"
        );
        std::memcpy(&getrs, &getrs_address, sizeof(getrs));
        if (getrf == nullptr || getrs == nullptr) return false;
        SetThreads set_threads = nullptr;
        const FARPROC set_threads_address = GetProcAddress(
            library, "MKL_Set_Num_Threads"
        );
        std::memcpy(&set_threads, &set_threads_address, sizeof(set_threads));
        if (set_threads != nullptr) set_threads(linear_solver_threads());
        dim = n;
        row_scale.assign(static_cast<std::size_t>(n), 1.0);
        column_scale.assign(static_cast<std::size_t>(n), 1.0);
        for (int row = 0; row < n; ++row) {
            double maximum = 0.0;
            for (int column = 0; column < n; ++column) {
                const double value = matrix[static_cast<std::size_t>(
                    row*n+column
                )];
                if (!std::isfinite(value)) return false;
                maximum = std::max(maximum, std::abs(value));
            }
            if (!(maximum > 0.0)) return false;
            row_scale[static_cast<std::size_t>(row)] = 1.0/maximum;
            for (int column = 0; column < n; ++column) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    row_scale[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < n; ++column) {
            double maximum = 0.0;
            for (int row = 0; row < n; ++row) {
                maximum = std::max(
                    maximum,
                    std::abs(matrix[static_cast<std::size_t>(row*n+column)])
                );
            }
            if (!(maximum > 0.0)) return false;
            column_scale[static_cast<std::size_t>(column)] = 1.0/maximum;
            for (int row = 0; row < n; ++row) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    column_scale[static_cast<std::size_t>(column)];
            }
        }
        pivot.assign(static_cast<std::size_t>(n), 0);
        std::vector<double> column_major(
            static_cast<std::size_t>(n*n), 0.0
        );
        for (int row = 0; row < n; ++row) {
            for (int column = 0; column < n; ++column) {
                column_major[static_cast<std::size_t>(column*n+row)] =
                    matrix[static_cast<std::size_t>(row*n+column)];
            }
        }
        const int info = getrf(
            kColumnMajor, n, n, column_major.data(), n, pivot.data()
        );
        if (info != 0) return false;
        lu = std::move(column_major);
        return true;
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (getrs == nullptr || static_cast<int>(right_hand_side.size()) != dim) {
            return false;
        }
        solution.resize(static_cast<std::size_t>(dim));
        for (int row = 0; row < dim; ++row) {
            solution[static_cast<std::size_t>(row)] =
                right_hand_side[static_cast<std::size_t>(row)]
                * row_scale[static_cast<std::size_t>(row)];
        }
        const int info = getrs(
            kColumnMajor, kNoTranspose, dim, 1, lu.data(), dim,
            pivot.data(), solution.data(), dim
        );
        if (info != 0) return false;
        for (int column = 0; column < dim; ++column) {
            solution[static_cast<std::size_t>(column)] *=
                column_scale[static_cast<std::size_t>(column)];
        }
        return finite_vec(solution);
    }
};

struct MklPardisoFactorization {
    using Pardiso = void (*)(
        void**, int*, int*, int*, int*, int*, double*, int*, int*, int*,
        int*, int*, int*, double*, double*, int*
    );

    int dim{0};
    void* pt[64]{};
    int maxfct{1};
    int mnum{1};
    int matrix_type{11};
    int nrhs{1};
    int message_level{0};
    int error{0};
    std::array<int, 64> iparm{};
    std::vector<int> row_ptr;
    std::vector<int> columns;
    std::vector<int> permutation;
    std::vector<double> values;
    std::vector<double> row_scale;
    std::vector<double> column_scale;
    bool analyzed{false};
    Pardiso pardiso_function{nullptr};
    HMODULE library{nullptr};

    ~MklPardisoFactorization() {
        release();
    }

    void release() {
        if (pardiso_function == nullptr || !analyzed) return;
        int phase = -1;
        pardiso_function(
            pt, &maxfct, &mnum, &matrix_type, &phase, &dim,
            nullptr, row_ptr.data(), columns.data(), permutation.data(),
            &nrhs, iparm.data(), &message_level, nullptr, nullptr, &error
        );
        analyzed = false;
        std::fill(std::begin(pt), std::end(pt), nullptr);
    }

    bool load() {
        if (pardiso_function != nullptr) return true;
        if (library == nullptr) library = LoadLibraryA(
            std::getenv("SUSPENSION_AXLE_MKL_DLL")
        );
        if (library == nullptr) return false;
        FARPROC address = GetProcAddress(library, "PARDISO");
        if (address == nullptr) address = GetProcAddress(library, "pardiso");
        if (address == nullptr) return false;
        std::memcpy(&pardiso_function, &address, sizeof(pardiso_function));
        return pardiso_function != nullptr;
    }

    bool factor(std::vector<double> matrix, int n) {
        if (n < 128 || matrix.size() != static_cast<std::size_t>(n*n) ||
            !load()) return false;
        dim = n;
        std::vector<double> next_row_scale(static_cast<std::size_t>(n), 1.0);
        std::vector<double> next_column_scale(
            static_cast<std::size_t>(n), 1.0
        );
        for (int row = 0; row < n; ++row) {
            double maximum = 0.0;
            for (int column = 0; column < n; ++column) {
                const double value = matrix[static_cast<std::size_t>(
                    row*n+column
                )];
                if (!std::isfinite(value)) return false;
                maximum = std::max(maximum, std::abs(value));
            }
            if (!(maximum > 0.0)) return false;
            next_row_scale[static_cast<std::size_t>(row)] = 1.0/maximum;
            for (int column = 0; column < n; ++column) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    next_row_scale[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < n; ++column) {
            double maximum = 0.0;
            for (int row = 0; row < n; ++row) {
                maximum = std::max(
                    maximum,
                    std::abs(matrix[static_cast<std::size_t>(row*n+column)])
                );
            }
            if (!(maximum > 0.0)) return false;
            next_column_scale[static_cast<std::size_t>(column)] = 1.0/maximum;
            for (int row = 0; row < n; ++row) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    next_column_scale[static_cast<std::size_t>(column)];
            }
        }
        // Numerical cancellation can make a structurally nonzero Jacobian
        // entry cross 1e-14 between Newton refreshes.  Keep those entries in
        // the fixed CSR structure so PARDISO can reuse its symbolic analysis;
        // values remain updated on every numeric factorization.
        constexpr double pattern_tolerance = 1.0e-14;
        std::vector<int> next_row_ptr(static_cast<std::size_t>(n+1), 0);
        std::vector<int> next_columns;
        for (int row = 0; row < n; ++row) {
            next_row_ptr[static_cast<std::size_t>(row)] =
                static_cast<int>(next_columns.size())+1;
            for (int column = 0; column < n; ++column) {
                if (row == column || std::abs(matrix[static_cast<std::size_t>(
                        row*n+column
                    )]) > pattern_tolerance) {
                    next_columns.push_back(column+1);
                }
            }
        }
        next_row_ptr[static_cast<std::size_t>(n)] =
            static_cast<int>(next_columns.size())+1;
        // Keep a monotonic structural superset for this factorization object.
        // Entries that are numerically zero in one Newton Jacobian can become
        // nonzero in the next one.  Retaining the old columns stores those
        // entries as explicit zeros and lets phase 22 reuse the symbolic
        // analysis without omitting a later physical Jacobian contribution.
        if (analyzed) {
            std::vector<int> merged_row_ptr(static_cast<std::size_t>(n+1), 0);
            std::vector<int> merged_columns;
            for (int row = 0; row < n; ++row) {
                merged_row_ptr[static_cast<std::size_t>(row)] =
                    static_cast<int>(merged_columns.size())+1;
                const int old_begin = row_ptr[static_cast<std::size_t>(row)]-1;
                const int old_end = row_ptr[static_cast<std::size_t>(row+1)]-1;
                const int new_begin = next_row_ptr[static_cast<std::size_t>(row)]-1;
                const int new_end = next_row_ptr[static_cast<std::size_t>(row+1)]-1;
                std::set_union(
                    columns.begin()+old_begin, columns.begin()+old_end,
                    next_columns.begin()+new_begin, next_columns.begin()+new_end,
                    std::back_inserter(merged_columns)
                );
            }
            merged_row_ptr[static_cast<std::size_t>(n)] =
                static_cast<int>(merged_columns.size())+1;
            next_row_ptr = std::move(merged_row_ptr);
            next_columns = std::move(merged_columns);
        }
        if (next_columns.size() > static_cast<std::size_t>(n*n/4)) {
            return false;
        }
        const bool same_pattern = analyzed && row_ptr == next_row_ptr &&
            columns == next_columns;
        if (mkl_pardiso_debug_enabled()) {
            std::fprintf(
                stderr,
                "PARDISO 模式: n=%d nnz=%zu same=%d analyzed=%d\n",
                n, next_columns.size(), same_pattern ? 1 : 0,
                analyzed ? 1 : 0
            );
        }
        if (!same_pattern && analyzed) release();
        row_ptr = std::move(next_row_ptr);
        columns = std::move(next_columns);
        values.clear();
        values.reserve(columns.size());
        for (int row = 0; row < n; ++row) {
            for (int index = row_ptr[static_cast<std::size_t>(row)]-1;
                 index < row_ptr[static_cast<std::size_t>(row+1)]-1; ++index) {
                const int column = columns[static_cast<std::size_t>(index)]-1;
                values.push_back(matrix[static_cast<std::size_t>(
                    row*n+column
                )]);
            }
        }
        row_scale = std::move(next_row_scale);
        column_scale = std::move(next_column_scale);
        permutation.assign(static_cast<std::size_t>(n), 0);
        if (!analyzed) {
            iparm.fill(0);
            iparm[0] = 1;
            iparm[1] = mkl_pardiso_ordering();
            iparm[2] = mkl_pardiso_threads();
            iparm[3] = 0;
            iparm[4] = 0;
            iparm[5] = 0;
            iparm[7] = 2;
            iparm[9] = mkl_pardiso_pivot_perturbation();
            iparm[10] = 1;
            iparm[12] = mkl_pardiso_matching_enabled() ? 1 : 0;
            iparm[17] = -1;
            iparm[18] = -1;
            int phase = 12;
            error = 0;
            pardiso_function(
                pt, &maxfct, &mnum, &matrix_type, &phase, &dim,
                values.data(), row_ptr.data(), columns.data(),
                permutation.data(), &nrhs, iparm.data(), &message_level,
                nullptr, nullptr, &error
            );
            if (error != 0) {
                release();
                return false;
            }
            analyzed = true;
        } else {
            int phase = 22;
            error = 0;
            pardiso_function(
                pt, &maxfct, &mnum, &matrix_type, &phase, &dim,
                values.data(), row_ptr.data(), columns.data(),
                permutation.data(), &nrhs, iparm.data(), &message_level,
                nullptr, nullptr, &error
            );
            if (error != 0) return false;
        }
        return true;
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (!analyzed || static_cast<int>(right_hand_side.size()) != dim) {
            return false;
        }
        std::vector<double> scaled_rhs(static_cast<std::size_t>(dim), 0.0);
        solution.assign(static_cast<std::size_t>(dim), 0.0);
        for (int row = 0; row < dim; ++row) {
            scaled_rhs[static_cast<std::size_t>(row)] =
                right_hand_side[static_cast<std::size_t>(row)]
                * row_scale[static_cast<std::size_t>(row)];
        }
        int phase = 33;
        int local_error = 0;
        pardiso_function(
            const_cast<void**>(pt), const_cast<int*>(&maxfct),
            const_cast<int*>(&mnum), const_cast<int*>(&matrix_type), &phase,
            const_cast<int*>(&dim), const_cast<double*>(values.data()),
            const_cast<int*>(row_ptr.data()), const_cast<int*>(columns.data()),
            const_cast<int*>(permutation.data()), const_cast<int*>(&nrhs),
            const_cast<int*>(iparm.data()), const_cast<int*>(&message_level),
            scaled_rhs.data(), solution.data(), &local_error
        );
        if (local_error != 0) return false;
        if (mkl_pardiso_debug_enabled()) {
            double solution_scale = 1.0;
            double residual_scale = 0.0;
            double rhs_scale = 1.0;
            for (double value : scaled_rhs) {
                rhs_scale = std::max(rhs_scale, std::abs(value));
            }
            for (double value : solution) {
                solution_scale = std::max(solution_scale, std::abs(value));
            }
            for (int row = 0; row < dim; ++row) {
                double value = 0.0;
                for (int index = row_ptr[static_cast<std::size_t>(row)]-1;
                     index < row_ptr[static_cast<std::size_t>(row+1)]-1;
                     ++index) {
                    const int column = columns[static_cast<std::size_t>(index)]-1;
                    value += values[static_cast<std::size_t>(index)] *
                        solution[static_cast<std::size_t>(column)];
                }
                residual_scale = std::max(
                    residual_scale,
                    std::abs(value-scaled_rhs[static_cast<std::size_t>(row)])
                );
            }
            std::fprintf(
                stderr,
                "PARDISO 线性残量: n=%d rhs=%.6g x=%.6g residual=%.6g\n",
                dim, rhs_scale, solution_scale, residual_scale
            );
        }
        for (int column = 0; column < dim; ++column) {
            solution[static_cast<std::size_t>(column)] *=
                column_scale[static_cast<std::size_t>(column)];
        }
        return finite_vec(solution);
    }
};

struct SparseGmresFactorization {
    int dim{0};
    int restart{48};
    int max_iterations{240};
    std::vector<int> row_ptr;
    std::vector<int> columns;
    std::vector<int> diagonal;
    std::vector<int> lookup;
    std::vector<double> values;
    std::vector<double> ilu;
    std::vector<double> row_scale;
    std::vector<double> column_scale;
    std::vector<int> permutation;
    std::vector<int> inverse_permutation;
    mutable std::vector<double> previous_solution;

    static double norm(const std::vector<double>& values) {
        double sum = 0.0;
        for (double value : values) sum += value*value;
        return std::sqrt(sum);
    }

    void multiply(
        const std::vector<double>& vector,
        std::vector<double>& result
    ) const {
        result.assign(static_cast<std::size_t>(dim), 0.0);
        for (int row = 0; row < dim; ++row) {
            double value = 0.0;
            for (int index = row_ptr[static_cast<std::size_t>(row)];
                 index < row_ptr[static_cast<std::size_t>(row+1)];
                 ++index) {
                value += values[static_cast<std::size_t>(index)]
                    * vector[static_cast<std::size_t>(
                        columns[static_cast<std::size_t>(index)]
                    )];
            }
            result[static_cast<std::size_t>(row)] = value;
        }
    }

    void precondition(
        const std::vector<double>& right_hand_side,
        std::vector<double>& result
    ) const {
        result = right_hand_side;
        for (int row = 0; row < dim; ++row) {
            double value = result[static_cast<std::size_t>(row)];
            for (int index = row_ptr[static_cast<std::size_t>(row)];
                 index < row_ptr[static_cast<std::size_t>(row+1)];
                 ++index) {
                const int column = columns[static_cast<std::size_t>(index)];
                if (column >= row) break;
                value -= ilu[static_cast<std::size_t>(index)]
                    * result[static_cast<std::size_t>(column)];
            }
            result[static_cast<std::size_t>(row)] = value;
        }
        for (int row = dim-1; row >= 0; --row) {
            double value = result[static_cast<std::size_t>(row)];
            for (int index = row_ptr[static_cast<std::size_t>(row)];
                 index < row_ptr[static_cast<std::size_t>(row+1)];
                 ++index) {
                const int column = columns[static_cast<std::size_t>(index)];
                if (column <= row) continue;
                value -= ilu[static_cast<std::size_t>(index)]
                    * result[static_cast<std::size_t>(column)];
            }
            const double pivot = ilu[static_cast<std::size_t>(
                diagonal[static_cast<std::size_t>(row)]
            )];
            result[static_cast<std::size_t>(row)] = value/pivot;
        }
    }

    bool factor(std::vector<double> matrix, int n) {
        if (n < 128 || matrix.size() != static_cast<std::size_t>(n*n)) {
            return false;
        }
        dim = n;
        restart = sparse_gmres_restart();
        max_iterations = sparse_gmres_max_iterations();
        std::vector<std::vector<int>> adjacency(
            static_cast<std::size_t>(n)
        );
        constexpr double graph_tolerance = 1.0e-13;
        for (int row = 0; row < n; ++row) {
            for (int column = 0; column < n; ++column) {
                if (row == column || std::abs(matrix[static_cast<std::size_t>(
                        row*n+column
                    )]) <= graph_tolerance) {
                    continue;
                }
                adjacency[static_cast<std::size_t>(row)].push_back(column);
                adjacency[static_cast<std::size_t>(column)].push_back(row);
            }
        }
        for (auto& neighbors : adjacency) {
            std::sort(neighbors.begin(), neighbors.end());
            neighbors.erase(
                std::unique(neighbors.begin(), neighbors.end()), neighbors.end()
            );
        }
        permutation.clear();
        permutation.reserve(static_cast<std::size_t>(n));
        std::vector<unsigned char> visited(static_cast<std::size_t>(n), 0);
        while (static_cast<int>(permutation.size()) < n) {
            int start = -1;
            int start_degree = n+1;
            for (int node = 0; node < n; ++node) {
                if (visited[static_cast<std::size_t>(node)] != 0) continue;
                const int degree = static_cast<int>(
                    adjacency[static_cast<std::size_t>(node)].size()
                );
                if (degree < start_degree) {
                    start = node;
                    start_degree = degree;
                }
            }
            if (start < 0) return false;
            std::vector<int> queue{start};
            visited[static_cast<std::size_t>(start)] = 1;
            for (std::size_t head = 0; head < queue.size(); ++head) {
                const int node = queue[head];
                permutation.push_back(node);
                std::vector<int> neighbors;
                for (int neighbor : adjacency[static_cast<std::size_t>(node)]) {
                    if (visited[static_cast<std::size_t>(neighbor)] == 0) {
                        neighbors.push_back(neighbor);
                    }
                }
                std::sort(
                    neighbors.begin(), neighbors.end(),
                    [&](int left, int right) {
                        return adjacency[static_cast<std::size_t>(left)].size()
                            < adjacency[static_cast<std::size_t>(right)].size();
                    }
                );
                for (int neighbor : neighbors) {
                    visited[static_cast<std::size_t>(neighbor)] = 1;
                    queue.push_back(neighbor);
                }
            }
        }
        std::reverse(permutation.begin(), permutation.end());
        inverse_permutation.assign(static_cast<std::size_t>(n), -1);
        for (int index = 0; index < n; ++index) {
            inverse_permutation[static_cast<std::size_t>(
                permutation[static_cast<std::size_t>(index)]
            )] = index;
        }
        std::vector<double> permuted_matrix(
            static_cast<std::size_t>(n*n), 0.0
        );
        for (int row = 0; row < n; ++row) {
            const int original_row = permutation[static_cast<std::size_t>(row)];
            for (int column = 0; column < n; ++column) {
                const int original_column = permutation[
                    static_cast<std::size_t>(column)
                ];
                permuted_matrix[static_cast<std::size_t>(row*n+column)] =
                    matrix[static_cast<std::size_t>(
                        original_row*n+original_column
                    )];
            }
        }
        matrix.swap(permuted_matrix);
        row_scale.assign(static_cast<std::size_t>(n), 1.0);
        column_scale.assign(static_cast<std::size_t>(n), 1.0);
        for (int row = 0; row < n; ++row) {
            double maximum = 0.0;
            for (int column = 0; column < n; ++column) {
                const double value = matrix[static_cast<std::size_t>(
                    row*n+column
                )];
                if (!std::isfinite(value)) return false;
                maximum = std::max(maximum, std::abs(value));
            }
            if (!(maximum > 0.0)) return false;
            row_scale[static_cast<std::size_t>(row)] = 1.0/maximum;
            for (int column = 0; column < n; ++column) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    row_scale[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < n; ++column) {
            double maximum = 0.0;
            for (int row = 0; row < n; ++row) {
                maximum = std::max(
                    maximum,
                    std::abs(matrix[static_cast<std::size_t>(row*n+column)])
                );
            }
            if (!(maximum > 0.0)) return false;
            column_scale[static_cast<std::size_t>(column)] = 1.0/maximum;
            for (int row = 0; row < n; ++row) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    column_scale[static_cast<std::size_t>(column)];
            }
        }
        row_ptr.assign(static_cast<std::size_t>(n+1), 0);
        columns.clear();
        values.clear();
        diagonal.assign(static_cast<std::size_t>(n), -1);
        constexpr double pattern_tolerance = 1.0e-13;
        for (int row = 0; row < n; ++row) {
            row_ptr[static_cast<std::size_t>(row)] =
                static_cast<int>(columns.size());
            for (int column = 0; column < n; ++column) {
                const double value = matrix[static_cast<std::size_t>(
                    row*n+column
                )];
                if (std::abs(value) <= pattern_tolerance && row != column) {
                    continue;
                }
                if (row == column) {
                    diagonal[static_cast<std::size_t>(row)] =
                        static_cast<int>(columns.size());
                }
                columns.push_back(column);
                values.push_back(value);
            }
        }
        row_ptr[static_cast<std::size_t>(n)] =
            static_cast<int>(columns.size());
        if (columns.size() > static_cast<std::size_t>(n*n/4)) return false;
        for (int row = 0; row < n; ++row) {
            if (diagonal[static_cast<std::size_t>(row)] < 0) return false;
        }
        lookup.assign(static_cast<std::size_t>(n*n), -1);
        for (int row = 0; row < n; ++row) {
            for (int index = row_ptr[static_cast<std::size_t>(row)];
                 index < row_ptr[static_cast<std::size_t>(row+1)];
                 ++index) {
                lookup[static_cast<std::size_t>(row*n+
                    columns[static_cast<std::size_t>(index)])] = index;
            }
        }
        ilu = values;
        for (int row = 0; row < n; ++row) {
            for (int index = row_ptr[static_cast<std::size_t>(row)];
                 index < row_ptr[static_cast<std::size_t>(row+1)];
                 ++index) {
                const int lower_column = columns[static_cast<std::size_t>(index)];
                if (lower_column >= row) break;
                const int pivot_index = diagonal[
                    static_cast<std::size_t>(lower_column)
                ];
                const double pivot = ilu[static_cast<std::size_t>(pivot_index)];
                if (!std::isfinite(pivot) || std::abs(pivot) < 1.0e-14) {
                    return false;
                }
                const double multiplier =
                    ilu[static_cast<std::size_t>(index)]/pivot;
                ilu[static_cast<std::size_t>(index)] = multiplier;
                for (int pivot_row_index = row_ptr[
                        static_cast<std::size_t>(lower_column)];
                     pivot_row_index < row_ptr[
                        static_cast<std::size_t>(lower_column+1)];
                     ++pivot_row_index) {
                    const int column = columns[
                        static_cast<std::size_t>(pivot_row_index)
                    ];
                    if (column <= lower_column) continue;
                    const int target = lookup[static_cast<std::size_t>(
                        row*n+column
                    )];
                    if (target >= 0) {
                        ilu[static_cast<std::size_t>(target)] -=
                            multiplier*ilu[static_cast<std::size_t>(
                                pivot_row_index
                            )];
                    }
                }
            }
            const double pivot = ilu[static_cast<std::size_t>(
                diagonal[static_cast<std::size_t>(row)]
            )];
            if (!std::isfinite(pivot) || std::abs(pivot) < 1.0e-14) {
                return false;
            }
        }
        previous_solution.clear();
        return true;
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (static_cast<int>(right_hand_side.size()) != dim) return false;
        std::vector<double> scaled_rhs(static_cast<std::size_t>(dim), 0.0);
        for (int row = 0; row < dim; ++row) {
            const int original_row = permutation[static_cast<std::size_t>(row)];
            scaled_rhs[static_cast<std::size_t>(row)] =
                right_hand_side[static_cast<std::size_t>(original_row)]
                * row_scale[static_cast<std::size_t>(row)];
        }
        std::vector<double> current(static_cast<std::size_t>(dim), 0.0);
        if (previous_solution.size() == current.size()) {
            current = previous_solution;
        }
        std::vector<double> product;
        multiply(current, product);
        std::vector<double> residual(static_cast<std::size_t>(dim), 0.0);
        for (int row = 0; row < dim; ++row) {
            residual[static_cast<std::size_t>(row)] =
                scaled_rhs[static_cast<std::size_t>(row)]
                - product[static_cast<std::size_t>(row)];
        }
        std::vector<double> preconditioned;
        precondition(residual, preconditioned);
        const double rhs_norm = std::max(1.0, norm(scaled_rhs));
        const double tolerance = 1.0e-10*rhs_norm;
        double residual_norm = norm(preconditioned);
        int iterations = 0;
        while (residual_norm > tolerance && iterations < max_iterations) {
            const int dimension = std::min(restart, max_iterations-iterations);
            std::vector<double> basis(
                static_cast<std::size_t>((dimension+1)*dim), 0.0
            );
            std::vector<double> hessenberg(
                static_cast<std::size_t>((dimension+1)*dimension), 0.0
            );
            std::vector<double> cosines(static_cast<std::size_t>(dimension), 0.0);
            std::vector<double> sines(static_cast<std::size_t>(dimension), 0.0);
            std::vector<double> transformed_rhs(
                static_cast<std::size_t>(dimension+1), 0.0
            );
            const double beta = residual_norm;
            transformed_rhs[0] = beta;
            for (int row = 0; row < dim; ++row) {
                basis[static_cast<std::size_t>(row)] =
                    preconditioned[static_cast<std::size_t>(row)]/beta;
            }
            int inner_count = 0;
            for (int column = 0; column < dimension; ++column) {
                std::vector<double> vector(static_cast<std::size_t>(dim), 0.0);
                for (int row = 0; row < dim; ++row) {
                    vector[static_cast<std::size_t>(row)] =
                        basis[static_cast<std::size_t>(column*dim+row)];
                }
                multiply(vector, product);
                precondition(product, vector);
                for (int basis_column = 0;
                     basis_column <= column; ++basis_column) {
                    double value = 0.0;
                    for (int row = 0; row < dim; ++row) {
                        value += vector[static_cast<std::size_t>(row)]
                            * basis[static_cast<std::size_t>(
                                basis_column*dim+row
                            )];
                    }
                    hessenberg[static_cast<std::size_t>(
                        basis_column*dimension+column
                    )] = value;
                    for (int row = 0; row < dim; ++row) {
                        vector[static_cast<std::size_t>(row)] -= value
                            * basis[static_cast<std::size_t>(
                                basis_column*dim+row
                            )];
                    }
                }
                const double next_norm = norm(vector);
                hessenberg[static_cast<std::size_t>(
                    (column+1)*dimension+column
                )] = next_norm;
                if (next_norm > 0.0) {
                    for (int row = 0; row < dim; ++row) {
                        basis[static_cast<std::size_t>((column+1)*dim+row)] =
                            vector[static_cast<std::size_t>(row)]/next_norm;
                    }
                }
                for (int row = 0; row < column; ++row) {
                    const double upper = hessenberg[static_cast<std::size_t>(
                        row*dimension+column
                    )];
                    const double lower = hessenberg[static_cast<std::size_t>(
                        (row+1)*dimension+column
                    )];
                    hessenberg[static_cast<std::size_t>(
                        row*dimension+column
                    )] = cosines[static_cast<std::size_t>(row)]*upper
                        + sines[static_cast<std::size_t>(row)]*lower;
                    hessenberg[static_cast<std::size_t>(
                        (row+1)*dimension+column
                    )] = -sines[static_cast<std::size_t>(row)]*upper
                        + cosines[static_cast<std::size_t>(row)]*lower;
                }
                const double upper = hessenberg[static_cast<std::size_t>(
                    column*dimension+column
                )];
                const double lower = hessenberg[static_cast<std::size_t>(
                    (column+1)*dimension+column
                )];
                const double denominator = std::hypot(upper, lower);
                if (!(denominator > 0.0)) {
                    inner_count = column+1;
                    break;
                }
                cosines[static_cast<std::size_t>(column)] = upper/denominator;
                sines[static_cast<std::size_t>(column)] = lower/denominator;
                hessenberg[static_cast<std::size_t>(
                    column*dimension+column
                )] = denominator;
                hessenberg[static_cast<std::size_t>(
                    (column+1)*dimension+column
                )] = 0.0;
                const double upper_rhs = transformed_rhs[
                    static_cast<std::size_t>(column)
                ];
                const double lower_rhs = transformed_rhs[
                    static_cast<std::size_t>(column+1)
                ];
                transformed_rhs[static_cast<std::size_t>(column)] =
                    cosines[static_cast<std::size_t>(column)]*upper_rhs
                    + sines[static_cast<std::size_t>(column)]*lower_rhs;
                transformed_rhs[static_cast<std::size_t>(column+1)] =
                    -sines[static_cast<std::size_t>(column)]*upper_rhs
                    + cosines[static_cast<std::size_t>(column)]*lower_rhs;
                inner_count = column+1;
                if (std::abs(transformed_rhs[static_cast<std::size_t>(
                        column+1
                    )]) <= tolerance) break;
            }
            if (inner_count <= 0) return false;
            std::vector<double> coefficients(
                static_cast<std::size_t>(inner_count), 0.0
            );
            for (int row = inner_count-1; row >= 0; --row) {
                double value = transformed_rhs[static_cast<std::size_t>(row)];
                for (int column = row+1; column < inner_count; ++column) {
                    value -= hessenberg[static_cast<std::size_t>(
                        row*dimension+column
                    )] * coefficients[static_cast<std::size_t>(column)];
                }
                const double pivot = hessenberg[static_cast<std::size_t>(
                    row*dimension+row
                )];
                if (!(std::abs(pivot) > 1.0e-14)) return false;
                coefficients[static_cast<std::size_t>(row)] = value/pivot;
            }
            for (int column = 0; column < inner_count; ++column) {
                const double coefficient = coefficients[
                    static_cast<std::size_t>(column)
                ];
                for (int row = 0; row < dim; ++row) {
                    current[static_cast<std::size_t>(row)] += coefficient
                        * basis[static_cast<std::size_t>(column*dim+row)];
                }
            }
            multiply(current, product);
            for (int row = 0; row < dim; ++row) {
                residual[static_cast<std::size_t>(row)] =
                    scaled_rhs[static_cast<std::size_t>(row)]
                    - product[static_cast<std::size_t>(row)];
            }
            precondition(residual, preconditioned);
            residual_norm = norm(preconditioned);
            iterations += inner_count;
        }
        if (!std::isfinite(residual_norm) || residual_norm > tolerance) {
            return false;
        }
        previous_solution = current;
        solution.resize(static_cast<std::size_t>(dim));
        for (int row = 0; row < dim; ++row) {
            const int original_row = permutation[static_cast<std::size_t>(row)];
            solution[static_cast<std::size_t>(original_row)] =
                current[static_cast<std::size_t>(row)]
                * column_scale[static_cast<std::size_t>(row)];
        }
        return finite_vec(solution);
    }
};

struct SparseLuFactorization {
    using SparseRow = std::unordered_map<int, double>;

    int dim{0};
    std::vector<SparseRow> rows;
    std::vector<int> pivot;
    std::vector<double> row_scale;
    std::vector<double> column_scale;

    bool factor(std::vector<double> matrix, int n) {
        if (n < 128 || matrix.size() != static_cast<std::size_t>(n*n)) {
            return false;
        }
        dim = n;
        row_scale.assign(static_cast<std::size_t>(n), 1.0);
        column_scale.assign(static_cast<std::size_t>(n), 1.0);
        for (int row = 0; row < n; ++row) {
            double maximum = 0.0;
            for (int column = 0; column < n; ++column) {
                const double value = matrix[static_cast<std::size_t>(
                    row*n+column
                )];
                if (!std::isfinite(value)) return false;
                maximum = std::max(maximum, std::abs(value));
            }
            if (!(maximum > 0.0)) return false;
            row_scale[static_cast<std::size_t>(row)] = 1.0/maximum;
            for (int column = 0; column < n; ++column) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    row_scale[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < n; ++column) {
            double maximum = 0.0;
            for (int row = 0; row < n; ++row) {
                maximum = std::max(
                    maximum,
                    std::abs(matrix[static_cast<std::size_t>(row*n+column)])
                );
            }
            if (!(maximum > 0.0)) return false;
            column_scale[static_cast<std::size_t>(column)] = 1.0/maximum;
            for (int row = 0; row < n; ++row) {
                matrix[static_cast<std::size_t>(row*n+column)] *=
                    column_scale[static_cast<std::size_t>(column)];
            }
        }
        rows.assign(static_cast<std::size_t>(n), SparseRow{});
        constexpr double drop_tolerance = 1.0e-15;
        for (int row = 0; row < n; ++row) {
            SparseRow& sparse_row = rows[static_cast<std::size_t>(row)];
            sparse_row.reserve(64);
            for (int column = 0; column < n; ++column) {
                const double value = matrix[static_cast<std::size_t>(
                    row*n+column
                )];
                if (row == column || std::abs(value) > drop_tolerance) {
                    sparse_row.emplace(column, value);
                }
            }
        }
        pivot.assign(static_cast<std::size_t>(n), -1);
        for (int step = 0; step < n; ++step) {
            int best_row = -1;
            double best = 0.0;
            for (int row = step; row < n; ++row) {
                const auto item = rows[static_cast<std::size_t>(row)].find(step);
                if (item != rows[static_cast<std::size_t>(row)].end() &&
                    std::abs(item->second) > best) {
                    best = std::abs(item->second);
                    best_row = row;
                }
            }
            if (best_row < 0 || best < 1.0e-14) return false;
            pivot[static_cast<std::size_t>(step)] = best_row;
            if (best_row != step) {
                std::swap(
                    rows[static_cast<std::size_t>(step)],
                    rows[static_cast<std::size_t>(best_row)]
                );
            }
            const double diagonal = rows[static_cast<std::size_t>(step)][step];
            if (!std::isfinite(diagonal) || std::abs(diagonal) < 1.0e-14) {
                return false;
            }
            const SparseRow pivot_row = rows[static_cast<std::size_t>(step)];
            for (int row = step+1; row < n; ++row) {
                SparseRow& target_row = rows[static_cast<std::size_t>(row)];
                auto target_pivot = target_row.find(step);
                if (target_pivot == target_row.end()) continue;
                const double multiplier = target_pivot->second/diagonal;
                target_pivot->second = multiplier;
                for (const auto& item : pivot_row) {
                    const int column = item.first;
                    if (column <= step) continue;
                    const double updated =
                        target_row[column]-multiplier*item.second;
                    if (std::abs(updated) <= drop_tolerance) {
                        target_row.erase(column);
                    } else {
                        target_row[column] = updated;
                    }
                }
            }
            if (step % 16 == 0) {
                std::size_t entries = 0;
                for (const SparseRow& row : rows) entries += row.size();
                if (entries > static_cast<std::size_t>(n*n/3)) return false;
            }
        }
        return true;
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (static_cast<int>(right_hand_side.size()) != dim) return false;
        solution.resize(static_cast<std::size_t>(dim));
        for (int row = 0; row < dim; ++row) {
            solution[static_cast<std::size_t>(row)] =
                right_hand_side[static_cast<std::size_t>(row)]
                * row_scale[static_cast<std::size_t>(row)];
        }
        for (int step = 0; step < dim; ++step) {
            const int row = pivot[static_cast<std::size_t>(step)];
            if (row != step) std::swap(
                solution[static_cast<std::size_t>(step)],
                solution[static_cast<std::size_t>(row)]
            );
        }
        for (int row = 0; row < dim; ++row) {
            double value = solution[static_cast<std::size_t>(row)];
            for (const auto& item : rows[static_cast<std::size_t>(row)]) {
                if (item.first < row) {
                    value -= item.second*solution[
                        static_cast<std::size_t>(item.first)
                    ];
                }
            }
            solution[static_cast<std::size_t>(row)] = value;
        }
        for (int row = dim-1; row >= 0; --row) {
            double value = solution[static_cast<std::size_t>(row)];
            double diagonal = 0.0;
            for (const auto& item : rows[static_cast<std::size_t>(row)]) {
                if (item.first == row) {
                    diagonal = item.second;
                } else if (item.first > row) {
                    value -= item.second*solution[
                        static_cast<std::size_t>(item.first)
                    ];
                }
            }
            if (!std::isfinite(diagonal) || std::abs(diagonal) < 1.0e-14) {
                return false;
            }
            solution[static_cast<std::size_t>(row)] = value/diagonal;
        }
        for (int column = 0; column < dim; ++column) {
            solution[static_cast<std::size_t>(column)] *=
                column_scale[static_cast<std::size_t>(column)];
        }
        return finite_vec(solution);
    }
};

struct BlockDiagonalMassFactorization {
    std::vector<Mat3> inverse_rotation;
    std::vector<double> inverse_mass;
    int dim{0};
    bool identity{false};

    bool factor(const std::vector<double>& matrix, int n) {
        if (n <= 0 || n % 6 != 0 ||
            matrix.size() != static_cast<std::size_t>(n*n)) {
            return false;
        }
        dim = n;
        identity = false;
        const int blocks = n/6;
        inverse_rotation.assign(static_cast<std::size_t>(blocks), Mat3{});
        inverse_mass.assign(static_cast<std::size_t>(blocks), 0.0);
        double matrix_scale = 1.0;
        for (double value : matrix) {
            if (!std::isfinite(value)) return false;
            matrix_scale = std::max(matrix_scale, std::abs(value));
        }
        const double tolerance = 1.0e-12*matrix_scale;
        for (int row = 0; row < n; ++row) {
            for (int column = 0; column < n; ++column) {
                if (row/6 == column/6) continue;
                if (std::abs(matrix[static_cast<std::size_t>(row*n+column)])
                    > tolerance) {
                    return false;
                }
            }
        }
        for (int block = 0; block < blocks; ++block) {
            const int offset = 6*block;
            const double mass = matrix[static_cast<std::size_t>(
                offset*n+offset
            )];
            if (!(std::abs(mass) > tolerance) ||
                !std::isfinite(mass)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    const double expected = row == column ? mass : 0.0;
                    if (std::abs(matrix[static_cast<std::size_t>(
                            (offset+row)*n+offset+column
                        )] - expected) > tolerance) {
                        return false;
                    }
                }
            }
            Mat3 rotation{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    rotation.a[row][column] = matrix[
                        static_cast<std::size_t>(
                            (offset+3+row)*n+offset+3+column
                        )
                    ];
                }
            }
            if (!inverse3(rotation,
                    inverse_rotation[static_cast<std::size_t>(block)])) {
                return false;
            }
            inverse_mass[static_cast<std::size_t>(block)] = 1.0/mass;
        }
        return true;
    }

    void set_identity(int n) {
        dim = n;
        identity = true;
        inverse_rotation.clear();
        inverse_mass.clear();
    }

    void solve(const std::vector<double>& right_hand_side,
               std::vector<double>& solution) const {
        if (identity) {
            solution = right_hand_side;
            return;
        }
        const int blocks = dim/6;
        solution.assign(static_cast<std::size_t>(dim), 0.0);
        for (int block = 0; block < blocks; ++block) {
            const int offset = 6*block;
            solution[static_cast<std::size_t>(offset)] =
                right_hand_side[static_cast<std::size_t>(offset)]
                * inverse_mass[static_cast<std::size_t>(block)];
            solution[static_cast<std::size_t>(offset+1)] =
                right_hand_side[static_cast<std::size_t>(offset+1)]
                * inverse_mass[static_cast<std::size_t>(block)];
            solution[static_cast<std::size_t>(offset+2)] =
                right_hand_side[static_cast<std::size_t>(offset+2)]
                * inverse_mass[static_cast<std::size_t>(block)];
            const Mat3& inverse = inverse_rotation[
                static_cast<std::size_t>(block)
            ];
            const Vec3 rhs{
                right_hand_side[static_cast<std::size_t>(offset+3)],
                right_hand_side[static_cast<std::size_t>(offset+4)],
                right_hand_side[static_cast<std::size_t>(offset+5)]
            };
            const Vec3 result = inverse*rhs;
            solution[static_cast<std::size_t>(offset+3)] = result.x;
            solution[static_cast<std::size_t>(offset+4)] = result.y;
            solution[static_cast<std::size_t>(offset+5)] = result.z;
        }
    }
};

struct ReducedNewtonFactorization {
    LuFactorization pose_factorization;
    BlockDiagonalMassFactorization acceleration_factorization;
    mutable LuFactorization reduced_factorization;
#ifdef _WIN32
    mutable MklDenseFactorization mkl_reduced_factorization;
    mutable MklPardisoFactorization mkl_pardiso_reduced_factorization;
#endif
    SparseGmresFactorization sparse_reduced_factorization;
    SparseLuFactorization sparse_lu_reduced_factorization;
    std::vector<double> q_from_a;
    std::vector<double> q_from_mu;
    std::vector<double> v_from_a;
    std::vector<double> remaining_q;
    std::vector<double> remaining_v;
    std::vector<std::vector<std::pair<int, double>>> remaining_q_terms_by_row;
    std::vector<std::vector<std::pair<int, double>>> remaining_v_terms_by_row;
    std::vector<std::vector<std::pair<int, double>>> q_reconstruction_terms_by_row;
    int total_dim{0};
    int pose_dim{0};
    int constraint_dim{0};
    int brush_dim{0};
    int reduced_dim{0};
    int acceleration_schur_dim{0};
    bool use_acceleration_schur{false};
    mutable bool use_sparse_reduced{false};
    mutable bool use_sparse_lu_reduced{false};
    mutable bool use_mkl_reduced{false};
    mutable bool use_mkl_pardiso_reduced{false};
    mutable bool sparse_fallback_ready{false};
    mutable std::vector<double> sparse_fallback_matrix;
    std::vector<double> acceleration_to_rest;
    std::vector<double> rest_from_acceleration;

    // These buffers are reused across factor/solve calls.  The factorization
    // is owned by one Newton solve, so mutable solve scratch is thread-safe at
    // the same level as the factorization itself and avoids per-step vectors.
    mutable std::vector<double> solve_pose_rhs;
    mutable std::vector<double> solve_q0;
    mutable std::vector<double> solve_v0;
    mutable std::vector<double> solve_reduced_rhs;
    mutable std::vector<double> solve_reduced_solution;
    mutable std::vector<double> solve_acceleration_rhs;
    mutable std::vector<double> solve_acceleration_base;
    mutable std::vector<double> solve_acceleration_schur_rhs;
    mutable std::vector<double> solve_acceleration_schur_solution;
    std::vector<std::vector<std::pair<int, double>>> rest_from_acceleration_terms_by_row;
    std::vector<std::vector<std::pair<int, double>>> acceleration_to_rest_terms_by_row;

    static int remaining_full_row(
        int row, int pose, int constraints, int brush
    ) {
        (void)brush;
        if (row < pose) return 2*pose+row;
        if (row < pose+constraints) {
            return 3*pose+(row-pose);
        }
        if (row < pose+2*constraints) {
            return 3*pose+constraints+(row-pose-constraints);
        }
        return 3*pose+2*constraints+(row-pose-2*constraints);
    }

    static int reduced_full_column(
        int column, int pose, int constraints, int brush
    ) {
        (void)brush;
        if (column < pose) return 2*pose+column;
        if (column < pose+constraints) {
            return 3*pose+(column-pose);
        }
        if (column < pose+2*constraints) {
            return 3*pose+constraints+(column-pose-constraints);
        }
        return 3*pose+2*constraints+(column-pose-2*constraints);
    }

    static bool row_has_only_blocks(
        const std::vector<double>& matrix, int dimension, int row,
        const std::vector<unsigned char>& allowed
    ) {
        double row_scale = 1.0;
        for (int column = 0; column < dimension; ++column) {
            row_scale = std::max(
                row_scale,
                std::abs(matrix[static_cast<std::size_t>(row*dimension+column)])
            );
        }
        const double tolerance = 1.0e-12*row_scale;
        for (int column = 0; column < dimension; ++column) {
            if (allowed[static_cast<std::size_t>(column)] != 0) continue;
            if (std::abs(matrix[
                    static_cast<std::size_t>(row*dimension+column)
                ]) > tolerance) {
                return false;
            }
        }
        return true;
    }

    bool factor(
        const std::vector<double>& matrix, int dimension,
        int pose, int constraints, int brush
    ) {
        total_dim = dimension;
        pose_dim = pose;
        constraint_dim = constraints;
        brush_dim = brush;
        reduced_dim = pose+2*constraints+brush;
        use_acceleration_schur = false;
        use_sparse_reduced = false;
        use_sparse_lu_reduced = false;
        use_mkl_reduced = false;
        use_mkl_pardiso_reduced = false;
        sparse_fallback_ready = false;
        sparse_fallback_matrix.clear();
        if (dimension != 3*pose+2*constraints+brush || pose <= 0) {
            return false;
        }
        const auto at = [dimension](int row, int column) {
            return static_cast<std::size_t>(row*dimension+column);
        };

        bool pose_identity = true;
        for (int row = 0; row < pose && pose_identity; ++row) {
            for (int column = 0; column < pose; ++column) {
                const double expected = row == column ? 1.0 : 0.0;
                if (matrix[at(row, column)] != expected) {
                    pose_identity = false;
                    break;
                }
            }
        }
        if (pose_identity) {
            pose_factorization.set_identity(pose);
        } else {
            std::vector<double> pose_matrix(
                static_cast<std::size_t>(pose*pose), 0.0
            );
            for (int row = 0; row < pose; ++row) {
                for (int column = 0; column < pose; ++column) {
                    pose_matrix[static_cast<std::size_t>(row*pose+column)] =
                        matrix[at(row, column)];
                }
            }
            if (!pose_factorization.factor(std::move(pose_matrix), pose)) {
                return false;
            }
        }

        std::vector<unsigned char> pose_allowed(
            static_cast<std::size_t>(dimension), 0
        );
        for (int column = 0; column < pose; ++column) {
            pose_allowed[static_cast<std::size_t>(column)] = 1;
            pose_allowed[static_cast<std::size_t>(2*pose+column)] = 1;
        }
        for (int column = 0; column < constraints; ++column) {
            pose_allowed[static_cast<std::size_t>(3*pose+constraints+column)] = 1;
        }
        for (int row = 0; row < pose; ++row) {
            if (!row_has_only_blocks(matrix, dimension, row, pose_allowed)) {
                return false;
            }
        }

        q_from_a.assign(static_cast<std::size_t>(pose*pose), 0.0);
        q_from_mu.assign(static_cast<std::size_t>(pose*constraints), 0.0);
        std::vector<double> right_hand_side(static_cast<std::size_t>(pose), 0.0);
        std::vector<double> solution;
        if (pose_identity) {
            for (int column = 0; column < pose; ++column) {
                for (int row = 0; row < pose; ++row) {
                    q_from_a[static_cast<std::size_t>(column*pose+row)] =
                        -matrix[at(row, 2*pose+column)];
                }
            }
            for (int column = 0; column < constraints; ++column) {
                for (int row = 0; row < pose; ++row) {
                    q_from_mu[static_cast<std::size_t>(column*pose+row)] =
                        -matrix[at(row, 3*pose+constraints+column)];
                }
            }
        } else {
            for (int column = 0; column < pose; ++column) {
                std::fill(right_hand_side.begin(), right_hand_side.end(), 0.0);
                for (int row = 0; row < pose; ++row) {
                    right_hand_side[static_cast<std::size_t>(row)] =
                        matrix[at(row, 2*pose+column)];
                }
                pose_factorization.solve(right_hand_side, solution);
                if (static_cast<int>(solution.size()) != pose ||
                    !finite_vec(solution)) return false;
                for (int row = 0; row < pose; ++row) {
                    q_from_a[static_cast<std::size_t>(column*pose+row)] =
                        -solution[static_cast<std::size_t>(row)];
                }
            }
            for (int column = 0; column < constraints; ++column) {
                std::fill(right_hand_side.begin(), right_hand_side.end(), 0.0);
                for (int row = 0; row < pose; ++row) {
                    right_hand_side[static_cast<std::size_t>(row)] =
                        matrix[at(row, 3*pose+constraints+column)];
                }
                pose_factorization.solve(right_hand_side, solution);
                if (static_cast<int>(solution.size()) != pose ||
                    !finite_vec(solution)) return false;
                for (int row = 0; row < pose; ++row) {
                    q_from_mu[static_cast<std::size_t>(column*pose+row)] =
                        -solution[static_cast<std::size_t>(row)];
                }
            }
        }
        q_reconstruction_terms_by_row.assign(
            static_cast<std::size_t>(pose), {}
        );
        for (int column = 0; column < pose; ++column) {
            for (int row = 0; row < pose; ++row) {
                const double value = q_from_a[
                    static_cast<std::size_t>(column*pose+row)
                ];
                if (value != 0.0) {
                    q_reconstruction_terms_by_row[
                        static_cast<std::size_t>(row)
                    ].emplace_back(column, value);
                }
            }
        }
        for (int column = 0; column < constraints; ++column) {
            for (int row = 0; row < pose; ++row) {
                const double value = q_from_mu[
                    static_cast<std::size_t>(column*pose+row)
                ];
                if (value != 0.0) {
                    q_reconstruction_terms_by_row[
                        static_cast<std::size_t>(row)
                    ].emplace_back(pose+constraints+column, value);
                }
            }
        }

        std::vector<unsigned char> velocity_allowed(
            static_cast<std::size_t>(dimension), 0
        );
        for (int column = 0; column < pose; ++column) {
            velocity_allowed[static_cast<std::size_t>(pose+column)] = 1;
            velocity_allowed[static_cast<std::size_t>(2*pose+column)] = 1;
        }
        for (int row = pose; row < 2*pose; ++row) {
            if (!row_has_only_blocks(matrix, dimension, row, velocity_allowed)) {
                return false;
            }
            const int local_row = row-pose;
            const double diagonal = matrix[at(row, pose+local_row)];
            if (std::abs(diagonal-1.0) > 1.0e-12) return false;
            for (int column = 0; column < pose; ++column) {
                if (column == local_row) continue;
                if (std::abs(matrix[at(row, pose+column)]) > 1.0e-12) {
                    return false;
                }
            }
        }
        v_from_a.assign(static_cast<std::size_t>(pose), 0.0);
        for (int row = 0; row < pose; ++row) {
            v_from_a[static_cast<std::size_t>(row)] =
                -matrix[at(pose+row, 2*pose+row)];
            for (int column = 0; column < pose; ++column) {
                if (column == row) continue;
                if (std::abs(matrix[at(pose+row, 2*pose+column)]) > 1.0e-12) {
                    return false;
                }
            }
        }

        std::vector<double> reduced_matrix(
            static_cast<std::size_t>(reduced_dim*reduced_dim), 0.0
        );
        remaining_q.assign(
            static_cast<std::size_t>(reduced_dim*pose), 0.0
        );
        remaining_v.assign(
            static_cast<std::size_t>(reduced_dim*pose), 0.0
        );
        remaining_q_terms_by_row.assign(
            static_cast<std::size_t>(reduced_dim), {}
        );
        remaining_v_terms_by_row.assign(
            static_cast<std::size_t>(reduced_dim), {}
        );
        std::vector<std::pair<int, double>> q_terms;
        std::vector<std::pair<int, double>> v_terms;
        q_terms.reserve(static_cast<std::size_t>(pose));
        v_terms.reserve(static_cast<std::size_t>(pose));
        for (int row = 0; row < reduced_dim; ++row) {
            const int full_row = remaining_full_row(
                row, pose, constraints, brush
            );
            for (int column = 0; column < pose; ++column) {
                remaining_q[static_cast<std::size_t>(row*pose+column)] =
                    matrix[at(full_row, column)];
                remaining_v[static_cast<std::size_t>(row*pose+column)] =
                    matrix[at(full_row, pose+column)];
            }
            q_terms.clear();
            v_terms.clear();
            for (int source = 0; source < pose; ++source) {
                const double q_value = remaining_q[
                    static_cast<std::size_t>(row*pose+source)
                ];
                if (q_value != 0.0) q_terms.emplace_back(source, q_value);
                const double v_value = remaining_v[
                    static_cast<std::size_t>(row*pose+source)
                ];
                if (v_value != 0.0) v_terms.emplace_back(source, v_value);
            }
            remaining_q_terms_by_row[static_cast<std::size_t>(row)] = q_terms;
            remaining_v_terms_by_row[static_cast<std::size_t>(row)] = v_terms;
            for (int column = 0; column < reduced_dim; ++column) {
                const int full_column = reduced_full_column(
                    column, pose, constraints, brush
                );
                double value = matrix[at(full_row, full_column)];
                if (column < pose) {
                    for (const auto& term : q_terms) {
                        value += term.second * q_from_a[
                            static_cast<std::size_t>(column*pose+term.first)
                        ];
                    }
                    for (const auto& term : v_terms) {
                        if (term.first == column) {
                            value += term.second * v_from_a[
                                static_cast<std::size_t>(column)
                            ];
                        }
                    }
                } else if (column >= pose+constraints &&
                           column < pose+2*constraints) {
                    const int mu_column = column-pose-constraints;
                    for (const auto& term : q_terms) {
                        value += term.second * q_from_mu[
                            static_cast<std::size_t>(mu_column*pose+term.first)
                        ];
                    }
                }
                reduced_matrix[static_cast<std::size_t>(
                    row*reduced_dim+column
                )] = value;
            }
        }
        const bool sparse_factorized = sparse_gmres_enabled() &&
            sparse_reduced_factorization.factor(reduced_matrix, reduced_dim);
        if (sparse_factorized) {
            sparse_fallback_matrix = reduced_matrix;
            sparse_fallback_ready = true;
            use_sparse_reduced = true;
            return true;
        }
#ifdef _WIN32
        if (mkl_pardiso_enabled() &&
            mkl_pardiso_reduced_factorization.factor(
                reduced_matrix, reduced_dim
            )) {
            sparse_fallback_matrix = reduced_matrix;
            sparse_fallback_ready = true;
            use_mkl_pardiso_reduced = true;
            use_mkl_reduced = true;
            return true;
        }
        if (mkl_dense_enabled() &&
            mkl_reduced_factorization.factor(reduced_matrix, reduced_dim)) {
            use_mkl_reduced = true;
            return true;
        }
#endif
        if (sparse_lu_enabled() &&
            sparse_lu_reduced_factorization.factor(reduced_matrix, reduced_dim)) {
            sparse_fallback_matrix = reduced_matrix;
            sparse_fallback_ready = true;
            use_sparse_lu_reduced = true;
            return true;
        }

        if (!acceleration_schur_probe_enabled()) {
            acceleration_to_rest.clear();
            rest_from_acceleration.clear();
            acceleration_to_rest_terms_by_row.clear();
            rest_from_acceleration_terms_by_row.clear();
            sparse_fallback_ready = false;
            sparse_fallback_matrix.clear();
            return reduced_factorization.factor(
                std::move(reduced_matrix), reduced_dim
            );
        }

        acceleration_schur_dim = reduced_dim-pose;
        std::vector<double> acceleration_matrix(
            static_cast<std::size_t>(pose*pose), 0.0
        );
        for (int row = 0; row < pose; ++row) {
            for (int column = 0; column < pose; ++column) {
                acceleration_matrix[static_cast<std::size_t>(
                    row*pose+column
                )] = reduced_matrix[static_cast<std::size_t>(
                    row*reduced_dim+column
                )];
            }
        }
        if (acceleration_factorization.factor(acceleration_matrix, pose)) {
            acceleration_to_rest.assign(
                static_cast<std::size_t>(pose*acceleration_schur_dim), 0.0
            );
            rest_from_acceleration.assign(
                static_cast<std::size_t>(acceleration_schur_dim*pose), 0.0
            );
            rest_from_acceleration_terms_by_row.assign(
                static_cast<std::size_t>(acceleration_schur_dim), {}
            );
            acceleration_to_rest_terms_by_row.assign(
                static_cast<std::size_t>(pose), {}
            );
            for (int row = 0; row < acceleration_schur_dim; ++row) {
                for (int column = 0; column < pose; ++column) {
                    const double value = reduced_matrix[static_cast<std::size_t>(
                        (pose+row)*reduced_dim+column
                    )];
                    rest_from_acceleration[static_cast<std::size_t>(
                        row*pose+column
                    )] = value;
                    if (value != 0.0) {
                        rest_from_acceleration_terms_by_row[
                            static_cast<std::size_t>(row)
                        ].emplace_back(column, value);
                    }
                }
            }
            std::vector<double> right_hand_side(
                static_cast<std::size_t>(pose), 0.0
            );
            std::vector<double> column_solution;
            for (int column = 0; column < acceleration_schur_dim; ++column) {
                for (int row = 0; row < pose; ++row) {
                    right_hand_side[static_cast<std::size_t>(row)] =
                        reduced_matrix[static_cast<std::size_t>(
                            row*reduced_dim+pose+column
                        )];
                }
                acceleration_factorization.solve(
                    right_hand_side, column_solution
                );
                if (static_cast<int>(column_solution.size()) != pose ||
                    !finite_vec(column_solution)) {
                    acceleration_to_rest.clear();
                    rest_from_acceleration.clear();
                    acceleration_to_rest_terms_by_row.clear();
                    rest_from_acceleration_terms_by_row.clear();
                    break;
                }
                for (int row = 0; row < pose; ++row) {
                    const double value = column_solution[
                        static_cast<std::size_t>(row)
                    ];
                    acceleration_to_rest[static_cast<std::size_t>(
                        row*acceleration_schur_dim+column
                    )] = value;
                    if (value != 0.0) {
                        acceleration_to_rest_terms_by_row[
                            static_cast<std::size_t>(row)
                        ].emplace_back(column, value);
                    }
                }
            }
            if (static_cast<int>(acceleration_to_rest.size()) ==
                    pose*acceleration_schur_dim) {
                std::vector<double> schur_matrix(
                    static_cast<std::size_t>(
                        acceleration_schur_dim*acceleration_schur_dim
                    ), 0.0
                );
                for (int row = 0; row < acceleration_schur_dim; ++row) {
                    for (int column = 0; column < acceleration_schur_dim;
                         ++column) {
                        double value = reduced_matrix[static_cast<std::size_t>(
                            (pose+row)*reduced_dim+pose+column
                        )];
                        for (const auto& term : rest_from_acceleration_terms_by_row[
                                 static_cast<std::size_t>(row)
                             ]) {
                            value -= term.second * acceleration_to_rest[
                                static_cast<std::size_t>(
                                    term.first*acceleration_schur_dim+column
                                )
                            ];
                        }
                        schur_matrix[static_cast<std::size_t>(
                            row*acceleration_schur_dim+column
                        )] = value;
                    }
                }
                if (reduced_factorization.factor(
                        std::move(schur_matrix), acceleration_schur_dim
                    )) {
                    use_acceleration_schur = true;
                    return true;
                }
            }
        }
        acceleration_to_rest.clear();
        rest_from_acceleration.clear();
        acceleration_to_rest_terms_by_row.clear();
        rest_from_acceleration_terms_by_row.clear();
        sparse_fallback_ready = false;
        sparse_fallback_matrix.clear();
        return reduced_factorization.factor(
            std::move(reduced_matrix), reduced_dim
        );
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (static_cast<int>(right_hand_side.size()) != total_dim) {
            return false;
        }
        solve_pose_rhs.assign(
            right_hand_side.begin(), right_hand_side.begin()+pose_dim
        );
        pose_factorization.solve(solve_pose_rhs, solve_q0);
        if (static_cast<int>(solve_q0.size()) != pose_dim ||
            !finite_vec(solve_q0)) {
            return false;
        }
        solve_v0.assign(
            right_hand_side.begin()+pose_dim,
            right_hand_side.begin()+2*pose_dim
        );
        solve_reduced_rhs.assign(
            static_cast<std::size_t>(reduced_dim), 0.0
        );
        // Each reduced right-hand-side row is independent after the two
        // triangular solves. Parallelize this O(reduced_dim*pose_dim) part;
        // the factorization and triangular dependencies remain unchanged.
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(linear_solver_threads()) \
    if(reduced_dim*pose_dim >= 100000)
#endif
        for (int row = 0; row < reduced_dim; ++row) {
            const int full_row = remaining_full_row(
                row, pose_dim, constraint_dim, brush_dim
            );
            double value = right_hand_side[static_cast<std::size_t>(full_row)];
            for (const auto& term : remaining_q_terms_by_row[
                     static_cast<std::size_t>(row)
                 ]) {
                value -= term.second * solve_q0[
                    static_cast<std::size_t>(term.first)
                ];
            }
            for (const auto& term : remaining_v_terms_by_row[
                     static_cast<std::size_t>(row)
                 ]) {
                value -= term.second * solve_v0[
                    static_cast<std::size_t>(term.first)
                ];
            }
            solve_reduced_rhs[static_cast<std::size_t>(row)] = value;
        }
        if (use_mkl_reduced) {
#ifdef _WIN32
            const bool solved = use_mkl_pardiso_reduced
                ? mkl_pardiso_reduced_factorization.solve(
                    solve_reduced_rhs, solve_reduced_solution
                )
                : mkl_reduced_factorization.solve(
                    solve_reduced_rhs, solve_reduced_solution
                );
            if (!solved ||
                static_cast<int>(solve_reduced_solution.size()) != reduced_dim ||
                !finite_vec(solve_reduced_solution)) {
                if (!sparse_fallback_ready ||
                    !reduced_factorization.factor(
                        sparse_fallback_matrix, reduced_dim
                    )) {
                    return false;
                }
                use_mkl_reduced = false;
                use_mkl_pardiso_reduced = false;
                sparse_fallback_ready = false;
                sparse_fallback_matrix.clear();
                reduced_factorization.solve(
                    solve_reduced_rhs, solve_reduced_solution
                );
                if (static_cast<int>(solve_reduced_solution.size()) !=
                        reduced_dim || !finite_vec(solve_reduced_solution)) {
                    return false;
                }
            }
#else
            return false;
#endif
        } else if (use_sparse_lu_reduced) {
            const bool sparse_solved = sparse_lu_reduced_factorization.solve(
                solve_reduced_rhs, solve_reduced_solution
            );
            if (!sparse_solved ||
                static_cast<int>(solve_reduced_solution.size()) != reduced_dim ||
                !finite_vec(solve_reduced_solution)) {
                if (!sparse_fallback_ready ||
                    !reduced_factorization.factor(
                        sparse_fallback_matrix, reduced_dim
                    )) {
                    return false;
                }
                use_sparse_lu_reduced = false;
                sparse_fallback_ready = false;
                sparse_fallback_matrix.clear();
                reduced_factorization.solve(
                    solve_reduced_rhs, solve_reduced_solution
                );
                if (static_cast<int>(solve_reduced_solution.size()) !=
                        reduced_dim || !finite_vec(solve_reduced_solution)) {
                    return false;
                }
            }
        } else if (use_sparse_reduced) {
            const bool sparse_solved = sparse_reduced_factorization.solve(
                solve_reduced_rhs, solve_reduced_solution
            );
            if (!sparse_solved ||
                static_cast<int>(solve_reduced_solution.size()) != reduced_dim ||
                !finite_vec(solve_reduced_solution)) {
                if (!sparse_fallback_ready ||
                    !reduced_factorization.factor(
                        sparse_fallback_matrix, reduced_dim
                    )) {
                    return false;
                }
                use_sparse_reduced = false;
                sparse_fallback_ready = false;
                sparse_fallback_matrix.clear();
                reduced_factorization.solve(
                    solve_reduced_rhs, solve_reduced_solution
                );
                if (static_cast<int>(solve_reduced_solution.size()) !=
                        reduced_dim || !finite_vec(solve_reduced_solution)) {
                    return false;
                }
            }
        } else if (use_acceleration_schur) {
            solve_acceleration_rhs.assign(
                solve_reduced_rhs.begin(), solve_reduced_rhs.begin()+pose_dim
            );
            acceleration_factorization.solve(
                solve_acceleration_rhs, solve_acceleration_base
            );
            if (static_cast<int>(solve_acceleration_base.size()) != pose_dim ||
                !finite_vec(solve_acceleration_base)) {
                return false;
            }
            solve_acceleration_schur_rhs.assign(
                static_cast<std::size_t>(acceleration_schur_dim), 0.0
            );
            for (int row = 0; row < acceleration_schur_dim; ++row) {
                double value = solve_reduced_rhs[static_cast<std::size_t>(
                    pose_dim+row
                )];
                for (const auto& term : rest_from_acceleration_terms_by_row[
                         static_cast<std::size_t>(row)
                     ]) {
                    value -= term.second * solve_acceleration_base[
                        static_cast<std::size_t>(term.first)
                    ];
                }
                solve_acceleration_schur_rhs[static_cast<std::size_t>(row)] =
                    value;
            }
            reduced_factorization.solve(
                solve_acceleration_schur_rhs,
                solve_acceleration_schur_solution
            );
            if (static_cast<int>(solve_acceleration_schur_solution.size()) !=
                    acceleration_schur_dim ||
                !finite_vec(solve_acceleration_schur_solution)) {
                return false;
            }
            solve_reduced_solution.assign(
                static_cast<std::size_t>(reduced_dim), 0.0
            );
            for (int column = 0; column < acceleration_schur_dim; ++column) {
                solve_reduced_solution[static_cast<std::size_t>(
                    pose_dim+column
                )] = solve_acceleration_schur_solution[
                    static_cast<std::size_t>(column)
                ];
            }
            for (int row = 0; row < pose_dim; ++row) {
                double value = solve_acceleration_base[
                    static_cast<std::size_t>(row)
                ];
                for (const auto& term : acceleration_to_rest_terms_by_row[
                         static_cast<std::size_t>(row)
                     ]) {
                    value -= term.second * solve_acceleration_schur_solution[
                        static_cast<std::size_t>(term.first)
                    ];
                }
                solve_reduced_solution[static_cast<std::size_t>(row)] = value;
            }
        } else {
            reduced_factorization.solve(
                solve_reduced_rhs, solve_reduced_solution
            );
            if (static_cast<int>(solve_reduced_solution.size()) != reduced_dim ||
                !finite_vec(solve_reduced_solution)) {
                return false;
            }
        }
        solution.resize(static_cast<std::size_t>(total_dim));
        std::fill(solution.begin(), solution.end(), 0.0);
        // The q reconstruction has no cross-row dependency once the reduced
        // solution is available.
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(linear_solver_threads()) \
    if(pose_dim*pose_dim >= 100000)
#endif
        for (int row = 0; row < pose_dim; ++row) {
            double value = solve_q0[static_cast<std::size_t>(row)];
            for (const auto& term : q_reconstruction_terms_by_row[
                     static_cast<std::size_t>(row)
                 ]) {
                value += term.second * solve_reduced_solution[
                    static_cast<std::size_t>(term.first)
                ];
            }
            solution[static_cast<std::size_t>(row)] = value;
        }
        for (int row = 0; row < pose_dim; ++row) {
            double value = solve_v0[static_cast<std::size_t>(row)];
            value += v_from_a[static_cast<std::size_t>(row)]
                * solve_reduced_solution[static_cast<std::size_t>(row)];
            solution[static_cast<std::size_t>(pose_dim+row)] = value;
        }
        for (int column = 0; column < reduced_dim; ++column) {
            solution[static_cast<std::size_t>(reduced_full_column(
                column, pose_dim, constraint_dim, brush_dim
            ))] = solve_reduced_solution[static_cast<std::size_t>(column)];
        }
        return finite_vec(solution);
    }
};

struct NewtonSystemFactorization {
    ReducedNewtonFactorization reduced;
    LuFactorization full;
#ifdef _WIN32
    MklPardisoFactorization mkl_pardiso_full;
#endif
    bool use_reduced{false};
    bool use_mkl_pardiso_full{false};

    bool factor(
        std::vector<double> matrix, int dimension,
        int pose, int constraints, int brush
    ) {
        use_mkl_pardiso_full = false;
#ifdef _WIN32
        // 对完整 KKT 矩阵直接使用稀疏分解，避免先构造约化稠密矩阵。
        // 该路径仅在显式打开时使用，便于与当前约化后端逐项验收。
        if (mkl_pardiso_full_enabled() &&
            mkl_pardiso_full.factor(matrix, dimension)) {
            use_reduced = false;
            use_mkl_pardiso_full = true;
            return true;
        }
#endif
        const char* disabled = std::getenv(
            "SUSPENSION_AXLE_DISABLE_REDUCED_KKT"
        );
        if (disabled == nullptr || disabled[0] == '\0' || disabled[0] == '0') {
            if (reduced.factor(matrix, dimension, pose, constraints, brush)) {
                use_reduced = true;
                return true;
            }
        }
        use_reduced = false;
        return full.factor(std::move(matrix), dimension);
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
#ifdef _WIN32
        if (use_mkl_pardiso_full) {
            if (!mkl_pardiso_full.solve(right_hand_side, solution)) {
                return false;
            }
            return finite_vec(solution);
        }
#endif
        if (use_reduced) {
            reduced.solve(right_hand_side, solution);
        } else {
            full.solve(right_hand_side, solution);
        }
        if (!finite_vec(solution)) return false;
        return finite_vec(solution);
    }
};

struct NewtonLinearizationCache {
    NewtonSystemFactorization factorization;
    int dim{0};
    double h{0.0};
    double alpha_m{0.0};
    double alpha_f{0.0};
    double beta{0.0};
    double gamma{0.0};
    double alpha_m_z{0.0};
    double alpha_f_z{0.0};
    double gamma_z{0.0};
    int reuse_steps{0};
    bool valid{false};

    bool matches(const ResidualContext& context, int dimension) const;

    void update_key(const ResidualContext& context, int dimension);

    void invalidate() {
        valid = false;
        reuse_steps = 0;
    }
};

} // namespace axle_kernel
