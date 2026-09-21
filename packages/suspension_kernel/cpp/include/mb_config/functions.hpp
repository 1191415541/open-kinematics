#pragma once

/// The free functions of the `mb_config` module.
///
/// Split out of the former `mb_base/functions.hpp` aggregate at subtask 03.
/// The declarations are grouped by the module that defines them.

#include "mb_config/prelude.hpp"
#include "mb_config/env.hpp"
#include "mb_config/diagnostics.hpp"

namespace axle_kernel {

bool profiling_enabled();

bool runtime_jacobian_validation_enabled();

bool static_debug_enabled();

int linearization_reuse_limit(int default_limit);

int newton_jacobian_refresh_period();

int linear_solver_threads();

bool blocked_lu_enabled();

bool blocked_lu_parallel_enabled();

bool lu_equilibration_enabled();

bool sparse_gmres_enabled();

int sparse_gmres_restart();

int sparse_gmres_max_iterations();

bool sparse_lu_enabled();

bool mkl_dense_enabled();

bool mkl_pardiso_enabled();

bool mkl_pardiso_full_enabled();

bool mkl_pardiso_debug_enabled();

bool mkl_pardiso_matching_enabled();

int mkl_pardiso_pivot_perturbation();

int mkl_pardiso_ordering();

int mkl_pardiso_threads();

int blocked_lu_block_size();

bool exact_fiala_relaxation_enabled();

bool light_fiala_relaxation_enabled();

bool acceleration_predictor_enabled(bool default_enabled = false);

bool acceleration_schur_probe_enabled();

} // namespace axle_kernel
