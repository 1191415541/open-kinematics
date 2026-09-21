#pragma once

/// The solver's environment switches (MB_BASE).
///
/// Every solver switch is read through one of these functions so there is a
/// single place a default lives.  They are declared here, ahead of the class
/// bodies that call them, because a class definition is parsed where it
/// appears and an inline member body cannot call a function declared later.


namespace axle_kernel {

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
bool acceleration_schur_probe_enabled();

/// The optional element-wrench channel (05 step 3).  Default off: only a caller
/// that sets `SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT` gets the extra block, so
/// the default path's result bytes do not move.
bool element_wrench_output_enabled();

} // namespace axle_kernel
