// The solver's runtime switches (env variables) and their defaults, split from
// `kernel_base.cpp` at subtask 03 into `mb_config`.

#include "mb_config/functions.hpp"

namespace axle_kernel {
bool profiling_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_PROFILE");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


bool runtime_jacobian_validation_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_VALIDATE_JACOBIAN");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


bool static_debug_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_DEBUG_STATIC");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


int linearization_reuse_limit(int default_limit) {

    const char* value = std::getenv(

        "SUSPENSION_AXLE_LINEARIZATION_REUSE_LIMIT"

    );

    if (value == nullptr || value[0] == '\0') return default_limit;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return default_limit;

    return static_cast<int>(std::max(1L, std::min(256L, parsed)));

}


int newton_jacobian_refresh_period() {

    static const int period = [] {

        const char* value = std::getenv(

            "SUSPENSION_AXLE_NEWTON_REFRESH_PERIOD"

        );

        if (value == nullptr || value[0] == '\0') return 64;

        char* end = nullptr;

        const long parsed = std::strtol(value, &end, 10);

        if (end == value || *end != '\0') return 64;

        return static_cast<int>(std::max(1L, std::min(64L, parsed)));

    }();

    return period;

}


int linear_solver_threads() {

#ifdef _OPENMP

    const int available = std::max(1, std::min(4, omp_get_max_threads()));

    const char* value = std::getenv("SUSPENSION_AXLE_LINEAR_THREADS");

    if (value == nullptr || value[0] == '\0') return available;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return available;

    return std::max(1, std::min(available, static_cast<int>(parsed)));

#else

    return 1;

#endif

}


bool blocked_lu_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_BLOCKED_LU");

    return value == nullptr || value[0] == '\0' || value[0] != '0';

}


bool blocked_lu_parallel_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_BLOCKED_LU_PARALLEL");

    return value == nullptr || value[0] == '\0' || value[0] != '0';

}


bool lu_equilibration_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_LU_EQUILIBRATE");

    return value == nullptr || value[0] == '\0' || value[0] != '0';

}


bool sparse_gmres_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_SPARSE_GMRES");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


int sparse_gmres_restart() {

    const char* value = std::getenv("SUSPENSION_AXLE_GMRES_RESTART");

    if (value == nullptr || value[0] == '\0') return 48;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return 48;

    return static_cast<int>(std::max(12L, std::min(96L, parsed)));

}


int sparse_gmres_max_iterations() {

    const char* value = std::getenv("SUSPENSION_AXLE_GMRES_MAX_ITERATIONS");

    if (value == nullptr || value[0] == '\0') return 240;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return 240;

    return static_cast<int>(std::max(48L, std::min(800L, parsed)));

}


bool sparse_lu_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_SPARSE_LU");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


bool mkl_dense_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_MKL_DLL");

    return value != nullptr && value[0] != '\0';

}


bool mkl_pardiso_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_PARDISO");

    return value != nullptr && value[0] != '\0' && value[0] != '0' &&

        mkl_dense_enabled();

}


bool mkl_pardiso_full_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_PARDISO_FULL");

    return value != nullptr && value[0] != '\0' && value[0] != '0' &&

        mkl_pardiso_enabled();

}


bool mkl_pardiso_debug_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_DEBUG_PARDISO");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


bool mkl_pardiso_matching_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_PARDISO_MATCHING");

    return value == nullptr || value[0] == '\0' || value[0] != '0';

}


int mkl_pardiso_pivot_perturbation() {

    const char* value = std::getenv(

        "SUSPENSION_AXLE_PARDISO_PIVOT_PERTURBATION"

    );

    if (value == nullptr || value[0] == '\0') return 13;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return 13;

    return static_cast<int>(std::max(1L, std::min(13L, parsed)));

}


int mkl_pardiso_ordering() {

    const char* value = std::getenv("SUSPENSION_AXLE_PARDISO_ORDERING");

    if (value == nullptr || value[0] == '\0') return 2;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return 2;

    return static_cast<int>(std::max(0L, std::min(3L, parsed)));

}


int mkl_pardiso_threads() {

#ifdef _OPENMP

    const int available = std::max(1, std::min(16, omp_get_max_threads()));

#else

    const int available = 1;

#endif

    const char* value = std::getenv("SUSPENSION_AXLE_PARDISO_THREADS");

    if (value == nullptr || value[0] == '\0') {

        return std::min(4, available);

    }

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return std::min(4, available);

    return static_cast<int>(std::max(1L, std::min(

        static_cast<long>(available), parsed

    )));

}


int blocked_lu_block_size() {

    const char* value = std::getenv("SUSPENSION_AXLE_LU_BLOCK_SIZE");

    if (value == nullptr || value[0] == '\0') return 36;

    char* end = nullptr;

    const long parsed = std::strtol(value, &end, 10);

    if (end == value || *end != '\0') return 36;

    return static_cast<int>(std::max(8L, std::min(128L, parsed)));

}


bool exact_fiala_relaxation_enabled() {

    const char* value = std::getenv(

        "SUSPENSION_AXLE_EXACT_FIALA_RELAXATION"

    );

    return value == nullptr || value[0] == '\0' || value[0] != '0';

}


bool light_fiala_relaxation_enabled() {

    const char* value = std::getenv(

        "SUSPENSION_AXLE_LIGHT_FIALA_RELAXATION"

    );

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


bool acceleration_predictor_enabled(bool default_enabled) {

    const char* value = std::getenv("SUSPENSION_AXLE_ACCELERATION_PREDICTOR");

    if (value == nullptr || value[0] == '\0') return default_enabled;

    return value[0] != '0';

}


bool acceleration_schur_probe_enabled() {

    const char* value = std::getenv("SUSPENSION_AXLE_ACCELERATION_SCHUR_PROBE");

    return value != nullptr && value[0] != '\0' && value[0] != '0';

}


} // namespace axle_kernel
