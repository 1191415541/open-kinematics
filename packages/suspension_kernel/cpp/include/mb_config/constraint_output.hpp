#pragma once

#include <cstddef>

namespace axle_kernel {

// Contract-only observer buffers. No pointer enters the flat ABI or a trial state.
struct ConstraintOutputSink {
    double* multipliers{nullptr};
    double* reactions{nullptr};
    std::size_t row_count{0};
    std::size_t constraint_count{0};
    double* jacobian{nullptr};
    std::size_t dof_count{0};
    double* coupler_reactions{nullptr};
    std::size_t coupler_count{0};

    void clear() { *this = {}; }
};

inline ConstraintOutputSink& constraint_output_sink() {
    static thread_local ConstraintOutputSink sink;
    return sink;
}

// Per end: world force, world moment about body COM, joint point, power.
inline constexpr std::size_t kConstraintReactionWidth = 10;

} // namespace axle_kernel
