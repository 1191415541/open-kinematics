#pragma once

/// Timing and profiling counters (MB_BASE).
///
/// The counters are per-run values passed by pointer, never module state:
/// the layer contract forbids module-level mutable state.

#include <atomic>
#include <chrono>
#include <cstdint>

namespace axle_kernel {

using PerformanceClock = std::chrono::steady_clock;

struct PerformanceCounters {
    std::atomic<std::uint64_t> residual_calls{0};
    std::atomic<std::uint64_t> residual_nanoseconds{0};
    std::atomic<std::uint64_t> constraint_jacobian_calls{0};
    std::atomic<std::uint64_t> constraint_jacobian_nanoseconds{0};
    std::atomic<std::uint64_t> force_evaluations{0};
    std::atomic<std::uint64_t> force_nanoseconds{0};
    std::atomic<std::uint64_t> mass_inverse_calls{0};
    std::atomic<std::uint64_t> mass_inverse_nanoseconds{0};
    std::atomic<std::uint64_t> reaction_nanoseconds{0};
    std::atomic<std::uint64_t> linear_factorizations{0};
    std::atomic<std::uint64_t> linear_factorization_nanoseconds{0};
    std::atomic<std::uint64_t> linear_solves{0};
    std::atomic<std::uint64_t> linear_solve_nanoseconds{0};
    std::atomic<std::uint64_t> line_search_trials{0};
    std::atomic<std::uint64_t> accepted_steps{0};
    std::atomic<std::uint64_t> rejected_attempts{0};
    std::atomic<std::uint64_t> newton_iterations{0};
    std::atomic<std::uint64_t> analytic_jacobian_columns{0};
    std::atomic<std::uint64_t> finite_difference_jacobian_columns{0};
    std::atomic<std::uint64_t> nonsmooth_fallback_columns{0};
    std::atomic<std::uint64_t> analytic_jacobian_nanoseconds{0};
    std::atomic<std::uint64_t> finite_difference_jacobian_nanoseconds{0};
    std::atomic<std::uint64_t> dynamic_integration_nanoseconds{0};
};

struct ScopedPerformanceTimer {
    std::atomic<std::uint64_t>* destination{nullptr};
    PerformanceClock::time_point started{};

    explicit ScopedPerformanceTimer(
        std::atomic<std::uint64_t>* destination_
    ) : destination(destination_) {
        if (destination != nullptr) started = PerformanceClock::now();
    }

    ~ScopedPerformanceTimer() {
        if (destination == nullptr) return;
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            PerformanceClock::now() - started
        ).count();
        destination->fetch_add(
            static_cast<std::uint64_t>(elapsed), std::memory_order_relaxed
        );
    }
};

} // namespace axle_kernel
