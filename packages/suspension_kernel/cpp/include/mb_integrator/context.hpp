#pragma once

/// The integrator's per-step context and workspace (MB_INTEGRATOR).
///
/// `ResidualContext` is what one residual evaluation is handed;
/// `ResidualWorkspace` is the storage it reuses across evaluations, and the
/// slot-aligned tire-state vectors in it are described by
/// `mb_tire_state/tire_state.hpp`.  `PoseJacobians` and `StepResult` are the
/// two remaining pieces of one step's state.
///
/// `ResidualContext` sits here rather than with `mb_linalg` because the
/// linearization cache's key comparison is the only thing that needs it, and
/// the cache is declared with a forward reference (epic section 8-D-H).

#include "mb_input/types.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_linalg/factorization_types.hpp"
#include "mb_model/types.hpp"
#include "mb_tire_state/tire_state.hpp"
#include <cstddef>
#include <vector>

namespace axle_kernel {

struct ResidualContext {
    const Model* model{};
    const AxleInput* input{};
    const SampleInput* previous_sample{};
    const SampleInput* evaluation_sample{};
    const SampleInput* next_sample{};
    const SampleInput* internal_evaluation_sample{};
    State previous{};
    double h{0.0};
    double alpha_m{0.0}, alpha_f{0.0}, beta{0.0}, gamma{0.0};
    double alpha_m_z{0.0}, alpha_f_z{0.0}, gamma_z{0.0};
    PerformanceCounters* performance{nullptr};
};

struct ResidualWorkspace {
    State next{};
    State evaluation{};
    State internal_evaluation{};
    std::vector<double> dy;
    std::vector<double> pose_increment;
    std::vector<double> a_next;
    std::vector<double> v_next;
    std::vector<double> mu;
    std::vector<double> lambda;
    std::vector<double> J_storage;
    std::vector<double> J_eval_storage;
    std::vector<double> pose_cache;
    bool pose_cache_valid{false};
    std::vector<double> tire_forces;
    std::vector<double> tire_state_derivatives;
    std::vector<double> tire_output;
    std::vector<double> generalized_force;
    std::vector<Vec3> body_force;
    std::vector<Vec3> body_torque;
    std::vector<double> q_old_v;
    std::vector<double> perturbed_x;
    std::vector<double> output;
    std::vector<double> dy_ga;
    std::vector<double> mass_mu;
    std::vector<double> analytic_unit;
    std::vector<double> analytic_scaled;
    std::vector<double> phi_storage;
    std::vector<double> internal_dy;
    std::vector<double> internal_tire_forces;
    std::vector<double> internal_tire_derivatives;
    std::vector<double> internal_tire_relaxation_rates;
    std::vector<double> internal_tire_relaxation_targets;
    std::vector<double> internal_tire_output;

    void ensure(const Model& model, int dimension) {
        const std::size_t body_count = model.bodies.size();
        const std::size_t tire_count = model.tires.size();
        const std::size_t n = static_cast<std::size_t>(model.ndof);
        const std::size_t m = static_cast<std::size_t>(model.rows);
        // The tire block is strided by its own width, not a fixed 2: the
        // contact-mass modes carry extra per-tire states.
        const std::size_t nz =
            static_cast<std::size_t>(tire_state_width(model));
        const auto ensure_state = [&model](State& state) {
            resize_tire_states(model, state);
            state.tire_sx_dot.resize(model.tires.size(), 0.0);
            state.tire_sy_dot.resize(model.tires.size(), 0.0);
        };
        ensure_state(next);
        ensure_state(evaluation);
        ensure_state(internal_evaluation);
        dy.resize(n);
        pose_increment.resize(n);
        if (pose_cache.size() != n) {
            pose_cache.resize(n);
            pose_cache_valid = false;
        }
        a_next.resize(n);
        v_next.resize(n);
        mu.resize(m);
        lambda.resize(m);
        J_storage.resize(m*n);
        J_eval_storage.resize(m*n);
        tire_forces.resize(tire_count);
        tire_state_derivatives.resize(nz);
        tire_output.resize(tire_count*kTireOutputWidth);
        generalized_force.resize(n);
        body_force.resize(body_count);
        body_torque.resize(body_count);
        q_old_v.resize(n);
        perturbed_x.resize(static_cast<std::size_t>(dimension));
        output.resize(static_cast<std::size_t>(dimension));
        dy_ga.resize(n);
        mass_mu.resize(n);
        analytic_unit.resize(n);
        analytic_scaled.resize(n);
        phi_storage.resize(m);
        internal_dy.resize(n);
        internal_tire_forces.resize(tire_count);
        internal_tire_derivatives.resize(nz);
        internal_tire_relaxation_rates.resize(nz);
        internal_tire_relaxation_targets.resize(nz);
        internal_tire_output.resize(tire_count*kTireOutputWidth);
    }
};

struct PoseJacobians {
    std::vector<double> at_next;
    std::vector<double> at_evaluation;
    std::vector<double> position_residual;
    bool valid{false};
};

struct StepResult {
    State state{};
    std::vector<double> constraint_multiplier;
    double position_residual{0.0};
    double velocity_residual{0.0};
    double dynamics_residual{0.0};
    int active_contacts{0};
    int iterations{0};
};

} // namespace axle_kernel
