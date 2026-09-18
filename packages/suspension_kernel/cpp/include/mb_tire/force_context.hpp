#pragma once

// The shared state of one force-assembly pass (K4).
//
// `assemble_tire_forces` is a single loop over the tires whose body reaches
// seventeen hundred lines, because the force blocks, the energy ledger and the
// observer outputs all read and write the same set of variables.  Splitting those
// blocks into named functions means passing that set, and passing forty-odd
// separate arguments is how a signature drifts from its body.
//
// This struct is that set, grouped by what a callee may do with each part:
//
// * `Input` is read-only.  A block may read it and may not keep a pointer to it
//   past the call.
// * the buffers are written.  They are references rather than pointers so a
//   callee cannot be handed null by accident; the tire loop has already decided
//   that each buffer exists.
// * the energy members are the accumulators, and three of them are references
//   for a reason that is easy to get wrong: `potential`, `external_power` and
//   `dissipation` are added to **several times per element**, and floating-point
//   addition is not associative.  Summing into a temporary and adding once
//   changes the result, so they must be carried by reference and accumulated in
//   place.  That is the single easiest way to break the kernel's bit-identical
//   guarantee while looking like a tidy refactor.
//
// K6 moved this header under `mb_tire` and gave it the concrete includes it
// needs: it used to sit in `cpp/axle_dynamics/` and reach every type through the
// transitional aggregate header.  Its users are the tire assembler, which owns
// it, and the vehicle layer's force assembly, which may use the tire layer.

#include <vector>

#include "mb_base/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_energy/types.hpp"

namespace axle_kernel {

/// Everything one force-assembly pass reads and writes.
struct ForceAssemblyContext {
    /// How much force a unit of input load produces.  Two scales rather than one
    /// because a static-trim pass and a dynamic pass differ in what they scale.
    struct Input {
        const Model& model;
        const State& state;
        const SampleInput& input;
        const StaticContactOverride* static_contact{nullptr};
        /// Per-tire stride into the brush-derivative and relaxation buffers.
        int stride{1};
        double external_load_scale{1.0};
        double internal_force_scale{1.0};
        /// Produce the public tire/component outputs.
        bool record_output{false};
        /// Update the energy ledger.
        bool record_energy{false};
        /// Produce only what the tire brush ODE needs; see `assemble_tire_forces`.
        bool brush_only{false};
        /// Newton residuals need forces and tire-state derivatives but no public
        /// observer output.
        bool dynamics_only{false};
    };

    /// The buffers a pass writes, by reference.
    struct Buffers {
        std::vector<Vec3>& force;
        std::vector<Vec3>& torque;
        std::vector<double>& tire_forces;
        std::vector<double>& tire_state_derivatives;
        std::vector<double>& tire_output;
        /// Optional local coefficients of the affine Fiala relaxation ODE.
        std::vector<double>* tire_relaxation_rates{nullptr};
        /// Optional per-tire vertical deflection (A5).
        std::vector<double>* tire_deflections{nullptr};
    };

    /// The energy ledger and the accumulators.
    ///
    /// `dissipation`, `potential` and `external_power` are the three that are
    /// added to repeatedly; see the header comment.  `energy_rates` and
    /// `energy_storage` may be null when the caller does not want the ledger.
    struct Energy {
        EnergyRates* energy_rates{nullptr};
        EnergyStorage* energy_storage{nullptr};
        double& dissipation;
        double& potential;
        double& external_power;
    };

    Input in;
    Buffers buffers;
    Energy energy;
};

} // namespace axle_kernel
