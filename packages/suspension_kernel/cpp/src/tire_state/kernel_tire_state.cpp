// K5 (epic MODULES.md section 2.3): the tire-state layer.
//
// One tire contributes a fixed-width run of slots to the unknown vector, and every
// part of the kernel that reads, writes or sizes that run goes through this file:
// the per-tire width (`tire_block_width`), the total width (`tire_state_width`),
// the per-tire storage (`resize_tire_states`), the slot accessor
// (`tire_state_value`) and the inverse mapping that unpacks the unknown vector
// (`read_tire_states`).
//
// The layer exists so the width and the slot meanings have one owner.  Before it,
// the total width was defined in the PAC2002 law translation unit while the storage
// and the accessors lived with the model, and the integrator compared tire model
// kinds to decide what to do with a slot.  Now the width is a property cached on
// `Tire` at registration (`state_slot_width`, D10), so nothing here calls back into
// the tire models; the integrator reads `Tire`'s state-semantics flags and the slot
// descriptor table (D14); and the slot layout is stated once, in
// `kTireSlotDescriptors` in `mb_tire_state/tire_state.hpp`.
//
// ### `ResidualWorkspace` slot alignment
//
// The workspace keeps five tire-state vectors.  Three of them -- and the two
// internal-evaluation ones -- are strided by the block width, which is what makes
// them slot-aligned; one is per tire and one is per tire and output column.  The
// list, with the index expression each one uses:
//
//   | member                             | length          | index for tire i, slot s |
//   |------------------------------------|-----------------|--------------------------|
//   | `tire_state_derivatives`           | `nz`            | `stride*i + s`           |
//   | `internal_tire_derivatives`        | `nz`            | `stride*i + s`           |
//   | `internal_tire_relaxation_rates`   | `nz`            | `stride*i + s`           |
//   | `internal_tire_relaxation_targets` | `nz`            | `stride*i + s`           |
//   | `tire_forces`                      | `tire_count`    | `i`                      |
//   | `internal_tire_forces`             | `tire_count`    | `i`                      |
//   | `tire_output`                      | `tire_count*kTireOutputWidth` | `i*kTireOutputWidth` |
//   | `internal_tire_output`             | `tire_count*kTireOutputWidth` | `i*kTireOutputWidth` |
//
// with `nz = tire_state_width(model) = stride*tire_count` and
// `stride = tire_block_width(model)`.  The `State` members a slot maps to are the
// `value`/`rate` pairs in `kTireSlotDescriptors`; the packed turn-slip group
// (slots 8..11) is the one place where the slot index and the vector index differ,
// which `tire_slot_index` performs.

#include "mb_tire_state/functions.hpp"

namespace axle_kernel {

int tire_block_width(const Model& model) {
    // The base width of 2 is the linear transient `sx`/`sy` pair and must be
    // kept: a model whose tires are all non-PAC2002 has no cached width above the
    // default, and starting from 0 would silently drop those two slots.
    int width = 2;
    for (const Tire& tire : model.tires) {
        width = std::max(width, tire.state_slot_width);
    }
    return width;
}

void resize_tire_states(const Model& model, State& state) {
    const std::size_t count = model.tires.size();
    state.tire_sx.resize(count, 0.0);
    state.tire_sy.resize(count, 0.0);
    // The Maxwell element is orthogonal to the USE_MODE state block, so it keeps its
    // own per-tire vector instead of widening every mode's layout.  It must be
    // allocated *here*, ahead of the mode-width early returns below: those returns
    // skip the tail of this function, so a mode with no contact-body slots (14, 23,
    // 24 all have a state width of 2) never got storage at all.  The visible symptom
    // was a Maxwell force permanently evaluated at z_m = 0, i.e. K_dyn*delta*decay
    // (~850 N) added to every vertical force: measured as a 20 Hz, +-800 N ride-mode
    // oscillation that dropped Fz from 3136.5 N to 2011 N inside 30 ms, while the same
    // tire with the element disabled held 3135.7 N and the Adams reference held 3133.3 N.
    bool maxwell = false;
    for (const Tire& tire : model.tires) {
        maxwell = maxwell || tire.maxwell_enabled;
    }
    if (maxwell) {
        state.tire_maxwell.resize(count, 0.0);
    } else {
        state.tire_maxwell.clear();
    }
    // The contact-body vectors are allocated for every mode that carries them
    // and cleared otherwise, so a read of an unallocated slot yields zero.
    const int slot_width = tire_block_width(model);
    if (slot_width < 4) {
        state.tire_u.clear();
        state.tire_u_dot.clear();
        state.tire_u_ddot.clear();
        state.tire_v.clear();
        state.tire_v_dot.clear();
        state.tire_v_ddot.clear();
        state.tire_beta.clear();
        state.tire_beta_dot.clear();
        state.tire_beta_ddot.clear();
        return;
    }
    state.tire_u.resize(count, 0.0);
    state.tire_u_dot.resize(count, 0.0);
    state.tire_u_ddot.resize(count, 0.0);
    if (slot_width < 6) {
        state.tire_v.clear();
        state.tire_v_dot.clear();
        state.tire_v_ddot.clear();
        state.tire_beta.clear();
        state.tire_beta_dot.clear();
        state.tire_beta_ddot.clear();
        return;
    }
    state.tire_v.resize(count, 0.0);
    state.tire_v_dot.resize(count, 0.0);
    state.tire_v_ddot.resize(count, 0.0);
    if (slot_width < 8) {
        state.tire_beta.clear();
        state.tire_beta_dot.clear();
        state.tire_beta_ddot.clear();
        return;
    }
    state.tire_beta.resize(count, 0.0);
    state.tire_beta_dot.resize(count, 0.0);
    state.tire_beta_ddot.resize(count, 0.0);
    if (slot_width < 12) {
        state.tire_phi.clear();
        state.tire_phi_dot.clear();
        return;
    }
    // Four packed turn-slip slots per tire (see the State declaration).  This has to
    // be a *resize*, not an assign: ``resize`` leaves a correctly sized vector alone,
    // while ``assign`` rewrites every element with zero.  It is called from
    // state_from_unknown() and from apply_brush_return_mapping() -- i.e. on the
    // accepted state after every step -- so an assign here wiped the four turn-slip
    // states once per step and the whole USE_MODE-25 turn-slip family was inert:
    // measured on the parking maneuver, the filter rates were ~5e2 1/s (the states
    // were being driven hard towards psi_dot/|Vx|) while the states read exactly zero
    // in every sample and the exposed output columns were all zero.
    state.tire_phi.resize(count*4, 0.0);
    state.tire_phi_dot.resize(count*4, 0.0);
}

double tire_state_value(const State& state, std::size_t tire, int slot) {
    // One mapping, read from the descriptor table: the residual's generalized-alpha
    // z update asks for the same slot's previous value and derivative, and a second
    // hand-written switch here is how the two drift apart.
    const TireSlotDescriptor* descriptor = tire_slot_descriptor(slot);
    return slot_value(
        state.*(descriptor->value), tire_slot_index(*descriptor, tire, slot)
    );
}

void read_tire_states(
    const std::vector<double>& x, int base, int index, int per_tire, State& state
) {
    const std::size_t i = static_cast<std::size_t>(index);
    const int offset = base + per_tire * index;
    if (i < state.tire_sx.size()) state.tire_sx[i] = x[offset];
    if (i < state.tire_sy.size()) state.tire_sy[i] = x[offset + 1];
    if (per_tire < 4) return;
    if (i < state.tire_u.size()) state.tire_u[i] = x[offset + 2];
    if (i < state.tire_u_dot.size()) state.tire_u_dot[i] = x[offset + 3];
    if (per_tire < 6) return;
    if (i < state.tire_v.size()) state.tire_v[i] = x[offset + 4];
    if (i < state.tire_v_dot.size()) state.tire_v_dot[i] = x[offset + 5];
    if (per_tire < 8) return;
    if (i < state.tire_beta.size()) state.tire_beta[i] = x[offset + 6];
    if (i < state.tire_beta_dot.size()) state.tire_beta_dot[i] = x[offset + 7];
    if (per_tire < 12) return;
    for (int k = 0; k < 4; ++k) {
        const std::size_t packed = i*4 + static_cast<std::size_t>(k);
        if (packed < state.tire_phi.size()) {
            state.tire_phi[packed] = x[offset + 8 + k];
        }
    }
}

int tire_state_width(const Model& model) {
    // Delegate so the total can never disagree with the per-tire block width the
    // residual and the Jacobian stride by.
    return tire_block_width(model) * static_cast<int>(model.tires.size());
}

} // namespace axle_kernel
