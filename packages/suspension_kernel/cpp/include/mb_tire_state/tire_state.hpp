#pragma once
#include "mb_model/types.hpp"

/// The tire-state slot layout (MB_TIRE_STATE).
///
/// One tire's state block is a fixed-width run of slots, and this is the
/// table that says what each slot means and where its value and its own
/// derivative live.  It is the epic's `ResidualWorkspace` slot alignment
/// list: the residual and the analytic Jacobian stride by slot, so the
/// mapping has to exist once instead of in a switch per consumer.

#include "mb_model/types.hpp"
#include <cstddef>
#include <vector>

namespace axle_kernel {

/// One slot of a tire's state block, and where its two values live.
///
/// This is the written-down form of the layout the residual, the analytic Jacobian
/// and the step driver stride by, and it is the epic's "`ResidualWorkspace` slot
/// alignment list".  Every slot means the same thing in every tire model -- that is
/// what lets the integrator iterate slots instead of dispatching on the model kind
/// -- and this table is where that claim is stated once.
///
/// `minimum_width` is the smallest block width that carries the slot, which is
/// exactly the ladder `resize_tire_states` allocates by: 2 for the linear transient
/// pair, 4 and 6 for the in-plane contact-body value/rate pairs, 8 for the
/// contact-body yaw pair, and 12 for the packed turn-slip states.  No current
/// `USE_MODE` produces a width of 4 or 6, but the ladder supports them and the
/// table agrees with the ladder rather than with today's modes.
struct TireSlotDescriptor {
    /// Index inside the per-tire block; also the bit index of `tire_slot_mask`.
    int slot;
    /// Short name, for tests and diagnostics.
    const char* name;
    /// Smallest block width that carries this slot.
    int minimum_width;
    /// How many consecutive slots the state storage packs into one vector.
    int packing;
    /// First slot of the packed group; ignored when `packing` is 1.
    int packed_first_slot;
    /// The `State` member holding the slot's value.
    std::vector<double> State::* value;
    /// The `State` member holding the slot's own time derivative.
    std::vector<double> State::* rate;
};

/// The tire state block's slots, in slot order.
///
/// The value/rate pairs mirror the residual's generalized-alpha z update: a slot's
/// own derivative is the one the update needs, which is why slot 3 is the rate of
/// slot 2 and slot 6's derivative is `tire_beta_dot` rather than the yaw
/// acceleration that belongs to slot 7.
inline constexpr TireSlotDescriptor kTireSlotDescriptors[] = {
    {0, "sx", 2, 1, 0, &State::tire_sx, &State::tire_sx_dot},
    {1, "sy", 2, 1, 0, &State::tire_sy, &State::tire_sy_dot},
    {2, "u", 4, 1, 0, &State::tire_u, &State::tire_u_dot},
    {3, "u_dot", 4, 1, 0, &State::tire_u_dot, &State::tire_u_ddot},
    {4, "v", 6, 1, 0, &State::tire_v, &State::tire_v_dot},
    {5, "v_dot", 6, 1, 0, &State::tire_v_dot, &State::tire_v_ddot},
    {6, "beta", 8, 1, 0, &State::tire_beta, &State::tire_beta_dot},
    {7, "beta_dot", 8, 1, 0, &State::tire_beta_dot, &State::tire_beta_ddot},
    {8, "phi_c", 12, 4, 8, &State::tire_phi, &State::tire_phi_dot},
    {9, "phi_f2", 12, 4, 8, &State::tire_phi, &State::tire_phi_dot},
    {10, "phi_1", 12, 4, 8, &State::tire_phi, &State::tire_phi_dot},
    {11, "phi_2", 12, 4, 8, &State::tire_phi, &State::tire_phi_dot},
};
inline constexpr int kTireSlotCount = static_cast<int>(
    sizeof(kTireSlotDescriptors)/sizeof(kTireSlotDescriptors[0])
);

/// The descriptor of one slot.
///
/// A slot past the end clamps to the last descriptor rather than running off the
/// table; callers that index a packed group with such a slot still read zero,
/// because the packing formula below still uses the requested slot.
inline const TireSlotDescriptor* tire_slot_descriptor(int slot) {
    return &kTireSlotDescriptors[
        slot >= 0 && slot < kTireSlotCount ? slot : kTireSlotCount - 1
    ];
}

/// The index inside a packed state vector that `slot` of `tire` occupies.
inline std::size_t tire_slot_index(
    const TireSlotDescriptor& descriptor, std::size_t tire, int slot
) {
    if (descriptor.packing <= 1) return tire;
    return tire*static_cast<std::size_t>(descriptor.packing)
        + static_cast<std::size_t>(slot - descriptor.packed_first_slot);
}

/// Bit `s` is set when a state block of `width` slots carries slot `s`.
///
/// This reproduces `resize_tire_states`' allocation ladder: a slot whose
/// `minimum_width` exceeds the block width is absent.
inline int tire_slot_mask(int width) {
    int mask = 0;
    for (const TireSlotDescriptor& descriptor : kTireSlotDescriptors) {
        if (descriptor.minimum_width <= width) mask |= 1 << descriptor.slot;
    }
    return mask;
}

int tire_state_width(const Model& model);
void resize_tire_states(const Model& model, State& state);
double tire_state_value(const State& state, std::size_t tire, int slot);

} // namespace axle_kernel
