#pragma once

// The tire model registry (MB_TIRE), §5.5 of the epic's module plan.
//
// A tire model is a row in `kTireModelDescriptors`: the integrator's questions and
// the model's own identity, with the kind as the key.  The point is that adding a
// model is adding a row, not editing the integrator -- which is what the flags in
// this table are for.  `Tire::model_kind` is still the model's own business; what
// the integrator reads is the descriptor.
//
// This header needs only the model layer: `Tire` for the inline semantics
// helper, and the tire-model enumerations it reaches through `mb_model/types.hpp`.
// The model layer, the tire layer and the integrator all depend on that, so all
// three can include this header.

#include "mb_model/types.hpp"

namespace axle_kernel {

/// What the integrator and the contact geometry need to know about a tire model.
struct TireModelDescriptor {
    /// One of `VehicleTireModelKind`.
    int kind;
    /// Short name, for diagnostics and tests.
    const char* name;
    /// The generic BDF2 relaxation row is replaced by the exact exponential of the
    /// affine relaxation ODE (Fiala's relaxation is linear in the slip with
    /// coefficients frozen within a step).
    bool uses_exact_relaxation;
    /// The tire state is projected back onto the model's admissible set after an
    /// accepted step (the brush model's friction-ellipse return mapping).
    bool has_state_return_mapping;
    /// The force responds to the normal load and the slip within a step, so the
    /// Newton linearization may be reused across several steps.
    bool uses_pac2002_law;
    /// Contact kinematics are evaluated at the wheel/spindle centre rather than at
    /// the carrier centre.
    bool evaluates_at_wheel_center;
    /// The compression is the wheel-plane/road-plane intersection rather than the
    /// plain vertical gap.
    bool projects_compression_on_spin;
    /// The state block's width comes from the PAC2002 `USE_MODE` table rather than
    /// from the model's own base width.
    bool uses_pac2002_mode_table;
};

/// The registered tire models, in `VehicleTireModelKind` order.
///
/// A new model is a new row here plus its own force law: nothing in the integrator
/// or the contact geometry has to learn about it.
inline constexpr TireModelDescriptor kTireModelDescriptors[] = {
    {
        VEHICLE_TIRE_NATIVE_BRUSH, "brush",
        false, true, false, false, false, false,
    },
    {
        VEHICLE_TIRE_PAC2002_PURE_SLIP, "pac2002_pure_slip",
        false, false, true, false, false, true,
    },
    {
        VEHICLE_TIRE_PAC2002_ADAMS_SOURCE, "pac2002_adams_source",
        false, false, true, true, false, true,
    },
    {
        VEHICLE_TIRE_FIALA, "fiala",
        true, false, false, true, true, false,
    },
};

inline constexpr int kTireModelCount = static_cast<int>(
    sizeof(kTireModelDescriptors)/sizeof(kTireModelDescriptors[0])
);
/// The base width of every model's state block: the linear transient `sx`/`sy`
/// pair.  A model whose width comes from the `USE_MODE` table may be wider.
inline constexpr int kTireModelBaseStateSlotWidth = 2;

/// The descriptor for a model kind, or nullptr when the kind is not registered.
inline const TireModelDescriptor* tire_model_descriptor(int kind) {
    for (const TireModelDescriptor& descriptor : kTireModelDescriptors) {
        if (descriptor.kind == kind) return &descriptor;
    }
    return nullptr;
}


/// kind.
inline void apply_tire_state_semantics(Tire& tire) {
    const TireModelDescriptor* descriptor =
        tire_model_descriptor(tire.model_kind);
    // An unregistered kind cannot reach here -- both registration paths reject it
    // by name first -- so the fallback only keeps the function total.
    if (descriptor == nullptr) descriptor = &kTireModelDescriptors[0];
    tire.uses_exact_relaxation = descriptor->uses_exact_relaxation;
    tire.has_state_return_mapping = descriptor->has_state_return_mapping;
    tire.uses_pac2002_law = descriptor->uses_pac2002_law;
    tire.evaluates_at_wheel_center = descriptor->evaluates_at_wheel_center;
    tire.projects_compression_on_spin = descriptor->projects_compression_on_spin;
    tire.state_slot_width = kTireModelBaseStateSlotWidth;
}

} // namespace axle_kernel
