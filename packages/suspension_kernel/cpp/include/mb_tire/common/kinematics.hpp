#pragma once

/// The scalar contact kinematics of a tire (MB_TIRE_COMMON).
///
/// `MODULES.md` section 2.3 keeps this type -- and `tire_contact_kinematics`,
/// which returns it -- in the tire common layer: the integrator and the statics
/// layer both ask for it, and neither is below the model layer.

namespace axle_kernel {

struct TireContactKinematics {
    double compression{0.0};
    double compression_rate{0.0};
    double loaded_radius{0.0};
};

} // namespace axle_kernel
