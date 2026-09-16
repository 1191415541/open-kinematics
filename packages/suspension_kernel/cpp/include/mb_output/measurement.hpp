#pragma once

/// One steering measurement, as the ABI reports it (MB_OUTPUT).
///
/// `measure_steering` produces it and `write_vehicle_steering_output` writes it,
/// both in the output layer, so it lives here rather than in the transitional
/// aggregate header that used to declare it.

namespace axle_kernel {

struct SteeringMeasurement {
    double angle{0.0};
    double rate{0.0};
    double torque{0.0};
};

} // namespace axle_kernel
