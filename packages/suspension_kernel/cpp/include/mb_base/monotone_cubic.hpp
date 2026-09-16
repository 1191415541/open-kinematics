#pragma once

/// The value-and-slope pair a monotone cubic interpolation returns
/// (MB_BASE).
///
/// It carries the derivative as well as the value because the directional
/// (dual-number) tire laws differentiate the interpolant analytically.


namespace axle_kernel {

struct CurveValue {
    double value{0.0};
    double slope{0.0};
};

} // namespace axle_kernel
