#pragma once

/// The kernel's width, tolerance and limit constants (MB_BASE).
///
/// They are here rather than with the output layer because several are
/// structural: `kTireOutputWidth` is the tire state block's stride in the
/// integrator and the ABI's capacity check, not just a column count.

#include <cstddef>

namespace axle_kernel {

inline constexpr double kEps = 1e-12;

inline constexpr double kPi = 3.141592653589793238462643383279502884;

inline constexpr double kPac2002CamberLimit = 0.26181;

inline constexpr int kStatePerBody = 19;

inline constexpr int kTireOutputWidth = 41;

inline constexpr int kConstraintOutputWidth = 6;

// The three axial structures report one block each.  Splitting the fused
// `Spring` record split its single 7-column ledger the same way: the elastic
// block keeps the length, the rate, the elastic force and its preload, the
// dissipative block reports the damping force and the power it removes, and the
// unilateral block reports the penetration, the stop force and whether the stop
// is engaged.  Together they carry every number the fused row carried.
inline constexpr int kSpringOutputWidth = 4;

inline constexpr int kDamperOutputWidth = 4;

inline constexpr int kBumpStopOutputWidth = 5;

inline constexpr int kBushingOutputWidth = 12;

inline constexpr int kAntiRollOutputWidth = 3;

inline constexpr int kSteeringOutputWidth = 4;

// The element-wrench channel's stride (05 step 3): world force, world moment
// about the receiving body's origin, element type code, action point in world
// coordinates, body a, body b, receiving body.  It is a structural constant
// like the other widths: the block's shape and its row addressing share it.
inline constexpr int kElementWrenchOutputWidth = 13;

inline constexpr int kDiagnosticsWidth = 16;

inline constexpr int kPerformanceWidth = 24;

inline constexpr int kEnergyOutputWidth = 21;

inline constexpr int kContactEventOutputWidth = 3;

} // namespace axle_kernel
