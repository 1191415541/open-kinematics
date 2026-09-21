#pragma once

// The single source of truth for every ABI version this kernel exports.
//
// This header has no dependencies of any kind, so both the public C ABI header
// and the internal model layer can include it.  It lives in the base layer
// rather than under `abi/` because the model layer needs the literals and a
// model -> abi edge would put the ABI aggregate back inside a dependency cycle.

namespace axle_kernel {

/// Axle (`AxleInput`/`AxleOutput`) ABI version.
///
/// Bumped 14 -> 15 at K7: the two structures gained the `struct_size` /
/// `abi_version` / `reserved` extension header and the generic element surface
/// (`element_count`/`elements`/`element_curves`/`topology_extension_count`/
/// `topology_extensions`).  Both additions are at the front or the back, so every
/// pre-existing field kept its offset; the version still moves because the
/// structure the caller must compile against is no longer the same size, and a
/// caller built against 14 would be told so rather than read past its own end.
inline constexpr int kAxleKernelAbiVersion = 15;

/// Whole-vehicle (`VehicleInput`/`VehicleOutput`) ABI version.
///
/// Bumped 29 -> 30 at K7 for the same reason: `VehicleInput` gained the same
/// element surface, and it embeds `AxleInput` by value, so its own size moved
/// with the axle structure's.
inline constexpr int kVehicleKernelAbiVersion = 30;

/// Generic core (`mb_core_*`) ABI version.
inline constexpr int kCoreKernelAbiVersion = 1;

} // namespace axle_kernel
