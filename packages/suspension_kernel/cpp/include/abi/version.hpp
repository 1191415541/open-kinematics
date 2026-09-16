#pragma once

// The single source of truth for every ABI version this kernel exports.
//
// Nothing else in the C++ sources may spell a version as a literal: the
// version-exporting entry points are defined in terms of these constants, and
// `kernel_abi.cpp` carries a `static_assert` pair tying each one to the literal
// the corresponding structure's `abi_version` field must carry.  A bump is
// therefore one edit, and a half-finished bump fails to compile rather than
// shipping a library that advertises one version and speaks another.
//
// This header has no dependencies of any kind, so both the public C ABI header
// and the internal model layer can include it.

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
