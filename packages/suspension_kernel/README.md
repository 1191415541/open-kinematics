# suspension-kernel

Generic multibody kernel: the shared C++ core, its CMake build, and the
product-independent part of the ctypes boundary.

## What lives here

| Path | Owns |
|---|---|
| `cpp/` | the C++ kernel sources and headers |
| `CMakeLists.txt` | the real build (CMake + Ninja, explicit source list) |
| `src/suspension_kernel/native/` | the built shared library and `native_build.json` |
| `src/suspension_kernel/binding/` | library path resolution, loading, exported-symbol probing, the ABI gate, metadata reading, and the error types |

## What does not live here

Anything that knows what a *product* is. The kernel owns the element semantics --
springs, bushings, anti-roll bars, the tire force laws, the static and dynamic
solvers -- because that is what it was built to take over; what it must never do
is reach back into a product package: `suspension_kernel` never imports
`suspension_multibody`, and the dependency runs one way only.

The boundary is the contract, not a set of structs: a caller sends a model
document and a case document and gets a result document back, through
`suspension_kernel_run`. The product keeps its own `ctypes.Structure` mirrors
and its marshalling for the older entry points and loads the shared library
through `suspension_kernel.binding`.

An earlier version of this section said the kernel contains no element
semantics. That was true of the kernel's first incarnation and has not been true
since the takeover: `mb_suspension`, `mb_tire` and `mb_static` are the element
and solver layers, and this file is the place that has to say so.

## Build

```
uv run python packages/suspension_kernel/scripts/build_suspension_kernel.py
```

The historical entry point
`packages/suspension_multibody/scripts/build_axle_native.py` still works: it
delegates here and then copies the product into the axle package's `native`
directory so that the axle ctypes boundary keeps loading it from the path it
always used.

## Flags

The compile and link flag set is a one-to-one mapping of the previous direct
compiler invocation, recorded from the generated build files into
`native_build.json`. No floating-point semantic flag is added or removed.
