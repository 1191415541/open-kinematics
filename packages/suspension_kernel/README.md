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

Anything that knows what a spring, bushing, or tire is. Those semantics stay in
the product package that owns them; `suspension_kernel` must never import
`suspension_multibody`. The axle product keeps its own `ctypes.Structure`
mirrors and its marshalling, and loads the shared library through
`suspension_kernel.binding`.

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
