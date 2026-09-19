"""Default compiler dispatch for assembly and case-family pairs."""

from __future__ import annotations

from functools import lru_cache

from .compiler import (
    AxleDynamicCompiler,
    CompilerRegistry,
    HandlingCompiler,
    KcQuasiStaticCompiler,
    RideFourPostCompiler,
    RideRandomRoadCompiler,
    VehicleDynamicCompiler,
    VehicleKcCompiler,
)


@lru_cache(maxsize=1)
def default_registry() -> CompilerRegistry:
    """Return the process-local registry for all supported family compilers."""
    registry = CompilerRegistry()
    for compiler in (
        AxleDynamicCompiler(),
        VehicleDynamicCompiler(),
        KcQuasiStaticCompiler(),
        VehicleKcCompiler(),
        HandlingCompiler(),
        RideFourPostCompiler(),
        RideRandomRoadCompiler(),
    ):
        registry.register(compiler)
    return registry


def compiler_for(assembly: str, family: str):
    """Resolve one compiler from the default registry."""
    return default_registry().resolve(assembly, family)


def dispatch_request(request, *, registry: CompilerRegistry | None = None):
    """Compile a request through the selected registry entry."""
    from .compiler import compile_request

    return compile_request(request, registry=registry)


__all__ = ["compiler_for", "default_registry", "dispatch_request"]
