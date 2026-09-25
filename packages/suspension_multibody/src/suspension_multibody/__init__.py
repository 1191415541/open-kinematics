"""
Independent quasi-static suspension K&C and load analysis package.

The public names below are reached through :pep:`562` module ``__getattr__``
rather than imported here.  The distinction is not stylistic: importing ``api``
at the package root executes most of the product, so *any* submodule import --
including the low ``modeling`` layer, which is meant to be usable on its own --
paid for the whole chain, and the resulting half-initialised modules are what
made import order load-bearing.  Resolving on first access keeps the same public
surface for callers while letting a leaf module be a leaf.

``TYPE_CHECKING`` imports give static tools the real types without running
anything.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .api import run_case, run_dynamic_case
    from .axle_dynamics import AxleDynamicsResult
    from .io.artifacts import read_artifact, write_artifact
    from .results.vehicle import VehicleDynamicsResult
    from .schema import (
        CaseSpec,
        FrontAxleModel,
        Manifest,
        ResultBundle,
        SchemaVersion,
        load_case,
        load_model,
        load_vehicle_dynamic_case,
        load_vehicle_model,
    )
    from .vehicle.service import run_vehicle_dynamics

__version__ = "0.1.0"

#: Public name -> the module attribute path that defines it.  Kept as data so
#: the surface is inspectable and a typo is a lookup error rather than a silent
#: missing attribute.
_PUBLIC_NAMES: dict[str, tuple[str, str]] = {
    "AxleDynamicsResult": (".axle_dynamics", "AxleDynamicsResult"),
    "CaseSpec": (".schema", "CaseSpec"),
    "FrontAxleModel": (".schema", "FrontAxleModel"),
    "Manifest": (".schema", "Manifest"),
    "ResultBundle": (".schema", "ResultBundle"),
    "SchemaVersion": (".schema", "SchemaVersion"),
    "VehicleDynamicsResult": (".results.vehicle", "VehicleDynamicsResult"),
    "load_case": (".schema", "load_case"),
    "load_model": (".schema", "load_model"),
    "load_vehicle_dynamic_case": (".schema", "load_vehicle_dynamic_case"),
    "load_vehicle_model": (".schema", "load_vehicle_model"),
    "read_artifact": (".io.artifacts", "read_artifact"),
    "run_case": (".api", "run_case"),
    "run_dynamic_case": (".api", "run_dynamic_case"),
    "run_vehicle_dynamics": (".vehicle.service", "run_vehicle_dynamics"),
    "write_artifact": (".io.artifacts", "write_artifact"),
}

__all__ = [
    "AxleDynamicsResult",
    "CaseSpec",
    "FrontAxleModel",
    "Manifest",
    "ResultBundle",
    "SchemaVersion",
    "VehicleDynamicsResult",
    "__version__",
    "load_case",
    "load_model",
    "load_vehicle_dynamic_case",
    "load_vehicle_model",
    "read_artifact",
    "run_case",
    "run_dynamic_case",
    "run_vehicle_dynamics",
    "write_artifact",
]


def __getattr__(name: str) -> Any:
    """Resolve a public name on first access, importing only its own module."""
    target = _PUBLIC_NAMES.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute = target
    from importlib import import_module

    value = getattr(import_module(module_name, __name__), attribute)
    # Cache on the module so the lookup happens once, exactly as an eager import
    # would have left it.
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    """List the public surface without importing any of it."""
    return sorted({*globals(), *__all__})
