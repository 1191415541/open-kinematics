"""
The axle model can declare the ground its tires are measured against.

A pad is a *surface*, not a body, so the model carries it as geometry.  What is asserted
here is the two-part contract this feature has to keep:

* the field is optional, so every existing model keeps constructing exactly as before;
* when given, it reaches the **runtime** -- because `contract.model_document` receives
  only the runtime, a surface left on the model alone would be silently dropped before the
  document is ever written.

The parameters are also checked against what the kernel accepts.  A `plane` that omits
`wavelength`/`bump_length`/`corner_scales` is refused at model registration
(`cpp/src/assembly/registration.cpp`), so the schema states those requirements rather than
letting a caller discover them as a run-time error.
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from suspension_multibody.schema import FrontAxleModel, MassSpec, RoadSurfaceSpec, Vec3
from suspension_multibody.subsystems.entry import compose_axle


def _axle(**overrides) -> FrontAxleModel:
    """
    Return a complete symmetric-proxy axle.

    The hardpoint set is the full one the geometry roles require -- a partial set is
    refused by `subsystems/geometry.py` before any composition happens.
    """
    payload: dict = {
        "hardpoints": {
            "uca_front": Vec3(x=-100.0, y=-500.0, z=400.0),
            "uca_rear": Vec3(x=100.0, y=-500.0, z=400.0),
            "uca_outer": Vec3(x=0.0, y=-700.0, z=450.0),
            "lca_front": Vec3(x=-120.0, y=-500.0, z=150.0),
            "lca_rear": Vec3(x=120.0, y=-500.0, z=150.0),
            "lca_outer": Vec3(x=0.0, y=-700.0, z=150.0),
            "tierod_inner": Vec3(x=100.0, y=-400.0, z=250.0),
            "tierod_outer": Vec3(x=50.0, y=-700.0, z=250.0),
            "wheel_center": Vec3(x=0.0, y=-700.0, z=300.0),
            "rack_center": Vec3(x=0.0, y=0.0, z=250.0),
        },
        "mass": MassSpec(sprung_mass=1000),
    }
    payload.update(overrides)
    return FrontAxleModel(**payload)


def test_a_model_without_a_road_still_constructs():
    """The field is optional: its absence is the ordinary case."""
    model = _axle()
    assert model.road is None


def test_a_plane_pad_is_accepted_and_carries_its_height():
    model = _axle(
        road=RoadSurfaceSpec(kind="plane", origin=Vec3(x=0.0, y=0.0, z=-20.0))
    )
    assert model.road is not None
    assert model.road.kind == "plane"
    assert model.road.origin.z == -20.0


def test_a_plane_with_amplitude_is_refused():
    """A `plane` that oscillates would make its own kind a lie."""
    with pytest.raises(ValidationError, match="zero amplitude"):
        RoadSurfaceSpec(kind="plane", amplitude=5.0)


def test_the_kernel_required_parameters_are_strictly_positive():
    """
    `wavelength` and `bump_length` must be positive even for a plane.

    The kernel refuses a zero at registration; stating it here turns a run-time failure
    into a construction-time one.
    """
    with pytest.raises(ValidationError):
        RoadSurfaceSpec(kind="plane", wavelength=0.0)
    with pytest.raises(ValidationError):
        RoadSurfaceSpec(kind="plane", bump_length=0.0)


def test_the_road_reaches_the_runtime():
    """
    The document is authored from the runtime, so the surface has to travel with it.

    This is the assertion that keeps the feature from being a field nobody reads: the
    runtime is the only thing `model_document` sees.
    """
    model = _axle(
        road=RoadSurfaceSpec(kind="plane", origin=Vec3(x=0.0, y=0.0, z=-20.0))
    )
    runtime = compose_axle(model, "K")
    assert runtime.road is not None
    assert runtime.road.kind == "plane"
    assert runtime.road.origin.z == -20.0


def test_a_model_without_a_road_gives_a_runtime_without_one():
    runtime = compose_axle(_axle(), "K")
    assert runtime.road is None
