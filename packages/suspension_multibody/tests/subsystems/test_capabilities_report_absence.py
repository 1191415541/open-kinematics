"""
The assembly reports what it carries, so a rig never has to guess.

Subtask 10 shrinks the KC rig's interface to whatever the assembly actually
offers.  It can only do that if the assembly says so: probing for a body named
`rack` is what turns a missing subsystem into a `StopIteration` deep inside the
case layer instead of a decision at the boundary.
"""

from __future__ import annotations

from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.subsystems import DEFAULT_AXLE_SUBSYSTEMS, AssemblyRequest
from tests.benchmark_fixture import benchmark_model


def _without_steering_bodies():
    hardpoints = {
        name: point
        for name, point in benchmark_model().hardpoints.items()
        if name != "rack_center"
    }
    return benchmark_model().model_copy(update={"hardpoints": hardpoints})


def test_an_axle_with_steering_offers_the_rack_coordinate() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    assert assembly.capabilities is not None
    assert assembly.capabilities.has("steering")
    assert assembly.capabilities.provides("rack_drive")
    assert assembly.capabilities.provides("wheel_drive_L")
    assert assembly.capabilities.provides("wheel_drive_R")


def test_an_axle_without_steering_offers_no_rack_coordinate() -> None:
    assembly = build_front_axle(
        _without_steering_bodies(),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    assert assembly.capabilities is not None
    assert not assembly.capabilities.has("steering")
    assert not any(
        coordinate.startswith("rack")
        for coordinate in assembly.capabilities.drive_coordinates
    )
    # The wheel coordinate is the suspension's, not steering's, so it stays.
    assert assembly.capabilities.provides("wheel_drive_L")


def test_the_capability_set_uses_the_six_role_names() -> None:
    assembly = build_front_axle(benchmark_model(), "C")
    assert assembly.capabilities is not None
    # `wheel`, not `tire`: the rig supplies the wheels on a single axle (D9).
    assert "wheel" in assembly.capabilities.subsystems
    assert "tire" not in assembly.capabilities.subsystems
    assert "brake" not in assembly.capabilities.subsystems
    assert "drive" not in assembly.capabilities.subsystems


def test_requiring_an_absent_coordinate_names_what_is_missing() -> None:
    assembly = build_front_axle(
        _without_steering_bodies(),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    assert assembly.capabilities is not None
    try:
        assembly.capabilities.require("rack_drive", rig="kc_quasi_static")
    except Exception as error:  # noqa: BLE001 - the message is the contract
        message = str(error)
        assert "rack_drive" in message
        assert "kc_quasi_static" in message
        assert "wheel_drive_L" in message
    else:  # pragma: no cover - the call must raise
        raise AssertionError("requiring an absent coordinate must raise")
