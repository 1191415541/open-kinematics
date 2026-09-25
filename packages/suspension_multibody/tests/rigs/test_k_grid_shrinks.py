"""
The K grid loses a dimension when the assembly has no steering.

A rig adapting is only half the story: the *grid* the rig solves over has to shrink
with it, or the case layer would carry a rack axis the assembly cannot drive.  This
is the concrete failure the earlier plan recorded -- `axis_map["rack"]` was built
with `next(...)` over the document's `rack_*` names, so a steered assembly without
a rack raised `StopIteration` from deep inside the case layer instead of saying at
the boundary that there was nothing to drive.
"""

from __future__ import annotations

from suspension_multibody.api import _k_drivable_coordinates, _k_grid
from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.rigs import resolve_combination
from suspension_multibody.schema import DisplacementControl
from suspension_multibody.subsystems import DEFAULT_AXLE_SUBSYSTEMS, AssemblyRequest
from tests.benchmark_fixture import benchmark_model


def _without_steering():
    model = benchmark_model()
    hardpoints = {
        name: point
        for name, point in model.hardpoints.items()
        if name != "rack_center"
    }
    return build_front_axle(
        model.model_copy(update={"hardpoints": hardpoints}),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )


def _named_coordinates(section: dict[str, object]) -> list[str]:
    """Return every driven coordinate a `k` section names, whichever spelling."""
    axes = section.get("axes")
    if isinstance(axes, list):
        return [str(axis["coordinate"]) for axis in axes]
    axis_map = section.get("axis_map")
    if isinstance(axis_map, dict):
        named = [str(name) for name in axis_map.get("wheel", ())]
        rack = axis_map.get("rack")
        if rack is not None:
            named.append(str(rack))
        return named
    return []


def _controls() -> list[DisplacementControl]:
    return [
        DisplacementControl(target="wheel_travel_left", values=(-10.0, 0.0, 10.0)),
        DisplacementControl(target="rack", values=(-5.0, 0.0, 5.0)),
    ]


def test_an_assembly_with_steering_offers_the_rack_coordinate() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    assert "rack_drive" in _k_drivable_coordinates(assembly)


def test_an_assembly_without_steering_offers_only_wheel_travel() -> None:
    assembly = _without_steering()
    drivable = _k_drivable_coordinates(assembly)
    assert drivable == frozenset({"wheel_drive_L", "wheel_drive_R"})
    assert not any(name.startswith("rack") for name in drivable)


def test_the_grid_keeps_the_rack_axis_when_the_assembly_has_steering() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    section, combinations = _k_grid(
        _controls(), drivable=_k_drivable_coordinates(assembly)
    )
    assert "rack" in section["axis_map"]
    # Three wheel values by three rack values.
    assert len(combinations) == 9


def test_the_grid_drops_the_rack_axis_when_it_cannot_be_driven() -> None:
    """
    The rack axis disappears; it is not filled with a single zero.

    A grid that kept a rack axis of `(0.0,)` would produce the same three states
    but would claim the assembly has a rack, and the case document would name a
    coordinate the model never declares.

    With no rack coordinate at all the grid leaves the *shorthand* behind: that
    spelling's contract is "wheel travel plus a rack axis", so it cannot express
    a run that has no rack.  The general `axes` form names exactly the axes the
    grid moves, and that is what a bench with no steering gets -- so the assertion
    is that the document names no rack, whichever spelling carries it.
    """
    assembly = _without_steering()
    section, combinations = _k_grid(
        _controls(), drivable=_k_drivable_coordinates(assembly)
    )
    assert "axis_map" not in section or "rack" not in section["axis_map"]
    coordinates = [
        axis["coordinate"] for axis in section.get("axes", ())
    ] or list(section.get("axis_map", {}).get("wheel", ()))
    assert not any(name.startswith("rack") for name in coordinates)
    assert section.get("rack_values_mm", []) == []
    assert len(combinations) == 3
    assert all(rack == 0.0 for _, _, rack in combinations)


def test_the_grid_without_a_capability_restriction_is_unchanged() -> None:
    """The historical call keeps its shape, so existing callers do not move."""
    section, combinations = _k_grid(_controls())
    assert "rack" in section["axis_map"]
    assert len(combinations) == 9


def test_the_rig_and_the_grid_shrink_together() -> None:
    """One assembly, one decision: what the rig drives is what the grid has."""
    assembly = _without_steering()
    composition = resolve_combination(
        "axle", "kc_quasi_static", assembly.capabilities
    )
    section, _ = _k_grid(_controls(), drivable=_k_drivable_coordinates(assembly))
    assert "rack_drive" in composition.dropped
    assert _named_coordinates(section) and not any(
        name.startswith("rack") for name in _named_coordinates(section)
    )
    for drive in composition.drives:
        assert drive.coordinate in _k_drivable_coordinates(assembly)


def test_an_explicit_axis_map_never_names_an_undrivable_rack() -> None:
    """The `next(...)` failure is gone: the mapping is built, not searched."""
    assembly = _without_steering()
    section, _ = _k_grid(_controls(), drivable=_k_drivable_coordinates(assembly))
    axis_map = section.get("axis_map")
    if axis_map is not None:
        # The shorthand was used, so it must name no rack.
        assert "rack" not in axis_map
        assert all(not name.startswith("rack") for name in axis_map["wheel"])
    # Whichever spelling carried it, the document names the wheel drives only.
    assert set(_named_coordinates(section)) == {"wheel_drive_L", "wheel_drive_R"}


def test_a_no_steering_axle_runs_through_the_public_entry() -> None:
    """
    GAP-2's real acceptance: a steering-less axle is *runnable*, and its rack
    channel is absent rather than zero.

    Sub-task 01 recorded this as the defect: the run had no way to be asked for
    through the public entry at all, so the shrink was only ever visible in a
    private helper.  Now the case carries the subsystem set, the run solves, and
    the result says what it did not steer by *omitting* the channel.
    """
    from suspension_multibody.api import run_case
    from suspension_multibody.schema import CaseSpec

    model = benchmark_model()
    hardpoints = {
        name: point for name, point in model.hardpoints.items() if name != "rack_center"
    }
    without = model.model_copy(update={"hardpoints": hardpoints})
    case = CaseSpec(
        mode="K",
        subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"},
        controls=(
            DisplacementControl(target="wheel_travel_left", values=(-10.0, 0.0, 10.0)),
        ),
    )

    bundle = run_case(without, case)

    assert len(bundle.states) == 3
    for state in bundle.states:
        assert "rack_displacement" not in state.drives
        assert set(state.drives) == {"wheel_travel_left", "wheel_travel_right"}
        assert state.converged

    # And the steering-carrying run still reports it, so the omission is a
    # consequence of the assembly and not of the code path.
    with_steering = run_case(
        model,
        CaseSpec(
            mode="K",
            controls=(
                DisplacementControl(target="wheel_travel_left", values=(-10.0, 0.0, 10.0)),
            ),
        ),
    )
    assert "rack_displacement" in with_steering.states[0].drives
