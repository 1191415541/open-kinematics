"""
Requirement 10 on the vehicle side: the tire owns the mass it declares.

Subtask 08 built the kernel and the contract field, subtask 09 the axle authoring
path.  The vehicle side has its own emitter, so it needed its own landing point --
and, as on the axle side, the default must not move a single recorded document.
"""

from __future__ import annotations

from suspension_multibody.axle_dynamics.schema import AxleTire
from suspension_multibody.cases.vehicle_dynamic import _tire_entry

_TIRE = dict(
    name="front_left",
    body="wheel_front_left",
    center_local_m=(0.0, 0.0, 0.0),
    unloaded_radius_m=0.3,
    maximum_compression_m=0.1,
    vertical_stiffness_n_per_m=2.0e5,
    vertical_damping_n_s_per_m=0.0,
    longitudinal_friction_coefficient=1.0,
    lateral_friction_coefficient=1.0,
    longitudinal_brush_stiffness_n_per_m=1.0,
    lateral_brush_stiffness_n_per_m=1.0,
    longitudinal_relaxation_length_m=1.0,
    lateral_relaxation_length_m=1.0,
    detached_relaxation_s=1.0,
)

_INERTIA = ((0.7, 0.0, 0.0), (0.0, 1.1, 0.0), (0.0, 0.0, 0.7))


def test_a_vehicle_tire_without_mass_emits_no_mass_field() -> None:
    """
    The historical vehicle document must not change.

    Every recorded full-vehicle baseline was authored by a model that could not
    declare a tire mass.  Growing a `mass: 0.0` would differ every one of those
    documents and force a re-record for no physical reason.
    """
    entry = _tire_entry(AxleTire(**_TIRE), 0, 0, 0)
    assert "mass" not in entry
    assert "inertia" not in entry


def test_a_vehicle_tire_with_mass_emits_it() -> None:
    entry = _tire_entry(AxleTire(**_TIRE, mass_kg=5.0), 0, 0, 0)
    assert entry["mass"] == 5.0


def test_a_vehicle_tire_inertia_is_emitted_as_a_matrix() -> None:
    entry = _tire_entry(
        AxleTire(**_TIRE, mass_kg=5.0, inertia_kg_m2=_INERTIA), 0, 0, 0
    )
    assert entry["inertia"] == [list(row) for row in _INERTIA]


def test_the_vehicle_entry_keeps_the_keys_the_contract_requires() -> None:
    entry = _tire_entry(AxleTire(**_TIRE, mass_kg=5.0), 0, 0, 0)
    for key in ("name", "model", "body", "parameters"):
        assert key in entry
