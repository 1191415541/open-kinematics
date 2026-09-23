"""
The tire's mass reaches the axle's own model, and the default document does not move.

Decision D2 makes the tire an inertia source, and subtask 08 built the kernel and
the contract side of that.  What was still missing is the axle authoring layer:
`AxleTire` had nowhere to declare a mass, so an axle tire could not be an inertia
source at all.  This file covers that half -- and, just as importantly, that a
model which declares no mass emits exactly the document it used to.
"""

from __future__ import annotations

from suspension_multibody.axle_dynamics.schema import AxleTire
from suspension_multibody.cases.axle_dynamic import _tire_entry

_TIRE = dict(
    name="tire_L",
    body="upright_L",
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


def test_a_tire_without_mass_emits_no_mass_field() -> None:
    """
    The historical document must not change.

    Every recorded baseline was authored by a model that could not declare a tire
    mass.  If the entry grew a `mass: 0.0`, every one of those documents would
    differ and every baseline would have to be re-recorded for no physical reason.
    """
    entry = _tire_entry(AxleTire(**_TIRE), 0, 0, 0)
    assert "mass" not in entry
    assert "inertia" not in entry


def test_a_tire_with_mass_emits_it() -> None:
    entry = _tire_entry(AxleTire(**_TIRE, mass_kg=20.0), 0, 0, 0)
    assert entry["mass"] == 20.0


def test_a_declared_inertia_is_emitted_as_a_matrix() -> None:
    entry = _tire_entry(
        AxleTire(**_TIRE, mass_kg=20.0, inertia_kg_m2=_INERTIA), 0, 0, 0
    )
    assert entry["inertia"] == [list(row) for row in _INERTIA]


def test_mass_alone_is_a_complete_statement() -> None:
    """A mass with no inertia is still the right model: the solver adds both."""
    entry = _tire_entry(AxleTire(**_TIRE, mass_kg=20.0), 0, 0, 0)
    assert entry["mass"] == 20.0
    assert "inertia" not in entry


def test_the_entry_keeps_the_keys_the_contract_requires() -> None:
    entry = _tire_entry(AxleTire(**_TIRE, mass_kg=20.0), 0, 0, 0)
    for key in ("name", "model", "body", "parameters"):
        assert key in entry
