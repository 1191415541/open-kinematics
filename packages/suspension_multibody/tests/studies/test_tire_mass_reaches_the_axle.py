"""Axle tire inertia survives offline migration and the shared compiler."""

from __future__ import annotations

from suspension_multibody.axle_dynamics.schema import AxleTire
from tests.authoring._tire_document import tire_document

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


def test_a_tire_without_mass_has_no_inertia_contribution() -> None:
    entry = tire_document(AxleTire(**_TIRE))
    assert entry["mass"] == 0
    assert entry["inertia"] == [[0, 0, 0]] * 3


def test_a_tire_with_mass_emits_it() -> None:
    entry = tire_document(AxleTire(**_TIRE, mass_kg=20.0))
    assert entry["mass"] == 20.0


def test_a_declared_inertia_is_emitted_as_a_matrix() -> None:
    entry = tire_document(AxleTire(**_TIRE, mass_kg=20.0, inertia_kg_m2=_INERTIA))
    assert entry["inertia"] == [list(row) for row in _INERTIA]


def test_mass_alone_is_a_complete_statement() -> None:
    """A mass with no inertia is still the right model: the solver adds both."""
    entry = tire_document(AxleTire(**_TIRE, mass_kg=20.0))
    assert entry["mass"] == 20.0
    assert entry["inertia"] == [[0, 0, 0]] * 3


def test_the_entry_keeps_the_keys_the_contract_requires() -> None:
    entry = tire_document(AxleTire(**_TIRE, mass_kg=20.0))
    for key in ("name", "model", "body", "parameters"):
        assert key in entry
