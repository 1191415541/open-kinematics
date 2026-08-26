"""The Adams Car importer must reproduce the source numbers exactly."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from suspension_multibody.adams.car_import import (
    import_blockers,
    read_adams_suspension,
    suspension_summary,
)

_ADAMS_ROOT = Path("G:/MSC.Software/Adams/2024_1")
_SUBSYSTEM = (
    _ADAMS_ROOT
    / "acar/achassis_gs.cdb/subsystems.tbl/acar_gs_front_suspension.sub"
)

pytestmark = pytest.mark.skipif(
    not _SUBSYSTEM.is_file(),
    reason="a local Adams Car installation is required",
)


def _suspension():
    return read_adams_suspension(
        _SUBSYSTEM,
        tire_unloaded_radius_m=0.300,
        tire_stiffness_n_per_m=200_000.0,
    )


def test_hardpoints_convert_millimetres_to_metres() -> None:
    """A wrong length scale would not raise, so it is pinned here."""
    suspension = _suspension()

    # Values read directly from the subsystem file, divided by 1000.
    assert suspension.hardpoints_m["wheel_center"] == pytest.approx(
        (1.5292, -0.74302, 0.51727)
    )
    assert suspension.hardpoints_m["lower_ball_joint"] == pytest.approx(
        (1.52234, -0.65, 0.425)
    )
    assert suspension.hardpoints_m["upper_ball_joint"] == pytest.approx(
        (1.55387, -0.57, 0.80475)
    )
    assert len(suspension.hardpoints_m) == 22


def test_masses_and_inertias_convert_to_si() -> None:
    """Adams writes kg*mm^2; a missed 1e6 factor stays finite and plausible."""
    parts = {part.name: part for part in _suspension().parts}

    upright = parts["upright"]
    assert upright.mass_kg == pytest.approx(10.17)
    assert upright.inertia_kg_m2 == pytest.approx((0.09473, 0.1261, 0.04371))
    assert upright.sprung_fraction == pytest.approx(0.0)

    arm = parts["lower_control_arm"]
    assert arm.mass_kg == pytest.approx(2.69)
    assert arm.inertia_kg_m2 == pytest.approx((0.04282, 0.00163, 0.04316))
    assert arm.sprung_fraction == pytest.approx(0.5)

    assert len(parts) == 11


def test_spring_and_stop_clearances_convert_to_si() -> None:
    suspension = _suspension()

    # 87.5 N/mm and a 315 mm free length in the property file.
    assert suspension.spring_rate_n_per_m == pytest.approx(87_500.0)
    assert suspension.spring_free_length_m == pytest.approx(0.315)
    assert suspension.bumpstop_clearance_m == pytest.approx(0.17888)
    assert suspension.reboundstop_clearance_m == pytest.approx(0.03454)


def test_damper_curve_is_imported_whole_and_strictly_increasing() -> None:
    """The measured curve must survive import, including its asymmetry."""
    suspension = _suspension()
    velocity = suspension.damper_velocity_m_per_s
    force = suspension.damper_force_n

    assert len(velocity) == len(force) >= 40
    assert all(b > a for a, b in zip(velocity, velocity[1:]))
    # mm/s in the file, m/s here.
    assert velocity[0] == pytest.approx(-4.0)
    # A real shock carries gas preload, so the force at zero is not zero.
    zero_index = velocity.index(0.0)
    assert force[zero_index] == pytest.approx(-142.3)


def test_bushing_rates_convert_from_adams_units() -> None:
    """N/mm becomes N/m and N*mm/deg becomes N*m/rad."""
    bushings = {bushing.name: bushing for bushing in _suspension().bushings}

    assert len(bushings) == 8
    lca_front = bushings["lca_front"]
    # 6450 N/mm radially and 775 N/mm axially in the property file.
    assert lca_front.translational_stiffness_n_per_m == pytest.approx(
        (6_450_000.0, 6_450_000.0, 775_000.0)
    )
    # Every imported bushing must be positive semidefinite on its diagonal.
    for bushing in bushings.values():
        assert min(bushing.translational_stiffness_n_per_m) >= 0.0
        assert min(bushing.rotational_stiffness_n_m_per_rad) >= 0.0
        assert min(bushing.translational_damping_n_s_per_m) >= 0.0
        assert min(bushing.rotational_damping_n_m_s_per_rad) >= 0.0

    # The hub compliance is rotational only.  Its curve is 9.8e7 N*mm per
    # degree, which is 9.8e4 N*m per degree, i.e. 5.61e6 N*m/rad.
    hub = bushings["hub_compliance"]
    assert hub.translational_stiffness_n_per_m == pytest.approx((0.0, 0.0, 0.0))
    assert hub.rotational_stiffness_n_m_per_rad[0] == pytest.approx(
        98_000_000.0 / 1000.0 * (180.0 / math.pi), rel=1e-9
    )
    assert hub.rotational_stiffness_n_m_per_rad[2] == pytest.approx(0.0)


def test_import_reports_no_blockers_for_this_subsystem() -> None:
    """Anything inexpressible must be reported rather than approximated."""
    suspension = _suspension()

    assert import_blockers(suspension) == ()
    summary = suspension_summary(suspension)
    assert summary["part_count"] == 11
    assert summary["bushing_count"] == 8
    # Unsprung mass is the upright plus the spindle plus half of each link.
    assert summary["unsprung_mass_kg"] == pytest.approx(15.84005)
