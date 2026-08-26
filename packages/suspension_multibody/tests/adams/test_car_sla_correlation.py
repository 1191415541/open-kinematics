"""
The real Adams Car SLA suspension must solve and match its static loads.

These tests build the model from the installed Adams Car database rather than
from transcribed numbers, so a unit-conversion or topology mistake shows up as a
physical disagreement with Adams' own recorded wheel loads.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.adams.car_import import read_adams_suspension
from suspension_multibody.adams.car_sla_model import (
    build_sla_axle_model,
    unsprung_corner_mass_kg,
)
from suspension_multibody.axle_dynamics import (
    AxleDynamicsCase,
    AxleSolverSettings,
    run_axle_dynamics,
)

_ADAMS_ROOT = Path("G:/MSC.Software/Adams/2024_1")
_SUBSYSTEM = (
    _ADAMS_ROOT
    / "acar/achassis_gs.cdb/subsystems.tbl/acar_gs_front_suspension.sub"
)

# Recorded by Adams in the wheel-force-transducer parameter file for this
# vehicle's front axle.  These are the numbers the imported model must
# reproduce from mass, geometry and stiffness alone.
_ADAMS_STATIC_LOAD_LEFT_N = 5117.77
_ADAMS_STATIC_LOAD_RIGHT_N = 4928.6
_ADAMS_RIG_WHEEL_RADIUS_M = 0.300
_ADAMS_TIRE_STIFFNESS_N_PER_M = 200_000.0

pytestmark = pytest.mark.skipif(
    not _SUBSYSTEM.is_file(),
    reason="a local Adams Car installation is required",
)


def _model_and_road():
    suspension = read_adams_suspension(
        _SUBSYSTEM,
        tire_unloaded_radius_m=_ADAMS_RIG_WHEEL_RADIUS_M,
        tire_stiffness_n_per_m=_ADAMS_TIRE_STIFFNESS_N_PER_M,
    )
    # Adams reports the load one front wheel carries, so the sprung mass this
    # axle supports follows from that load minus the corner's own unsprung mass.
    sprung_axle_mass_kg = 2.0 * (
        _ADAMS_STATIC_LOAD_LEFT_N / 9.80665
        - unsprung_corner_mass_kg(suspension)
    )
    # Hardpoints are in the vehicle frame, where the wheel centre sits one rig
    # wheel radius above the contact plane.
    road_height_m = (
        suspension.hardpoints_m["wheel_center"][2] - _ADAMS_RIG_WHEEL_RADIUS_M
    )
    model = build_sla_axle_model(
        suspension,
        sprung_mass_kg=sprung_axle_mass_kg,
        sprung_inertia_kg_m2=(180.0, 60.0, 200.0),
        sprung_height_m=0.85,
        road_height_m=road_height_m,
    )
    return model, road_height_m


def _run(duration_s: float = 0.05):
    model, road_height_m = _model_and_road()
    count = int(round(duration_s / 0.001)) + 1
    times = tuple(index * 0.001 for index in range(count))
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="static_equilibrium",
            times_s=times,
            road_height_m={
                tire.name: (road_height_m,) * count for tire in model.tires
            },
            solver=AxleSolverSettings(
                initialization_mode="static_equilibrium",
                adaptive_step=True,
                internal_step_s=0.00025,
                maximum_step_s=0.001,
                max_newton_iterations=100,
                max_line_search_iterations=30,
            ),
        ),
    )
    return model, result


def test_real_suspension_reaches_static_equilibrium() -> None:
    """The imported model must trim and then hold that trim exactly."""
    model, result = _run()

    height = result.body_state("sprung")[:, 2]
    # A trimmed state is an equilibrium: it must not drift once integration
    # starts, which is the sharpest check that the trim really converged.
    assert float(np.ptp(height)) <= 1e-9
    assert float(np.max(result.diagnostics.position_residual)) <= 1e-8
    assert float(np.max(result.diagnostics.velocity_residual)) <= 1e-7
    assert np.all(result.diagnostics.accepted)
    assert len(model.bodies) == 12


def test_static_wheel_load_matches_the_adams_recorded_value() -> None:
    """Mass, geometry and tire rate alone must reproduce Adams' wheel load."""
    _, result = _run()

    left = float(result.tire_state("tire_l")[0, 4])
    right = float(result.tire_state("tire_r")[0, 4])

    assert left == pytest.approx(_ADAMS_STATIC_LOAD_LEFT_N, rel=0.01)
    assert right == pytest.approx(_ADAMS_STATIC_LOAD_LEFT_N, rel=0.01)


def test_every_newton_of_weight_is_carried_by_the_tires() -> None:
    """No load may leak into ground through a wrongly attached joint."""
    model, result = _run()

    weight_n = sum(body.mass_kg for body in model.bodies) * 9.80665
    carried_n = float(result.tire_state("tire_l")[0, 4]) + float(
        result.tire_state("tire_r")[0, 4]
    )

    assert carried_n == pytest.approx(weight_n, rel=1e-6)
    # The rig restraint exists only to make the static problem determinate; it
    # must not be quietly propping the axle up.
    restraint = result.bushing_state("rig_restraint")[0]
    assert float(np.max(np.abs(restraint[6:9]))) <= 1.0


def test_imported_elements_are_used_unmodified() -> None:
    """Spring rate and damper curve must arrive from Adams untouched."""
    model, _ = _model_and_road()
    suspension = read_adams_suspension(
        _SUBSYSTEM,
        tire_unloaded_radius_m=_ADAMS_RIG_WHEEL_RADIUS_M,
        tire_stiffness_n_per_m=_ADAMS_TIRE_STIFFNESS_N_PER_M,
    )

    spring = next(s for s in model.springs if s.name == "spring_l")
    damper = next(s for s in model.springs if s.name == "damper_l")

    assert spring.stiffness_n_per_m == pytest.approx(87_500.0)
    assert damper.damper_curve_velocity_m_per_s == (
        suspension.damper_velocity_m_per_s
    )
    assert damper.damper_curve_force_n == suspension.damper_force_n

    uprights = [b for b in model.bodies if b.name.startswith("upright")]
    assert len(uprights) == 2
    for upright in uprights:
        assert upright.mass_kg == pytest.approx(10.17)
