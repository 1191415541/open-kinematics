"""
Prove the three structures compute the forces the fused record computed.

The fused `Spring` evaluated three laws on one point pair and summed them into
one scalar.  The split evaluates each law on its own point pair and adds them to
the same accumulators.  This compares the reported ledgers against the fused
formulas evaluated by hand at the same state, so a coefficient that landed in
the wrong structure, or a sign that flipped, shows up as a mismatch rather than
as a plausible-looking number.
"""
from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleBumpStop,
    AxleDamper,
    AxleDynamicsCase,
    AxleJoint,
    AxleSolverSettings,
    AxleSpring,
)
from suspension_multibody.axle_dynamics.schema import AxleDynamicsModel
from tests.axle_dynamics._unified_entry import solve_axle

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_Z = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

# One corner: every law on the same two points, which is the case the fused
# record expressed and the case the split has to reproduce.
_K = 10_000.0
_FREE = 0.25
_PRELOAD = 123.0
_C_COMPRESSION = 100.0
_C_REBOUND = 300.0
_CLEARANCE = 0.20
_STOP_K = 40_000.0
_STOP_C = 50.0


def _model() -> AxleDynamicsModel:
    return AxleDynamicsModel(
        name="three-laws",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(name="fixture", mass_kg=0.0, inertia_kg_m2=_Z, fixed=True),
            AxleBody(
                name="body",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.15),
                linear_velocity_m_per_s=(0.0, 0.0, -0.05),
            ),
        ),
        joints=(
            AxleJoint(
                name="slide",
                kind="prismatic",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
        springs=(
            AxleSpring(
                name="spring",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=_K,
                free_length_m=_FREE,
                preload_n=_PRELOAD,
            ),
        ),
        dampers=(
            AxleDamper(
                name="damper",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                compression_damping_n_s_per_m=_C_COMPRESSION,
                rebound_damping_n_s_per_m=_C_REBOUND,
            ),
        ),
        bump_stops=(
            AxleBumpStop(
                name="stop",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                clearance_m=_CLEARANCE,
                stiffness_n_per_m=_STOP_K,
                direction=1.0,
                damping_n_s_per_m=_STOP_C,
            ),
        ),
    )


def _run():
    return solve_axle(
        _model(),
        AxleDynamicsCase(
            name="laws",
            times_s=(0.0, 1.0e-4),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=1.0e-4,
            ),
        ),
    )


def test_the_elastic_law_is_the_fused_elastic_law() -> None:
    """`k*(free_length - length) + preload` is what the elastic ledger reports."""
    result = _run()
    spring = result.spring_state("spring")
    length = spring[:, 0]
    assert float(length[0]) == pytest.approx(0.15, abs=1e-12)
    np.testing.assert_allclose(
        spring[:, 2],
        _K * (_FREE - length),
        rtol=1e-12,
        atol=1e-9,
    )
    np.testing.assert_allclose(spring[:, 3], _PRELOAD, rtol=0.0, atol=0.0)


def test_the_dissipative_law_is_the_fused_damping_law() -> None:
    """The damper's force is `-c*rate`, with c chosen by the rate's sign."""
    result = _run()
    damper = result.damper_state("damper")
    rate = damper[:, 1]
    expected = np.where(rate < 0.0, -_C_COMPRESSION, -_C_REBOUND) * rate
    np.testing.assert_allclose(damper[:, 2], expected, rtol=1e-12, atol=1e-9)
    # The compression branch is the one a compressing corner takes, and the two
    # coefficients are different, so the branch is actually being decided.
    assert float(rate[0]) < 0.0
    assert _C_COMPRESSION != _C_REBOUND


def test_the_unilateral_law_is_the_fused_stop_law() -> None:
    """Below the clearance the stop pushes back with `k*penetration`."""
    result = _run()
    stop = result.bump_stop_state("stop")
    length = stop[:, 0]
    penetration = _CLEARANCE - length
    assert float(penetration[0]) > 0.0, "the fixture must start inside the stop"
    np.testing.assert_allclose(stop[:, 2], penetration, rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(
        stop[:, 3], _STOP_K * penetration, rtol=1e-12, atol=1e-9
    )
    np.testing.assert_allclose(stop[:, 4], 1.0, rtol=0.0, atol=0.0)


def test_the_corner_force_is_the_sum_of_the_three_laws() -> None:
    """
    The whole axial force is the fused record's sum, term for term.

    This is the assertion the split exists to preserve: the fused record's
    scalar was `elastic + damping + stop`, and the three structures have to add
    up to exactly that on the first sample, where no integration error has had a
    chance to accumulate.
    """
    result = _run()
    spring = result.spring_state("spring")[0]
    damper = result.damper_state("damper")[0]
    stop = result.bump_stop_state("stop")[0]

    # A positive scalar pushes the two ends apart, so a corner in compression
    # pushes the body up; the slider's acceleration is the corner's force over
    # its mass, and gravity is off.  The stop's damping is the one term its
    # ledger does not carry as a column, so it is rebuilt from the rate and the
    # coefficient the model declares, under the same closing-side rule the law
    # applies.
    stop_rate = stop[1]
    stop_damping = _STOP_C * stop_rate if stop_rate < 0.0 else 0.0
    fused = (
        (spring[2] + spring[3])        # elastic force plus its constant offset
        + damper[2]                     # dissipative
        + stop[3]                       # unilateral elastic
        - stop_damping                  # unilateral damping, resisting closure
    )
    acceleration = float(result.body_state("body")[0, 15])
    assert acceleration == pytest.approx(fused / 10.0, rel=1e-9)
