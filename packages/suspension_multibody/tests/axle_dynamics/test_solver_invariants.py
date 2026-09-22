"""
Physical invariants of the retired Python solvers, asserted on the native path.

``core/rank.py``, ``core/reactions.py`` and ``model/mass.py`` were deleted with
the rest of the old solver surface: no production path called them any more.
The claims they encoded are not deleted with them, so each claim is asserted
here against the axle contract the kernel actually answers:

* ``diagnose_rank`` classified a constraint Jacobian as over- or
  under-constrained.  The native model build refuses a rank-deficient joint set
  outright, and a body the model does not constrain is left free instead of
  being held by a constraint that was never declared.
* ``recover_reactions`` turned KKT multipliers into per-body wrenches and
  ``body_equilibrium_wrench`` summed them.  The native solver reports the
  constraint wrench itself, and in a hanging equilibrium it carries exactly the
  gravity load.
* ``body_mass_properties``/``mass_matrix`` assembled the spatial inertia and
  ``spatial_bias_wrench`` the Newton-Euler velocity bias.  The native solver
  integrates the same tensor: the angular acceleration is ``torque / I``, and a
  body spinning about a non-principal axis shows the ``omega x (I omega)`` term.

The per-assertion mapping is recorded in the epic's coverage registry
(``tasks/20260921-08-delete/raw/step4_assertion_coverage.md``).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleAntiRollBar,
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    NativeAxleError,
    run_axle_dynamics,
)

_I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_GRAVITY = 9.80665


def _fixture() -> AxleBody:
    """Return the fixed body every axle model needs."""
    return AxleBody(name="ground", mass_kg=0.0, inertia_kg_m2=_I3, fixed=True)


def _case(name: str, *, consistent_state: bool = False) -> AxleDynamicsCase:
    """Return a short run; the initializer is chosen by the caller."""
    settings = (
        AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.0005,
        )
        if consistent_state
        else AxleSolverSettings(internal_step_s=0.00025)
    )
    return AxleDynamicsCase(name=name, times_s=(0.0, 0.02), solver=settings)


def _vertical_slider() -> AxleDynamicsModel:
    """Return a mass on a frictionless vertical guide, in static equilibrium."""
    mass = 10.0
    stiffness = 10_000.0
    equilibrium_length = 0.25 - mass * _GRAVITY / stiffness
    return AxleDynamicsModel(
        name="vertical-slider",
        bodies=(
            _fixture(),
            AxleBody(
                name="slider",
                mass_kg=mass,
                inertia_kg_m2=_I3,
                position_m=(0.0, 0.0, equilibrium_length),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=stiffness,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.25,
            ),
        ),
    )


# --------------------------------------------------------------------------- #
# core/rank.py: under- and over-constrained systems
# --------------------------------------------------------------------------- #


def test_a_rank_deficient_joint_set_is_refused_by_the_native_model_build() -> None:
    """
    ``diagnose_rank(...).overconstrained``: a duplicated joint adds rows the
    Jacobian cannot support, and the native build refuses the model.
    """
    model = _vertical_slider()
    doubled = model.model_copy(
        update={
            "joints": (
                *model.joints,
                AxleJoint(
                    name="guide_again",
                    kind="prismatic",
                    body_a="ground",
                    body_b="slider",
                    point_a_m=(0.0, 0.0, 0.0),
                    point_b_m=(0.0, 0.0, 0.0),
                ),
            )
        }
    )

    with pytest.raises(NativeAxleError, match="rank deficient"):
        run_axle_dynamics(doubled, _case("rank-deficient"))


def test_an_unconstrained_body_is_refused_as_an_equilibrium_and_left_free() -> None:
    """
    ``diagnose_rank(...).underconstrained``: nothing holds this body, so the
    static initializer cannot pin it -- and once it is given a consistent state
    the solver lets it fall rather than inventing a constraint.
    """
    model = AxleDynamicsModel(
        name="unconstrained",
        bodies=(
            _fixture(),
            AxleBody(
                name="free",
                mass_kg=10.0,
                inertia_kg_m2=_I3,
                position_m=(0.0, 0.0, 1.0),
            ),
        ),
        joints=(),
    )

    with pytest.raises(NativeAxleError, match="static equilibrium"):
        run_axle_dynamics(model, _case("unconstrained"))

    result = run_axle_dynamics(model, _case("unconstrained", consistent_state=True))
    state = result.body_state("free")
    np.testing.assert_allclose(
        state[0, 13:16], (0.0, 0.0, -_GRAVITY), rtol=0.0, atol=1e-12
    )
    assert state[-1, 2] < state[0, 2], "an unconstrained body was held in place"


# --------------------------------------------------------------------------- #
# core/reactions.py: reaction recovery and body equilibrium
# --------------------------------------------------------------------------- #


def test_a_revolute_reaction_recovers_the_gravity_load() -> None:
    """
    ``recover_reactions`` + ``body_equilibrium_wrench``: the reported constraint
    wrench is what balances the load, and the hanging body is at rest.
    """
    mass = 10.0
    model = AxleDynamicsModel(
        name="pendulum",
        bodies=(
            _fixture(),
            AxleBody(
                name="bob",
                mass_kg=mass,
                inertia_kg_m2=_I3,
                position_m=(0.0, 0.0, -0.5),
            ),
        ),
        joints=(
            AxleJoint(
                name="hinge",
                kind="revolute",
                body_a="ground",
                body_b="bob",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.5),
                axis_a=(0.0, 1.0, 0.0),
                axis_b=(0.0, 1.0, 0.0),
            ),
        ),
    )

    result = run_axle_dynamics(model, _case("hang"))
    wrench = result.joint_wrench_on_body_b("hinge")
    np.testing.assert_allclose(wrench[:, 2], mass * _GRAVITY, rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(wrench[:, [0, 1]], 0.0, atol=1e-9)
    np.testing.assert_allclose(wrench[:, 3:], 0.0, atol=1e-9)

    state = result.body_state("bob")
    np.testing.assert_allclose(state[:, 2], -0.5, rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(state[:, 7:13], 0.0, atol=1e-9)


# --------------------------------------------------------------------------- #
# model/mass.py: the mass matrix and the Newton-Euler bias
# --------------------------------------------------------------------------- #


def test_the_native_mass_matrix_uses_the_body_inertia_tensor() -> None:
    """
    ``body_mass_properties``/``mass_matrix``: the angular acceleration the solver
    produces is the applied torque divided by the body's own inertia.
    """
    angle = 0.04
    rate = 0.2
    stiffness = 5000.0
    damping = 50.0
    inertia_zz = 2.5
    model = AxleDynamicsModel(
        name="anti-roll-inertia",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            _fixture(),
            AxleBody(
                name="arm",
                mass_kg=10.0,
                inertia_kg_m2=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, inertia_zz)),
                quaternion_body_to_world=(
                    math.cos(0.5 * angle),
                    0.0,
                    0.0,
                    math.sin(0.5 * angle),
                ),
                angular_velocity_rad_per_s=(0.0, 0.0, rate),
            ),
        ),
        joints=(),
        anti_roll_bars=(
            AxleAntiRollBar(
                name="bar",
                body_a="ground",
                body_b="arm",
                axis_a=(0.0, 0.0, 1.0),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness_n_m_per_rad=stiffness,
                damping_n_m_s_per_rad=damping,
            ),
        ),
    )

    result = run_axle_dynamics(model, _case("anti-roll", consistent_state=True))
    torque = result.anti_roll_bar_state("bar")[0, 2]
    alpha_z = result.body_state("arm")[0, 18]

    assert torque == pytest.approx(-stiffness * angle - damping * rate)
    assert alpha_z == pytest.approx(torque / inertia_zz, rel=0.0, abs=1e-9)


def test_a_body_spinning_about_a_non_principal_axis_shows_the_bias_term() -> None:
    """
    ``spatial_bias_wrench``: ``I dw/dt = -omega x (I omega)``.  A free body with
    a diagonal but anisotropic inertia and a non-principal spin axis therefore
    changes its angular velocity while its kinetic energy stays put.
    """
    inertia = np.diag([1.0, 2.0, 3.0])
    omega = np.array([1.0, 0.0, 1.0])
    model = AxleDynamicsModel(
        name="free-spin",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            _fixture(),
            AxleBody(
                name="body",
                mass_kg=1.0,
                inertia_kg_m2=tuple(tuple(float(value) for value in row) for row in inertia),
                angular_velocity_rad_per_s=tuple(float(value) for value in omega),
            ),
        ),
        joints=(),
    )

    result = run_axle_dynamics(model, _case("spin", consistent_state=True))
    state = result.body_state("body")
    momentum = inertia @ state[0, 10:13]
    expected_alpha = -np.linalg.solve(inertia, np.cross(state[0, 10:13], momentum))

    np.testing.assert_allclose(expected_alpha, (0.0, 1.0, 0.0), rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(state[0, 16:19], expected_alpha, rtol=0.0, atol=1e-12)
    rotational = 0.5 * float(omega @ inertia @ omega)
    np.testing.assert_allclose(result.energy[:, 0], rotational, rtol=0.0, atol=1e-9)
    assert not np.allclose(
        state[-1, 10:13], state[0, 10:13]
    ), "the bias term did not act on the spin axis"
