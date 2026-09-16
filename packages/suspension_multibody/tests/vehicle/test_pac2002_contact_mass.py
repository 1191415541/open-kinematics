"""
Native PAC2002 advanced (non-linear) transient contact-mass modes 23/24.

The contact patch of USE_MODE 21-25 is its own body: mass ``mc`` on the carcass
springs ``cx``/``cy`` with dampers ``kx``/``ky``.  Adams documents the governing
equations in "Transient Behavior in PAC2002" (eqs 20-25).  The kernel carries the
contact body's in-plane deflection ``(u, v)`` and its rate as extra tire states.

The decisive property is that the extra layer may only change the *transient*:
at equilibrium ``u_dot = 0`` and ``cx*u = Fx``, so the patch slide velocity the
Magic Formula sees equals the rim's and the advanced modes have to reproduce the
linear transient modes they mirror (23 -> 13, 24 -> 14).  A second layer that
perturbs the steady state is wrong by construction, so that equality is asserted
here rather than merely "the run does not crash".
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import numpy as np
import pytest

from suspension_multibody.axle_dynamics.schema import PAC2002_PARAMETER_DEFAULTS
from suspension_multibody.pac2002_scope import (
    PAC2002_SUPPORTED_NATIVE_USE_MODES,
    pac2002_unsupported_native_reasons,
)
from suspension_multibody.schema import RoadSurfaceSpec, Vec3
from suspension_multibody.schema.dynamic import DynamicSolverSettings
from suspension_multibody.vehicle_dynamics import run_vehicle_dynamics


def _load_sibling_helpers():
    """
    Reuse the PAC2002 vehicle fixtures from ``test_native_vehicle``.

    pytest runs with ``--import-mode=importlib``, so a sibling test module is not
    importable by name; load it by path instead of duplicating the fixtures.
    """
    name = "_native_vehicle_helpers"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        name, pathlib.Path(__file__).with_name("test_native_vehicle.py")
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_helpers = _load_sibling_helpers()
_case = _helpers._case
_pac2002_model = _helpers._pac2002_model
_uniform_velocity_initial_states = _helpers._uniform_velocity_initial_states

# The contact-patch parameters the native kernel consumes.  IC/KP/CP (the yaw
# degree of freedom), the half contact length and the turn-slip/enveloping
# helpers stay fail-closed, so a tire that declares them cannot reach these modes.
_CONTACT_PATCH_COEFFICIENTS = (
    "MC",
    "KX",
    "KY",
    "CX",
    "CY",
    "CXZ1",
    "CXZ2",
    "CXX1",
    "CYZ1",
    "CYZ2",
    "CYY1",
    "PA1",
    "PA2",
    "IC",
    "KP",
    "CP",
)

# Step size for the degeneracy checks.  The advanced modes carry their own
# relaxation lengths (Eq3982-Eq3984), so at a *finite* step they legitimately
# track a changing slip differently from the linear modes -- that is the whole
# point of the model.  Only in the limit of a frozen kinematic slip must the two
# agree, and that limit is reached here: at 1e-4 s the difference is at machine
# precision, while at the 1e-3 s the rest of the suite uses it is ~19 N.
_DEGENERACY_STEP_S = 1.0e-4
_FINITE_STEP_S = 1.0e-3


@pytest.fixture(autouse=True)
def _use_the_spring_series_relaxation_length(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    Exercise the contact-body layer with the switch set to the spring-series length.

    These tests isolate the contact body (springs, dampers, yaw degree of freedom) and
    its degeneracy against the linear transient modes.  The synthetic fixture always
    runs at full longitudinal slip -- kappa = 1, because the wheel starts unspun and
    the initial-state API does not set the tire spin -- and that pins the *shipped*
    contact relaxation ``sigma_c = a*(1 - theta*zeta)`` at its floor, which makes the
    slip ODE ~4e5 1/s stiff.  The fixture's fixed step then cannot converge at any
    operating point: measured over vy = 0.5-5 m/s and wheel spin 0-50 rad/s, every
    combination fails on the first step with "Newton solve did not converge".

    So the layer is exercised with the other documented length (Eq3982-Eq3984), which
    is what the switch exists for, and the shipped convention is pinned where it can
    be measured: the Adams mode-23/24 step-steer and parking gates, which discriminate
    it (mode-24 parking lateral acceleration is 81.3 % NRMSE without sigma_c and
    13.5 % with it).
    """
    monkeypatch.setenv("PAC2002_CONTACT_MASS_RELAXATION_SIGMA_C", "0")


def _run(
    use_mode: int,
    step_s: float = _FINITE_STEP_S,
    vy_mm_s: float = 5_000.0,
    **overrides: float,
):
    extra = {
        name: float(PAC2002_PARAMETER_DEFAULTS[name])
        for name in _CONTACT_PATCH_COEFFICIENTS
        if name in PAC2002_PARAMETER_DEFAULTS
    }
    extra.update(overrides)
    extra["USE_MODE"] = float(use_mode)
    model = _pac2002_model(
        combined=True,
        parameter_source="adams_builtin",
        extra_coefficients=extra,
    )
    case = _case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(
                model, vy_mm_s=vy_mm_s
            ),
            "solver": DynamicSolverSettings(
                end_time=step_s,
                step_size=step_s,
                internal_step_size=step_s,
                min_internal_step_size=step_s,
                adaptive_substepping=False,
                integrator="generalized_alpha",
                gravity=Vec3(x=0, y=0, z=0),
            ),
        }
    )
    result = run_vehicle_dynamics(model, case)
    assert np.all(result.diagnostics.accepted)
    return result.axle.tire_output[-1]


def test_advanced_transient_modes_are_in_native_scope() -> None:
    assert 23 in PAC2002_SUPPORTED_NATIVE_USE_MODES
    assert 24 in PAC2002_SUPPORTED_NATIVE_USE_MODES
    coefficients = dict.fromkeys(_CONTACT_PATCH_COEFFICIENTS, 1.0)
    coefficients["USE_MODE"] = 24.0
    assert pac2002_unsupported_native_reasons(coefficients) == ()


def test_contact_patch_geometry_and_yaw_still_fail_closed() -> None:
    """
    A tire requesting what the kernel cannot consume must be rejected.

    EP/EP12/BF2/BP1 used to be on this list.  They are the turn-slip relaxation set of
    Eq3963-Eq3977, which the USE_MODE 25 kernel now evaluates, so they were moved out
    of the fail-closed set together with the mode itself (task D, step 71); what
    remains here is the non-point contact geometry (PB*/PBE/PCE/PLS and the enveloping
    ``N_*``/PAE helpers), which is still rejected.
    """
    coefficients = dict.fromkeys(_CONTACT_PATCH_COEFFICIENTS, 1.0)
    coefficients["USE_MODE"] = 24.0
    assert pac2002_unsupported_native_reasons(coefficients) == ()
    for name in (
        "N_WIDTH",
        "PAE",
        "PB1",
        "PB2",
        "PB3",
        "PBE",
        "PCE",
        "PLS",
        "BP3",
        "BP4",
    ):
        coefficients = dict.fromkeys(_CONTACT_PATCH_COEFFICIENTS, 1.0)
        coefficients["USE_MODE"] = 24.0
        coefficients[name] = 1.0
        reasons = pac2002_unsupported_native_reasons(coefficients)
        assert any(name in reason for reason in reasons), name


def test_yaw_degree_of_freedom_is_consumed_but_inert_at_mode_24() -> None:
    """
    IC/KP/CP are inside the native scope, and at mode 24 they do not move the
    forces.

    Adams' own mode-24 step steer is bit-identical when the turn-slip and parking
    coefficients are zeroed, but aborts with "NaN value detected in AsMath::step()"
    when IC/KP/CP are, so a mode-24 tire has to keep the yaw degree of freedom
    non-singular.  Its *effect* on the force channels is nevertheless nil: Eq26's
    ``- Vx*beta`` term belongs to turn-slip, which modes 23/24 do not have.
    Measured on the mode-24 Adams reference over the full 5 s step steer, feeding
    beta into the lateral slip target gives 5.63/2.17/25.5/22.6/28.0/23.2 % NRMSE
    where leaving it out gives 0.24/0.69/1.03/0.85/0.65/1.16 %.

    So the contract pinned here is: the coefficients are accepted, and changing the
    yaw stiffness -- with an aligning moment present to excite the yaw equation --
    leaves the tire forces alone.  Mode 25, where the yaw state *does* act on the
    slip, stays fail-closed.
    """
    coefficients = dict.fromkeys(_CONTACT_PATCH_COEFFICIENTS, 1.0)
    coefficients["USE_MODE"] = 24.0
    for name, value in (
        ("IC", 0.05), ("KP", 11.9), ("CP", 2019.0),
        ("QBZ1", 8.0), ("QDZ1", 0.1),
    ):
        coefficients[name] = value
    assert pac2002_unsupported_native_reasons(coefficients) == ()

    trail = {"QBZ1": 8.0, "QDZ1": 0.1}
    stiff = _run(24, step_s=_FINITE_STEP_S, CP=2019.0, **trail)
    soft = _run(24, step_s=_FINITE_STEP_S, CP=20.0, **trail)
    # The aligning moment must be present, otherwise the yaw equation is
    # unexcited and the inertness below would hold vacuously.
    assert np.max(np.abs(stiff[:, 14])) > 1.0e-6, "aligning moment is zero"
    for column in (4, 5, 6, 12, 13, 14):
        delta = float(np.max(np.abs(soft[:, column] - stiff[:, column])))
        assert delta < 1.0e-9, (column, delta)


def test_nonlinear_transient_half_contact_length_is_consumed() -> None:
    """PA1/PA2 feed Eq3978/3982-3984, so they are no longer blockers."""
    coefficients = dict.fromkeys(_CONTACT_PATCH_COEFFICIENTS, 1.0)
    coefficients["USE_MODE"] = 24.0
    coefficients["PA1"] = 0.4147
    coefficients["PA2"] = 1.9129
    assert pac2002_unsupported_native_reasons(coefficients) == ()


def test_load_and_slip_stiffness_corrections_are_consumed() -> None:
    """CXZ*/CXX*/CYZ*/CYY* now reach the contact body, so they are not blockers."""
    coefficients = dict.fromkeys(_CONTACT_PATCH_COEFFICIENTS, 1.0)
    coefficients["USE_MODE"] = 24.0
    assert pac2002_unsupported_native_reasons(coefficients) == ()


@pytest.mark.parametrize(("advanced", "linear"), [(23, 13), (24, 14)])
def test_stiffness_corrections_stay_out_of_the_steady_state(
    advanced: int, linear: int
) -> None:
    """
    The load/slip corrections scale the contact body's carcass spring only.

    They change the resting deflection (``u = Fx/Cx``) and the contact body's
    natural frequency, but never the slip the Magic Formula sees, because that
    depends on ``u_dot`` alone.  Wiring them into the slip path instead would move
    the steady state, so this is the guard that keeps them where they belong.
    """
    reference = _run(linear, step_s=_DEGENERACY_STEP_S)
    corrected = _run(
        advanced,
        step_s=_DEGENERACY_STEP_S,
        CXZ1=0.05,
        CXZ2=0.02,
        # The slip terms are scaled by (kappa*Fz0/Fz)^2, and this fixture runs at
        # ~200 N against a 4850 N reference load, so a physically sized cxx1
        # would make the contact spring ~100x stiffer than the step can resolve.
        # A small value still exercises the term.
        CXX1=1.0e-5,
        CYZ1=-0.05,
        CYZ2=0.02,
        CYY1=1.0e-5,
    )
    for column in (4, 5, 6, 12, 13, 14):
        delta = float(np.max(np.abs(corrected[:, column] - reference[:, column])))
        assert delta < 1.0e-9, (column, delta)


@pytest.mark.parametrize(
    ("advanced", "linear"), [(23, 13), (24, 14)]
)
def test_contact_mass_degenerates_to_the_linear_transient_mode(
    advanced: int, linear: int
) -> None:
    """
    At equilibrium the contact layer must vanish, so the advanced mode reduces to
    the linear one it mirrors.

    The layer-1 fixed point is ``sx = kappa`` regardless of the relaxation length,
    so with the slip frozen the layer-2 correction is zero and the two modes have
    to agree.  ``_DEGENERACY_STEP_S`` is small enough that the kinematic slip is
    effectively frozen; the measured difference there is at machine precision
    (~1e-12), which is what the 1e-9 bound pins.
    """
    reference = _run(linear, step_s=_DEGENERACY_STEP_S)
    candidate = _run(advanced, step_s=_DEGENERACY_STEP_S)
    for column in (4, 5, 6, 12, 13, 14):
        delta = float(
            np.max(np.abs(candidate[:, column] - reference[:, column]))
        )
        assert delta < 1.0e-9, (column, delta)


@pytest.mark.parametrize(
    ("advanced", "linear"), [(23, 13), (24, 14)]
)
def test_advanced_transient_tracks_a_changing_slip_differently(
    advanced: int, linear: int
) -> None:
    """
    Falsification for the test above: the two must *not* be identical in general.

    The advanced modes use the Eq3982-Eq3984 relaxation lengths instead of the
    tire file's pTx/pTy, so over a step long enough for the slip to move they
    answer differently.  If this ever stops holding, the non-linear relaxation
    length has been lost and the degeneracy test above has become vacuous.
    """
    reference = _run(linear, step_s=_FINITE_STEP_S)
    candidate = _run(advanced, step_s=_FINITE_STEP_S)
    delta = float(np.max(np.abs(candidate[:, 5] - reference[:, 5])))
    assert delta > 1.0e-3, delta


def test_contact_mass_modes_keep_the_axis_gating_of_their_linear_twins() -> None:
    """23 is "not combined" and 24 is "combined", exactly like 13 and 14."""
    not_combined = _run(23)
    combined = _run(24)
    assert np.max(np.abs(not_combined[:, 5])) > 1.0e-3
    assert np.max(np.abs(not_combined[:, 6])) > 1.0e-3
    assert np.max(np.abs(combined[:, 5])) > 1.0e-3
    assert np.max(np.abs(combined[:, 6])) > 1.0e-3
    assert np.max(np.abs(combined[:, 5] - not_combined[:, 5])) > 1.0e-3


# Columns 15-20 of the tire output are the contact-body states
# (u, u_dot, v, v_dot, beta, beta_dot); see axle_dynamics/result.py.
_CONTACT_BODY_COLUMNS = slice(15, 21)


@pytest.mark.parametrize("linear", [13, 14])
def test_linear_transient_modes_expose_no_contact_body(linear: int) -> None:
    """
    A mode without a contact body must expose exact zeros, not stale storage.

    The columns are written for every mode so a consumer can read them
    unconditionally, which only works if "this mode has no contact body" is
    distinguishable from "the contact body happens to be at rest".  Bitwise zero is
    what makes that distinction available; the advanced modes are checked below.
    """
    state = _run(linear)
    assert np.all(state[:, _CONTACT_BODY_COLUMNS] == 0.0)


@pytest.mark.parametrize("advanced", [23, 24])
def test_advanced_transient_modes_expose_the_moving_contact_body(advanced: int) -> None:
    """
    The second layer has to be observable, which is why these columns exist.

    Measured on this 1 ms fixture: mode 24 reports |u| = 1.8e-4 m, |v| = 5.4e-4 m
    and non-zero rates, so the threshold is three orders of magnitude below the
    measurement.  Before the columns existed "the contact body is active" could only
    be inferred from a force difference, which cannot tell a moving patch from a
    differently relaxed one.
    """
    state = _run(advanced)
    body = state[:, _CONTACT_BODY_COLUMNS]
    assert np.max(np.abs(body)) > 1.0e-6, body
    assert np.max(np.abs(state[:, 15])) > 1.0e-6  # longitudinal deflection
    assert np.max(np.abs(state[:, 17])) > 1.0e-6  # lateral deflection
