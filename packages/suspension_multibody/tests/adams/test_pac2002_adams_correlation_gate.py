"""
Automated Adams correlation gate for native PAC2002.

This is the gate the PAC2002 parity work depends on: the ``exact_pac2002``
manifest label is only meaningful while something in the repository re-runs the
native model against the Adams reference and fails when the error grows.

It regenerates the native side from the stored Adams source case and applies the
frozen thresholds in ``diagnose_native_pac2002_correlation``.  Thresholds are
deliberately not tuned here -- they live with the diagnostic script so changing
them shows up as a change to the gate, not to a test.

Everything is skipped when the Adams reference case or the Adams tire library is
absent, because the comparison cannot be constructed without them.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

_SOURCE_CASE = Path("artifacts/adams-full-source-2025_1_1/step_steer")
# The earlier case was produced by Adams 2024.1.  Native PAC2002 results against
# it are bit-identical to the 2025.1.1 case, so it remains a supported fallback
# for a checkout that only carries the older artifact.
_LEGACY_SOURCE_CASE = Path("artifacts/adams-full-source/step_steer")
_SCRIPTS = Path(__file__).parents[2] / "scripts"

# Modules referenced by the generated comparison manifest.
_REQUIRED_CONTRACT = {
    "same_initial_state_and_inputs": True,
    "native_steering_input": "prescribed_adams_rack_displacement",
    "native_wheel_torque_input": "direct_adams_drive_brake_replay",
}


def _load_script(name: str, module_name: str):
    """Import a comparison script by path (they are not packaged modules)."""
    path = _SCRIPTS / name
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def comparison(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the native PAC2002 comparison once for every test in the module."""
    if not _SOURCE_CASE.is_dir():
        pytest.skip(f"Adams source case unavailable: {_SOURCE_CASE}")

    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        "pac2002_gate_comparison",
    )
    root = tmp_path_factory.mktemp("pac2002_correlation")
    comparison_module.generate(
        _SOURCE_CASE,
        root,
        end_time=5.0,
        output_step=0.01,
        internal_step=0.01,
        tire_kinds=("pac2002",),
    )
    return root


@pytest.fixture(scope="module")
def legacy_comparison(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the same comparison against the Adams 2024.1 case, if present."""
    if not _LEGACY_SOURCE_CASE.is_dir():
        pytest.skip(f"legacy Adams source case unavailable: {_LEGACY_SOURCE_CASE}")

    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        "pac2002_gate_comparison_legacy",
    )
    root = tmp_path_factory.mktemp("pac2002_correlation_legacy")
    comparison_module.generate(
        _LEGACY_SOURCE_CASE,
        root,
        end_time=5.0,
        output_step=0.01,
        internal_step=0.01,
        tire_kinds=("pac2002",),
    )
    return root


def _native_channels(root: Path) -> dict[str, list[float]]:
    payload = json.loads((root / "native_pac2002_time_history.json").read_text("utf-8"))
    return payload["channels"]


def test_adams_release_does_not_change_native_results(
    comparison: Path, legacy_comparison: Path
) -> None:
    """
    Adding Adams 2025.1.1 to the machine must not move the native results.

    The 2024.1 and 2025.1.1 references for this maneuver agree exactly, and the
    native run against each is bit-identical.  Pinning that here keeps the
    equivalence claim release-independent: if a future Adams release changes the
    PAC2002 physics, this fails and the gate has to name the release it covers.
    """
    new = _native_channels(comparison)
    old = _native_channels(legacy_comparison)

    assert set(new) == set(old), "native channel sets differ between references"
    differing = [
        name
        for name in sorted(new)
        if not np.array_equal(np.asarray(new[name]), np.asarray(old[name]))
    ]
    assert differing == [], (
        "native state changed between the Adams 2024.1 and 2025.1.1 references "
        f"for these channels: {differing}"
    )


def test_native_pac2002_stays_inside_frozen_adams_thresholds(comparison: Path) -> None:
    """Native PAC2002 must keep matching the Adams reference within the gate."""
    diagnostic = _load_script(
        "diagnose_native_pac2002_correlation.py",
        "pac2002_gate_diagnostic",
    )

    result = diagnostic.diagnose(comparison, split_time_s=1.0)
    limits = diagnostic.DEFAULT_LIMITS_PERCENT
    observed = {
        **{
            component: result["force"][component]["combined"]["nrmse_percent"]
            for component in ("normal_force", "longitudinal_force", "lateral_force")
        },
        **{
            channel: result["handling"][channel]["nrmse_percent"]
            for channel in ("lateral_acceleration", "yaw_rate", "body_roll")
        },
    }

    failures = {
        name: value for name, value in observed.items() if value > limits[name]
    }
    assert failures == {}, (
        "native PAC2002 left the frozen Adams correlation envelope: "
        f"{ {k: round(v, 4) for k, v in failures.items()} } "
        f"(limits {limits}, observed { {k: round(v, 4) for k, v in observed.items()} })"
    )


def test_comparison_manifest_declares_the_diagnostic_contract(comparison: Path) -> None:
    """
    The manifest must satisfy the contract the diagnostic gate validates.

    ``tire_force_coordinates`` previously said
    ``adams_fiala_or_pac2002_tire_iso_output`` while the diagnostic expected
    ``pac2002_tire_iso_output``, so the gate could never pass.  Pin both the
    manifest field and the per-tire history metadata.
    """
    manifest = json.loads((comparison / "comparison_manifest.json").read_text("utf-8"))

    assert manifest["contract"] == "full-native-model-comparison-v2"
    assert manifest["tire_force_coordinates"] == "pac2002_tire_iso_output"
    for key, expected in _REQUIRED_CONTRACT.items():
        assert manifest[key] == expected, f"manifest contract key {key!r}"

    history = json.loads(
        (comparison / "native_pac2002_time_history.json").read_text("utf-8")
    )
    metadata = history["metadata"]
    assert metadata["tire_force_coordinates"] == "pac2002_tire_iso_output"
    assert metadata["complete_vehicle_reference"] is True


def test_reference_tire_requests_no_unimplemented_feature() -> None:
    """
    The covered reference case must not rely on a fail-closed feature.

    If this ever fails, the correlation gate above is comparing a model that was
    accepted only because a scope check regressed.
    """
    from suspension_multibody.adams.full_vehicle_model import (
        load_adams_full_vehicle_input,
    )
    from suspension_multibody.kernel.capabilities import (
        pac2002_unsupported_native_reasons,
    )

    if not _SOURCE_CASE.is_dir():
        pytest.skip(f"Adams source case unavailable: {_SOURCE_CASE}")

    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    assert data.pac2002_coefficients.get("USE_MODE") == pytest.approx(14.0)
    assert pac2002_unsupported_native_reasons(data.pac2002_coefficients) == ()


# ---------------------------------------------------------------------------
# USE_MODE 23 and 24 (advanced transient contact mass)
# ---------------------------------------------------------------------------
#
# The mode-14 case above cannot exercise the contact-body layer: its tire carries
# no contact-mass coefficients at all.  This second gate covers the advanced
# transient modes against references built for them.
#
# Both modes need their own reference: they differ in the combined-slip axis gating
# (23 is "not combined", 24 is "combined"), exactly like 13 and 14, so one cannot
# gate the other.  Each reference is the stock parking tire cloned to the mode with
# the fourteen coefficients the native scope rejects *and* that modes 23/24 provably
# do not use zeroed -- EP/EP12/BF2/BP1/BP2 (the turn-slip relaxation set) and the
# nine spin / parking torque terms.  Zeroing them was measured on Adams to leave the
# mode-24 step steer bit-identical to the stock reference, which is what makes the
# two sides comparable at all: IC/KP/CP cannot be zeroed, so without this clone no
# tire could be both Adams-solvable and inside the native scope.
#
# The comparison runs at the same 10 ms internal step as the mode-14 gate.  The
# contact-body oscillator (~105 Hz) made that step fail outright before; it is
# integrable now because the fixed-step path halves a step that Newton cannot
# solve (the floor has to be below the nominal step for that to be possible, which
# is why the comparison no longer pins the floor to the step).
_CONTACT_MASS_REFERENCES: dict[int, tuple[Path, Path]] = {
    23: (
        Path("artifacts/adams-mode-ref/um23-native"),
        Path(
            "artifacts/adams-mode-ref/clones/"
            "pac2002_205_55R16_parking_um23-native.tir"
        ),
    ),
    24: (
        Path("artifacts/adams-mode-ref/um24-native"),
        Path("artifacts/adams-mode-ref/clones/pac2002_um24_native_scope.tir"),
    ),
}
_MODE24_LIMITS_PERCENT = {
    "normal_force": 3.0,
    "longitudinal_force": 50.0,
    "lateral_force": 20.0,
    "lateral_acceleration": 50.0,
    "yaw_rate": 20.0,
    "body_roll": 50.0,
}


@pytest.fixture(scope="module", params=sorted(_CONTACT_MASS_REFERENCES))
def contact_mass_comparison(
    request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory
) -> tuple[int, Path]:
    """Generate the native comparison for one advanced-transient mode."""
    use_mode = request.param
    reference, tire = _CONTACT_MASS_REFERENCES[use_mode]
    if not reference.is_dir() or not tire.is_file():
        pytest.skip(
            f"advanced-transient reference unavailable for USE_MODE {use_mode}: "
            f"{reference} / {tire}"
        )

    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        f"pac2002_gate_comparison_mode{use_mode}",
    )
    root = tmp_path_factory.mktemp(f"pac2002_correlation_mode{use_mode}")
    comparison_module.generate(
        reference,
        root,
        end_time=5.0,
        output_step=0.01,
        internal_step=0.01,
        tire_kinds=("pac2002",),
        pac2002_tire_file=tire,
    )
    return use_mode, root


def test_contact_mass_modes_stay_inside_frozen_adams_thresholds(
    contact_mass_comparison: tuple[int, Path],
) -> None:
    """
    Native USE_MODE 23/24 must keep matching their Adams advanced-transient references.

    Measured with the shipped contact relaxation: mode 23 gives normal 0.07 %,
    longitudinal 0.64 %, lateral 0.23 %, lateral acceleration 0.13 %, yaw rate
    0.12 %, body roll 0.61 %; mode 24 gives 0.07/0.68/0.22/0.13/0.13/0.62 %.  The
    earlier spring-series relaxation length gave 0.24/0.64/1.04/0.86/0.65/1.19 and
    0.24/0.69/1.04/0.86/0.65/1.20 -- also inside the envelope, which is exactly why
    the parking gate below exists: this maneuver cannot tell the two apart.
    """
    use_mode, root = contact_mass_comparison
    diagnostic = _load_script(
        "diagnose_native_pac2002_correlation.py",
        f"pac2002_gate_diagnostic_mode{use_mode}",
    )

    result = diagnostic.diagnose(root, split_time_s=1.0)
    observed = {
        **{
            component: result["force"][component]["combined"]["nrmse_percent"]
            for component in ("normal_force", "longitudinal_force", "lateral_force")
        },
        **{
            channel: result["handling"][channel]["nrmse_percent"]
            for channel in ("lateral_acceleration", "yaw_rate", "body_roll")
        },
    }
    baseline = diagnostic.DEFAULT_LIMITS_PERCENT
    failures = {
        name: value
        for name, value in observed.items()
        if value > baseline[name]
    }
    assert failures == {}, (
        f"native USE_MODE {use_mode} left the frozen Adams correlation envelope: "
        f"{ {k: round(v, 4) for k, v in failures.items()} } "
        f"(limits {baseline}, observed "
        f"{ {k: round(v, 4) for k, v in observed.items()} })"
    )
    # The envelope is deliberately the mode-14 one; record it so a future change to
    # either set is a visible decision rather than an accident.
    assert _MODE24_LIMITS_PERCENT == baseline
    _assert_contact_body_is_active(root, use_mode)


def _assert_contact_body_is_active(
    root: Path,
    use_mode: int,
    lateral_threshold: float = 1.0e-3,
    yaw_threshold: float = 1.0e-3,
) -> None:
    """
    Assert the stable multi-step transient case shows the second layer moving.

    The gate above can only see forces, so an accidentally inert contact layer would
    pass it while being indistinguishable from the linear transient mode.  The tire
    output now carries the contact-body states, and the thresholds are set an order of
    magnitude below what the maneuver measures:

    * step steer -- 7.9-22 mm of lateral deflection and 8.6-53 mrad of contact-body yaw
      per wheel (thresholds 1 mm / 1 mrad);
    * parking steer -- 0.22-0.26 mm and 0.33-0.64 mrad, because the maneuver is slow
      (thresholds 0.1 mm / 0.1 mrad, passed in by the caller).

    A dead layer reads exactly zero in both cases, so the margin is what makes the
    assertion meaningful rather than the exact value.
    """
    reference = json.loads(
        (root / "adams_pac2002_time_history.json").read_text("utf-8")
    )["channels"]
    native = json.loads(
        (root / "native_pac2002_time_history.json").read_text("utf-8")
    )["channels"]
    samples = len(reference["yaw_rate"])
    assert samples > 100, samples  # "multi-step" is the point of this case
    for wheel in ("front_left", "front_right", "rear_left", "rear_right"):
        for name, threshold in (
            ("contact_body_lateral_m", lateral_threshold),
            ("contact_body_yaw_rad", yaw_threshold),
        ):
            key = f"{wheel}.tire_{name}"
            values = native[key]
            assert len(values) == samples, (use_mode, key, len(values))
            span = max(values) - min(values)
            assert span > threshold, (use_mode, key, span)


@pytest.mark.parametrize("use_mode", sorted(_CONTACT_MASS_REFERENCES))
def test_contact_mass_reference_tire_is_inside_the_native_scope(
    use_mode: int,
) -> None:
    """The reference tire must not be accepted only because a scope check regressed."""
    from suspension_multibody.adams.full_vehicle_model import _parse_tire
    from suspension_multibody.kernel.capabilities import (
        pac2002_unsupported_native_reasons,
    )

    _, tire = _CONTACT_MASS_REFERENCES[use_mode]
    if not tire.is_file():
        pytest.skip(f"advanced-transient reference tire unavailable: {tire}")

    coefficients = _parse_tire(tire)
    assert coefficients.get("USE_MODE") == pytest.approx(float(use_mode))
    assert pac2002_unsupported_native_reasons(coefficients) == ()


# ---------------------------------------------------------------------------
# USE_MODE 24 on the parking maneuver (which relaxation length ships)
# ---------------------------------------------------------------------------
#
# The step-steer gates above cannot see which of the two documented slip relaxation
# lengths the advanced transient modes use: over 5 s of steering they agree to a few
# tenths of a percent, and switching the shipped convention moves the mode-24 gate
# from 0.24/0.69/1.04 % to 0.07/0.68/0.22 % -- better, but inside the envelope either
# way.  The low-speed parking maneuver is where they separate, because the contact
# relaxation sigma_c = a*(1 - theta*zeta) shortens with slip while the spring-series
# length |CF|/C + a does not.
#
# Measured native NRMSE against the mode-24 parking reference: the spring-series
# length gives 81.3/67.5/88.9 % on lateral acceleration / yaw rate / body roll
# (outside the frozen envelope), sigma_c gives 13.5/12.8/24.1 % (inside).  This gate
# is therefore the evidence for the shipped convention, not just a regression test.
#
# The per-wheel force channels are deliberately *not* gated: their native
# disagreement is ~73 % at both conventions, so they measure something else (the
# still-open mode-25 item) and a threshold here would only encode that gap.
# The comparison runs at a 2 ms step because the parking maneuver's high steering
# rate collapses the contact relaxation; at 10 ms the mode-25 side of that work does
# not converge.
#
# The reference is the same parking maneuver run with the fourteen inert coefficients
# zeroed, so the native side can load the tire at all.  Measured: zeroing them leaves
# the parking result *bit-identical* to the stock reference (all four handling channels
# agree to 0.0), which is the same inertness step 25 measured on the step steer.
_PARKING_REFERENCE = Path("artifacts/adams-mode-ref/um24-parking_steer-native")
_PARKING_TIRE = Path(
    "artifacts/adams-mode-ref/clones/"
    "pac2002_205_55R16_parking_um24-parking_steer-native.tir"
)
_PARKING_LIMITS_PERCENT = {
    "normal_force": 3.0,
    "lateral_force": 20.0,
    "lateral_acceleration": 50.0,
    "yaw_rate": 20.0,
    "body_roll": 50.0,
}


@pytest.fixture(scope="module")
def parking_comparison(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the native mode-24 comparison against the parking reference."""
    if not _PARKING_REFERENCE.is_dir() or not _PARKING_TIRE.is_file():
        pytest.skip(
            "parking reference unavailable: "
            f"{_PARKING_REFERENCE} / {_PARKING_TIRE}"
        )
    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        "pac2002_gate_comparison_parking24",
    )
    root = tmp_path_factory.mktemp("pac2002_correlation_parking24")
    comparison_module.generate(
        _PARKING_REFERENCE,
        root,
        end_time=4.0,
        output_step=0.01,
        internal_step=0.002,
        tire_kinds=("pac2002",),
        pac2002_tire_file=_PARKING_TIRE,
    )
    return root


def _assert_axle_lateral_forces_match(root: Path) -> None:
    """
    Assert the axle-level lateral forces agree, which the per-wheel channels cannot.

    The Adams per-wheel rear channels carry a large anti-symmetric component (each leg
    spans ~173 N on this maneuver while the two legs *sum* to 48 N, and the native's
    legs are 29-37 N), so per-wheel comparison exaggerates the difference; the axle
    sum is what the vehicle actually feels and it is mapping-robust (a left/right swap
    cannot change it).  Measured here: front axle r = +0.999 with ratio 0.993, rear
    axle r = +0.996 with ratio 0.977.

    The same measurement also shows the harness's ``til``/``tir`` request prefixes are
    left/right swapped against this model's wheel labels -- native ``front_left``
    correlates +0.995 with the ``tir`` channel and +0.855 with ``til`` -- which is why
    the per-wheel names are not used for the assertion.
    """
    adams = json.loads(
        (root / "adams_pac2002_time_history.json").read_text("utf-8")
    )["channels"]
    native = json.loads(
        (root / "native_pac2002_time_history.json").read_text("utf-8")
    )["channels"]
    for axle, wheels in (
        ("front", ("front_left", "front_right")),
        ("rear", ("rear_left", "rear_right")),
    ):
        reference = np.asarray(
            [adams[f"{wheel}.tire_lateral_force"] for wheel in wheels], dtype=float
        ).sum(axis=0)
        candidate = np.asarray(
            [native[f"{wheel}.tire_lateral_force"] for wheel in wheels], dtype=float
        ).sum(axis=0)
        correlation = float(np.corrcoef(reference, candidate)[0, 1])
        ratio = (candidate.max() - candidate.min())/(reference.max() - reference.min())
        assert correlation > 0.99, (axle, correlation)
        assert 0.9 < ratio < 1.1, (axle, ratio)


def test_parking_steer_pins_the_shipped_contact_relaxation(
    parking_comparison: Path,
) -> None:
    """
    Native USE_MODE 24 must match the Adams parking reference on the handling channels.

    Measured when frozen: normal 0.09 %, lateral force 16.8 %, lateral acceleration
    6.7 %, yaw rate 2.9 %, body roll 15.5 %.  The lateral-force channel is what makes
    this gate discriminate the low-speed slip convention: the tire file's VXLOW (2 m/s
    here, against a 1 m/s maneuver) used to floor the slip denominator, which measured
    73.4 % on that channel and left the rear axle's lateral force and aligning moment
    anti-phase (r = -0.78/-0.71); dividing by the true |Vx| gives 16.8 % and
    +0.85/+0.89, and improves every other channel too.  The alignment envelope is the
    mode-14 one for the handling channels.

    Still outside the envelope and deliberately not gated: the rear axle's lateral
    force *magnitude* (0.17x Adams even with the sign correct) and the aligning moment
    aggregate (86 %).  Both are known open items; gating them now would freeze a defect
    rather than prevent one.
    """
    diagnostic = _load_script(
        "diagnose_native_pac2002_correlation.py",
        "pac2002_gate_diagnostic_parking24",
    )
    result = diagnostic.diagnose(parking_comparison, split_time_s=1.0)
    observed = {
        **{
            component: result["force"][component]["combined"]["nrmse_percent"]
            for component in ("normal_force", "lateral_force")
        },
        **{
            channel: result["handling"][channel]["nrmse_percent"]
            for channel in ("lateral_acceleration", "yaw_rate", "body_roll")
        },
    }
    baseline = diagnostic.DEFAULT_LIMITS_PERCENT
    failures = {
        name: value
        for name, value in observed.items()
        if value > _PARKING_LIMITS_PERCENT[name]
    }
    assert failures == {}, (
        "native USE_MODE 24 left the parking envelope: "
        f"{ {k: round(v, 4) for k, v in failures.items()} } "
        f"(limits {_PARKING_LIMITS_PERCENT}, observed "
        f"{ {k: round(v, 4) for k, v in observed.items()} })"
    )
    # The three handling limits are the shared envelope; record that they are not a
    # parking-specific relaxation of it.
    assert _PARKING_LIMITS_PERCENT == {
        name: baseline[name] for name in _PARKING_LIMITS_PERCENT
    }
    _assert_contact_body_is_active(
        parking_comparison, 24, lateral_threshold=1.0e-4, yaw_threshold=1.0e-4
    )
    _assert_axle_lateral_forces_match(parking_comparison)


# ---------------------------------------------------------------------------
# USE_MODE 25 on the parking maneuver (turn slip and the parking torque)
# ---------------------------------------------------------------------------
#
# Mode 25 is the nonlinear-transient mode *with* turn-slip and parking modelling, and
# it is the only mode that reproduces the stand-still parking torque -- which is why
# it exists (Adams ``Parking_Torque``: "the maximum parking torque is mainly
# determined by parameter qCrj1, while the stiffness is due to the yaw stiffness cy
# value").  It is fail-closed in ``pac2002_scope`` until this gate exists and passes.
#
# What actually needs gating is not the aggregate error.  Measured on this maneuver
# (native against the ``um25-parking_steer`` reference, 1.2 s = one steering cycle at
# a 2 ms step, mode 24 for contrast):
#
#   * the axle **sums** of lateral force and aligning moment are close in *both*
#     modes (mode 25: r = 0.998/0.992 lateral, 0.994/0.956 aligning, ratio
#     0.86-1.52), and the (25 - 24) increment of the front aligning moment
#     correlates +0.9966 with Adams' own increment at amplitude 0.80 -- so the
#     parking torque itself is right;
#   * the axle **left-minus-right difference** is where the two modes differ.  Mode 24
#     under-produces it tenfold (ratios 0.10 and 0.23 against Adams); mode 25 produces
#     it at 0.46-0.71 with the right shape.
#
# So the assertions below pin the antisymmetric content, which is the part a
# left/right channel mix-up cannot hide and which no aggregate NRMSE reports.
_MODE25_REFERENCE = Path("artifacts/adams-mode-ref/um25-parking_steer")
_MODE25_TIRE = Path(
    "artifacts/adams-mode-ref/clones/"
    "pac2002_205_55R16_parking_um25-parking_steer.tir"
)
# The mode-24 clone of the same parking tire, used as the control for "which model
# reproduces Adams' mode 25".
_MODE24_ON_MODE25_TIRE = Path(
    "artifacts/adams-mode-ref/clones/"
    "pac2002_205_55R16_parking_um24-parking_steer-native.tir"
)
# Mode 25 must fit the same frozen envelope mode 24 does: being inside the declared
# native scope is exactly the claim that it does not need a wider one.
_MODE25_LIMITS_PERCENT = _PARKING_LIMITS_PERCENT
_AXLES = {
    "front": ("front_left", "front_right"),
    "rear": ("rear_left", "rear_right"),
}
# (minimum correlation, minimum ratio, maximum ratio) of candidate over reference.
#
# Every entry here is a criterion that USE_MODE 24 *cannot* meet and USE_MODE 25 does,
# which is what makes the gate a statement about the capability mode 25 adds rather
# than a restatement of what mode 24 already did.  Measured on the same maneuver:
#
#   key                        mode 24        mode 25 (shipped)   requirement
#   front.aligning_moment      +0.954/1.580   +0.994/0.881        r>=0.9, 0.7..1.3
#   front.lateral_force.diff   -0.686/0.306   +0.797/2.392        r>0.5, 0.25..4.0
#   front.aligning_moment.diff -0.714/0.608   +0.791/3.352        r>0.5, 0.25..4.0
#   rear.lateral_force.diff    +0.931/0.103   +0.996/0.459        r>0.5, 0.25..4.0
#   rear.aligning_moment.diff  +0.963/0.231   +0.958/0.710        r>0.5, 0.25..4.0
#
# The front's antisymmetric correlation (0.79) is the weakest value the shipped model
# reaches and is a *known open item*: it is not a threshold picked to pass, it is the
# smallest bound that still rejects mode 24's negative correlation.  Tightening it is
# tracked in tasks/D/TODO.csv D23.
#
# The rear axle's aligning-moment *sum* is deliberately not gated: it misses by the
# same factor in both modes (2.103 in mode 24 against 1.578 in mode 25), so gating it
# would freeze a mode-independent gap rather than prevent a regression -- the same
# reason the mode-24 gate above leaves the rear lateral magnitude ungated.
_MODE25_AXLE_STATISTICS = {
    "front.aligning_moment": (0.9, 0.7, 1.3),
    "front.lateral_force.diff": (0.5, 0.25, 4.0),
    "front.aligning_moment.diff": (0.5, 0.25, 4.0),
    "rear.lateral_force.diff": (0.5, 0.25, 4.0),
    "rear.aligning_moment.diff": (0.5, 0.25, 4.0),
}


def _axle_series(channels: dict, key: str, wheels: tuple[str, str]) -> np.ndarray:
    """
    Axle sum, or left-minus-right difference, of one tire channel.

    ``key`` is ``"<component>"`` for the sum or ``"<component>.diff"`` for the
    antisymmetric channel.  The sum is what the vehicle feels and is what a left/right
    channel mix-up cannot corrupt; the difference is the antisymmetric part, which on a
    single-frequency steering maneuver is collinear with the sum, so only its magnitude
    and sign separate the two.
    """
    difference = key.endswith(".diff")
    component = key[: -len(".diff")] if difference else key
    left, right = wheels
    left_series = np.asarray(channels[f"{left}.tire_{component}"], dtype=float)
    right_series = np.asarray(channels[f"{right}.tire_{component}"], dtype=float)
    return left_series - right_series if difference else left_series + right_series


def _mode25_axle_statistics(root: Path, key: str) -> tuple[float, float]:
    """Return (correlation, candidate span / reference span) for one axle key."""
    adams = json.loads(
        (root / "adams_pac2002_time_history.json").read_text("utf-8")
    )["channels"]
    native = json.loads(
        (root / "native_pac2002_time_history.json").read_text("utf-8")
    )["channels"]
    axle, _, component = key.partition(".")
    reference = _axle_series(adams, component, _AXLES[axle])
    candidate = _axle_series(native, component, _AXLES[axle])
    span = float(reference.max() - reference.min())
    ratio = float(candidate.max() - candidate.min())/span if span > 0.0 else float("nan")
    if reference.std() <= 0.0 or candidate.std() <= 0.0:
        return (float("nan"), ratio)
    return (float(np.corrcoef(reference, candidate)[0, 1]), ratio)


@pytest.fixture(scope="module")
def mode25_parking_comparison(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the native USE_MODE 25 comparison against the parking reference."""
    if not _MODE25_REFERENCE.is_dir() or not _MODE25_TIRE.is_file():
        pytest.skip(
            "mode-25 parking reference unavailable: "
            f"{_MODE25_REFERENCE} / {_MODE25_TIRE}"
        )
    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        "pac2002_gate_comparison_parking25",
    )
    root = tmp_path_factory.mktemp("pac2002_correlation_parking25")
    comparison_module.generate(
        _MODE25_REFERENCE,
        root,
        end_time=1.2,
        output_step=0.01,
        internal_step=0.002,
        tire_kinds=("pac2002",),
        pac2002_tire_file=_MODE25_TIRE,
    )
    return root


def test_parking_steer_pins_the_turn_slip_and_parking_torque(
    mode25_parking_comparison: Path,
) -> None:
    """
    Native USE_MODE 25 must reproduce the parking torque without leaving the envelope.

    Measured when frozen (1.2 s = one steering cycle, 2 ms step): the aggregate
    channels are normal 0.11 %, lateral 13.4 %, lateral acceleration 11.7 %, yaw rate
    11.7 %, body roll 15.9 %; the axle statistics are front aligning-moment sum
    r = +0.994 at amplitude 0.88, and the antisymmetric left-minus-right channels
    front/rear lateral r = +0.80/+1.00 at ratio 2.4/0.46, front/rear aligning moment
    r = +0.79/+0.96 at ratio 3.4/0.71.

    Mode 24 cannot express any of the antisymmetric part: its ratios there are
    0.31/0.10 (lateral) and 0.61/0.23 (aligning), with the front's correlation
    *negative* (-0.686, -0.714).  The convergence loop that produced these numbers is
    in ``tasks/D/TURN_SLIP_PARKING_NOTES.md`` steps 68-70; the fix that moved the
    needle was making Eq3967's ``beta_st`` read the same relaxed slip the yaw equation
    is driven by, which alone took the lateral-force aggregate from 47.3 % (a breach)
    to 13.4 % and turned the rear's antisymmetric correlation from -0.98 to +1.00.
    """
    diagnostic = _load_script(
        "diagnose_native_pac2002_correlation.py",
        "pac2002_gate_diagnostic_parking25",
    )
    result = diagnostic.diagnose(mode25_parking_comparison, split_time_s=1.0)
    observed = {
        **{
            component: result["force"][component]["combined"]["nrmse_percent"]
            for component in ("normal_force", "longitudinal_force", "lateral_force")
        },
        **{
            channel: result["handling"][channel]["nrmse_percent"]
            for channel in ("lateral_acceleration", "yaw_rate", "body_roll")
        },
    }
    failures = {
        name: value
        for name, value in observed.items()
        if name in _MODE25_LIMITS_PERCENT
        and value > _MODE25_LIMITS_PERCENT[name]
    }
    assert failures == {}, (
        "native USE_MODE 25 left the parking envelope: "
        f"{ {k: round(v, 4) for k, v in failures.items()} } "
        f"(limits {_MODE25_LIMITS_PERCENT}, observed "
        f"{ {k: round(v, 4) for k, v in observed.items()} })"
    )

    statistics = {
        key: _mode25_axle_statistics(mode25_parking_comparison, key)
        for key in _MODE25_AXLE_STATISTICS
    }
    bad = {
        key: {"r": round(statistics[key][0], 4), "ratio": round(statistics[key][1], 4)}
        for key, (minimum_r, low, high) in _MODE25_AXLE_STATISTICS.items()
        if not (
            statistics[key][0] >= minimum_r
            and low <= statistics[key][1] <= high
        )
    }
    assert bad == {}, (
        "native USE_MODE 25 lost the axle-level turn-slip content "
        f"(requirements {_MODE25_AXLE_STATISTICS}, observed {bad})"
    )


@pytest.fixture(scope="module")
def mode24_on_the_mode25_reference(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """
    Solve USE_MODE 24 against the *mode-25* reference.

    Comparing mode 25's error against ``um25`` with mode 24's error against ``um24``
    compares two different targets, so neither number says one model is worse.  This
    run is what answers that question, and it is the run the assertion below uses.
    """
    if not _MODE25_REFERENCE.is_dir() or not _MODE24_ON_MODE25_TIRE.is_file():
        pytest.skip(
            "mode-24 tire or mode-25 reference unavailable: "
            f"{_MODE24_ON_MODE25_TIRE} / {_MODE25_REFERENCE}"
        )
    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        "pac2002_gate_comparison_parking24_on_25",
    )
    root = tmp_path_factory.mktemp("pac2002_correlation_parking24_on_25")
    comparison_module.generate(
        _MODE25_REFERENCE,
        root,
        end_time=1.2,
        output_step=0.01,
        internal_step=0.002,
        tire_kinds=("pac2002",),
        pac2002_tire_file=_MODE24_ON_MODE25_TIRE,
    )
    return root


def test_mode25_beats_mode24_against_the_same_reference(
    mode25_parking_comparison: Path,
    mode24_on_the_mode25_reference: Path,
) -> None:
    """
    Against Adams' mode-25 parking maneuver, mode 25 must be the more accurate model.

    This is the sound form of "mode 25 must not be worse than mode 24".  The unsound
    form compares mode 25's error against ``um25`` with mode 24's against ``um24``;
    measured, mode 24 scores better on lateral acceleration (7.4 % against 11.7 %) and
    yaw rate (2.4 % against 11.7 %) that way, but only because Adams' own turn slip
    barely moves those channels -- its (25 - 24) increment is 4.3 % and 3.5 % of the
    signal -- so a model that ignores turn slip altogether is trivially close.  Against
    the *same* reference, mode 25 wins where turn slip governs:

      channel                    mode 24        mode 25
      front.aligning_moment      +0.824/0.187   +0.994/0.881
      normal_force               0.149 %        0.113 %
      lateral_force              17.55 %        13.35 %
      body_roll                  18.62 %        15.95 %
      rear lateral difference    +0.931/0.096   +0.996/0.459

    Mode 24 produces 19 % of Adams' front-axle parking torque; mode 25 produces 88 %.
    """
    diagnostic = _load_script(
        "diagnose_native_pac2002_correlation.py",
        "pac2002_gate_diagnostic_parking24_on_25",
    )
    mode25 = diagnostic.diagnose(mode25_parking_comparison, split_time_s=1.0)
    mode24 = diagnostic.diagnose(mode24_on_the_mode25_reference, split_time_s=1.0)

    # The parking torque: the front axle's aligning moment is the channel that carries
    # it, and the mode-24 run is the control that proves the assertion has content.
    mode25_torque = _mode25_axle_statistics(
        mode25_parking_comparison, "front.aligning_moment"
    )
    mode24_torque = _mode25_axle_statistics(
        mode24_on_the_mode25_reference, "front.aligning_moment"
    )
    assert mode24_torque[1] < 0.5, (
        "the mode-24 control now reproduces Adams' parking torque, so this comparison "
        f"no longer separates the two modes (ratio {mode24_torque[1]:.4f})"
    )
    assert mode25_torque[0] > mode24_torque[0] + 0.1, (
        "mode 25 must track the parking torque better than mode 24 does against the "
        f"same reference (r {mode25_torque[0]:.4f} against {mode24_torque[0]:.4f})"
    )
    assert mode25_torque[1] > mode24_torque[1] + 0.3, (
        "mode 25 must produce more of Adams' parking-torque amplitude than mode 24 "
        f"(ratio {mode25_torque[1]:.4f} against {mode24_torque[1]:.4f})"
    )

    # And the channels turn slip governs at the vehicle level, also on the same
    # reference.
    for channel in ("normal_force", "lateral_force"):
        better = mode25["force"][channel]["combined"]["nrmse_percent"]
        worse = mode24["force"][channel]["combined"]["nrmse_percent"]
        assert better < worse, (
            f"mode 25 is farther from Adams' mode-25 reference on {channel} than mode "
            f"24 is ({better:.4f} % against {worse:.4f} %)"
        )
    roll25 = mode25["handling"]["body_roll"]["nrmse_percent"]
    roll24 = mode24["handling"]["body_roll"]["nrmse_percent"]
    assert roll25 < roll24, (
        "mode 25 is farther from Adams' mode-25 reference on body roll than mode 24 is "
        f"({roll25:.4f} % against {roll24:.4f} %)"
    )


# ---------------------------------------------------------------------------
# USE_MODE 3 and 13 (steady state, and linear transient not combined)
# ---------------------------------------------------------------------------
#
# The mode-14 gate replays the stock case, whose tire is USE_MODE 14.  Modes 3 and
# 13 differ only in the force-axis gating and the slip relaxation, so they are
# referenced by cloning that same stock tire to the mode and replaying the same
# maneuver; the clone carries none of the fail-closed coefficients, so it is inside
# the native scope as it stands.
#
# Modes 0, 1, 2, 11 and 12 are the ones this gate cannot reach.  Modes 1/11 emit
# only Fx and My, 2/12 only Fy/Mx/Mz and 0 no slip force at all, and every maneuver
# the Adams harness can run is a steering maneuver that demands the missing axis: a
# mode-0 or mode-1 reference does not produce a dynamic result at all, and a mode-2
# one cannot hold the vehicle.  Measured: generating a mode-0 reference fails with
# "Adams result has no dynamic_001 data".  Gating them needs a straight-line
# braking case, which this harness does not have yet.
_ADDITIONAL_MODE_REFERENCES: dict[int, tuple[Path, Path]] = {
    3: (
        Path("artifacts/adams-mode-ref/um3"),
        Path("artifacts/adams-mode-ref/clones/pac2002_235_60R16_um3.tir"),
    ),
    4: (
        Path("artifacts/adams-mode-ref/um4"),
        Path("artifacts/adams-mode-ref/clones/pac2002_235_60R16_um4.tir"),
    ),
    13: (
        Path("artifacts/adams-mode-ref/um13"),
        Path("artifacts/adams-mode-ref/clones/pac2002_235_60R16_um13.tir"),
    ),
}


@pytest.mark.parametrize("use_mode", sorted(_ADDITIONAL_MODE_REFERENCES))
def test_other_claimed_modes_match_their_own_adams_reference(
    use_mode: int, tmp_path_factory: pytest.TempPathFactory
) -> None:
    """
    Every claimed USE_MODE that the handling maneuver can trim has a live gate.

    Measured when frozen: mode 3 gives 0.08/0.69/0.43/0.08/0.13/0.70 %,
    mode 4 gives 0.09/0.92/0.50/0.08/0.13/0.81 % and mode 13 gives
    0.09/0.68/0.53/0.12/0.05/0.75 % over the 5 s step steer, against the same frozen
    envelope the mode-14 gate uses.
    """
    source, tire = _ADDITIONAL_MODE_REFERENCES[use_mode]
    if not source.is_dir() or not tire.is_file():
        pytest.skip(f"reference unavailable for USE_MODE {use_mode}: {source}")

    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        f"pac2002_gate_comparison_mode{use_mode}",
    )
    diagnostic = _load_script(
        "diagnose_native_pac2002_correlation.py",
        f"pac2002_gate_diagnostic_mode{use_mode}",
    )
    root = tmp_path_factory.mktemp(f"pac2002_correlation_mode{use_mode}")
    comparison_module.generate(
        source,
        root,
        end_time=5.0,
        output_step=0.01,
        internal_step=0.01,
        tire_kinds=("pac2002",),
        pac2002_tire_file=tire,
    )

    result = diagnostic.diagnose(root, split_time_s=1.0)
    observed = {
        **{
            component: result["force"][component]["combined"]["nrmse_percent"]
            for component in ("normal_force", "longitudinal_force", "lateral_force")
        },
        **{
            channel: result["handling"][channel]["nrmse_percent"]
            for channel in ("lateral_acceleration", "yaw_rate", "body_roll")
        },
    }
    limits = diagnostic.DEFAULT_LIMITS_PERCENT
    failures = {
        name: value for name, value in observed.items() if value > limits[name]
    }
    assert failures == {}, (
        f"native USE_MODE {use_mode} left the frozen Adams correlation envelope: "
        f"{ {k: round(v, 4) for k, v in failures.items()} } "
        f"(limits {limits}, observed "
        f"{ {k: round(v, 4) for k, v in observed.items()} })"
    )


# ---------------------------------------------------------------------------
# Gate coverage
# ---------------------------------------------------------------------------


def test_every_declared_gated_use_mode_has_a_live_reference_gate() -> None:
    """
    A mode may only be declared gated if this module actually replays a reference.

    ``PAC2002_ADAMS_GATED_USE_MODES`` is what the scope audit publishes as "has a
    passing automated Adams threshold", so a mode added to that set without a test
    here would be an unverified claim.  Mode 14 is the stock case replayed by
    ``test_native_pac2002_stays_inside_frozen_adams_thresholds``; the comparison
    itself proves which tire that case uses (USE_MODE 14).  Mode 25 is gated by
    ``test_parking_steer_pins_the_turn_slip_and_parking_torque``, which is the only
    maneuver in the installed Adams set that exercises turn slip at all -- a step
    steer measures the turn slip as inert (um23/um24/um25 references differ by
    0.07-0.21 % on the handling channels).
    """
    from suspension_multibody.adams.pac2002_evidence import (
        PAC2002_ADAMS_GATED_USE_MODES,
    )

    covered = (
        {14, 25}
        | set(_ADDITIONAL_MODE_REFERENCES)
        | set(_CONTACT_MASS_REFERENCES)
    )
    assert covered == set(PAC2002_ADAMS_GATED_USE_MODES), (
        "declared gated USE_MODEs and the modes this module gates disagree: "
        f"declared {sorted(PAC2002_ADAMS_GATED_USE_MODES)}, "
        f"gated {sorted(covered)}"
    )
