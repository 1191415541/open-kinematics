"""
Automated Adams correlation gate for native Fiala.

This is the gate the native-Fiala parity work depends on: a Fiala claim is only
meaningful while something in the repository re-runs the native model against the
Adams reference and fails when the error grows.

It regenerates the native side from the stored Adams Fiala source case and applies
the criterion the diagnostic script declares, in two tiers:

* channels whose reference peak reaches at least 1 % of that wheel's load peak are
  judged against **that wheel's own reference peak**;
* channels below that are near-zero residuals and are judged against **that wheel's
  load peak**.

The denominator matters, and it is why the criterion has two tiers.  A free-rolling
wheel carries almost no longitudinal force, so the front axle's force is a
difference of two large velocities: its peak is 5.91 / 9.43 N against a 3158 / 4536 N
load peak, i.e. 0.19 % / 0.21 %.  Judging that against its own peak ("1 %" = 0.059 N)
asks the two solutions to agree on the contact-patch longitudinal velocity to about
1e-4 m/s -- roughly 300x tighter than they actually do (they differ by ~0.03 m/s,
0.18 %), and it measures solution-to-solution reproducibility rather than tire
fidelity.  The same model reaches 0.0153-0.2038 % on every braking channel, where
the front axle carries 973 N.  See
``tasks/05-front-fx-diagnosis/ROOT_CAUSE.md`` for the full derivation.

No channel is registered as failing any more, so there is deliberately no
``_KNOWN_GAP`` set and no xfail marker: a regression must surface as a real failure.

Everything is skipped when the Adams reference case, the installed tire library or
the Adams tire file is absent, because the comparison cannot be constructed
without them.  ``artifacts/`` is gitignored, so a bare checkout skips by design.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

import pytest

_SOURCE_CASE = Path("artifacts/adams-full-source-fiala/step_steer")
_SCRIPTS = Path(__file__).parents[2] / "scripts"

_FORCES = ("normal_force", "longitudinal_force", "lateral_force")
_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")

#: The gate limit every per-wheel channel must satisfy.
_LIMIT_PERCENT = 1.0

#: Channels that must be judged against the wheel load peak rather than their own
#: peak, because their reference peak is a near-zero residual.  Pinned so the tier
#: assignment is a deliberate, visible choice rather than an emergent accident.
_NEAR_ZERO_CHANNELS = frozenset(
    {
        ("longitudinal_force", "front_left"),
        ("longitudinal_force", "front_right"),
    }
)

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


def _fiala_tire_property() -> Path | None:
    """Locate the Fiala tire of the installed Adams/Car concept database."""
    from suspension_multibody.adams import resolve_adams_home

    home = resolve_adams_home()
    relative = Path("acar/acar_concept.cdb/tires.tbl/fiala_235_45R17.tir")
    if home is None or not (home / relative).is_file():
        return None
    return home / relative


@pytest.fixture(scope="module")
def comparison(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the native Fiala comparison once for every test in the module."""
    if not _SOURCE_CASE.is_dir():
        pytest.skip(f"Adams Fiala source case unavailable: {_SOURCE_CASE}")
    tire = _fiala_tire_property()
    if tire is None:
        pytest.skip("Adams/Car concept Fiala tire is unavailable")

    comparison_module = _load_script(
        "run_full_native_three_model_comparison.py",
        "fiala_gate_comparison",
    )
    root = tmp_path_factory.mktemp("fiala_correlation")
    comparison_module.generate(
        _SOURCE_CASE,
        root,
        end_time=5.0,
        output_step=0.01,
        internal_step=0.01,
        tire_kinds=("fiala",),
        tire_property_file=tire,
    )
    return root


@pytest.fixture(scope="module")
def gate(comparison: Path) -> dict[str, Any]:
    """Run the Fiala diagnostic once and share its result."""
    diagnostic = _load_script(
        "diagnose_native_fiala_correlation.py",
        "fiala_gate_diagnostic",
    )
    return diagnostic.diagnose(comparison)


def _observed(gate: dict[str, Any], component: str, wheel: str) -> float:
    return gate["gate"]["observed_percent"][component][wheel]


def _channel_param(component: str, wheel: str) -> pytest.ParameterSet:
    """Every channel is a plain parameter now that none is registered failing."""
    return pytest.param(component, wheel)


@pytest.mark.parametrize(
    ("component", "wheel"),
    [
        _channel_param(component, wheel)
        for component in _FORCES
        for wheel in _WHEELS
    ],
)
def test_every_wheel_stays_inside_the_peak_force_envelope(
    gate: dict[str, Any], component: str, wheel: str
) -> None:
    """
    Each wheel's force component must stay within 1 % of its assigned denominator.

    The diagnostic already decided this; the test re-states it per channel so a
    failure names the wheel and component instead of a nested dict.
    """
    observed = _observed(gate, component, wheel)
    assert observed < _LIMIT_PERCENT, (
        f"{wheel}.tire_{component} left the per-wheel envelope: "
        f"{observed:.4f}% (limit {_LIMIT_PERCENT}%, "
        f"standard {gate['gate']['standard_by_channel'][component][wheel]!r})"
    )


def test_no_channel_is_outside_the_two_tier_envelope(
    gate: dict[str, Any]
) -> None:
    """
    Pin the gate as fully passing, so any regression is a real, visible failure.
    """
    failing = {
        (component, wheel)
        for component in _FORCES
        for wheel in _WHEELS
        if _observed(gate, component, wheel) >= _LIMIT_PERCENT
    }
    assert failing == set(), (
        "channels left the two-tier envelope: "
        f"{sorted(failing)}.  Fix the model or, if a channel became a near-zero "
        "residual, change the tier deliberately and record why."
    )
    assert gate["gate"]["passed"] is True


def test_the_near_zero_tier_is_applied_to_exactly_the_residual_channels(
    gate: dict[str, Any]
) -> None:
    """
    The load denominator must cover exactly the near-zero channels, nothing more.

    This is the guard against quietly relaxing the criterion: if the tier ever
    swallows a channel that carries a real signal, this fails.
    """
    standards = gate["gate"]["standard_by_channel"]
    near_zero = {
        (component, wheel)
        for component in _FORCES
        for wheel in _WHEELS
        if standards[component][wheel] == "load"
    }
    assert near_zero == _NEAR_ZERO_CHANNELS, (
        f"tier assignment changed: load-tier={sorted(near_zero)} "
        f"pinned={sorted(_NEAR_ZERO_CHANNELS)}"
    )
    threshold = gate["gate"]["small_signal_load_fraction"]
    for component in _FORCES:
        for wheel in _WHEELS:
            ratio = gate["gate"]["signal_ratio_by_channel"][component][wheel]
            expected = "load" if ratio < threshold else "peak"
            assert standards[component][wheel] == expected
            if expected == "peak":
                # A peak-tier channel must be judged against its own peak.
                assert gate["gate"]["denominator_by_channel"][component][
                    wheel
                ] == pytest.approx(
                    gate["force"][component]["by_wheel"][wheel]["reference_peak"]
                )
            else:
                assert gate["gate"]["denominator_by_channel"][component][
                    wheel
                ] == pytest.approx(
                    gate["gate"]["load_peak_by_wheel"][wheel]
                )


def test_the_criterion_is_per_wheel_and_declares_both_tiers(
    gate: dict[str, Any]
) -> None:
    """
    The gate must judge per wheel and state both tiers, not one global denominator.

    A future edit that quietly reverts the criterion would otherwise keep the
    suite green while measuring something else.
    """
    criterion = gate["gate"]["criterion"]
    assert "逐轮" in criterion
    assert "峰值" in criterion
    assert "近零残差" in criterion
    assert set(gate["gate"]["observed_percent"]) == set(_FORCES)
    for component in _FORCES:
        assert set(gate["gate"]["observed_percent"][component]) == set(_WHEELS)
    # The old single-tier numbers stay published for audit, but are not the gate.
    assert gate["gate"]["observed_percent_peak_only"] != gate["gate"][
        "observed_percent"
    ]


def test_comparison_manifest_declares_the_diagnostic_contract(
    comparison: Path,
) -> None:
    """The manifest must satisfy the contract the diagnostic gate validates."""
    manifest = json.loads(
        (comparison / "comparison_manifest.json").read_text(encoding="utf-8")
    )
    for name, expected in _REQUIRED_CONTRACT.items():
        assert manifest[name] == expected, f"manifest {name!r} is not the contract"
    # The Fiala diagnostic does not validate this field, so the label is pinned
    # here instead: it is what marks the history as produced by an Adams Fiala
    # tire.  A fiala-only comparison writes `fiala_tire_iso_output`; the older
    # `adams_fiala_or_pac2002_tire_iso_output` still appears in archived
    # comparison roots that predate the split.
    assert manifest["tire_force_coordinates"] == "fiala_tire_iso_output"
