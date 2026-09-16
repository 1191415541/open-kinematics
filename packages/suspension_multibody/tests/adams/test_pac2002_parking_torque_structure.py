"""
Structural acceptance for the USE_MODE 25 parking torque.

Adams ``Parking_Torque`` states the acceptance criterion for the turn-slip / parking
model in one sentence:

    "The maximum parking torque is mainly determined by parameter qCrj1, while the
    stiffness is due to the yaw stiffness cy value."

That is a claim about *which coefficient does what*, so it can be checked without an
absolute reference: mutate the coefficient in the tire file and require the predicted
part of the response to move, while the rest of the tire stays put.  The gate in
``test_pac2002_adams_correlation_gate`` compares against Adams; these tests compare
against the documentation, and they cover the two clauses the reference gate cannot
separate (``qCrj1`` sets the peak, ``cy`` sets the stiffness).

What is measured, on the steering-wheel sine parking maneuver (1.2 s at a 2 ms step):

  * with ``QCRP1`` -- the "turning moment at constant turning with zero speed" that
    scales the spin moment ``Mz_phi_inf`` of Eq3348 -- zeroed, the front tire's
    aligning moment loses most of its amplitude.  That *is* the parking torque: the
    moment that is present only because the tire is being steered while it stands;
  * the contact body's yaw deflection equals ``Mz / CP`` sample by sample (Eq3954's
    equilibrium and Eq3967), and doubling ``CP`` in the file halves the deflection for
    the same moment.  If the kernel used a built-in stiffness instead of the tire's
    ``CP`` the second part would fail.

Both are falsifiable by construction: a kernel that ignored either coefficient, or
that hard-coded the yaw stiffness, fails them.
"""

from __future__ import annotations

import importlib.util
import json
import re
from pathlib import Path

import numpy as np
import pytest

_SCRIPTS = Path(__file__).parents[2] / "scripts"
_REFERENCE = Path("artifacts/adams-mode-ref/um25-parking_steer")
_TIRE = Path(
    "artifacts/adams-mode-ref/clones/"
    "pac2002_205_55R16_parking_um25-parking_steer.tir"
)
_END_TIME = 1.2
_INTERNAL_STEP = 0.002
# The parking torque lives on the steered axle.
_FRONT = ("front_left", "front_right")


def _load_script(name: str, module_name: str):
    """Import a comparison script by path (they are not packaged modules)."""
    path = _SCRIPTS / name
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _rewrite_tire(source: Path, destination: Path, overrides: dict[str, str]) -> Path:
    """
    Copy a tire property file with some ``KEY = value`` lines replaced.

    Every requested key must be present exactly once, so a typo cannot silently turn
    into a test that measures the unmutated tire.
    """
    text = source.read_text(encoding="ascii", errors="replace")
    for key, value in overrides.items():
        pattern = rf"^(\s*{re.escape(key)}\s*=\s*)[^\s$]+"
        text, count = re.subn(pattern, rf"\g<1>{value}", text, flags=re.MULTILINE)
        assert count == 1, f"expected one {key} assignment in {source}, found {count}"
    destination.write_text(text, encoding="ascii")
    return destination


@pytest.fixture(scope="module")
def parking_runs(tmp_path_factory: pytest.TempPathFactory):
    """
    Solve the parking maneuver for a set of tire overrides, caching each result.

    Returns a callable ``run(**overrides) -> channels`` so two tests can share the
    unmutated run and each mutation is solved once.
    """
    if not _REFERENCE.is_dir() or not _TIRE.is_file():
        pytest.skip(f"mode-25 parking reference unavailable: {_REFERENCE} / {_TIRE}")
    comparison = _load_script(
        "run_full_native_three_model_comparison.py",
        "parking_torque_structure_comparison",
    )
    scratch = tmp_path_factory.mktemp("parking_torque_tires")
    cached: dict[tuple[tuple[str, str], ...], dict[str, list[float]]] = {}
    counter = {"runs": 0}

    def run(**overrides: str) -> dict[str, list[float]]:
        """Solve the parking maneuver with the given tire overrides."""
        key = tuple(sorted(overrides.items()))
        if key in cached:
            return cached[key]
        tire = _TIRE
        if overrides:
            tire = _rewrite_tire(
                _TIRE, scratch / f"parking_{counter['runs']}.tir", overrides
            )
        root = scratch / f"comparison_{counter['runs']}"
        counter["runs"] += 1
        comparison.generate(
            _REFERENCE,
            root,
            end_time=_END_TIME,
            output_step=0.01,
            internal_step=_INTERNAL_STEP,
            tire_kinds=("pac2002",),
            pac2002_tire_file=tire,
        )
        channels = json.loads(
            (root / "native_pac2002_time_history.json").read_text("utf-8")
        )["channels"]
        cached[key] = channels
        return channels

    return run


def _front_aligning_moment(channels: dict[str, list[float]]) -> np.ndarray:
    """Sum of the two front tires' aligning moment."""
    return np.asarray(
        [channels[f"{wheel}.tire_aligning_moment"] for wheel in _FRONT], dtype=float
    ).sum(axis=0)


def test_parking_torque_peak_is_the_spin_moment_coefficient(parking_runs) -> None:
    """
    Zeroing ``QCRP1`` must remove most of the front axle's aligning moment.

    ``QCRP1`` is Adams' "Turning moment at constant turning with zero speed"; it scales
    Eq3348's ``Mz_phi_inf``, the amplitude of the spin moment that only exists while
    the tire is being steered at low speed.  The rest of the front aligning moment
    (the pneumatic-trail term ``- t*Fy``) does not scale with it, so the assertion is
    a large *reduction*, not a disappearance.
    """
    stock = _front_aligning_moment(parking_runs())
    without = _front_aligning_moment(parking_runs(QCRP1="0"))
    stock_peak = float(np.max(np.abs(stock)))
    without_peak = float(np.max(np.abs(without)))
    assert without_peak < 0.7*stock_peak, (
        "zeroing QCRP1 did not remove the parking torque: front axle aligning moment "
        f"peak {stock_peak:.4g} -> {without_peak:.4g} N*m"
    )


def test_parking_torque_stiffness_is_the_tire_contact_yaw_stiffness(
    parking_runs,
) -> None:
    """
    The contact-body yaw deflection must be ``Mz / CP`` with the tire's own ``CP``.

    Eq3954 gives the contact body's yaw equilibrium ``c_psi*beta = Mz`` and Eq3967
    defines ``beta_st = Mz/c_psi``; ``c_psi`` is the tire property ``CP``.  The check
    runs twice: once as the per-sample identity ``beta = Mz/CP``, and once by doubling
    ``CP`` in the file and requiring the deflection to halve for the same moment.  The
    second part is what makes this a test of the *file* rather than of the algebra -- a
    kernel with a built-in stiffness would satisfy neither.
    """
    stock = parking_runs()
    doubled = parking_runs(CP="4038")
    for label, channels, stiffness in (
        ("stock", stock, 2019.0),
        ("CP doubled", doubled, 4038.0),
    ):
        deflection = np.asarray(
            channels["front_left.tire_contact_body_yaw_rad"], dtype=float
        )
        moment = np.asarray(
            channels["front_left.tire_aligning_moment"], dtype=float
        )
        residual = deflection - moment/stiffness
        scale = max(float(np.mean(np.abs(deflection))), 1e-12)
        assert float(np.mean(np.abs(residual)))/scale < 0.05, (
            f"{label}: contact yaw is not Mz/CP "
            f"(mean |beta - Mz/CP| / mean |beta| = "
            f"{float(np.mean(np.abs(residual)))/scale:.4f})"
        )
    stock_peak = float(
        np.max(np.abs(np.asarray(stock["front_left.tire_contact_body_yaw_rad"])))
    )
    doubled_peak = float(
        np.max(np.abs(np.asarray(doubled["front_left.tire_contact_body_yaw_rad"])))
    )
    ratio = doubled_peak/stock_peak
    assert 0.3 < ratio < 0.7, (
        "doubling the tire's CP did not halve the contact yaw deflection "
        f"(peak ratio {ratio:.4f}); the kernel is not reading CP from the tire file"
    )
