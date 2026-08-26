"""The acceptance runner must never claim Adams accuracy without evidence."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from suspension_multibody.adams import load_axle_acceptance_contract

_SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "run_axle_dynamics_acceptance.py"
)


def _script() -> ModuleType:
    specification = importlib.util.spec_from_file_location(
        "axle_dynamics_acceptance", _SCRIPT
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_every_frozen_case_is_buildable_on_the_public_grid() -> None:
    """Each frozen case must exist and land on the 1 kHz comparison grid."""
    module = _script()
    acceptance = load_axle_acceptance_contract()
    expected_step = 1.0 / float(
        acceptance["comparison"]["public_sample_rate_hz"]
    )

    for entry in acceptance["case_matrix"]:
        case = module.build_case(str(entry["name"]))
        times = np.asarray(case.times_s)
        assert case.name == entry["name"]
        np.testing.assert_allclose(np.diff(times), expected_step, atol=1e-12)
        # Road velocity must be the exact derivative of the road height, or the
        # energy ledger would attribute real road work to a residual.
        for tire, height in case.road_height_m.items():
            rate = np.asarray(case.road_velocity_m_per_s[tire])
            assert rate.shape == np.asarray(height).shape


def test_smooth_cases_declare_no_prescribed_contact_loss() -> None:
    """A case frozen as smooth must not be driven past tire liftoff."""
    module = _script()
    acceptance = load_axle_acceptance_contract()
    unsprung_kg = 18.0 + 22.0
    # The static corner load sets how hard the road may accelerate downward
    # before the tire unloads; a smooth case must stay under it.
    preload_n = 3334.0
    limit_m_per_s2 = 9.80665 + preload_n / unsprung_kg

    smooth = [
        str(entry["name"])
        for entry in acceptance["case_matrix"]
        if entry["energy_class"] == "smooth"
    ]
    assert "large_amplitude_high_frequency" in smooth
    peak = (
        module._HIGH_FREQUENCY_AMPLITUDE_M
        * (2.0 * np.pi * module._HIGH_FREQUENCY_HZ) ** 2
    )
    assert peak < limit_m_per_s2


def test_missing_adams_evidence_blocks_the_accuracy_and_speed_claim(
    tmp_path: Path,
) -> None:
    """Without real Adams evidence both Adams gates must report BLOCKED."""
    module = _script()
    status = module.main(
        [
            "--output",
            str(tmp_path),
            "--case",
            "static_equilibrium",
        ]
    )
    report = json.loads(
        (tmp_path / "acceptance_report.json").read_text(encoding="utf-8")
    )

    assert status == 0
    assert report["solver_self_convergence_status"] == "PASSED"
    assert report["adams_accuracy_status"] == "BLOCKED"
    assert report["adams_speed_status"] == "BLOCKED"
    assert report["accuracy_claim_permitted"] is False
    assert report["adams_comparisons"] == []
    assert any("no real Adams evidence" in item for item in report["blockers"])
    assert report["parameter_provenance"] == module.PARAMETER_PROVENANCE


def test_cases_outside_the_frozen_matrix_are_rejected(tmp_path: Path) -> None:
    module = _script()
    with pytest.raises(SystemExit, match="outside the frozen matrix"):
        module.main(
            ["--output", str(tmp_path), "--case", "invented_case"]
        )
