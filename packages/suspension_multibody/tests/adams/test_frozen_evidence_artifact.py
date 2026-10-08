"""Frozen evidence bytes remain comparable after public artifact migration."""

import hashlib
import json
import runpy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "packages/suspension_multibody"


def test_unified_runner_preserves_the_frozen_comparison_artifact(tmp_path):
    producer = runpy.run_path(str(PACKAGE / "scripts/run_axle_dynamics_acceptance.py"))
    producer["_run_case"]("static_equilibrium", tmp_path, producer["load_axle_acceptance_contract"](),
        measure_performance=False)
    destination = tmp_path / "static_equilibrium/native"
    sentinel = runpy.run_path(str(PACKAGE / "scripts/dynamic_hash_sentinel.py"))
    baseline = json.loads((PACKAGE / "tests/data/dynamic_hash_baseline.json").read_text(encoding="utf-8"))
    for variant in ("native_result", "native_refined_result"):
        expected = next(row for row in baseline["entries"] if row["artifact"] == f"static_equilibrium/native/{variant}")
        saved = destination / variant
        assert hashlib.sha256((saved / "arrays.npz").read_bytes()).hexdigest() == expected["arrays_npz_sha256"]
        payload = json.loads((saved / "manifest.json").read_text(encoding="utf-8"))
        assert hashlib.sha256(sentinel["_canonical_manifest"](payload)).hexdigest() == expected["manifest_sha256"]
