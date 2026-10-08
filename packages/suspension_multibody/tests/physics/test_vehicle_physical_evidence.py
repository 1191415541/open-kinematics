"""Approved contact-frame evidence still rejects unrelated numerical changes."""

import importlib.util
import json
import subprocess
import sys
import textwrap
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts import pack_container, unpack_container

from suspension_multibody.results.envelope import ResultEnvelope
from suspension_multibody.simulation import run_compiled

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "packages/suspension_multibody/scripts"


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name+".py"))
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def evidence():
    return load_script("vehicle_physical_evidence")


@pytest.fixture(scope="module")
def steering(evidence):
    gate = load_script("case_parity_check")
    baseline = gate._VEHICLE_BASELINE
    layout = json.loads(baseline.with_name("entity_layout.json").read_text(encoding="utf-8"))["cases"]["steering"]
    _, case = gate._vehicle_case_matrix(gate._load_vehicle_fixture())["steering"]
    compiled = evidence.compile_evidence(case, layout)
    reference = evidence.reference_case(baseline.parent / "reference", "steering", baseline)
    return compiled, layout, reference


def test_gate_reports_strict_and_physical_status_separately(tmp_path):
    gate = load_script("case_parity_check")
    passed, message = gate.check_vehicle_dynamic(artifact_dir=tmp_path)
    assert passed, message
    report = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    strict = [row for row in report["cases"].values() if row["status"] == "STRICT"]
    physical = [row for row in report["cases"].values() if row["status"] == "PHYSICAL_DIFFERENCE"]
    assert len(strict) == 5
    assert all(row["original_hashes_match"] and row["accepted"] for row in strict)
    assert len(physical) == 3
    assert all(not row["original_hashes_match"] and row["physical_checks"]["passed"] for row in physical)
    for row in physical:
        assert row["case_differences"] == []
        assert {channel["unit"] for channel in row["channel_differences"]} >= {"m", "N", "Nm", "J"}
        assert any(channel["max_abs_difference"] > 0 for channel in row["channel_differences"])
        assert row["physical_checks"]["equalities"]["kinetic_energy_J"]["max_error_ratio"] <= 1
        assert len(row["physical_checks"]["constraint_balances"]) in (28, 36)
        equalities = row["physical_checks"]["equalities"]
        assert any(name.endswith(":longitudinal_force_N") for name in equalities)
        assert any(name.endswith(":force_N") for name in equalities)


def test_vehicle_fixture_preserves_inputs_without_importing_legacy_producers():
    gate = load_script("case_parity_check")
    script = textwrap.dedent(f"""
        import importlib.abc, importlib.util, hashlib, json, sys
        class BlockLegacy(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                prefixes = ('suspension_multibody.preparation', 'suspension_multibody.vehicle.service',
                    'suspension_multibody.subsystems.entry', 'tests.vehicle.test_native_vehicle')
                if any(fullname == prefix or fullname.startswith(prefix + '.') for prefix in prefixes):
                    raise ImportError('vehicle fixture imports a legacy producer: ' + fullname)
        sys.meta_path.insert(0, BlockLegacy())
        spec = importlib.util.spec_from_file_location('gate', {str(SCRIPTS / 'case_parity_check.py')!r})
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cases = module._vehicle_case_matrix(module._load_vehicle_fixture())
        print(json.dumps({{name: hashlib.sha256(case.model_copy(update={{'vehicle': model}})
            .model_dump_json().encode()).hexdigest() for name, (model, case) in cases.items()}}))
    """)
    completed = subprocess.run([sys.executable, "-c", script], cwd=ROOT,
        capture_output=True, text=True, encoding="utf-8", check=False)
    assert completed.returncode == 0, completed.stderr
    fingerprints = json.loads(completed.stdout)
    reference = json.loads((gate._VEHICLE_BASELINE.parent / "reference/manifest.json").read_text(encoding="utf-8"))
    assert fingerprints == {name: row["source_sha256"] for name, row in reference["cases"].items()}


def test_missing_reaction_identity_manifest_is_a_failure(evidence, steering):
    compiled, layout, _ = steering
    result = run_compiled(compiled).result

    class MissingReactions:
        constraint_ids = ()

        def __getattr__(self, name):
            return getattr(result, name)

    checks = evidence.physical_checks(MissingReactions(), compiled, layout)
    assert not checks["passed"]
    assert any(row["check"] == "constraint_coverage" for row in checks["failures"])


@pytest.mark.parametrize("channel", ["tire", "steering"])
def test_corrupted_force_channel_fails_independent_law(evidence, steering, channel):
    compiled, layout, _ = steering
    result = run_compiled(compiled).result
    assert isinstance(result, ResultEnvelope)

    class CorruptedForce:
        def __getattr__(self, name):
            return getattr(result, name)

        def tire_state(self, name):
            values = result.tire_state(name).copy()
            if channel == "tire":
                values[-1, 5] += 1
            return values

        def element_state(self, name):
            if name in result.raw.tire_names:
                return self.tire_state(name)
            values = result.element_state(name).copy()
            if channel == "steering":
                values[-1, 3] += 1
            return values

    checks = evidence.physical_checks(CorruptedForce(), compiled, layout)
    assert not checks["passed"]
    suffix = ":longitudinal_force_N" if channel == "tire" else ":force_N"
    assert any(row["check"].endswith(suffix) for row in checks["failures"])


def test_reference_arrays_must_match_original_channel_hash(evidence, steering):
    _, _, reference = steering
    arrays = deepcopy(reference[4])
    baseline = load_script("case_parity_check")._VEHICLE_BASELINE
    expected = json.loads(baseline.read_text(encoding="utf-8"))["cases"]["steering"]
    arrays["states"][0, 0, 0] += 1e-3
    with pytest.raises(ValueError, match="differs from frozen hash"):
        evidence._verify_reference_arrays(arrays, expected, "steering")


def test_identity_mapping_rejects_truncated_ledger(evidence, steering):
    _, layout, reference = steering
    old = deepcopy(reference[0])
    old["tires"].pop()
    with pytest.raises(ValueError, match="one-to-one"):
        evidence._id_maps(old, layout)


def test_mass_change_is_not_covered_by_contact_frame_approval(evidence, steering):
    compiled, layout, reference = steering
    model = deepcopy(compiled.model_document)
    model["bodies"][0]["mass"] += .1
    _, blob = unpack_container(compiled.model_payload)
    altered = replace(compiled, model_document=model, model_payload=pack_container(model, blob))
    delta, case_delta, _ = evidence.input_differences(altered, *reference[:4], layout)
    checks = evidence.allowed_input_changes(delta, case_delta)
    assert not checks["passed"]
    assert any(row["field"] == "bodies[0].mass" for row in checks["unexpected"])


def test_excitation_table_change_is_not_covered_by_approval(evidence, steering):
    compiled, layout, reference = steering
    document, blob = unpack_container(compiled.case_payload)
    blob = bytearray(blob)
    descriptor = next(row for row in document["blobs"] if row["role"] == "steering_target")
    offset = descriptor["offset"]
    values = np.frombuffer(blob, dtype="<f8", count=descriptor["length"]//8, offset=offset)
    values[-1] += .001
    altered = replace(compiled, case_payload=pack_container(document, bytes(blob)))
    delta, case_delta, _ = evidence.input_differences(altered, *reference[:4], layout)
    assert case_delta
    assert not evidence.allowed_input_changes(delta, case_delta)["passed"]


def test_independent_energy_check_detects_corrupted_result(evidence, steering):
    compiled, layout, _ = steering
    result = run_compiled(compiled).result
    assert isinstance(result, ResultEnvelope)
    original = result.energy.copy()

    class CorruptedEnergy:
        def __getattr__(self, name):
            return getattr(result, name)

        @property
        def energy(self):
            altered = original.copy()
            altered[-1, 0] += 1
            return altered

    checks = evidence.physical_checks(CorruptedEnergy(), compiled, layout)
    assert not checks["passed"]
    assert any(row["check"] == "kinetic_energy_J" for row in checks["failures"])
