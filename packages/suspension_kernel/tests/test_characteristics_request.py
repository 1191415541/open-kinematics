"""
P0 acceptance: the native reader rejects a malformed characteristics request.

The Python contract validator and the native reader are two independent
implementations of one contract.  A test that only drives the Python side would
still pass if the kernel silently ignored everything it did not recognise, which
is exactly the failure this guards: a reader that drops unknown fields turns a
typo into a different physical question and reports success.

These tests therefore bypass the Python schema entirely.  They compile a valid
document, tamper with the compiled case document, repack it and hand it straight
to ``suspension_kernel_run``.  The kernel is then the only thing that can refuse.

The bundle is compiled in a **subprocess**, not in this interpreter.  Building it
in-process pulls in ``suspension_multibody``'s numba-accelerated primitives, and
running that JIT after this session has already loaded the kernel shared library
through ctypes faults on Windows (access violation inside ``_quaternion_to_matrix_unit``).
The isolation is not about speed: it keeps a real crash out of the suite while
still exercising the kernel's own reader on a genuinely compiled document.
"""

from __future__ import annotations

import copy
import ctypes
import json
import subprocess
import sys
from pathlib import Path

import pytest
import suspension_contracts as contracts

FIXTURE = (
    Path(__file__).resolve().parents[2]
    / "suspension_multibody"
    / "tests"
    / "data"
    / "benchmark_axle.json"
)

OUTPUT_CAPACITY = 1 << 24
ERROR_CAPACITY = 4096

#: Runs in a fresh interpreter.  It compiles the document pair and writes both
#: payloads to disk, so this test session never triggers the numba JIT.
_BUILDER = '''
import json, pathlib, sys
from suspension_multibody.authoring import migrate_v1_kc_case
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation import resolved as compilation
from suspension_multibody.kernel import capabilities
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.schema.model import AxleDeclaration

fixture, request_json, out_dir = sys.argv[1], sys.argv[2], pathlib.Path(sys.argv[3])
payload = json.loads(pathlib.Path(fixture).read_text(encoding="utf-8"))
model = AxleDeclaration.model_validate(payload["model"])
assembly, case = migrate_v1_kc_case(
    model, mode="K", wheel_values_mm=(-10.0, 0.0, 10.0), rack_values_mm=(-5.0, 0.0, 5.0)
)
document = case.to_payload()
document["characteristics"] = json.loads(request_json)
plan = ResolvedSolvePlan(document)
bundle = DocumentLoader().load(assembly, plan.to_document())
# This build reads requests but answers none, so the capability gate would
# refuse.  The reader is what is under test here; the gate has its own test.
capabilities.require_characteristics_answerable = lambda kind: None
compiled = compilation.compile_resolved(bundle.resolve(), plan)
(out_dir / "model.bin").write_bytes(compiled.model_payload)
(out_dir / "case.bin").write_bytes(compiled.case_payload)
(out_dir / "case.json").write_text(json.dumps(compiled.case_document), encoding="utf-8")
'''


def _request_body() -> dict:
    return {
        "request_id": "p0-native",
        "domain": "dynamic",
        "samples": [0, 1],
        "measurements": [
            {
                "measurement_id": "front.{left;right}.wheel_center",
                "definition_version": 1,
                "units": "N/m",
                "response_kind": "frozen_force_tangent",
                "response_version": 1,
            }
        ],
        "boundary": {"profile": "susp_force_balance_body_fixed_v1", "version": 1},
        "definition": {"kind": "frozen_force_tangent", "version": 1},
        "frozen_conditions": {"protocol": "frozen_inertial_load_v1", "version": 1},
        "response_operator": {"kind": "local_response_v1", "version": 1},
    }



@pytest.fixture(scope="module")
def compiled(tmp_path_factory):
    """
    Compile a bundle whose case document carries a valid request.

    The compiler refuses a request this build cannot *answer* (the kernel reads
    requests and computes no response yet), so the builder suppresses that gate.
    Reaching the ABI with a request present is the whole point: only then can the
    native reader's own strictness be observed.
    """
    out_dir = tmp_path_factory.mktemp("characteristics")
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            _BUILDER,
            str(FIXTURE),
            json.dumps(_request_body()),
            str(out_dir),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"bundle build failed:\n{result.stderr}"
    return _Bundle(
        model_payload=(out_dir / "model.bin").read_bytes(),
        case_payload=(out_dir / "case.bin").read_bytes(),
        case_document=json.loads((out_dir / "case.json").read_text(encoding="utf-8")),
    )


class _Bundle:
    """The compiled artefacts the assertions below tamper with."""

    def __init__(self, *, model_payload: bytes, case_payload: bytes, case_document: dict) -> None:
        self.model_payload = model_payload
        self.case_payload = case_payload
        self.case_document = case_document


def _mirror_library() -> Path:
    """Locate the kernel mirror the multibody package loads."""
    path = (
        Path(__file__).resolve().parents[2]
        / "suspension_multibody"
        / "src"
        / "suspension_multibody"
        / "native"
        / "suspension_kernel.dll"
    )
    if not path.is_file():
        pytest.skip(f"kernel mirror not built at {path}; run packages/suspension_multibody/scripts/build_axle_native.py")
    return path


def _run(model_payload: bytes, case_payload: bytes) -> tuple[int, str]:
    """
    Call the ABI directly, so no Python contract layer can pre-empt the answer.

    The library is the *mirror the multibody package was built against*, not the
    kernel package's own copy: the bundle under test was produced by that
    package, and a stale mirror would answer a different question.
    """
    library = ctypes.CDLL(str(_mirror_library()))
    entry = library.suspension_kernel_run
    entry.restype = ctypes.c_int32
    entry.argtypes = [
        ctypes.c_char_p,
        ctypes.c_size_t,
        ctypes.c_char_p,
        ctypes.c_size_t,
        ctypes.c_char_p,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    out = ctypes.create_string_buffer(OUTPUT_CAPACITY)
    out_length = ctypes.c_size_t(OUTPUT_CAPACITY)
    error = ctypes.create_string_buffer(ERROR_CAPACITY)
    code = entry(
        model_payload,
        len(model_payload),
        case_payload,
        len(case_payload),
        out,
        ctypes.byref(out_length),
        error,
        ERROR_CAPACITY,
    )
    return int(code), error.value.decode("utf-8", "replace")


def test_the_valid_request_is_accepted(compiled) -> None:
    """The control: a well-formed request runs, so rejection below means something."""
    code, error = _run(compiled.model_payload, compiled.case_payload)
    assert code == 0, f"a valid request was refused: {error}"


@pytest.mark.parametrize(
    ("label", "mutate"),
    [
        ("unknown measurement field", lambda c: c["measurements"][0].update(unit="N/m")),
        ("unknown top-level field", lambda c: c.update(extra=1)),
        ("unknown domain", lambda c: c.update(domain="nonsense")),
        ("unknown unit", lambda c: c["measurements"][0].update(units="furlong")),
        ("unknown response kind", lambda c: c["measurements"][0].update(response_kind="made_up")),
        ("duplicate sample", lambda c: c.update(samples=[0, 0])),
        ("negative sample", lambda c: c.update(samples=[-1])),
        ("zero boundary version", lambda c: c["boundary"].update(version=0)),
        ("zero response version", lambda c: c["measurements"][0].update(response_version=0)),
        ("frozen kind without protocol", lambda c: c.pop("frozen_conditions")),
        ("unknown definition kind", lambda c: c["definition"].update(kind="vibes")),
        ("unknown request schema version", lambda c: None),
    ],
)
def test_malformed_requests_are_refused_by_the_kernel(compiled, label, mutate) -> None:
    """Each malformed request fails in the kernel, with a message naming the path."""
    tampered = copy.deepcopy(compiled.case_document)
    body = tampered["characteristics"]["characteristics"]
    if label == "unknown request schema version":
        tampered["characteristics"]["schema_version"] = 99
    else:
        mutate(body)
    case_payload = contracts.pack_container(tampered, compiled.case_payload)
    code, error = _run(compiled.model_payload, case_payload)
    assert code != 0, f"the kernel accepted a malformed request: {label}"
    assert "characteristics" in error, (
        f"the refusal does not name the characteristics request: {error!r}"
    )


def test_absent_request_is_not_an_error(compiled) -> None:
    """A document without a request is the ordinary case and must still run."""
    tampered = copy.deepcopy(compiled.case_document)
    tampered.pop("characteristics")
    # The blob section must survive: the case's own sampled tables live there.
    case_payload = contracts.pack_container(tampered, compiled.case_payload)
    code, error = _run(compiled.model_payload, case_payload)
    assert code == 0, f"a document without a request was refused: {error}"


def test_a_tampered_request_still_uses_the_original_payload(compiled) -> None:
    """The rejection above is about the request, not about a damaged payload."""
    tampered = copy.deepcopy(compiled.case_document)
    tampered["characteristics"]["characteristics"]["domain"] = "nonsense"
    case_payload = contracts.pack_container(tampered, compiled.case_payload)
    code, error = _run(compiled.model_payload, case_payload)
    assert code != 0
    assert "unknown domain" in error
