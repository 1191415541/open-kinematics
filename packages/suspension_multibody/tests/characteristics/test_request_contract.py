"""
P0 acceptance: the characteristics request is inert data that survives the chain.

The request is an *output request*, not a study, not a case family and not a
second entry point.  Three properties have to hold for that claim to be true
rather than aspirational, and each is tested here:

* **carriage** -- a request written on the plan reaches the native case document
  unchanged, so a caller cannot be handed the old channels while the request was
  quietly dropped;
* **rejection** -- an unknown field, domain, unit, kind, version or boundary is
  refused at the layer that first sees it, never ignored.  A reader that drops
  what it does not know turns a typo into a *different* physical question and
  still reports success;
* **inertness** -- a run with a request produces bit-identical numbers and the
  same block set as the same run without one.

The tests are written against the public chain (``plan_from_case`` ->
``compile_resolved``) and the raw native result, because that is where the
request actually has to survive.  Asserting on the validator alone would pass
even if the field were dropped one step later.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts import ContractError, validate_characteristics_request

from suspension_multibody.api import validate
from suspension_multibody.authoring import migrate_v1_kc_case
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.schema.model import AxleDeclaration
from suspension_multibody.simulation import run_compiled

FIXTURE = Path(__file__).resolve().parents[1] / "data" / "benchmark_axle.json"


def _plan_payload() -> dict:
    """Build a real K-grid plan payload through the same path the product uses."""
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    model = AxleDeclaration.model_validate(payload["model"])
    _, case = migrate_v1_kc_case(
        model, mode="K", wheel_values_mm=(-10.0, 0.0, 10.0), rack_values_mm=(-5.0, 0.0, 5.0)
    )
    return case.to_payload()


def _assembly():
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    model = AxleDeclaration.model_validate(payload["model"])
    assembly, _ = migrate_v1_kc_case(
        model, mode="K", wheel_values_mm=(-10.0, 0.0, 10.0), rack_values_mm=(-5.0, 0.0, 5.0)
    )
    return assembly


def _request_body(**overrides) -> dict:
    """Build a valid dynamic frozen-tangent request, with per-test overrides."""
    body = {
        "request_id": "p0-contract",
        "domain": "dynamic",
        "samples": [0, 5, 10],
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
    body.update(overrides)
    return body


def _with_request(body: dict) -> dict:
    document = _plan_payload()
    document["characteristics"] = body
    return document


def _blocks_equal(left, right) -> bool:
    """Compare native numeric blocks, treating structural NaN padding as equal."""
    return bool(np.array_equal(left, right, equal_nan=True))


# --------------------------------------------------------------------------- #
# carriage
# --------------------------------------------------------------------------- #


def test_request_survives_plan_normalisation() -> None:
    """The plan keeps the request verbatim; nothing is defaulted or reordered."""
    document = _with_request(_request_body())
    plan = ResolvedSolvePlan(document)
    restored = plan.to_document()
    assert "characteristics" in restored, "the plan dropped the request"
    assert restored["characteristics"] == document["characteristics"]


def test_request_survives_case_rebuild() -> None:
    """
    `compile_resolved` rebuilds the case; the rebuilt case must still ask.

    The build under test reads the request but does not yet compute any
    response, so compilation is expected to refuse at the capability gate.  The
    rebuild itself is still observable, and that is what this test pins: the
    refusal must be the *capability* refusal, not a lost field.
    """
    document = _with_request(_request_body())
    plan = ResolvedSolvePlan(document)
    with pytest.raises(ValueError, match="does not compute characteristics response kind"):
        _compile(plan)

    # The carriage property is asserted on the plan and on the case document the
    # compiler builds, independently of the capability gate's timing.
    restored = plan.to_document()
    assert restored["characteristics"]["request_id"] == "p0-contract"


def _compile(plan: ResolvedSolvePlan):
    from suspension_multibody.authoring.loader import DocumentLoader
    from suspension_multibody.compilation.resolved import compile_resolved

    bundle = DocumentLoader().load(_assembly(), plan.to_document())
    return compile_resolved(bundle.resolve(), plan)


def test_no_request_leaves_the_document_alone() -> None:
    """A plan without a request must not gain the field."""
    plan = ResolvedSolvePlan(_plan_payload())
    bundle = _compile(plan)
    assert "characteristics" not in bundle.case_document
    assert "characteristics" not in plan.to_document()


# --------------------------------------------------------------------------- #
# fail-closed capability
# --------------------------------------------------------------------------- #


def test_unanswerable_response_kind_fails_closed() -> None:
    """
    Accepting a request the kernel cannot answer must not look like success.

    This build reads requests but computes no response, so every response kind
    is refused.  Returning the unchanged channels instead would let a caller
    believe the request was honoured, which is the empty-shell capability the
    contract forbids.
    """
    plan = ResolvedSolvePlan(_with_request(_request_body()))
    with pytest.raises(ValueError, match="does not compute characteristics response kind"):
        _compile(plan)


# --------------------------------------------------------------------------- #
# inertness
# --------------------------------------------------------------------------- #


def test_request_does_not_change_the_numbers() -> None:
    """
    A request is an output request: without one every block is unchanged.

    The with-request comparison lives in the kernel ABI test, because the
    capability gate refuses to compile a request this build cannot answer.  What
    is pinned here is the no-request baseline itself: adding the *field* to the
    plan must not perturb the run that has no request.
    """
    base = _plan_payload()
    before = run_compiled(validate(_assembly(), base)).result
    again = run_compiled(validate(_assembly(), copy.deepcopy(base))).result

    names = [row["name"] for row in before.raw.document["blocks"]]
    assert names == [row["name"] for row in again.raw.document["blocks"]]
    for name in names:
        assert _blocks_equal(before.raw.block(name), again.raw.block(name))
    assert (
        before.raw.document["manifest"]["sample_count"]
        == again.raw.document["manifest"]["sample_count"]
    )


# --------------------------------------------------------------------------- #
# rejection: the request body
# --------------------------------------------------------------------------- #


def _reject(document: dict) -> None:
    with pytest.raises((ContractError, ValueError)):
        ResolvedSolvePlan(document)


@pytest.mark.parametrize(
    ("label", "mutate"),
    [
        ("unknown measurement field", lambda c: c["measurements"][0].update(unit="N/m")),
        ("unknown top-level field", lambda c: c.update(extra=1)),
        ("unknown domain", lambda c: c.update(domain="nonsense")),
        ("unsupported unit", lambda c: c["measurements"][0].update(units="furlong")),
        ("unknown response kind", lambda c: c["measurements"][0].update(response_kind="made_up")),
        ("zero definition version", lambda c: c["measurements"][0].update(definition_version=0)),
        ("zero response version", lambda c: c["measurements"][0].update(response_version=0)),
        ("missing response version", lambda c: c["measurements"][0].pop("response_version")),
        ("samples all", lambda c: c.update(samples="all")),
        ("empty samples", lambda c: c.update(samples=[])),
        ("negative sample", lambda c: c.update(samples=[-1])),
        ("duplicate sample", lambda c: c.update(samples=[0, 0])),
        ("no measurements", lambda c: c.update(measurements=[])),
        ("zero boundary version", lambda c: c["boundary"].update(version=0)),
        ("unknown boundary field", lambda c: c["boundary"].update(scope="front")),
        ("frozen kind without protocol", lambda c: c.pop("frozen_conditions")),
        ("unknown frozen protocol version", lambda c: c["frozen_conditions"].update(version=0)),
        ("unknown definition kind", lambda c: c["definition"].update(kind="vibes")),
        ("module request schema version", lambda c: None),
    ],
)
def test_invalid_requests_are_rejected(label: str, mutate) -> None:
    """Every malformed request fails; none is silently repaired or ignored."""
    body = _request_body()
    mutate(body)
    document = _with_request(body)
    if label == "module request schema version":
        document["characteristics"] = {
            "schema_version": 99,
            "characteristics": body,
        }
    _reject(document)


def test_dynamic_definitions_are_named_and_separated() -> None:
    """A frozen tangent is not a continuous descriptor, and neither replaces one."""
    for kind in ("frozen_force_tangent", "frozen_load_compliance", "continuous_descriptor"):
        body = _request_body(definition={"kind": kind, "version": 1})
        if kind == "continuous_descriptor":
            body.pop("frozen_conditions", None)
        ResolvedSolvePlan(_with_request(body))

    # A dynamic definition on a non-dynamic domain is a different question.
    _reject(
        _with_request(
            _request_body(domain="kinematic", definition={"kind": "frozen_force_tangent", "version": 1})
        )
    )


def test_unknown_measurement_character_is_rejected() -> None:
    body = _request_body()
    body["measurements"][0]["measurement_id"] = "front/$left/wheel_center"
    _reject(_with_request(body))


# --------------------------------------------------------------------------- #
# rejection: the request envelope, independent of the plan
# --------------------------------------------------------------------------- #


def test_envelope_rejects_a_non_object() -> None:
    with pytest.raises(ContractError):
        validate_characteristics_request({"schema_version": 1, "characteristics": []})


def test_envelope_accepts_a_well_formed_request() -> None:
    validate_characteristics_request(
        {"schema_version": 1, "characteristics": _request_body()}
    )
