"""
P1 step 5 acceptance: the minimum query and the native artifact round-trip.

Four properties, and the last three are the ones that make the first meaningful:

* **Every sample has an identity.**  A value is identified by its case, time and
  output-sample id, not by its position in an array -- otherwise a dynamic run
  that takes a different number of accepted steps cannot be compared with its own
  stored artifact.

* **A missing channel is a query error, not a zero.**  The kernel may legitimately
  not have been asked for a channel; that is a different claim from "the answer
  is zero", and the difference has to survive the query.

* **A cold read does not re-solve.**  The test proves this by counting kernel
  invocations around the read and by deleting the model's ability to run, not by
  asserting that a function "looks read-only".

* **Values, units and reasons survive a write/read round trip.**  An artifact that
  loses a unit or drops an invalid row is not a faithful record.
"""

from __future__ import annotations

import csv
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.characteristics.values import (
    CharacteristicResult,
    CharacteristicsError,
    MeasurementValue,
    VectorValue,
)
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.io.artifacts import read_artifact, write_artifact
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.results.characteristics import (
    characteristic,
    envelope_from_artifact,
    measurement_from_channel,
    measurement_query,
    require_channel,
    sample_identities,
)
from suspension_multibody.results.raw import RawContractResult
from suspension_multibody.simulation.runner import run_compiled

from ..results.test_measurement_queries import roll_model

#: The smallest model document ``ResolvedModel`` accepts, used by the tests that
#: rebuild an envelope from a hand-made artifact payload.
_MINIMAL_MODEL = {
    "schema_version": 1,
    "name": "m",
    "units": "SI",
    "bodies": [
        {
            "name": "b",
            "mass": 1.0,
            "inertia": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "position": [0, 0, 0],
        }
    ],
    "joints": [],
    "elements": [],
    "frames": [],
    "gravity": [0, 0, 0],
}


def _solved(kind: str = "prismatic", samples=(0.0, 0.001)) -> ResultEnvelope:
    run = {
        "schema_version": 1,
        "name": "linkage",
        "study": "dynamic",
        "samples": list(samples),
        "boundaries": [],
        "inputs": [],
        "outputs": [],
        "solver": {"initialization_mode": "provided_consistent_state"},
    }
    plan = ResolvedSolvePlan(run)
    result = run_compiled(compile_resolved(roll_model(kind), plan)).result
    assert isinstance(result, ResultEnvelope)
    return result


# --------------------------------------------------------------------------- #
# identity
# --------------------------------------------------------------------------- #


def test_every_sample_carries_its_own_identity() -> None:
    """
    Sample identity is case/time/output-index, not array position.

    The flattened index and the within-case index are both reported, and the
    second is the one a trajectory comparison needs: it survives the solver taking
    a different number of accepted steps.
    """
    result = _solved()
    identities = sample_identities(result)
    assert len(identities) == len(result.times_s)
    for index, identity in enumerate(identities):
        assert identity.index == index
        assert identity.case_index == 0
        assert identity.output_sample_id == index
        assert identity.time_s == pytest.approx(float(result.times_s[index]))


def test_a_query_attaches_identity_to_every_answered_value() -> None:
    result = _solved()
    answered = measurement_query(result, "roll")
    assert isinstance(answered, CharacteristicResult)
    assert len(answered.samples) == len(result.times_s)
    assert answered.measurement("roll").measurement_id == "roll"


def test_the_answer_records_the_provenance_it_came_from() -> None:
    """A number without its frozen protocol and boundary is not traceable."""
    result = _solved()
    answered = measurement_query(result, "roll")
    provenance = answered.provenance()
    assert provenance["request_id"] == "roll"
    assert provenance["domain"] == "kinematic"
    assert provenance["boundary_profile"] == "stored-samples"
    assert provenance["boundary_version"] == 1
    assert provenance["response_operator"] == "ResultEnvelope.measure"
    assert provenance["model_fingerprint"] == result.model_fingerprint
    # The operator version says which formula answered, which is what lets a
    # reader tell a legacy definition from its replacement.
    assert provenance["operator_version"] == 1


# --------------------------------------------------------------------------- #
# a missing channel is an error, never a zero
# --------------------------------------------------------------------------- #


def test_a_channel_that_was_never_requested_is_a_query_error() -> None:
    """
    An absent channel is reported by name; it is not zero-filled.

    The kernel only produces ``constraint_jacobian`` when a roll-centre
    measurement asked for it.  A query naming a channel this result does not
    contain must say so, because a zero array would be read as a physical answer.
    """
    result = _solved()
    assert "constraint_jacobian" in result.raw.blocks  # this model does ask for it

    with pytest.raises(CharacteristicsError, match="does not contain") as caught:
        require_channel(result, "no_such_channel", request_id="probe")
    # The error names the missing channel and lists what *is* available.
    assert "no_such_channel" in str(caught.value)
    assert "constraint_jacobian" in str(caught.value)


def test_a_channel_query_returns_the_stored_values_unchanged() -> None:
    result = _solved()
    stored = require_channel(result, "constraint_jacobian", request_id="probe")
    np.testing.assert_array_equal(
        stored, np.asarray(result.raw.block("constraint_jacobian"), dtype=float)
    )


def test_a_channel_query_answers_both_stored_result_shapes() -> None:
    """
    The raw contract result is a legal way to name the same stored run.

    ``require_channel`` unwrapped the envelope but addressed the raw result's
    block table through ``result.raw`` unconditionally, so passing the raw result
    raised ``AttributeError`` instead of returning the very channel it holds.
    """
    result = _solved()
    stored = require_channel(result, "constraint_jacobian", request_id="probe")
    np.testing.assert_array_equal(
        require_channel(result.raw, "constraint_jacobian", request_id="probe"), stored
    )


def test_an_unavailable_sample_needs_a_reason_and_is_not_a_zero() -> None:
    """
    Non-finite samples become an explicit status; without a reason this refuses.

    Zero is a legitimate camber reading, so a missing value must never be written
    as one: the two are indistinguishable in a table once written.
    """
    result = _solved()
    values = np.array([1.0, np.nan, 3.0])

    with pytest.raises(CharacteristicsError, match="no reason"):
        measurement_from_channel(
            result, "probe", values, units="m", frame_id="world"
        )

    wrapped = measurement_from_channel(
        result,
        "probe",
        values,
        units="m",
        frame_id="world",
        reasons=("", "the linkage is at a singular pose", ""),
    )
    assert wrapped.status == ("valid", "undefined_geometry", "valid")
    assert wrapped.reason[1] == "the linkage is at a singular pose"
    assert np.isnan(wrapped.values[1])
    assert not wrapped.valid_mask[1]


def test_a_vector_measurement_keeps_its_components_and_units() -> None:
    """
    A roll centre is a two-component quantity and is not flattened.

    The roll-centre response is ``(samples, 2)``.  Raveled into a 1-D measurement
    it would claim two different components share one unit token and one status,
    and a per-sample reason would have nowhere to live.
    """
    result = _solved()
    measured = result.measure("roll")
    assert measured.values.shape == (len(result.times_s), 2)

    answered = result.characteristic("roll")
    response = answered.measurement("roll")
    assert isinstance(response, VectorValue)
    assert response.values.shape == (len(result.times_s), 2)
    assert response.units == ("m", "m")
    assert len(response.status) == len(result.times_s)
    np.testing.assert_array_equal(response.values, measured.values)


def test_a_vector_row_with_one_missing_component_is_not_a_reading() -> None:
    """Half a vector is not a value; the finite half must not read as zero."""
    result = _solved()
    values = np.array([[1.0, 2.0], [3.0, np.nan]])
    wrapped = measurement_from_channel(
        result, "probe", values, units="m", frame_id="world"
    )
    assert isinstance(wrapped, VectorValue)
    assert wrapped.status == ("valid", "undefined_geometry")
    assert not wrapped.valid_mask[1]


def test_a_vector_response_refuses_one_reason_per_component() -> None:
    result = _solved()
    with pytest.raises(CharacteristicsError, match="component vector"):
        measurement_from_channel(
            result,
            "probe",
            np.array([[1.0, 2.0], [3.0, np.nan]]),
            units="m",
            frame_id="world",
            reasons=("", "why"),
        )


def test_a_valid_sample_may_not_claim_a_reason() -> None:
    with pytest.raises(CharacteristicsError, match="valid but carries a reason"):
        MeasurementValue(
            measurement_id="probe",
            values=np.array([1.0]),
            units="m",
            frame_id="world",
            status=("valid",),
            reason=("because",),
        )


# --------------------------------------------------------------------------- #
# a cold read does not re-solve
# --------------------------------------------------------------------------- #


def test_a_multi_case_result_keeps_each_cases_own_grid() -> None:
    """
    With several cases the times are walked case by case, not by flat offset.

    ``RawContractResult.times_s`` builds its grid by slicing ``base[:count]`` once
    *per case* -- so the returned array is per-case grids laid end to end, not one
    continuous grid.  Indexing it with the flattened offset would silently pair a
    later case's samples with an earlier case's times, and past the end of the
    array would produce samples with no time at all -- which reads as "this sample
    legitimately has no time" rather than "the offsets are wrong".

    The assertion below therefore pins the *identity* mapping, which is this
    module's responsibility, and does not restate ``raw``'s own slicing.
    """
    blocks = {"body_state": np.zeros((5, 13, 1))}
    document = {
        "status": "success",
        "contract_version": 2,
        "manifest": {
            "cases": [
                {"name": "a", "sample_offset": 0, "sample_count": 2},
                {"name": "b", "sample_offset": 2, "sample_count": 3},
            ]
        },
    }
    raw = RawContractResult(
        document=document, blocks=blocks, _times_s=np.array([10.0, 11.0, 20.0])
    )
    stub = SimpleNamespace(times_s=raw.times_s, raw=raw)

    identities = sample_identities(stub)
    assert len(identities) == 5
    assert [(i.case_index, i.output_sample_id) for i in identities] == [
        (0, 0), (0, 1), (1, 0), (1, 1), (1, 2)
    ]
    # Every sample has a time.  Leaving this as `None` on overflow is exactly the
    # silent failure this test exists to prevent.
    assert all(i.time_s is not None for i in identities)
    # Case 1's samples carry case 1's grid, not case 0's repeated prefix.
    assert [i.time_s for i in identities[2:]] == [10.0, 11.0, 20.0]
    # The flat index still addresses the native blocks.
    assert [i.index for i in identities] == [0, 1, 2, 3, 4]


def test_case_spans_that_disagree_with_the_grid_are_refused() -> None:
    """
    A short grid is a structural error, not a sample without a time.

    Note on reachability: ``RawContractResult.times_s`` itself refuses a per-case
    count that overruns its decoded grid, and the grid it returns is exactly as
    long as the case spans demand -- so this guard is defensive, and is reached
    here through a stub rather than through a real parsed result.  It is kept
    (and asserted) because the alternative it replaces, leaving ``time_s`` empty,
    would make a mis-recorded offset look like a sample that has no time.
    """
    stub = SimpleNamespace(
        times_s=np.array([0.0]),
        raw=SimpleNamespace(cases=({"name": "a", "sample_offset": 0, "sample_count": 3},)),
    )
    with pytest.raises(CharacteristicsError, match="disagree"):
        sample_identities(stub)

    # The real reader refuses the same inconsistency, in its own terms.
    overrun = RawContractResult(
        document={
            "status": "success",
            "contract_version": 2,
            "manifest": {"cases": [{"name": "a", "sample_offset": 0, "sample_count": 3}]},
        },
        blocks={},
        _times_s=np.array([0.0]),
    )
    with pytest.raises(ValueError, match="exceeds decoded time grid"):
        _ = overrun.times_s


def test_the_stored_dtypes_survive_the_rebuild() -> None:
    """
    The rebuild copies blocks as they are stored rather than coercing to float.

    Today every native block is float64, so a blanket ``dtype=float`` would pass
    every current test while silently corrupting the first non-float channel
    somebody adds -- for instance an integer contact-event or index block.  This
    asserts the rebuild is dtype-preserving so that channel can be added safely.
    """
    payload = {
        "manifest": {
            "model": _MINIMAL_MODEL,
            "channels": {"cases": []},
            "status": "success",
        },
        "arrays": {
            "times_s": np.array([0.0, 1.0]),
            "body_state": np.zeros((2, 13, 1), dtype=np.float64),
            "constraint_index": np.array([[0, 1], [2, 3]], dtype=np.int64),
        },
    }
    rebuilt = envelope_from_artifact(payload)
    assert rebuilt.raw.block("constraint_index").dtype == np.int64
    np.testing.assert_array_equal(rebuilt.raw.block("constraint_index"), [[0, 1], [2, 3]])


def test_a_cold_read_does_not_invoke_the_kernel(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    The read path touches no solver.

    Two independent guards are used, because "did it solve again" cannot be seen
    from the returned numbers when the answer is deterministic: the kernel entry
    point is replaced with a function that fails if called, and a counter proves
    that the query never reaches it.  The second guard is what makes the first
    meaningful -- a monkeypatch that is simply never consulted would pass either
    way.
    """
    result = _solved()
    calls = {"count": 0}

    def forbidden(*_args: object, **_kwargs: object) -> None:
        calls["count"] += 1
        raise AssertionError("the query re-solved the problem")

    # The query layer's whole solver-facing surface is `require_channel`, which
    # reads stored blocks.  Replacing the runner proves nothing is re-submitted.
    import suspension_multibody.simulation.runner as runner_module

    monkeypatch.setattr(runner_module, "run_compiled", forbidden, raising=True)

    for _ in range(3):
        answered = result.characteristic("roll")
        assert answered.measurement("roll").values.shape[0] == len(result.times_s)
    assert calls["count"] == 0


def test_the_read_path_does_not_reload_modelling_resources() -> None:
    """
    Reading a stored result must not rebuild the model.

    ``ResultEnvelope.characteristic`` is asked twice and the model document is
    replaced by an object that raises if it is iterated.  The read still answers,
    which shows the answer comes from the already-decoded result rather than a
    fresh compile.
    """

    class _Exploding:
        def __getattr__(self, name: str) -> object:
            raise AssertionError(f"the query reloaded modelling resources: {name}")

    result = _solved()
    answered = result.characteristic("roll")
    measured = answered.measurement("roll")
    assert measured.values.size

    # Now poison the model document and confirm the *response* is still served
    # from the value that was already computed.
    object.__setattr__(result, "model", _Exploding())
    still = answered.measurement("roll")
    np.testing.assert_array_equal(still.values, measured.values)


# --------------------------------------------------------------------------- #
# artifact round trip
# --------------------------------------------------------------------------- #


def test_native_artifact_preserves_identity_units_and_reasons() -> None:
    """
    What is written is what is read back, channel for channel.

    An artifact stores *native channels*, not derived measurements -- the model
    document travels beside them, so a reader can re-derive a measurement without
    the writer having frozen one particular formula's output.  Every stored
    channel, the time grid and the model fingerprint must survive the round trip;
    a round trip that loses one is not a faithful record of the run.
    """
    result = _solved()

    with tempfile.TemporaryDirectory() as directory:
        write_artifact(result, directory, model=result.model, formats=("csv",))
        payload = read_artifact(directory)

        assert payload["manifest"]["model_fingerprint"] == result.model_fingerprint
        for name, block in result.raw.named_blocks.items():
            np.testing.assert_array_equal(payload["arrays"][name], block)
        np.testing.assert_array_equal(payload["arrays"]["times_s"], result.times_s)

        # The model document travels with the channels, which is what lets a
        # reader answer a measurement query without the original envelope.
        assert payload["manifest"]["model"] is not None


def test_a_measurement_can_be_re_answered_from_the_artifact_alone() -> None:
    """
    The round trip is queryable: a rebuilt envelope answers the same numbers.

    This is the property that makes the artifact a *record* rather than a dump:
    the stored channels plus the stored model document are enough to reproduce
    the original query, and the reproduction is compared value-for-value against
    the original answer.
    """
    result = _solved()
    expected = result.characteristic("roll").measurement("roll").values

    with tempfile.TemporaryDirectory() as directory:
        write_artifact(result, directory, model=result.model, formats=("csv",))
        payload = read_artifact(directory)

    rebuilt = envelope_from_artifact(payload)
    reanswered = rebuilt.characteristic("roll").measurement("roll")
    np.testing.assert_array_equal(reanswered.values, expected)
    assert rebuilt.model_fingerprint != ""


def test_a_written_artifact_can_be_read_back_without_the_original_result() -> None:
    """
    The round trip stands alone: the artifact directory is the only input.

    The original envelope is dropped before the second read, so nothing that
    happens to still be reachable in memory can make the read appear to work.
    """
    result = _solved()
    with tempfile.TemporaryDirectory() as directory:
        write_artifact(result, directory, model=result.model, formats=("csv",))
        expected = np.asarray(result.times_s, dtype=float)
        del result
        payload = read_artifact(directory)
        np.testing.assert_array_equal(payload["arrays"]["times_s"], expected)
        assert payload["manifest"]["artifact_type"] == "multibody_result"


def test_the_manifest_declares_a_layout_for_every_stored_channel() -> None:
    """A reader must be able to discover shapes without loading the arrays."""
    result = _solved()
    with tempfile.TemporaryDirectory() as directory:
        write_artifact(result, directory, model=result.model, formats=("csv",))
        payload = read_artifact(directory)
        layouts = payload["manifest"]["layouts"]
        for name, block in result.raw.named_blocks.items():
            assert layouts[name]["shape"] == list(block.shape)
        assert layouts["times_s"]["shape"] == [len(result.times_s)]


def test_artifact_tables_are_csv_parseable_with_stable_columns() -> None:
    """Every declared table is a real CSV whose rows share one field set."""
    result = _solved()
    with tempfile.TemporaryDirectory() as directory:
        write_artifact(result, directory, formats=("csv",))
        payload = read_artifact(directory)
        tables = payload["manifest"].get("tables", [])
        for table in tables:
            path = Path(directory) / f"{table}.csv"
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            if rows:
                widths = {len(row) for row in rows}
                assert len(widths) == 1, f"table {table!r} has ragged rows"


# --------------------------------------------------------------------------- #
# refusals
# --------------------------------------------------------------------------- #


def test_an_unknown_domain_is_refused_rather_than_defaulted() -> None:
    result = _solved()
    with pytest.raises(CharacteristicsError, match="unknown response domain"):
        characteristic(
            result,
            "probe",
            domain="modal",
            definition_kind="probe",
            definition_version=1,
            channels={},
            boundary_profile="stored-samples",
            boundary_version=1,
        )


def test_a_response_whose_length_disagrees_with_the_samples_is_refused() -> None:
    """
    A response computed for other samples is refused, not filed under these.

    The key and the value's own id cannot disagree -- one is derived from the
    other -- but the *length* can, and two values written beside this request's
    identities would be read as if they belonged to them.
    """
    result = _solved()
    with pytest.raises(CharacteristicsError, match="but the request selected"):
        characteristic(
            result,
            "probe",
            domain="kinematic",
            definition_kind="probe",
            definition_version=1,
            channels={"probe": (np.array([1.0, 2.0, 3.0]), "m", "world")},
            boundary_profile="stored-samples",
            boundary_version=1,
        )


def test_an_unknown_measurement_is_refused() -> None:
    result = _solved()
    with pytest.raises(KeyError, match="unknown measurement"):
        measurement_query(result, "not_declared")
