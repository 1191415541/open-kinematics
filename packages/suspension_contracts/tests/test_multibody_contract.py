"""Multibody Contract V1: canonical form, container and validation."""

from __future__ import annotations

import math
import random
import struct

import pytest

from suspension_contracts import (
    CONTRACT_MAGIC,
    CONTRACT_VERSION,
    ContractError,
    blob_slice,
    canonical_json,
    contract_hash,
    pack_container,
    parse_json,
    unpack_container,
    validate_case,
    validate_model,
    validate_result,
)


def _model() -> dict:
    return {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": "minimal",
        "units": {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"},
        "bodies": [
            {
                "name": "chassis",
                "mass": 1000.0,
                "inertia": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            }
        ],
    }


def _tire_model() -> dict:
    """Return a minimal model with one tire, for the tire-entry coverage."""
    document = _model()
    document["tires"] = [
        {
            "name": "front_left",
            "model": "native_brush",
            "body": "chassis",
        }
    ]
    return document


def _case() -> dict:
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "grid",
        "k": {
            "wheel_values_mm": [-10.0, 0.0, 10.0],
            "rack_values_mm": [-5.0, 0.0, 5.0],
            "drive": "wheel_center",
            "left_right_mode": "symmetric",
        },
    }


def _result() -> dict:
    return {
        "contract": "multibody-result",
        "contract_version": 1,
        "kind": "result",
        "case_identity": {"model_sha256": "a" * 64, "case_sha256": "b" * 64},
        "status": "success",
    }


def test_canonical_form_is_key_order_independent() -> None:
    left = {"b": 1, "a": {"y": 2, "x": 3}}
    right = {"a": {"x": 3, "y": 2}, "b": 1}
    assert canonical_json(left) == canonical_json(right)
    assert contract_hash(left) == contract_hash(right)


def test_canonical_form_uses_seventeen_significant_digits() -> None:
    assert canonical_json({"x": 0.1}) == '{"x":0.10000000000000001}'


def test_float_round_trip_is_bit_exact() -> None:
    """Gate A depends on this: a double must survive the canonical text form."""
    rng = random.Random(20260917)
    values = [
        0.0,
        -0.0,
        1.0,
        -1.0,
        math.pi,
        1e-300,
        1e300,
        5e-324,
        2.2250738585072014e-308,
        0.1,
        1.0 / 3.0,
    ]
    values.extend(struct.unpack(">d", rng.randbytes(8))[0] for _ in range(5000))
    checked = 0
    for value in values:
        if math.isnan(value) or math.isinf(value):
            continue
        text = canonical_json({"v": value})
        restored = parse_json(text)["v"]
        assert struct.pack(">d", restored) == struct.pack(">d", value), text
        checked += 1
    assert checked > 4000


def test_nan_and_infinity_are_rejected() -> None:
    for value in (math.nan, math.inf, -math.inf):
        with pytest.raises(ContractError):
            canonical_json({"v": value})


def test_container_round_trip_carries_the_blob() -> None:
    blob = struct.pack("<4d", 1.0, 2.0, 3.0, 4.0)
    payload = pack_container(_model(), blob)
    assert payload[:4] == CONTRACT_MAGIC
    assert struct.unpack_from("<I", payload, 4)[0] == CONTRACT_VERSION
    document, restored = unpack_container(payload)
    assert document == _model()
    assert restored == blob
    assert blob_slice(restored, {"offset": 8, "length": 16}) == blob[8:24]


def test_container_rejects_a_truncated_payload() -> None:
    payload = pack_container(_model())
    with pytest.raises(ContractError):
        unpack_container(payload[:-1])


def test_container_rejects_foreign_magic() -> None:
    payload = b"NOPE" + pack_container(_model())[4:]
    with pytest.raises(ContractError):
        unpack_container(payload)


def test_blob_slice_rejects_out_of_range_descriptors() -> None:
    with pytest.raises(ContractError):
        blob_slice(b"1234", {"offset": 2, "length": 8})


def test_model_validation_accepts_a_minimal_document() -> None:
    validate_model(_model())


def test_tire_validation_accepts_an_entry_with_mass_and_inertia() -> None:
    """The tire's own mass and inertia are optional entries of the tire object."""
    document = _tire_model()
    tire = document["tires"][0]
    tire["mass"] = 12.5
    tire["inertia"] = [[0.4, 0.0, 0.0], [0.0, 0.7, 0.0], [0.0, 0.0, 0.7]]
    validate_model(document)


def test_tire_validation_rejects_a_malformed_mass_by_name() -> None:
    """A mass of the wrong type is refused, and the message names the field."""
    document = _tire_model()
    document["tires"][0]["mass"] = "heavy"
    with pytest.raises(ContractError) as captured:
        validate_model(document)
    assert "$/tires[0]/mass" in str(captured.value)
    assert "expected number" in str(captured.value)


def test_tire_validation_rejects_a_negative_or_non_finite_mass_by_name() -> None:
    """A mass below zero and a non-finite mass are both refused by name."""
    for bad in (-1.0, math.inf, math.nan):
        document = _tire_model()
        document["tires"][0]["mass"] = bad
        with pytest.raises(ContractError) as captured:
            validate_model(document)
        message = str(captured.value)
        assert "$/tires[0]/mass" in message, bad
        assert ("must be >= 0" in message) or ("must be finite" in message), bad


def test_tire_validation_rejects_a_malformed_inertia_by_name() -> None:
    """
    Check that a tensor with a bad entry is refused by name.

    The four entries cover the wrong shape in both directions, a non-numeric
    element and a negative element; each message has to name the inertia path.
    """
    cases = (
        ([[1.0, 0.0], [0.0, 1.0]], "needs at least 3 items"),
        ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
         "at most 3 items"),
        ([[1.0, 0.0, 0.0], [0.0, "x", 0.0], [0.0, 0.0, 1.0]], "expected number"),
        ([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]], "must be >= 0"),
    )
    for bad, expected in cases:
        document = _tire_model()
        document["tires"][0]["inertia"] = bad
        with pytest.raises(ContractError) as captured:
            validate_model(document)
        message = str(captured.value)
        assert "$/tires[0]/inertia" in message, bad
        assert expected in message, (bad, message)


def test_tire_validation_rejects_an_unknown_tire_field_by_name() -> None:
    """`additionalProperties: false` still holds for the new optional pair."""
    document = _tire_model()
    document["tires"][0]["unsprung_mass"] = 12.5
    with pytest.raises(ContractError) as captured:
        validate_model(document)
    message = str(captured.value)
    assert "$/tires[0]" in message
    assert "unsprung_mass" in message


def test_model_validation_rejects_an_unknown_field() -> None:
    document = _model()
    document["extra"] = 1
    with pytest.raises(ContractError):
        validate_model(document)


def test_model_validation_rejects_a_missing_required_field() -> None:
    document = _model()
    del document["bodies"]
    with pytest.raises(ContractError):
        validate_model(document)


def test_model_validation_rejects_a_unknown_element_type() -> None:
    document = _model()
    document["elements"] = [{"name": "x", "type": "not_a_real_element"}]
    with pytest.raises(ContractError):
        validate_model(document)


def test_case_validation_accepts_every_declared_family() -> None:
    for family in (
        "kc_quasi_static",
        "axle_dynamic",
        "vehicle_kc",
        "vehicle_dynamic",
        "handling",
        "ride_four_post",
        "ride_random_road",
        "comparison",
    ):
        document = _case()
        document["family"] = family
        validate_case(document)


def test_case_validation_rejects_an_unknown_family() -> None:
    document = _case()
    document["family"] = "not_a_family"
    with pytest.raises(ContractError):
        validate_case(document)


def test_result_validation_requires_a_case_identity() -> None:
    document = _result()
    del document["case_identity"]
    with pytest.raises(ContractError):
        validate_result(document)


def test_result_validation_rejects_a_short_identity_hash() -> None:
    document = _result()
    document["case_identity"]["model_sha256"] = "abc"
    with pytest.raises(ContractError):
        validate_result(document)
