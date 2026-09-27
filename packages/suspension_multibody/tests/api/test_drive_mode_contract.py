"""
The case contract's three drive modes, and the legacy boolean that maps onto them.

`drive_mode` selects *how a run decides the assembly's pose*, which is a property of the
analysis rather than of the model:

* ``kinematics``    -- the constraint equations alone; nothing elastic, no tire;
* ``force_balance`` -- the elastic elements balance against the driven targets (default);
* ``pad``           -- a ground height is driven instead of the wheel centre.

Two things are asserted here that would otherwise only be discovered in production:

* the **normative** JSON schema accepts the new fields.  It has
  ``additionalProperties: false`` at both the top level and under ``k``, so a Python-side
  field that the schema does not know about is rejected by ``validate_case`` -- the failure
  a caller would see as "unexpected fields";
* the legacy ``drive_wheels`` boolean keeps its meaning.  It is spelled in 29 files across
  the repository, so it stays supported, and the mapping lives in one place rather than
  being re-derived by each caller.
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from suspension_contracts import validate_case

from suspension_multibody.schema.case import (
    DRIVE_MODES,
    DRIVE_WHEELS_TO_MODE,
    CaseSpec,
    drive_mode_for,
)


def _case_document() -> dict:
    """Return a minimal but valid K case document, as the contract writes one."""
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "drive-mode-probe",
        "k": {
            "wheel_values_mm": [0.0],
            "rack_values_mm": [0.0],
            "axis_map": {
                "wheel": ["wheel_drive_L", "wheel_drive_R"],
                "rack": "rack_drive",
            },
        },
    }


def test_force_balance_is_the_default():
    assert CaseSpec(name="t", mode="K").drive_mode == "force_balance"


def test_the_three_modes_are_the_whole_set():
    for mode in DRIVE_MODES:
        assert CaseSpec(name="t", mode="K", drive_mode=mode).drive_mode == mode


def test_an_unknown_mode_is_refused_and_the_message_names_the_valid_ones():
    with pytest.raises(ValidationError) as caught:
        CaseSpec(name="t", mode="K", drive_mode="bogus")  # type: ignore[arg-type]
    message = str(caught.value)
    for mode in DRIVE_MODES:
        assert mode in message, f"the error should name {mode!r} as valid"


def test_the_legacy_boolean_maps_onto_the_modes():
    """`True` is the wheel-centre reading, `False` is the C/loaded one."""
    assert drive_mode_for(True) == "force_balance"
    assert drive_mode_for(False) == "pad"
    assert DRIVE_WHEELS_TO_MODE == {True: "force_balance", False: "pad"}


def test_an_absent_boolean_yields_the_default():
    assert drive_mode_for(None) == "force_balance"
    assert drive_mode_for(None, default="kinematics") == "kinematics"


def test_the_normative_schema_accepts_drive_mode():
    """
    The JSON schema is the contract's normative face, and it forbids unknown fields.

    Asserting against it (not only against the Python model) is what proves the schema was
    updated too: a Python-only field would be rejected here as "unexpected fields".
    """
    document = _case_document()
    document["drive_mode"] = "pad"
    validate_case(document)


def test_the_normative_schema_accepts_pad_heights():
    document = _case_document()
    document["k"]["pad_height_mm"] = [0.0, 5.0, 10.0, 20.0]
    validate_case(document)


def test_the_normative_schema_refuses_an_unknown_mode():
    document = _case_document()
    document["drive_mode"] = "bogus"
    with pytest.raises(Exception, match="bogus"):
        validate_case(document)
