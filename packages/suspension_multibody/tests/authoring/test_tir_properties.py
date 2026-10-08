"""TIR property bindings retain source units, curves and explicit scope errors."""
from pathlib import Path

import pytest

from suspension_multibody.authoring.errors import ElementPropertyError
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.properties.tir import parse_tire_tables_text, parse_tire_text

TIR = """[UNITS]
LENGTH = 'meter'
FORCE = 'newton'
TIME = 'second'
ANGLE = 'radian'
MASS = 'kg'
[MODEL]
PROPERTY_FILE_FORMAT = 'PAC2002'
USE_MODE = 14
TYRESIDE = 'RIGHT'
[DIMENSION]
UNLOADED_RADIUS = 0.344D+0
[VERTICAL]
VERTICAL_STIFFNESS = 200000
VERTICAL_DAMPING = 50
FNOMIN = 4850
[DEFLECTION_LOAD_CURVE]
0.0 0.0
0.01 1000.0
"""


def test_tir_file_and_memory_produce_identical_law_and_hash(tmp_path: Path) -> None:
    path = tmp_path / "law.tir"
    path.write_text(TIR, encoding="ascii")
    file = ElementPropertyDocument.load(path, expected_type="tire", allowed_models=["pac2002"])
    memory = ElementPropertyDocument.from_tir_text(TIR, name="law")
    assert file.content_hash == memory.content_hash
    assert file.effective_values_hash == memory.effective_values_hash
    assert file.resolved["pac2002_coefficients"] == memory.resolved["pac2002_coefficients"]
    assert file.resolved["pac2002_tables"] == {"deflection_load_curve": ((0.0, 0.0), (0.01, 1000.0))}
    assert file.resolved["unloaded_radius"] == 344.0
    assert file.resolved["vertical_stiffness"] == 200.0
    assert file.resolved["vertical_damping"] == 0.05
    assert file.resolved["pac2002_coefficients"]["USE_MODE"] == -14.0


def test_tir_parsing_keeps_fortran_numbers_and_separate_si_tables() -> None:
    assert parse_tire_text(TIR)["UNLOADED_RADIUS"] == 0.344
    assert parse_tire_tables_text(TIR)["deflection_load_curve"][1] == (0.01, 1000.0)


@pytest.mark.parametrize("change,field", [
    ("BELT_DYNAMICS = 'YES'", "BELT_DYNAMICS"),
    ("CONTACT_MODEL = '3D_ENVELOPING'", "CONTACT_MODEL"),
    ("FE_METHOD = 1", "FE_METHOD"),
])
def test_tir_unsupported_feature_names_source_and_field(change: str, field: str) -> None:
    text = TIR.replace("USE_MODE = 14", f"USE_MODE = 14\n{change}")
    with pytest.raises(ElementPropertyError, match=field) as error:
        ElementPropertyDocument.from_tir_text(text, name="unsupported.tir")
    assert "unsupported.tir" in str(error.value)


def test_tir_binding_rejects_wrong_type_model_and_unknown_format(tmp_path: Path) -> None:
    path = tmp_path / "law.tir"
    path.write_text(TIR, encoding="ascii")
    with pytest.raises(ElementPropertyError, match="expected element_type"):
        ElementPropertyDocument.load(path, expected_type="spring")
    with pytest.raises(ElementPropertyError, match="not allowed"):
        ElementPropertyDocument.load(path, allowed_models=["fiala"])
    with pytest.raises(ElementPropertyError, match="PROPERTY_FILE_FORMAT"):
        ElementPropertyDocument.from_tir_text(TIR.replace("PAC2002", "PAC-MC"), name="law")


def test_fiala_tir_preserves_adams_relaxation_length_convention() -> None:
    text = TIR.replace("PAC2002", "FIALA").replace("USE_MODE = 14", "USE_MODE = 12")
    text += "\n[FIALA]\nCSLIP = 1000\nCALPHA = 800\nRELAX_LENGTH_X = 0.05\nRELAX_LENGTH_Y = 0.15\n"
    law = ElementPropertyDocument.from_tir_text(text, name="fiala")
    assert law.model == "fiala"
    assert law.resolved["fiala_parameters"]["RELAX_LENGTH_X"] == 50.0
