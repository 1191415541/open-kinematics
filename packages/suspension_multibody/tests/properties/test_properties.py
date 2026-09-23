"""
Properties files: the format, the refusals, and the two comparisons that matter.

A properties file is only useful if a bad one is *rejected*.  A missing stiffness
that silently becomes zero produces a floating linkage -- a plausible-looking
wrong vehicle -- so the negative tests here assert the message names the file, the
property, the field and the reason, not merely that something raised.

The positive tests are the other half: the same template with the same file is
reproducible, and with a different file changes stiffness and damping while the
geometry does not move at all.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.properties import (
    ENTRY_KINDS,
    PropertiesError,
    load_properties,
)
from suspension_multibody.subsystems import DEFAULT_AXLE_SUBSYSTEMS, AssemblyRequest
from suspension_multibody.templates import (
    DOUBLE_WISHBONE,
    TemplateError,
    instantiate,
    resolve_properties,
)
from tests.benchmark_fixture import benchmark_model

DATA = Path(__file__).parents[1] / "data" / "properties"
BASELINE = DATA / "baseline_compliance.json"
STIFFER = DATA / "stiffer_compliance.json"

#: The built-in template has no default for these two, so a properties file that
#: claims to fill it must supply them.
_MODEL_OWNED = ("spring", "damper")


def _write(tmp_path: Path, payload: object, name: str = "p.json") -> Path:
    target = tmp_path / name
    target.write_text(json.dumps(payload), encoding="utf-8")
    return target


def _minimal(**properties: object) -> dict:
    base = {
        "schema_version": 1,
        "name": "minimal",
        "units": "mm-N-N*mm-kg-deg",
        "properties": {},
    }
    base["properties"].update(properties)
    return base


def _stiffness_norms(assembly) -> set[float]:
    return {
        float(np.linalg.norm(element.stiffness))
        for element in assembly.elements
        if element.name.endswith(("inner_front", "inner_rear"))
    }


def test_the_shipped_files_load() -> None:
    baseline = load_properties(BASELINE)
    stiffer = load_properties(STIFFER)
    assert baseline.name == "baseline_compliance"
    assert stiffer.name == "stiffer_compliance"
    assert baseline.keys() == stiffer.keys() == ("bushing", "damper", "spring")
    assert baseline["spring"]["kind"] == "spring"
    assert baseline.kind_of("bushing") == "bushing6x6"


def test_the_default_file_reproduces_the_template_defaults_bit_for_bit() -> None:
    """
    The shipped baseline file must not move any number.

    This is what keeps the frozen C snapshot valid: the file exists to show the
    plumbing, and its bushing entry is the template's own zero.
    """
    baseline = load_properties(BASELINE)
    from_file = instantiate(
        DOUBLE_WISHBONE, mode="C", properties=resolve_properties(DOUBLE_WISHBONE, baseline)
    )
    from_defaults = instantiate(
        DOUBLE_WISHBONE,
        mode="C",
        properties={
            "spring": baseline["spring"]["stiffness"],
            "damper": baseline["damper"]["viscous_damping"],
            "bushing": 0.0,
        },
    )
    assert from_file.stiffness_for("uca_mount_L_inner_front") == 0.0
    assert from_file == from_defaults


def test_same_file_twice_is_reproducible() -> None:
    model = benchmark_model()
    first = load_properties(BASELINE)
    second = load_properties(BASELINE)
    assert first.as_values() == second.as_values()
    request = AssemblyRequest(
        mode="C",
        subsystems=DEFAULT_AXLE_SUBSYSTEMS,
        suspension_template=instantiate(
            DOUBLE_WISHBONE, mode="C", properties=resolve_properties(DOUBLE_WISHBONE, first)
        ),
    )
    other = AssemblyRequest(
        mode="C",
        subsystems=DEFAULT_AXLE_SUBSYSTEMS,
        suspension_template=instantiate(
            DOUBLE_WISHBONE, mode="C", properties=resolve_properties(DOUBLE_WISHBONE, second)
        ),
    )
    a = build_front_axle(model, "C", request)
    b = build_front_axle(model, "C", other)
    assert list(a.bodies) == list(b.bodies)
    assert {f"{x}::{y}" for x, y in a.points} == {f"{x}::{y}" for x, y in b.points}
    for key, point in a.points.items():
        assert np.array_equal(np.asarray(point), np.asarray(b.points[key])), key
    assert [c.name for c in a.constraints] == [c.name for c in b.constraints]
    assert [e.name for e in a.elements] == [e.name for e in b.elements]


def test_a_different_file_changes_stiffness_and_nothing_else() -> None:
    """
    The comparison the requirement is actually about: same template, same
    geometry, different numbers.
    """
    model = benchmark_model()
    assemblies = {}
    for label, path in (("baseline", BASELINE), ("stiffer", STIFFER)):
        loaded = load_properties(path)
        assemblies[label] = build_front_axle(
            model,
            "C",
            AssemblyRequest(
                mode="C",
                subsystems=DEFAULT_AXLE_SUBSYSTEMS,
                suspension_template=instantiate(
                    DOUBLE_WISHBONE,
                    mode="C",
                    properties=resolve_properties(DOUBLE_WISHBONE, loaded),
                ),
            ),
        )
    soft, hard = assemblies["baseline"], assemblies["stiffer"]

    # Geometry is untouched: bodies, points, connections, hardpoints, constraints.
    assert list(soft.bodies) == list(hard.bodies)
    assert set(soft.points) == set(hard.points)
    for key, point in soft.points.items():
        assert np.array_equal(np.asarray(point), np.asarray(hard.points[key])), key
    assert [c for c in soft.connections] == [c for c in hard.connections]
    assert sorted(soft.hardpoints) == sorted(hard.hardpoints)
    assert [c.name for c in soft.constraints] == [c.name for c in hard.constraints]

    # Only the numbers move.
    soft_norms = _stiffness_norms(soft)
    hard_norms = _stiffness_norms(hard)
    assert soft_norms == {0.0}
    assert len(hard_norms) == 1
    assert abs(next(iter(hard_norms)) - np.sqrt(3.0) * 25_000.0) < 1e-6
    assert soft_norms != hard_norms


# --- R1: document level ---------------------------------------------------


def test_a_non_object_root_is_refused(tmp_path: Path) -> None:
    target = _write(tmp_path, [1, 2, 3])
    with pytest.raises(PropertiesError, match="must be an object"):
        load_properties(target)


def test_a_missing_or_wrong_schema_version_is_refused(tmp_path: Path) -> None:
    for version in (None, 2, "1"):
        payload = _minimal()
        if version is None:
            del payload["schema_version"]
        else:
            payload["schema_version"] = version
        target = _write(tmp_path, payload, f"v{version}.json")
        with pytest.raises(PropertiesError, match="schema_version must be 1"):
            load_properties(target)


def test_an_unknown_root_key_is_refused(tmp_path: Path) -> None:
    payload = _minimal()
    payload["extra"] = 1
    target = _write(tmp_path, payload)
    with pytest.raises(PropertiesError) as error:
        load_properties(target)
    message = str(error.value)
    assert "unknown root key" in message
    assert "extra" in message


def test_invalid_json_names_the_file(tmp_path: Path) -> None:
    target = tmp_path / "broken.json"
    target.write_text("{ not json", encoding="utf-8")
    with pytest.raises(PropertiesError) as error:
        load_properties(target)
    assert "broken.json" in str(error.value)


def test_a_missing_file_names_the_path(tmp_path: Path) -> None:
    missing = tmp_path / "nope.json"
    with pytest.raises(PropertiesError) as error:
        load_properties(missing)
    assert "nope.json" in str(error.value)


# --- R2: entry level ------------------------------------------------------


def test_an_entry_without_a_kind_is_refused(tmp_path: Path) -> None:
    target = _write(tmp_path, _minimal(thing={"stiffness": 1.0}))
    with pytest.raises(PropertiesError, match="has no 'kind'"):
        load_properties(target)


def test_an_unknown_kind_is_refused_and_the_known_kinds_are_listed(
    tmp_path: Path,
) -> None:
    target = _write(tmp_path, _minimal(thing={"kind": "springy", "stiffness": 1.0}))
    with pytest.raises(PropertiesError) as error:
        load_properties(target)
    message = str(error.value)
    assert "unknown kind 'springy'" in message
    for kind in ENTRY_KINDS:
        assert kind in message


def test_an_unknown_entry_field_is_refused(tmp_path: Path) -> None:
    target = _write(
        tmp_path,
        _minimal(
            spring={"kind": "spring", "stiffness": 45.0, "free_length": 250.0, "wat": 1}
        ),
    )
    with pytest.raises(PropertiesError) as error:
        load_properties(target)
    message = str(error.value)
    assert "unknown field" in message
    assert "wat" in message


# --- R3 plus the three named negatives ------------------------------------


def test_a_missing_required_entry_is_refused_by_the_resolver(
    tmp_path: Path,
) -> None:
    """
    Missing a slot the role requires, with no template default, is an error.

    The message has to name both the slot and the file: "key error" would leave a
    reader guessing which of two inputs is wrong.
    """
    target = _write(tmp_path, _minimal(**{"bushing": {"kind": "bushing6x6", "stiffness": [
        [0.0] * 6 for _ in range(6)
    ]}}))
    loaded = load_properties(target)
    with pytest.raises(TemplateError) as error:
        resolve_properties(DOUBLE_WISHBONE, loaded)
    message = str(error.value)
    assert "spring" in message and "damper" in message
    assert str(target) in message


def test_a_string_stiffness_is_refused_naming_property_and_field(
    tmp_path: Path,
) -> None:
    payload = _minimal(spring={"kind": "spring", "stiffness": "stiff", "free_length": 1.0})
    target = _write(tmp_path, payload)
    with pytest.raises(PropertiesError) as error:
        load_properties(target)
    message = str(error.value)
    assert "spring" in message
    assert "stiffness" in message
    assert str(target) in message
    assert "number" in message


def test_a_wrong_matrix_shape_is_refused_naming_property_and_field(
    tmp_path: Path,
) -> None:
    payload = _minimal(
        bushing={"kind": "bushing6x6", "stiffness": [[0.0] * 3 for _ in range(3)]}
    )
    target = _write(tmp_path, payload)
    with pytest.raises(PropertiesError) as error:
        load_properties(target)
    message = str(error.value)
    assert "bushing" in message
    assert "6x6" in message


def test_out_of_range_values_are_refused_naming_property_and_field(
    tmp_path: Path,
) -> None:
    cases = {
        "spring": {"kind": "spring", "stiffness": -1.0, "free_length": 250.0},
        "damper": {"kind": "damper", "viscous_damping": -1.0},
        "tire": {"kind": "tire", "stiffness": 250.0, "unloaded_radius": 0.0},
        "bump_stop": {"kind": "bump_stop", "clearance": -1.0, "stiffness": 1.0},
    }
    for name, entry in cases.items():
        target = _write(tmp_path, _minimal(**{name: entry}), f"{name}.json")
        with pytest.raises(PropertiesError) as error:
            load_properties(target)
        message = str(error.value)
        assert name in message, (name, message)
        assert str(target) in message, (name, message)


def test_an_anisotropic_matrix_cannot_fill_a_scalar_slot(tmp_path: Path) -> None:
    matrix = [[float(i == j) * (1.0 + i) for j in range(6)] for i in range(6)]
    target = _write(tmp_path, _minimal(
        bushing={"kind": "bushing6x6", "stiffness": matrix},
        spring={"kind": "spring", "stiffness": 45.0, "free_length": 250.0},
        damper={"kind": "damper", "viscous_damping": 1.5},
    ))
    loaded = load_properties(target)
    with pytest.raises(TemplateError) as error:
        resolve_properties(DOUBLE_WISHBONE, loaded)
    message = str(error.value)
    assert "anisotropic" in message
    assert "bushing" in message


def test_a_property_the_template_does_not_declare_is_ignored_not_guessed(
    tmp_path: Path,
) -> None:
    """
    A file may hold entries a template does not read.

    That is not an error: one file is shared by several templates, and refusing
    the extra entries would force a file per template.  What must not happen is
    the extra entry changing anything -- the resolver simply does not look at it.
    """
    target = _write(tmp_path, _minimal(
        spring={"kind": "spring", "stiffness": 45.0, "free_length": 250.0},
        damper={"kind": "damper", "viscous_damping": 1.5},
        unrelated={"kind": "bushing6x6", "stiffness": [[1.0] * 6 for _ in range(6)]},
    ))
    loaded = load_properties(target)
    values = resolve_properties(DOUBLE_WISHBONE, loaded)
    assert "unrelated" not in values
