"""
An assembly file may state its own pairings, and the loader reads them.

The section is optional on purpose: a file that states nothing keeps the inferred
behaviour, which is what every assembly file written before this row said.  What
this module checks is the reading side -- that a stated pairing arrives on the
entry it belongs to, that an entry with none reads as empty rather than as an
error, and that a file stating one requirement twice is refused while loading
instead of quietly letting the last one win.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring.documents import AssemblyDocument, AuthoringError

from .fixtures import write_axle_project


def _assembly_path(root: Path) -> Path:
    """Return the assembly file the shared fixture wrote."""
    for candidate in sorted(root.rglob("*.json")):
        payload = json.loads(candidate.read_text(encoding="utf-8"))
        if isinstance(payload, dict) and payload.get("document") == "assembly":
            return candidate
    raise AssertionError(f"the fixture wrote no assembly file under {root}")


def _state_pairings(path: Path, pairings: list[dict[str, str]]) -> None:
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["subsystems"][0]["pairings"] = pairings
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def test_a_file_without_pairings_reads_as_empty(tmp_path: Path) -> None:
    write_axle_project(tmp_path)
    document = AssemblyDocument.load(_assembly_path(tmp_path))
    assert dict(document.entries[0].pairings) == {}


def test_a_stated_pairing_arrives_on_its_entry(tmp_path: Path) -> None:
    write_axle_project(tmp_path)
    path = _assembly_path(tmp_path)
    _state_pairings(path, [{"requirement_role": "mount", "port": "chassis_mount"}])
    document = AssemblyDocument.load(path)
    assert dict(document.entries[0].pairings) == {"mount": "chassis_mount"}


def test_a_requirement_stated_twice_is_refused_while_loading(tmp_path: Path) -> None:
    write_axle_project(tmp_path)
    path = _assembly_path(tmp_path)
    _state_pairings(
        path,
        [
            {"requirement_role": "mount", "port": "one"},
            {"requirement_role": "mount", "port": "another"},
        ],
    )
    with pytest.raises(AuthoringError) as caught:
        AssemblyDocument.load(path)
    assert "mount" in str(caught.value)


def test_an_unknown_field_on_a_pairing_is_refused_while_loading(tmp_path: Path) -> None:
    write_axle_project(tmp_path)
    path = _assembly_path(tmp_path)
    _state_pairings(
        path, [{"requirement_role": "mount", "port": "one", "mystery": "value"}]
    )
    with pytest.raises(AuthoringError) as caught:
        AssemblyDocument.load(path)
    # The contract validator names the offending key; the loader propagates it.
    assert "mystery" in str(caught.value)
