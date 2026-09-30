"""
A file says which sides it writes, and whether the other one is mirrored from it.

Subtask 06's remaining half.  The file format writes one side once and mirrors it
-- that is the shorthand every existing template uses, and it is why an axle is
one file rather than two.  The other form is a file that writes both sides itself,
and then **nothing** may be mirrored: a twin of a declared body is a second
declaration of one name, which the composition refuses as a duplicate.

What is asserted here is the conversion's own answer, because that is where the
mirroring happens: the declared sides, whether the mirror applies, and that the
side a file wrote itself is not written a second time for it.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring.documents import (
    SimulationAssembly,
    TemplateDocument,
)
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.solver import (
    assembly_request_for,
    runtime_template_from,
)
from tests.authoring.fixtures import write_axle_project


def _both_sides_written(tmp_path: Path):
    """Write the fixture's template again with both sides spelled out."""
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["template"].read_text(encoding="utf-8"))
    payload["sides"] = ["left", "right"]
    payload["mirror"] = False
    # The right-hand bodies, declared by the file rather than invented by the
    # conversion: one entry per left-hand body, same mass.
    payload["bodies"] = list(payload["bodies"]) + [
        {"name": str(row["name"])[:-2] + "_R", "mass": row.get("mass", 0.0)}
        for row in payload["bodies"]
        if str(row["name"]).endswith("_L")
    ]
    paths["template"].write_text(json.dumps(payload), encoding="utf-8")
    return TemplateDocument.load(paths["template"]), paths


def test_an_undeclared_template_still_mirrors_one_side(tmp_path: Path) -> None:
    """
    The default is the shorthand, and that is what keeps every existing file valid.

    A template that says nothing writes its left side and mirrors it, so the
    assembly carries both sides -- which is exactly the pair it carried before the
    declaration existed.
    """
    paths = write_axle_project(tmp_path)
    document = TemplateDocument.load(paths["template"])
    assert document.mirrors is True
    assert document.declared_sides == ("left",)

    request = assembly_request_for(SimulationAssembly.load(paths["assembly"]))
    assert request.sides == ("L", "R")


def test_a_file_that_writes_both_sides_is_not_mirrored_again(tmp_path: Path) -> None:
    """
    "A file that writes both sides is not mirrored twice": the conversion adds no twin.

    Read at the conversion, because that is the layer that would add one: the
    template's parts are exactly the bodies the file declared, every right-hand body
    in the result is one the file wrote, and the sides it declares are the sides the
    request carries.
    """
    document, paths = _both_sides_written(tmp_path)
    template = runtime_template_from(document)

    declared = {str(row["name"]) for row in document.payload["bodies"]}
    assert {part.name for part in template.parts} == declared
    assert any(name.endswith("_R") for name in declared)
    assert {part.name for part in template.parts} - declared == set()

    request = assembly_request_for(SimulationAssembly.load(paths["assembly"]))
    assert request.sides == ("L", "R")


def test_a_one_sided_file_is_a_corner_and_the_incoherent_pairs_are_refused(
    tmp_path: Path,
) -> None:
    """
    Two of the four combinations describe something the conversion can build.

    ``left + mirror`` is the shorthand every existing template uses, and
    ``both sides + no mirror`` is the file that spells both sides out.  The other
    two are refused where they are written:

    * ``right + mirror`` -- the mirror writes the right side *from* the left one, so
      a file that declares the right side and mirrors would produce no left side at
      all: an assembly half of whose bodies nobody wrote;
    * ``both sides + mirror`` -- a twin of a body the file already declared is a
      second declaration of one name.

    A file that writes one side and mirrors nothing is a third, legal shape: it is
    a single corner, which is what a one-sided assembly is made of.
    """
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["template"].read_text(encoding="utf-8"))

    # The single corner: one side written, nothing mirrored, and it is honoured.
    corner = dict(payload, mirror=False)
    paths["template"].write_text(json.dumps(corner), encoding="utf-8")
    document = TemplateDocument.load(paths["template"])
    assert (document.declared_sides, document.mirrors) == (("left",), False)
    request = assembly_request_for(SimulationAssembly.load(paths["assembly"]))
    assert request.sides == ("L",)

    # Right side plus the mirror: refused, because the mirror writes from the left.
    paths["template"].write_text(
        json.dumps(dict(payload, sides=["right"], mirror=True)), encoding="utf-8"
    )
    with pytest.raises(AuthoringError, match="mirror writes the right side from the left"):
        TemplateDocument.load(paths["template"])

    # Both sides plus the mirror: refused, because the twin would name a body twice.
    paths["template"].write_text(
        json.dumps(dict(payload, sides=["left", "right"], mirror=True)), encoding="utf-8"
    )
    with pytest.raises(AuthoringError, match="mirrors nothing"):
        TemplateDocument.load(paths["template"])
