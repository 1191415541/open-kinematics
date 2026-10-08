"""
Presets: the common topologies, offered as ordinary documents.

A preset is a *convenience*, not a type.  Every function here returns the same
kind of object a file load returns -- a template, a subsystem, or an assembly
bound to a rig -- so a caller can take a preset apart, change one coordinate,
save it and run it, exactly as if it had written the declaration by hand.  That
is the whole difference between this module and the model classes it replaces:
those *were* the model, and a caller who wanted something they did not describe
had no door; these are a starting point and nothing more.

Nothing here is an alias for a legacy model, and nothing here builds one.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .generic import generic_template

if TYPE_CHECKING:
    from ..authoring.documents import (
        AssemblyDocument,
        RigDocument,
        SubsystemDocument,
        TemplateDocument,
    )

__all__ = [
    "generic_template",
    "double_wishbone_template",
    "double_wishbone_subsystem",
    "standalone_axle",
]


def double_wishbone_template() -> "TemplateDocument":
    """
    Return the symmetric double-wishbone template as an ordinary document.

    The built-in is exported through the same conversion the file route uses and
    then rebuilt in memory, so the preset is not a *second* description of the
    template: it is the registered one, in the shape a caller can edit.
    """
    from ..authoring.documents import TemplateDocument
    from ..templates.builtin import DOUBLE_WISHBONE
    from ..templates.export import template_document_from
    return TemplateDocument.from_payload(template_document_from(DOUBLE_WISHBONE))


def double_wishbone_subsystem(
    *,
    hardpoints: dict[str, Any],
    spring: Any,
    damper: Any,
    name: str = "front",
    placement_role: str = "front",
    template: TemplateDocument | None = None,
) -> "SubsystemDocument":
    """
    Return one placed double-wishbone subsystem.

    ``hardpoints`` are the coordinates the subsystem places, keyed by the names
    the template declares.  ``spring`` and ``damper`` are the loaded constitutive
    laws for the template's two required slots.  All three are required rather
    than defaulted: a preset that invented coordinates or stiffnesses would be a
    model nobody chose, and the point of a preset is to spare the caller the
    boilerplate rather than to make the decisions for them.
    """
    from ..authoring.documents import SubsystemDocument
    resolved = double_wishbone_template() if template is None else template
    return SubsystemDocument.from_payload(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": name,
            "template": resolved.name,
            "functional_role": resolved.functional_role,
            "placement_role": placement_role,
            "hardpoints": {k: list(v) for k, v in hardpoints.items()},
            "property_bindings": {"spring": "spring.json", "damper": "damper.json"},
        },
        template=resolved,
        properties={"spring": spring, "damper": damper},
    )


def standalone_axle(
    subsystems: tuple[SubsystemDocument, ...],
    *,
    rig: "RigDocument",
    name: str = "axle",
) -> "AssemblyDocument":
    """
    Return a suspension-axle assembly from the subsystems a caller supplies.

    The assembly is a plain document: what makes it an axle is the role set it
    carries, which the policy checks on construction, not a class it belongs to.
    """
    from ..authoring.documents import AssemblyDocument
    return AssemblyDocument.from_payload(
        {
            "document": "assembly",
            "schema_version": 1,
            "name": name,
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {
                    "ref": f"{subsystem.payload['name']}.sub.json",
                    "functional_role": subsystem.functional_role,
                    "placement_role": subsystem.placement_role,
                }
                for subsystem in subsystems
            ],
            "rig": rig.name,
        },
        subsystems={
            f"{subsystem.payload['name']}.sub.json": subsystem
            for subsystem in subsystems
        },
        rig=rig,
    )
