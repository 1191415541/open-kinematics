"""
The bridge from authoring documents to the axle facts the composition reads.

The file layer answers "what did the author write"; the composition reads an
axle *declaration*.  Keeping the conversion in one place is what stops the file
format from becoming a second description of physics: every field the
declaration needs is derived here from a declaration the documents already
carry, and the properties the elements read are the *resolved* constitutive
values of the bound property files rather than anything re-parsed from the file
text.

Three rules this module holds:

* the declaration's hardpoint keys are chosen through ``HARDPOINT_ALIASES``, so
  a template may name a point by its role (``tie_inner``) and still land on the
  spelling the existing lookup accepts (``TIE_ROD_INBOARD``).  Writing the role
  name straight through would make those lookups fail, and the failure would
  surface as "missing required hardpoint" three layers away;
* the left side is the one the declaration describes.  A v1 axle is left-hand
  and generates the right by mirroring, so the bridge resolves every point
  against the left side and lets the composition do the mirroring it already
  does;
* there is exactly one conversion.  A function that produced a *different* value
  type for the document route and the v1 route would be two descriptions of one
  axle, and the two would drift the moment either was touched -- so both origins
  end in :func:`axle_declaration_from` and the same helpers underneath it.
"""

from __future__ import annotations

import re
from dataclasses import replace
from typing import Any, Mapping, Sequence

from ..schema import (
    BumpStop,
    LinearSpring,
    MassSpec,
    RigidBodySpec,
    StaticDamper,
    Vec3,
    VerticalTire,
)
from ..schema.model import AxleDeclaration
from ..templates.model import PartDefinition
from .geometry import HARDPOINT_ALIASES

__all__ = [
    "BridgeError",
    "axle_declaration_from",
    "hardpoint_document",
    "resolve_hardpoint_keys",
]

#: The side the model describes, and therefore the side every point is resolved
#: against.  The model mirrors it; the bridge never does the mirroring itself.
_MODEL_SIDE = "L"


class BridgeError(ValueError):
    """A subsystem cannot be turned into the model the solver reads."""


def canonical_hardpoint(name: str) -> str:
    """
    Return the spelling the model's hardpoint lookup accepts for ``name``.

    A template may name a point by its *role* (``tie_inner``), by a car-style
    label, or by a legacy upper-case key.  ``lookup_hardpoint`` accepts a fixed
    alias list per role, and that list happens to contain each role's own name for
    some roles but not for all, so matching only the alias list would let
    ``tie_inner`` through as-is and fail three layers later with "missing required
    hardpoint".  The role name is therefore accepted alongside its aliases, and the
    canonical alias is what the model is given.
    """
    normalized = name.upper().replace("-", "_")
    for role, aliases in HARDPOINT_ALIASES.items():
        if normalized == role.upper() or normalized in aliases:
            return aliases[0]
    return name


def resolve_hardpoint_keys(declared: Sequence[str]) -> dict[str, str]:
    """
    Map declared hardpoint names onto the spellings the model's lookup expects.

    A declared name that satisfies no role is left as written rather than refused:
    a template may legitimately carry a point that models nothing yet (a measuring
    marker), and the failure that matters -- an element attaching to a point the
    subsystem does not place -- is reported where the attachment is read.
    """
    return {name: canonical_hardpoint(name) for name in declared}


def hardpoint_document(hardpoints: Mapping[str, Sequence[float]]) -> dict[str, Vec3]:
    """
    Convert a subsystem's coordinates into the model's hardpoint mapping.

    The keys are normalised through the alias table, so a subsystem written with
    role names and one written with legacy keys produce the same model.
    """
    document: dict[str, Vec3] = {}
    for name, coordinates in hardpoints.items():
        values = tuple(float(component) for component in coordinates)
        if len(values) != 3:
            raise BridgeError(
                f"hardpoint {name!r} has {len(values)} components; three are required"
            )
        document[canonical_hardpoint(name)] = Vec3(
            x=values[0], y=values[1], z=values[2]
        )
    return document


def axle_declaration_from(
    subsystem: Any,
    *,
    name: str = "file_driven_axle",
    bodies: Sequence[str] = (),
    sprung_mass: float = 600.0,
) -> AxleDeclaration:
    """
    Build the axle facts a K/C run reads from one effective suspension subsystem.

    ``bodies`` names the dynamic bodies the declaration should carry.  It
    defaults to the subsystem's template bodies minus the fixed ones, which is
    what the composition expects: a fixed support is a property of the assembly,
    not a body the equations of motion integrate.  Each body's mass is the
    template's own declaration, so a template that states no mass yields a
    zero-mass body rather than a number invented here.
    A body's inertia and centre of mass are the template's declarations too, read
    when the file states them; a file that states neither yields the declaration's
    own defaults, which is the axle every existing template produced.

    The elastic elements come from the template's element declarations and the
    *resolved* property bindings, so the constitutive data reaching the kernel is
    the one the property files describe.  Swapping a linear file for a curve file
    therefore changes these values and nothing else, which is the property the
    file format exists to provide.

    This is the one conversion from documents to axle facts.  The document route
    used to have a second function producing a second value type, which made the
    file format and the value type two descriptions of one axle; both origins now
    end here, so a change to how a file's elements are read reaches every caller
    at once.
    """
    template = subsystem.template.payload
    hardpoints = hardpoint_document(subsystem.hardpoints)
    declared = {str(row["name"]): row for row in template["bodies"]}
    # The parts are the *mirrored* ones, because a file states one side and the
    # composition builds both: an axle whose body list stopped at the declared side
    # would name half of itself, and a vehicle declaration asks an axle for every
    # body it has -- so the missing half shows up as "requires positive mass specs
    # for upper_arm_R" rather than as a missing body.
    parts = _mirrored_parts(template)
    dynamic = tuple(part.name for part in parts if not part.fixed)
    selected = tuple(bodies) if bodies else dynamic
    mass_of = {part.name: part.mass for part in parts}

    elements = _element_rows(subsystem, hardpoints)
    try:
        return AxleDeclaration(
            name=name,
            hardpoints=hardpoints,
            bodies=tuple(
                _body_spec(body, _declared_row(declared, body), mass=mass_of[body])
                for body in selected
            ),
            mass=MassSpec(sprung_mass=float(sprung_mass)),
            springs=tuple(elements["spring"]),
            dampers=tuple(elements["damper"]),
            stops=tuple(elements["bump_stop"]),
            tires=_tire_rows(subsystem, hardpoints),
        )
    except Exception as exc:  # noqa: BLE001 - the declaration's own message is the report
        raise BridgeError(
            f"subsystem {subsystem.name!r} does not describe a solvable axle: {exc}"
        ) from exc
def _body_spec(name: str, row: Mapping[str, Any], *, mass: float) -> RigidBodySpec:
    """
    Build one rigid body from a template's declaration of it.

    Mass is stated by the caller because a body the template declares but the
    composition excludes still has one, and a template that states no mass yields
    a zero-mass body rather than a number invented here.

    Inertia and centre of mass are read when the file states them, and left at the
    model's own defaults when it does not.  They are the reason a body needs more
    than one number: a dynamic run integrates the rotational equations, and an
    identity inertia is a *default*, not a description of a real arm.  A file that
    omits them therefore produces the model the format always produced, which is
    what keeps the existing templates' numbers where they were.
    """
    center = row.get("center_of_mass")
    inertia = row.get("inertia")
    extra: dict[str, Any] = {"mass": float(mass)}
    if center is not None:
        extra["center_of_mass"] = Vec3(
            x=float(center[0]), y=float(center[1]), z=float(center[2])
        )
    if inertia is not None:
        extra["inertia"] = tuple(tuple(float(item) for item in line) for line in inertia)
    return RigidBodySpec(name=name, **extra)


def _element_rows(
    subsystem: Any, hardpoints: Mapping[str, Vec3]
) -> dict[str, list[Any]]:
    """
    Build the model's per-kind element lists from the template declarations.

    Each declaration names its attachment by *body name and hardpoint name*, and
    the model wants body-local geometry.  The coordinates come from the hardpoint
    the declaration names, so a subsystem that moves a hardpoint moves the
    element with it -- which is the whole point of placing coordinates in the
    subsystem rather than in the template.
    """
    template = subsystem.template.payload
    rows: dict[str, list[Any]] = {"spring": [], "damper": [], "bump_stop": []}
    for element in template["elements"]:
        kind = str(element["type"])
        if kind not in rows:
            continue
        slot = str(element["property_slot"])
        resolved = dict(subsystem.resolved_property(slot))
        body_a = _element_body(str(element["body_a"]))
        body_b = _element_body(str(element["body_b"]))
        point_a = _point_of(hardpoints, str(element["point_a"]))
        point_b = _point_of(hardpoints, str(element["point_b"]))
        name = f"{element['name']}_{_MODEL_SIDE}"
        if kind == "spring":
            rows["spring"].append(
                LinearSpring(
                    name=name,
                    body_a=body_a,
                    body_b=body_b,
                    point_a=point_a,
                    point_b=point_b,
                    stiffness=float(resolved["stiffness"]),
                    free_length=float(resolved["free_length"]),
                    force_curve=_curve_of(resolved),
                )
            )
        elif kind == "damper":
            rows["damper"].append(
                StaticDamper(
                    name=name,
                    body_a=body_a,
                    body_b=body_b,
                    point_a=point_a,
                    point_b=point_b,
                    viscous_damping=float(resolved["viscous_damping"]),
                    force_curve=_curve_of(resolved),
                )
            )
        elif kind == "bump_stop":
            rows["bump_stop"].append(
                BumpStop(
                    name=name,
                    body_a=body_a,
                    body_b=body_b,
                    point_a=point_a,
                    point_b=point_b,
                    clearance=float(resolved.get("clearance", 0.0)),
                    stiffness=float(resolved["stiffness"]),
                    force_curve=_curve_of(resolved),
                )
            )
    return rows


def _curve_of(resolved: Mapping[str, Any]) -> tuple[tuple[float, float], ...]:
    """
    Return the samples a property file declared, as the model's own curve type.

    A linear law carries no curve, so the empty tuple is the ordinary case and
    the element classes read it as "the scalar parameters are the whole law".
    Carrying the samples -- rather than only the first slope the resolver derives
    from them -- is what makes a nonlinear file a *different law* to the kernel
    instead of a differently-sloped line: the slope is what the kernel uses
    outside the curve's range, and the curve is what it interpolates inside it.
    """
    return tuple(
        (float(independent), float(dependent))
        for independent, dependent in resolved.get("force_curve", ())
    )


def _point_of(hardpoints: Mapping[str, Vec3], name: str) -> Vec3:
    """
    Return the coordinate of the hardpoint an element attaches to.

    The name comes from the template, which may spell it as a role, so it is
    canonicalised the same way the subsystem's own coordinates were; the two then
    meet under one spelling instead of the lookup missing a name that is present
    under another.
    """
    try:
        return hardpoints[canonical_hardpoint(name)]
    except KeyError as exc:
        raise BridgeError(
            f"element attachment names hardpoint {name!r}, which the subsystem does "
            f"not place; it places {sorted(hardpoints)}"
        ) from exc


def _tire_rows(
    subsystem: Any, hardpoints: Mapping[str, Vec3]
) -> tuple[VerticalTire, ...]:
    """
    Build the model's tires from the template's tire elements.

    A tire hangs off the wheel centre, and its contact point is that point lowered
    by the unloaded radius in the body's own frame.  Deriving it here is what keeps
    the *file* from having to state geometry the subsystem already knows: a tire
    element names its slot and its point, and the law carries the two numbers.

    No tire means no vertical support at the wheel centre, which is exactly what
    the pad reading loads -- so a file that declares one is the difference between
    a solvable C run and a model the solver refuses.
    """
    template = subsystem.template.payload
    rows: list[VerticalTire] = []
    for element in template["elements"]:
        if str(element["type"]) != "tire":
            continue
        resolved = dict(subsystem.resolved_property(str(element["property_slot"])))
        centre = _point_of(hardpoints, str(element["point_a"]))
        radius = float(resolved["unloaded_radius"])
        rows.append(
            VerticalTire(
                stiffness=float(resolved["stiffness"]),
                unloaded_radius=radius,
                contact_point=Vec3(x=centre.x, y=centre.y, z=centre.z - radius),
            )
        )
    return tuple(rows)


def _element_body(name: str) -> str:
    """
    Return the body name an element attachment should carry in the model.

    The model's element lists are read *per side*: ``suspension.elements`` resolves
    each attachment through ``resolve_body(name, side, bodies)``, which appends the
    side to a body that exists once per side and leaves a shared body alone.  What
    belongs here is therefore the **stem** (``lower_arm``), not the generated name
    the template writes (``lower_arm_L``): handing over the generated name makes
    ``resolve_body`` find it immediately, and then both sides' springs hang off the
    left arm -- which the K reading tolerates and the C reading cannot balance.

    Stripping the suffix unconditionally is right because the conversion mirrors
    every ``_L`` part into its ``_R`` twin (``_mirrored_parts``), so a sided name in
    a file always has a twin in the template the assembly sees.  A shared name
    (``chassis``, ``rack``) keeps its spelling.
    """
    if name.endswith(("_L", "_R")):
        return name[:-2]
    return name


#: A side token inside a generated name: ``_L`` or ``_R`` bounded by a ``_`` or the
#: end of the name.  Rewriting the token rather than the suffix is what lets a name
#: like ``uca_mount_L_inner_front`` mirror to ``uca_mount_R_inner_front`` -- its side
#: is in the middle, and a suffix-only rule would leave both sides named alike.
_SIDE_TOKEN = re.compile(r"_(L|R)(?=_|$)")


def _side_name(name: str, side: str) -> str:
    """
    Rewrite one *body* name's side token, leaving an unsided name alone.

    A body that exists once -- the chassis, the rack -- must stay shared when a
    connection is mirrored, so this function never invents a side for it: a
    ``chassis_R`` is a body no contribution produces.
    """
    return _SIDE_TOKEN.sub(f"_{side}", name, count=1)


def _mirrored_parts(
    payload: Mapping[str, Any], *, mirror: bool = True
) -> tuple[PartDefinition, ...]:
    """
    Build the template's parts, mirrored like its connections.

    A part named for the left side gets a right-side twin with the same mass, so
    the mirroring of connections lands on bodies that exist.  An unsided part --
    the chassis, the rack -- is declared once, because it exists once.

    ``mirror`` is the template's own declaration: a file that writes both sides
    already names its right-hand bodies, and adding twins for them would put two
    declarations on one name -- the composition refuses that as a duplicate, and
    rightly.  So a file that mirrors nothing gets its own list back unchanged.
    """
    declared = [
        PartDefinition(
            name=str(row["name"]),
            mass=float(row.get("mass", 0.0)),
            fixed=bool(row.get("fixed", False)),
        )
        for row in payload["bodies"]
    ]
    if not mirror:
        return tuple(declared)
    # Two passes, not one: every declared part is listed before its right-side twin.
    # The sequence is a fact about the template -- it is the order the document
    # records its bodies in -- so interleaving the twins would produce a different
    # document from the same file even though both carry the same bodies.
    mirrored = [
        replace(part, name=_side_name(part.name, "R"))
        for part in declared
        if part.name.endswith("_L")
    ]
    return tuple(declared + mirrored)


def _declared_row(
    declared: Mapping[str, Mapping[str, Any]], name: str
) -> Mapping[str, Any]:
    """
    Return the file's own declaration of a body, given a mirrored body's name.

    A file states one side, so the right-side twin has no row of its own: its
    geometry is its left twin's.  A body with neither spelling gets an empty row,
    which is a name and the model's own defaults -- the same model a template that
    states only a name produces.
    """
    return declared.get(name) or declared.get(_side_name(name, "L")) or {"name": name}
