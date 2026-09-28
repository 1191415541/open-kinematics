"""
The bridge from authoring documents to the model the solver reads.

The file layer answers "what did the author write"; the solver reads a
``FrontAxleModel``.  Keeping the conversion in one place is what stops the file
format from becoming a second description of physics: every field the model needs
is derived here from a declaration the documents already carry, and the
properties the elements read are the *resolved* constitutive values of the bound
property files rather than anything re-parsed from the file text.

Two rules this module holds:

* the model's hardpoint keys are chosen through ``HARDPOINT_ALIASES``, so a
  template may name a point by its role (``tie_inner``) and still land on the
  spelling the existing lookup accepts (``TIE_ROD_INBOARD``).  Writing the role
  name straight through would make those lookups fail, and the failure would
  surface as "missing required hardpoint" three layers away;
* the left side is the one the model describes.  ``FrontAxleModel`` is
  left-hand and generates the right by mirroring, so the bridge resolves every
  point against the left side and lets the model do the mirroring it already does.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from ..schema import (
    BumpStop,
    FrontAxleModel,
    LinearSpring,
    MassSpec,
    RigidBodySpec,
    StaticDamper,
    Vec3,
)
from ..subsystems.geometry import HARDPOINT_ALIASES

__all__ = [
    "BridgeError",
    "front_axle_model_from",
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


def front_axle_model_from(
    subsystem: Any,
    *,
    name: str = "file_driven_axle",
    bodies: Sequence[str] = (),
    sprung_mass: float = 600.0,
) -> FrontAxleModel:
    """
    Build the model a K/C run reads from one effective suspension subsystem.

    ``bodies`` names the dynamic bodies the model should carry.  It defaults to
    the subsystem's template bodies minus the fixed ones, which is what the
    composition expects: a fixed support is a property of the assembly, not a body
    the equations of motion integrate.  Each body's mass is the template's own
    declaration, so a template that states no mass yields a zero-mass body rather
    than a number invented here.

    The elastic elements come from the template's element declarations and the
    *resolved* property bindings, so the constitutive data reaching the kernel is
    the one the property files describe.  Swapping a linear file for a curve file
    therefore changes these values and nothing else, which is the property the
    file format exists to provide.
    """
    template = subsystem.template.payload
    hardpoints = hardpoint_document(subsystem.hardpoints)
    declared_mass = {
        str(row["name"]): float(row.get("mass", 0.0)) for row in template["bodies"]
    }
    fixed = {str(row["name"]) for row in template["bodies"] if row.get("fixed")}
    dynamic = tuple(name for name in declared_mass if name not in fixed)
    selected = tuple(bodies) if bodies else dynamic

    elements = _element_rows(subsystem, hardpoints)
    try:
        return FrontAxleModel(
            name=name,
            hardpoints=hardpoints,
            bodies=tuple(
                RigidBodySpec(name=body, mass=declared_mass.get(body, 0.0))
                for body in selected
            ),
            mass=MassSpec(sprung_mass=float(sprung_mass)),
            springs=tuple(elements["spring"]),
            dampers=tuple(elements["damper"]),
            stops=tuple(elements["bump_stop"]),
        )
    except Exception as exc:  # noqa: BLE001 - the model's own message is the report
        raise BridgeError(
            f"subsystem {subsystem.name!r} does not describe a solvable axle: {exc}"
        ) from exc


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
        body_a = _side_body(str(element["body_a"]))
        body_b = _side_body(str(element["body_b"]))
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
                )
            )
    return rows


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


def _side_body(name: str) -> str:
    """
    Return the body name the model's left-side element attachment should carry.

    A template names its bodies exactly as the composition generates them: a part
    that exists once per side carries its side suffix (``upper_arm_L``), and a part
    that exists once (``chassis``, ``rack``) does not.  The model describes the
    left side, so a name written for the *right* is read as its left twin and an
    unsided name is left alone -- appending a suffix to every name would invent a
    ``chassis_L`` that no contribution produces.
    """
    if name.endswith("_R"):
        return f"{name[:-2]}_L"
    return name

