"""
Versioned file driven authoring documents.

This module owns the boundary between editable documents and runtime assembly.  A
subsystem document can carry values and property references only; topology belongs
to its template document, and an assembly may override values while never touching
either the template or the subsystem file it refers to.

The four layers, and what each one is allowed to say:

* ``TemplateDocument`` -- topology: bodies, hardpoints, joints, elements, property
  slots, ports.  Written by an expert.
* ``SubsystemDocument`` -- values for one template: hardpoint coordinates and
  property file bindings.  Written by a user.
* ``AssemblyDocument`` -- which subsystems, at which functional and placement
  role, plus copy-on-write overrides that never leave this file.
* ``RigDocument`` -- the bench a simulation assembly is bound to.

Nothing here solves.  A document layer that also solved would have to know the
model schema, and the file format would then be a second way to describe physics
rather than a way to *author* it.
"""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping

from suspension_contracts import (
    ContractError,
    canonical_json_bytes,
    validate_assembly,
    validate_rig,
    validate_subsystem,
    validate_template,
)

from ..connections.policy import (
    RuleViolation,
    check_assembly_shape,
    check_forbidden_roles,
)
from .errors import AuthoringError, ElementPropertyError
from .properties import ElementPropertyDocument

#: The two modes a template's columns may be activated in.  Named here rather than
#: imported from the solver so a document can be validated without the solver layer
#: being importable.
MODES: tuple[str, ...] = ("K", "C")

#: Element kinds whose two ends sit on two different points.  Anything else acts at
#: one point, and the file states one point for it.
TWIN_ENDED_ELEMENTS = frozenset({"spring", "damper", "bump_stop", "anti_roll_bar"})
FUNCTIONAL_ROLES = frozenset(
    {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
)
#: Where a subsystem may sit.  The *shape* is what is fixed -- a placement must be a
#: name, not an arbitrary string -- while which axes exist is the file's own
#: declaration, so a three-axle truck needs no entry here and a typo is still
#: caught by the shape rule naming the placement it does not recognise.
PLACEMENT_ROLES = frozenset(
    {"any", "front", "rear", "middle", "third"}
    | {"front_left", "front_right", "rear_left", "rear_right"}
    | {"middle_left", "middle_right", "third_left", "third_right"}
)
#: The bodies a joint may name without the template declaring them: the ones another
#: role owns.
#:
#: A mount states *what it attaches to*, and a template mounts to parts it does not
#: build.  The arm mounts' far end is the chassis -- the chassis role's body, or the
#: ground when the assembly carries neither -- and a tie rod's inner end is the rack,
#: which belongs to the steering role.  Nothing in the document format makes a
#: template declare its neighbour's bodies, and the ownership itself is stated once
#: (`subsystems.suspension._FOREIGN_STEMS` states the same set from the other side).
#: Every other body name is still checked, because a typo in an arm's name is a
#: model that validates and means something else.
_ASSEMBLY_SUPPLIED_BODIES = frozenset({"chassis", "ground", "rack", "rack_housing"})
#: The fields that describe a template's *topology*.  A subsystem or an assembly
#: override that names any of them is rejected by name rather than by omission:
#: "you cannot add a body here" is the message the boundary exists to produce.
TOPOLOGY_KEYS = frozenset(
    {"bodies", "hardpoints", "joints", "elements", "property_slots", "ports", "outputs"}
)
SUBSYSTEM_VALUE_KEYS = frozenset({"hardpoints", "property_bindings", "parameters"})
#: Values an assembly override may restate.  Assembly overrides are narrower than
#: subsystem documents on purpose: an assembly assigns and tunes what already
#: exists, and instance parameters belong with the subsystem that owns them.
OVERRIDE_KEYS = frozenset({"hardpoints", "property_bindings"})
#: The four wheel ends a full vehicle must account for.
WHEEL_ENDS = ("front_left", "front_right", "rear_left", "rear_right")


def _hash(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()

def _check_modes(path: Path, what: str, modes: Any) -> None:
    """Refuse a mode list that repeats an entry or names no mode at all."""
    if modes is None:
        return
    listed = [str(mode) for mode in modes]
    if not listed:
        raise AuthoringError(f"{path}: {what} declares an empty 'modes' list")
    if len(listed) != len(set(listed)):
        raise AuthoringError(f"{path}: {what} repeats a mode in 'modes'")
    unknown = sorted(set(listed) - set(MODES))
    if unknown:
        raise AuthoringError(
            f"{path}: {what} activates in unknown mode(s) {unknown}; modes are "
            f"{sorted(MODES)}"
        )


def _hash(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _read(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise AuthoringError(f"{path}: cannot read document: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise AuthoringError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise AuthoringError(f"{path}: document root must be an object")
    return payload


def _check_contract(payload: Mapping[str, Any], path: Path, validator: Any) -> None:
    """Run a bundled schema, naming the file and the failing field path."""
    try:
        validator(payload)
    except ContractError as exc:
        raise AuthoringError(f"{path}: {exc}") from exc


def _check_role(functional_role: str, placement_role: str, path: Path) -> None:
    if functional_role not in FUNCTIONAL_ROLES:
        raise AuthoringError(f"{path}: unknown functional_role {functional_role!r}")
    if placement_role not in PLACEMENT_ROLES:
        raise AuthoringError(f"{path}: unknown placement_role {placement_role!r}")


def _frozen_numbers(values: Mapping[str, Any]) -> Mapping[str, Any]:
    """Return a read-only copy of a mapping of numeric vectors."""
    frozen: dict[str, Any] = {}
    for key, value in values.items():
        if isinstance(value, (list, tuple)):
            frozen[str(key)] = tuple(float(component) for component in value)
        else:
            frozen[str(key)] = value
    return MappingProxyType(frozen)


@dataclass(frozen=True)
class TemplateDocument:
    """One template: the topology an expert owns."""

    path: Path
    payload: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "TemplateDocument":
        target = Path(path).resolve()
        payload = _read(target)
        _check_contract(payload, target, validate_template)
        _check_role(str(payload["functional_role"]), "any", target)
        cls._check_topology(target, payload)
        cls._check_sides(target, payload)
        return cls(target, copy.deepcopy(payload))

    @staticmethod
    def _check_sides(path: Path, payload: Mapping[str, Any]) -> None:
        """
        Refuse a symmetry declaration that describes half a topology.

        One side plus the mirror is the ordinary form, and writing both sides is
        the other one: a file states which sides it writes and whether the side it
        does not write is mirrored from it.  The two answers are only meaningful
        together -- "I write one side and I mirror nothing" describes an axle with
        one corner, which is a legal *assembly* but never a legal template.
        """
        declared = payload.get("sides", ("left",))
        if not isinstance(declared, (list, tuple)) or not declared:
            raise AuthoringError(f"{path}: sides must be a non-empty list of sides")
        unknown = sorted({str(side) for side in declared} - {"left", "right"})
        if unknown:
            raise AuthoringError(f"{path}: unknown side(s) {unknown}; sides are left, right")
        if len(set(declared)) != len(declared):
            raise AuthoringError(f"{path}: duplicate side(s) {sorted(declared)}")
        both = set(map(str, declared)) == {"left", "right"}
        mirrors = bool(payload.get("mirror", True))
        # The two answers are checked against each other, because only two of the
        # four combinations describe something the conversion can build:
        #
        # * **left + mirror** -- the shorthand: the file writes the left side, the
        #   conversion mirrors it into the right one.  This is what every existing
        #   template does, and mirroring *writes from* the left, so "right +
        #   mirror" is not a variant of it: it would declare the right side and
        #   then produce no left one at all -- an assembly half of whose bodies
        #   nobody wrote;
        # * **both sides, no mirror** -- the file spells both sides out, so a twin
        #   of either would name a body twice.
        if mirrors and not both and set(map(str, declared)) != {"left"}:
            raise AuthoringError(
                f"{path}: sides is {sorted(map(str, declared))} with mirror true; the "
                "mirror writes the right side from the left one, so a mirrored file "
                "declares sides: [left].  A file that writes the right side has to "
                "write the left one too, and state mirror: false"
            )
        if mirrors and both:
            raise AuthoringError(
                f"{path}: sides is both sides with mirror true; a file that writes "
                "both sides mirrors nothing -- set mirror: false"
            )

    @property
    def declared_sides(self) -> tuple[str, ...]:
        """Return the sides this template writes, in the file's own spelling."""
        return tuple(str(side) for side in self.payload.get("sides", ("left",)))

    @property
    def mirrors(self) -> bool:
        """
        Return whether the side this file does not write is mirrored from it.

        True is the ordinary answer, and the reason the file format is usable for a
        symmetric axle: one side is written once and the other is its mirror.  A
        file that writes both sides says so -- and then nothing is mirrored,
        because mirroring a side that is already declared would produce two
        descriptions of one body.
        """
        return bool(self.payload.get("mirror", True))

    @staticmethod
    def _check_topology(path: Path, payload: Mapping[str, Any]) -> None:
        """
        Refuse a template whose own references do not resolve.

        Every failure names the entity and the reference: "template is invalid"
        would not tell an author which body to add.  The checks are the ones a
        composition would otherwise discover much later, in a merged model where
        the originating file is no longer visible.
        """
        bodies = [str(row["name"]) for row in payload["bodies"]]
        if len(bodies) != len(set(bodies)):
            duplicates = sorted({n for n in bodies if bodies.count(n) > 1})
            raise AuthoringError(f"{path}: duplicate body name(s) {duplicates}")
        hardpoints = [str(row["name"]) for row in payload["hardpoints"]]
        if len(hardpoints) != len(set(hardpoints)):
            duplicates = sorted({n for n in hardpoints if hardpoints.count(n) > 1})
            raise AuthoringError(f"{path}: duplicate hardpoint name(s) {duplicates}")
        slots = [str(row["name"]) for row in payload["property_slots"]]
        if len(slots) != len(set(slots)):
            duplicates = sorted({n for n in slots if slots.count(n) > 1})
            raise AuthoringError(f"{path}: duplicate property slot name(s) {duplicates}")

        body_names = set(bodies)
        hardpoint_names = set(hardpoints)
        owners = {
            str(row["name"]): str(row.get("owner", "")) for row in payload["hardpoints"]
        }
        for name, owner in owners.items():
            # An ownerless point is legal and is not a gap: a role that owns no
            # bodies -- the simplified brake and drive, which contribute a torque
            # and nothing else -- declares the mount some *other* role's body
            # carries, exactly as the built-in template declares the steering,
            # wheel and chassis mounts alongside its own.  Requiring an owner would
            # make those two roles impossible to write as files.
            if owner and owner not in body_names:
                raise AuthoringError(
                    f"{path}: hardpoint {name!r} is owned by {owner!r}, which is not "
                    "one of this template's bodies"
                )
        slot_types = {
            str(row["name"]): str(row["element_type"]) for row in payload["property_slots"]
        }

        joint_names: list[str] = []
        for joint in payload["joints"]:
            joint_names.append(str(joint["name"]))
            for key in ("body_a", "body_b"):
                if str(joint[key]) not in body_names | _ASSEMBLY_SUPPLIED_BODIES:
                    raise AuthoringError(
                        f"{path}: joint {joint['name']!r} references unknown body "
                        f"{joint[key]!r} in {key}"
                    )
            for key in ("point_a", "point_b"):
                point = joint.get(key)
                if point is not None and str(point) not in hardpoint_names:
                    raise AuthoringError(
                        f"{path}: joint {joint['name']!r} references unknown {key} "
                        f"{point!r}"
                    )
        if len(joint_names) != len(set(joint_names)):
            duplicates = sorted({n for n in joint_names if joint_names.count(n) > 1})
            raise AuthoringError(f"{path}: duplicate joint name(s) {duplicates}")

        element_names: list[str] = []
        for element in payload["elements"]:
            element_names.append(str(element["name"]))
            for key in ("body_a", "body_b"):
                if str(element[key]) not in body_names | _ASSEMBLY_SUPPLIED_BODIES:
                    raise AuthoringError(
                        f"{path}: element {element['name']!r} references unknown body "
                        f"{element[key]!r} in {key}"
                    )
            # A two-point element (a spring, a damper, a bump stop) must say where
            # both of its ends sit; a bushing acts at one point and states one.
            # Requiring the second point of every element would refuse a legal
            # bushing, and accepting a missing one of a spring would defer the
            # failure to the builder, which has no file to name.
            needs_both = str(element["type"]) in TWIN_ENDED_ELEMENTS
            for key in ("point_a", "point_b") if needs_both else ("point_a",):
                value = element.get(key)
                if value is None:
                    raise AuthoringError(
                        f"{path}: element {element['name']!r} of type "
                        f"{element['type']!r} needs a {key!r}, since its two ends are "
                        "placed on two different points"
                    )
                if str(value) not in hardpoint_names:
                    raise AuthoringError(
                        f"{path}: element {element['name']!r} references unknown {key} "
                        f"{value!r}"
                    )
            slot = str(element["property_slot"])
            if slot not in slot_types:
                raise AuthoringError(
                    f"{path}: element {element['name']!r} references unknown property "
                    f"slot {slot!r}"
                )
            slot_type = slot_types[slot]
            if slot_type not in {str(element["type"]), "generic"}:
                raise AuthoringError(
                    f"{path}: property slot {slot!r} declares element_type "
                    f"{slot_type!r}, which does not match element "
                    f"{element['name']!r} of type {element['type']!r}"
                )
        if len(element_names) != len(set(element_names)):
            duplicates = sorted({n for n in element_names if element_names.count(n) > 1})
            raise AuthoringError(f"{path}: duplicate element name(s) {duplicates}")

        # A declaration that says which modes it activates in must say something
        # real: an empty or repeated list is a declaration whose author meant to
        # restrict it and typed something that restricts nothing.
        for element in payload["elements"]:
            _check_modes(path, f"element {element['name']!r}", element.get("modes"))
        for joint in payload["joints"]:
            _check_modes(path, f"joint {joint['name']!r}", joint.get("modes"))
            reference = joint.get("axis_reference")
            if reference is not None and str(reference) not in hardpoint_names:
                raise AuthoringError(
                    f"{path}: joint {joint['name']!r} takes its axis from "
                    f"{reference!r}, which is not a hardpoint this template declares"
                )
            overrides = joint.get("kind_by_mode", ())
            declared_modes = {str(mode) for mode in joint.get("modes", ())} or {"K", "C"}
            for pair in overrides:
                if str(pair["mode"]) not in declared_modes:
                    raise AuthoringError(
                        f"{path}: joint {joint['name']!r} gives {pair['mode']!r} a type "
                        f"of its own but does not activate in {pair['mode']!r}"
                    )

        placements = [str(role) for role in payload["allowed_placement_roles"]]
        if len(placements) != len(set(placements)):
            raise AuthoringError(f"{path}: allowed_placement_roles repeats an entry")

        port_names = [str(port["name"]) for port in payload.get("ports", ())]
        if len(port_names) != len(set(port_names)):
            duplicates = sorted({n for n in port_names if port_names.count(n) > 1})
            raise AuthoringError(f"{path}: duplicate port name(s) {duplicates}")
        for port in payload.get("ports", ()):
            owner = str(port.get("owner", ""))
            if owner and owner not in body_names:
                raise AuthoringError(
                    f"{path}: port {port['name']!r} is attached to {owner!r}, which is "
                    "not one of this template's bodies"
                )

    @property
    def name(self) -> str:
        return str(self.payload["name"])

    @property
    def functional_role(self) -> str:
        return str(self.payload["functional_role"])

    @property
    def allowed_placement_roles(self) -> tuple[str, ...]:
        return tuple(str(role) for role in self.payload["allowed_placement_roles"])

    @property
    def topology_hash(self) -> str:
        """
        The hash of this template's topology.

        Only topology: a subsystem that changes a coordinate or repoints a
        property file must leave this value untouched, which is what makes "the
        user cannot change the template" checkable rather than merely claimed.
        """
        topology = {key: self.payload.get(key, []) for key in sorted(TOPOLOGY_KEYS)}
        return _hash(topology)

    @property
    def hardpoint_names(self) -> frozenset[str]:
        return frozenset(str(row["name"]) for row in self.payload["hardpoints"])

    @property
    def hardpoint_owners(self) -> Mapping[str, str]:
        return MappingProxyType(
            {str(row["name"]): str(row["owner"]) for row in self.payload["hardpoints"]}
        )

    @property
    def body_names(self) -> frozenset[str]:
        return frozenset(str(row["name"]) for row in self.payload["bodies"])

    @property
    def fixed_bodies(self) -> frozenset[str]:
        return frozenset(
            str(row["name"]) for row in self.payload["bodies"] if row.get("fixed")
        )

    @property
    def property_slots(self) -> Mapping[str, Mapping[str, Any]]:
        return MappingProxyType(
            {str(row["name"]): row for row in self.payload["property_slots"]}
        )

    @property
    def ports(self) -> Mapping[str, Mapping[str, Any]]:
        return MappingProxyType(
            {str(row["name"]): row for row in self.payload.get("ports", ())}
        )


@dataclass(frozen=True)
class EffectiveSubsystem:
    """
    One subsystem resolved against its template: the read-only effective model.

    ``hardpoints`` and ``property_bindings`` are read-only mappings, and the
    template payload is never handed out for mutation, so a caller that wants a
    different value has to build a new effective instance through an override --
    which is exactly the copy-on-write rule an assembly relies on.
    """

    name: str
    template: TemplateDocument
    functional_role: str
    placement_role: str
    hardpoints: Mapping[str, Any]
    parameters: Mapping[str, Any]
    property_bindings: Mapping[str, ElementPropertyDocument]

    @property
    def topology_hash(self) -> str:
        return self.template.topology_hash

    @property
    def values_hash(self) -> str:
        return _hash(
            {
                "hardpoints": {k: list(v) for k, v in self.hardpoints.items()},
                "parameters": dict(self.parameters),
            }
        )

    @property
    def property_bindings_hash(self) -> str:
        return _hash(
            {
                name: document.content_hash
                for name, document in sorted(self.property_bindings.items())
            }
        )

    @property
    def effective_values_hash(self) -> str:
        """Hash of the resolved constitutive values the solve path consumes."""
        return _hash(
            {
                name: document.effective_values_hash
                for name, document in sorted(self.property_bindings.items())
            }
        )

    def resolved_property(self, slot: str) -> Mapping[str, Any]:
        """Return the kernel-facing parameters of one bound property slot."""
        try:
            return self.property_bindings[slot].resolved
        except KeyError as exc:
            raise AuthoringError(
                f"subsystem {self.name!r} has no binding for property slot {slot!r}"
            ) from exc

    def plan(self) -> Mapping[str, Any]:
        """
        Return the declaration a declarative build reads.

        This is the effective model in the shape a composition needs: the
        template's topology, plus the coordinates and constitutive values this
        subsystem supplies.  It contains no Python and no callables, which is what
        lets a file-only template be built without a registered builder.
        """
        return MappingProxyType(
            {
                "name": self.name,
                "functional_role": self.functional_role,
                "placement_role": self.placement_role,
                "template": self.template.payload,
                "hardpoints": self.hardpoints,
                "parameters": self.parameters,
                "property_bindings": {
                    name: dict(document.resolved)
                    for name, document in self.property_bindings.items()
                },
                "hashes": {
                    "topology": self.topology_hash,
                    "values": self.values_hash,
                    "property_bindings": self.property_bindings_hash,
                    "effective_values": self.effective_values_hash,
                },
            }
        )


@dataclass(frozen=True)
class SubsystemDocument:
    """One subsystem: values for exactly one template."""

    path: Path
    payload: dict[str, Any]
    template: TemplateDocument

    @classmethod
    def load(cls, path: str | Path) -> "SubsystemDocument":
        target = Path(path).resolve()
        payload = _read(target)
        _check_contract(payload, target, validate_subsystem)
        _check_role(
            str(payload["functional_role"]), str(payload["placement_role"]), target
        )
        template_path = (target.parent / str(payload["template"])).resolve()
        template = TemplateDocument.load(template_path)

        if str(payload["functional_role"]) != template.functional_role:
            raise AuthoringError(
                f"{target}: functional_role {payload['functional_role']!r} disagrees "
                f"with template {template.name!r} of role {template.functional_role!r}"
            )
        placement = str(payload["placement_role"])
        if placement not in template.allowed_placement_roles:
            raise AuthoringError(
                f"{target}: placement_role {placement!r} is not allowed by template "
                f"{template.name!r}, which allows "
                f"{sorted(template.allowed_placement_roles)}"
            )
        missing_points = sorted(template.hardpoint_names - set(payload["hardpoints"]))
        if missing_points:
            raise AuthoringError(
                f"{target}: template {template.name!r} declares hardpoint(s) "
                f"{missing_points} that this subsystem does not place; every declared "
                "hardpoint needs a coordinate"
            )
        unknown_points = sorted(set(payload["hardpoints"]) - template.hardpoint_names)
        if unknown_points:
            raise AuthoringError(
                f"{target}: unknown hardpoint(s) {unknown_points}; a subsystem may only "
                "place the hardpoints its template declares"
            )
        unknown_slots = sorted(
            set(payload["property_bindings"]) - set(template.property_slots)
        )
        if unknown_slots:
            raise AuthoringError(
                f"{target}: unknown property binding(s) {unknown_slots}; a subsystem may "
                "only bind the slots its template declares"
            )
        return cls(target, copy.deepcopy(payload), template)

    @property
    def placement_role(self) -> str:
        return str(self.payload["placement_role"])

    @property
    def functional_role(self) -> str:
        return str(self.payload["functional_role"])

    @property
    def topology_hash(self) -> str:
        return self.template.topology_hash

    @property
    def values_hash(self) -> str:
        return _hash(
            {
                "hardpoints": self.payload["hardpoints"],
                "parameters": self.payload.get("parameters", {}),
            }
        )

    @property
    def property_bindings_hash(self) -> str:
        return _hash(self.payload["property_bindings"])

    def _load_bindings(
        self, bindings: Mapping[str, str]
    ) -> Mapping[str, ElementPropertyDocument]:
        """
        Load each bound property file against the slot that names it.

        The reference is resolved relative to *this* subsystem document, so a
        subsystem can be moved with its own property files and keep working, and a
        property file is checked against the slot's declared element type and model
        list before it is accepted.
        """
        slots = self.template.property_slots
        loaded: dict[str, ElementPropertyDocument] = {}
        for slot_name, reference in sorted(bindings.items()):
            slot = slots[slot_name]
            allowed = slot.get("allowed_models")
            try:
                loaded[slot_name] = ElementPropertyDocument.load(
                    self.path.parent / str(reference),
                    expected_type=str(slot["element_type"]),
                    allowed_models=tuple(str(m) for m in allowed) if allowed else None,
                )
            except ElementPropertyError as exc:
                raise AuthoringError(
                    f"{self.path}: property slot {slot_name!r}: {exc}"
                ) from exc
        # A slot needs a binding only when the role requires one *and* the slot
        # carries no default of its own.  A default is a real value of the
        # simplified model -- the built-in's compliant mounts really are at zero
        # stiffness -- so demanding a property file for one would be demanding a
        # number the template already states.
        missing = sorted(
            name
            for name, slot in slots.items()
            if slot.get("required") and name not in loaded and slot.get("default") is None
        )
        if missing:
            raise AuthoringError(
                f"{self.path}: required property slot(s) {missing} of template "
                f"{self.template.name!r} have no binding"
            )
        return MappingProxyType(loaded)

    def effective(
        self, *, overrides: Mapping[str, Any] | None = None
    ) -> EffectiveSubsystem:
        """
        Return the read-only effective subsystem, with ``overrides`` applied.

        An override is applied on top of the document and never written back: the
        result is a new value and ``self.payload`` is untouched, which is what
        makes an assembly's override copy-on-write rather than a mutation of the
        subsystem file it refers to.
        """
        hardpoints = {
            str(name): tuple(float(c) for c in value)
            for name, value in self.payload["hardpoints"].items()
        }
        parameters = dict(self.payload.get("parameters", {}))
        bindings = dict(self.payload["property_bindings"])
        if overrides:
            illegal = sorted(set(overrides) - OVERRIDE_KEYS)
            if illegal:
                raise AuthoringError(
                    f"{self.path}: an assembly override may only restate "
                    f"{sorted(OVERRIDE_KEYS)}; found {illegal}"
                )
            hardpoints.update(
                {
                    str(name): tuple(float(c) for c in value)
                    for name, value in overrides.get("hardpoints", {}).items()
                }
            )
            bindings.update(
                {str(k): str(v) for k, v in overrides.get("property_bindings", {}).items()}
            )
        unknown = sorted(set(hardpoints) - self.template.hardpoint_names)
        if unknown:
            raise AuthoringError(
                f"{self.path}: override names unknown hardpoint(s) {unknown}"
            )
        unknown = sorted(set(bindings) - set(self.template.property_slots))
        if unknown:
            raise AuthoringError(
                f"{self.path}: override names unknown property slot(s) {unknown}"
            )
        return EffectiveSubsystem(
            name=str(self.payload["name"]),
            template=self.template,
            functional_role=self.functional_role,
            placement_role=self.placement_role,
            hardpoints=_frozen_numbers(hardpoints),
            parameters=MappingProxyType(parameters),
            property_bindings=self._load_bindings(bindings),
        )


@dataclass(frozen=True)
class AssemblyEntry:
    """One subsystem reference inside an assembly, with its local overrides."""

    ref: str
    functional_role: str
    placement_role: str
    overrides: Mapping[str, Any]
    subsystem: SubsystemDocument
    #: Explicit pairings this entry states: requirement role -> port name.
    #:
    #: Optional, and empty means "let the matcher infer", which is what every
    #: assembly file written before the pairing section said.  It lives on the
    #: *entry* rather than on the document because a requirement belongs to the
    #: subsystem that declares it, and a document-wide table would let two
    #: entries state the same requirement role and disagree.
    pairings: Mapping[str, str] = MappingProxyType({})

    def effective(self) -> EffectiveSubsystem:
        """Return this entry's effective subsystem, overrides applied."""
        return self.subsystem.effective(overrides=self.overrides)


@dataclass(frozen=True)
class AssemblyDocument:
    """One assembly: which subsystems, at which roles, with local overrides."""

    path: Path
    payload: dict[str, Any]
    entries: tuple[AssemblyEntry, ...]

    @classmethod
    def load(cls, path: str | Path) -> "AssemblyDocument":
        target = Path(path).resolve()
        payload = _read(target)
        # Three questions, asked in the order their repairs become possible.  A role
        # the kind forbids is refused first: it is wrong no matter which file is
        # named.  Then each reference is checked against the file it names, because
        # "this assignment disagrees with the subsystem" is the fault to fix before
        # any count is worth reporting.  Only then is the shape of the whole set
        # judged -- a count taken over assignments that do not match their files
        # would report a number rather than the mistake behind it.
        _check_contract(payload, target, validate_assembly)
        cls._check_forbidden(target, str(payload["assembly_kind"]), payload["subsystems"])
        entries: list[AssemblyEntry] = []
        for row in payload["subsystems"]:
            subsystem = SubsystemDocument.load(
                (target.parent / str(row["ref"])).resolve()
            )
            functional = str(row["functional_role"])
            placement = str(row["placement_role"])
            _check_role(functional, placement, target)
            if functional != subsystem.functional_role:
                raise AuthoringError(
                    f"{target}: assignment for {row['ref']!r} says functional_role "
                    f"{functional!r}, but that subsystem is a "
                    f"{subsystem.functional_role!r}"
                )
            if not _placement_matches(placement, subsystem.placement_role):
                raise AuthoringError(
                    f"{target}: assignment for {row['ref']!r} says placement_role "
                    f"{placement!r}, but that subsystem declares "
                    f"{subsystem.placement_role!r}; a subsystem may only be reassigned "
                    "when it declares 'any'"
                )
            overrides = dict(row.get("overrides", {}))
            illegal = sorted(set(overrides) - OVERRIDE_KEYS)
            if illegal:
                raise AuthoringError(
                    f"{target}: override for {row['ref']!r} may only restate "
                    f"{sorted(OVERRIDE_KEYS)}; found {illegal}"
                )
            stated = row.get("pairings", [])
            pairings = {
                str(item["requirement_role"]): str(item["port"]) for item in stated
            }
            if len(pairings) != len(stated):
                repeated = sorted(
                    {
                        str(item["requirement_role"])
                        for item in stated
                        if [str(other["requirement_role"]) for other in stated].count(
                            str(item["requirement_role"])
                        )
                        > 1
                    }
                )
                raise AuthoringError(
                    f"{target}: the pairings for {row['ref']!r} state "
                    f"{repeated} more than once; one requirement gets one pairing, "
                    "or the file does not say which port is meant"
                )
            entry = AssemblyEntry(
                ref=str(row["ref"]),
                functional_role=functional,
                placement_role=placement,
                overrides=MappingProxyType(overrides),
                subsystem=subsystem,
                pairings=MappingProxyType(pairings),
            )
            # Resolve once here so an unreachable property file, an unknown
            # hardpoint or a topology-shaped override fails while loading the
            # assembly, naming this file, rather than three steps later.
            entry.effective()
            entries.append(entry)
        # Only once every reference has been checked against the file it names is the
        # shape of the whole set worth judging.
        cls._check_shape(target, str(payload["assembly_kind"]), entries)
        return cls(target, copy.deepcopy(payload), tuple(entries))

    @property
    def name(self) -> str:
        return str(self.payload["name"])

    @property
    def assembly_kind(self) -> str:
        return str(self.payload["assembly_kind"])

    @property
    def subsystems(self) -> tuple[SubsystemDocument, ...]:
        """The referenced subsystem documents, in assembly order."""
        return tuple(entry.subsystem for entry in self.entries)

    def role_counts(self) -> Mapping[tuple[str, str], int]:
        counts: dict[tuple[str, str], int] = {}
        for entry in self.entries:
            key = (entry.functional_role, entry.placement_role)
            counts[key] = counts.get(key, 0) + 1
        return MappingProxyType(counts)

    def wheel_ends(self) -> frozenset[str]:
        """
        Return the wheel ends this assembly accounts for.

        A suspension placed at ``front`` covers both front corners, because the
        role's own note says the left/right pair is one subsystem; a suspension
        placed at an explicit corner covers that corner only.  Reading coverage
        from the placements rather than from a count is what makes "four wheel
        ends" checkable for both of those spellings.
        """
        covered: set[str] = set()
        for entry in self.entries:
            if entry.functional_role == "wheel":
                covered.update(_corners_of(entry.placement_role, expand=True))
            elif entry.functional_role == "suspension":
                covered.update(_corners_of(entry.placement_role, expand=True))
        return frozenset(covered)

    def provenance(self) -> Mapping[str, Any]:
        """Return the file hashes this assembly resolves to, per subsystem."""
        return MappingProxyType(
            {
                "assembly": _hash(self.payload),
                "subsystems": {
                    entry.ref: {
                        "template": str(entry.subsystem.template.path),
                        "topology": entry.subsystem.template.topology_hash,
                        "values": entry.subsystem.values_hash,
                        "property_bindings": entry.subsystem.property_bindings_hash,
                        "effective_values": entry.effective().effective_values_hash,
                    }
                    for entry in self.entries
                },
            }
        )

    @staticmethod
    def _check_forbidden(
        path: Path,
        kind: str,
        entries: Iterable[AssemblyEntry | Mapping[str, Any]],
    ) -> None:
        """
        Refuse a role the assembly kind must not carry, before anything is loaded.

        Asked first because "an axle has no brake" is true whatever else the
        assembly says: reporting a missing suspension instead would name the wrong
        repair, and the role that is actually forbidden would stay in the file.  The
        rule itself lives in ``connections.policy``, which is its one home.
        """
        assignments = [_assignment_of(entry) for entry in entries]
        try:
            check_forbidden_roles(kind, [role for role, _placement in assignments])
        except RuleViolation as exc:
            raise AuthoringError(f"{path}: {exc}") from exc

    @staticmethod
    def _check_shape(
        path: Path,
        kind: str,
        entries: Iterable[AssemblyEntry | Mapping[str, Any]],
    ) -> None:
        """Judge the counts and placements of an assembly whose references resolve."""
        assignments = [_assignment_of(entry) for entry in entries]
        try:
            check_assembly_shape(kind, assignments)
        except RuleViolation as exc:
            raise AuthoringError(f"{path}: {exc}") from exc


def _assignment_of(entry: AssemblyEntry | Mapping[str, Any]) -> tuple[str, str]:
    """Return one entry's ``(functional_role, placement_role)`` assignment."""
    if isinstance(entry, Mapping):
        return str(entry["functional_role"]), str(entry["placement_role"])
    return entry.functional_role, entry.placement_role


def _corners_covered(assignments: Iterable[tuple[str, str]]) -> set[str]:
    """Return the wheel ends the suspensions and wheels in ``assignments`` cover."""
    covered: set[str] = set()
    for functional, placement in assignments:
        if functional in {"suspension", "wheel"}:
            covered.update(_corners_of(placement, expand=True))
    return covered


def _corners_of(placement: str, *, expand: bool) -> set[str]:
    if placement == "front" and expand:
        return {"front_left", "front_right"}
    if placement == "rear" and expand:
        return {"rear_left", "rear_right"}
    if placement == "any" and expand:
        return set(WHEEL_ENDS)
    return {placement}


def _placement_matches(assigned: str, declared: str) -> bool:
    """Return whether an assembly may place a subsystem at ``assigned``."""
    if assigned == declared:
        return True
    # A subsystem that declares it may sit anywhere can be assigned anywhere, and
    # one that widens to a whole axle covers both of its corners.
    if declared == "any":
        return assigned in PLACEMENT_ROLES
    expanded = _corners_of(declared, expand=False)
    if assigned == declared:
        return True
    return bool(expanded & _corners_of(assigned, expand=False))


@dataclass(frozen=True)
class RigDocument:
    """One test rig: what it supports, and the ports it insists on."""

    path: Path
    payload: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "RigDocument":
        target = Path(path).resolve()
        payload = _read(target)
        _check_contract(payload, target, validate_rig)
        for key in ("required_ports", "optional_ports"):
            names = [str(name) for name in payload.get(key, ())]
            if len(names) != len(set(names)):
                duplicates = sorted({n for n in names if names.count(n) > 1})
                raise AuthoringError(f"{target}: {key} repeats {duplicates}")
        overlap = sorted(
            set(str(name) for name in payload.get("required_ports", ()))
            & set(str(name) for name in payload.get("optional_ports", ()))
        )
        if overlap:
            raise AuthoringError(
                f"{target}: port(s) {overlap} are declared both required and optional"
            )
        cls._check_supports(target, payload)
        bench = payload.get("bench")
        if bench is not None:
            from ..rigs import get_rig, rig_names

            known = rig_names()
            if str(bench) not in known:
                raise AuthoringError(
                    f"{target}: bench {bench!r} is not a registered test bench; the "
                    f"registered benches are {list(known)}"
                )
            cls._check_actuators(target, payload, get_rig(str(bench)))
        return cls(target, copy.deepcopy(payload))

    @staticmethod
    def _check_supports(path: Path, payload: Mapping[str, Any]) -> None:
        """
        Refuse a rig whose supports do not agree with the ports it insists on.

        A support is the rig's own side of an attachment: this is the port it
        reaches for and what it can do there.  A port a rig *requires* but does
        not declare supporting is the fault worth naming -- the rig cannot both
        insist on an attachment and say it has none -- while declaring support
        for a port it merely tolerates stays legal, which is what makes a bench
        usable on more than the one assembly it was written for.
        """
        names = [str(row["name"]) for row in payload.get("supports", ())]
        if len(names) != len(set(names)):
            repeated = sorted({name for name in names if names.count(name) > 1})
            raise AuthoringError(f"{path}: supports repeats {repeated}")
        if not names:
            return
        declared = set(names) | {str(name) for name in payload.get("optional_ports", ())}
        unsupported = sorted(set(str(name) for name in payload["required_ports"]) - declared)
        if unsupported:
            raise AuthoringError(
                f"{path}: required port(s) {unsupported} are not declared in supports "
                f"or optional_ports; a rig cannot require an attachment it declares "
                f"no support for"
            )

    @staticmethod
    def _check_actuators(path: Path, payload: Mapping[str, Any], spec: Any) -> None:
        """
        Refuse an actuator the bench this rig names does not drive.

        The declaration is checked rather than restated: the bench stays the code
        that drives, loads and measures, and a file that names an actuator the
        bench has no drive for is refused by name instead of silently driving
        nothing.
        """
        actuators = [str(name) for name in payload.get("actuators", ())]
        if len(actuators) != len(set(actuators)):
            repeated = sorted({name for name in actuators if actuators.count(name) > 1})
            raise AuthoringError(f"{path}: actuators repeats {repeated}")
        unknown = sorted(set(actuators) - set(spec.coordinate_names()))
        if unknown:
            raise AuthoringError(
                f"{path}: actuator(s) {unknown} are not driven by bench "
                f"{spec.name!r}; it drives {list(spec.coordinate_names())}"
            )

    @property
    def name(self) -> str:
        return str(self.payload["name"])

    @property
    def supported_kinds(self) -> frozenset[str]:
        return frozenset(str(kind) for kind in self.payload["supported_assembly_kinds"])

    @property
    def required_ports(self) -> frozenset[str]:
        return frozenset(str(name) for name in self.payload["required_ports"])

    @property
    def optional_ports(self) -> frozenset[str]:
        """
        Return the ports this rig uses when offered, but does not insist on.

        The distinction is what lets one bench serve an axle with steering and one
        without: a required port is a fault when absent, an optional one is a
        connection when present.
        """
        return frozenset(str(name) for name in self.payload.get("optional_ports", ()))

    @property
    def supports(self) -> Mapping[str, tuple[str, ...]]:
        """Return the attachments this rig declares, by name, with their capabilities."""
        return MappingProxyType(
            {
                str(row["name"]): tuple(str(entry) for entry in row.get("capabilities", ()))
                for row in self.payload.get("supports", ())
            }
        )

    @property
    def actuators(self) -> tuple[str, ...]:
        """
        Return the bench coordinates this rig asks to be driven.

        The names are *drive coordinates* -- the ones the bench's own drive
        declarations use -- because naming the case layer's grouping keys instead
        would describe the same axis in a second vocabulary, and the two would be
        free to drift apart.
        """
        return tuple(str(name) for name in self.payload.get("actuators", ()))

    @property
    def measurements(self) -> tuple[str, ...]:
        """Return the channels this rig reports, in declaration order."""
        return tuple(str(name) for name in self.payload.get("measurements", ()))

    @property
    def bench(self) -> str | None:
        """Return the registered test bench this rig drives through, if it names one."""
        declared = self.payload.get("bench")
        return None if declared is None else str(declared)

    def bench_spec(self) -> object | None:
        """
        Return the registered bench this rig binds to.

        A file rig may declare which of the registered benches it *is*.  That is
        what connects the files to the solver's existing benches: the rig file says
        which of them to drive, and the driving, loading and measuring stay in the
        implementation that already does it rather than being restated in a file.
        """
        name = self.bench
        if name is None:
            return None
        from ..rigs import get_rig

        return get_rig(name)

    def check_assembly(self, assembly: AssemblyDocument) -> None:
        """
        Refuse an assembly this rig cannot support, naming the reason.

        Three separate questions, asked separately, because they have three
        different repairs: the rig does not support this kind of assembly, the rig
        wants a port the assembly does not offer, or a port exists but a subsystem
        role is not carried at all.
        """
        if assembly.assembly_kind not in self.supported_kinds:
            raise AuthoringError(
                f"{self.path}: does not support assembly kind "
                f"{assembly.assembly_kind!r}; it supports "
                f"{sorted(self.supported_kinds)}"
            )
        offered = self.offered_ports(assembly)
        missing = sorted(self.required_ports - set(offered))
        if missing:
            raise AuthoringError(
                f"{self.path}: assembly {assembly.name!r} does not offer required "
                f"port(s) {missing}; it offers {sorted(offered)}"
            )

    @staticmethod
    def offered_ports(assembly: AssemblyDocument) -> dict[str, str]:
        """Return the ports an assembly offers, keyed by port name."""
        offered: dict[str, str] = {}
        for entry in assembly.entries:
            for name, port in entry.subsystem.template.ports.items():
                offered.setdefault(str(name), entry.ref)
                offered.setdefault(
                    f"{entry.functional_role}:{name}", entry.ref
                )
        return offered


@dataclass(frozen=True)
class SimulationAssembly:
    """
    An assembly bound to a rig: the model a solve path reads.

    ``bindings`` records which subsystem supplies each port the rig asked for, so
    a match can be audited rather than guessed at, and ``provenance`` carries the
    file hashes of everything that went into the model.
    """

    assembly: AssemblyDocument
    rig: RigDocument
    bindings: Mapping[str, str]

    @classmethod
    def load(cls, assembly_path: str | Path) -> "SimulationAssembly":
        assembly = AssemblyDocument.load(assembly_path)
        rig_ref = assembly.payload.get("rig")
        if not rig_ref:
            raise AuthoringError(
                f"{assembly.path}: a SimulationAssembly requires a rig reference"
            )
        rig = RigDocument.load(assembly.path.parent / str(rig_ref))
        rig.check_assembly(assembly)
        offered = rig.offered_ports(assembly)
        # Both halves of the interface, resolved once: a required port is bound
        # because `check_assembly` refused the assembly without it, and an
        # optional one is bound when the assembly turns out to offer it.  The
        # result is the connection set the rig actually attaches to, which is
        # what makes it auditable rather than a restatement of the file.
        wanted = sorted(rig.required_ports | rig.optional_ports)
        bindings = {name: offered[name] for name in wanted if name in offered}
        return cls(assembly, rig, MappingProxyType(bindings))

    @property
    def channels(self) -> tuple[str, ...]:
        """Return the channels this simulation assembly is measured through."""
        return self.rig.measurements

    @property
    def name(self) -> str:
        return f"{self.assembly.name}@{self.rig.name}"

    def provenance(self) -> Mapping[str, Any]:
        """Return every file hash this simulation assembly depends on."""
        return MappingProxyType(
            {
                "rig": _hash(self.rig.payload),
                "bindings": dict(self.bindings),
                **dict(self.assembly.provenance()),
            }
        )

    def effective_subsystems(self) -> tuple[EffectiveSubsystem, ...]:
        """Return each subsystem's effective model, overrides applied."""
        return tuple(entry.effective() for entry in self.assembly.entries)
