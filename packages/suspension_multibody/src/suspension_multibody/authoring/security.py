"""
Who may change what: the expert and user editing surfaces.

The plan draws a line between two authors, and the line is about *fields*, not
about who is trustworthy:

* an **expert** defines a template: its bodies, points, joints, elements, slots and
  ports -- the topology;
* a **user** places a template's declared hardpoints, points the property slots at
  files, and assigns subsystems inside an assembly.  A user never adds a body or a
  joint, because a subsystem that changed the topology would silently stop being an
  instance of its template.

Enforcing that in the API rather than in a UI is the difference between a rule and
a suggestion: a caller that hand-edits a file is outside this module, and the
documents' own schemas still refuse a subsystem that carries a body.  What this
module adds is that the *supported* editing calls cannot be talked into writing one
in the first place, and that the payload they write is read back through the loader
before the call returns -- so an API that produced an unloadable file would fail
where it was called rather than at the next load.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from .documents import AssemblyDocument, SubsystemDocument, TemplateDocument
from .errors import AuthoringError
from .project import SUPPORTED_SCHEMA_VERSION

__all__ = [
    "EXPERT_FIELDS",
    "USER_ASSEMBLY_FIELDS",
    "USER_FIELDS",
    "AuthoringPermissionError",
    "ExpertAuthoring",
    "Revision",
    "UserAuthoring",
]


class AuthoringPermissionError(AuthoringError):
    """An editing call asked for something its author is not allowed to change."""


#: What an expert may state in a template: the topology and its outward promises.
EXPERT_FIELDS: frozenset[str] = frozenset(
    {
        "document",
        "schema_version",
        "name",
        "functional_role",
        "allowed_placement_roles",
        "bodies",
        "hardpoints",
        "joints",
        "elements",
        "property_slots",
        "ports",
        "needs",
        "outputs",
        "suspension_kind",
        "description",
    }
)

#: What a user may state in a subsystem.  ``hardpoints`` is here because placing a
#: *declared* point is a value; the same key is topology in a template, which is why
#: the two surfaces keep separate lists rather than sharing one.
USER_FIELDS: frozenset[str] = frozenset(
    {
        "document",
        "schema_version",
        "name",
        "template",
        "functional_role",
        "placement_role",
        "hardpoints",
        "property_bindings",
        "parameters",
    }
)

#: What a user may state in an assembly.
USER_ASSEMBLY_FIELDS: frozenset[str] = frozenset(
    {
        "document",
        "schema_version",
        "name",
        "assembly_kind",
        "subsystems",
        "rig",
        "description",
    }
)

#: What a user may state in one assembly entry, and in its overrides.
USER_ENTRY_FIELDS: frozenset[str] = frozenset(
    {"ref", "functional_role", "placement_role", "overrides"}
)
USER_OVERRIDE_FIELDS: frozenset[str] = frozenset({"hardpoints", "property_bindings"})


@dataclass(frozen=True)
class Revision:
    """One written document: where it went and the hash of what was written."""

    path: Path
    content_hash: str


def _reject(what: str, payload: Mapping[str, Any], allowed: frozenset[str]) -> None:
    """
    Refuse a payload naming a field its author may not state.

    The refusal names the offending field and why it is out of reach, because "you
    may not edit this document" would not tell a caller which argument to drop -- and
    the interesting case is precisely the caller who *meant* to add a body.
    """
    illegal = sorted(set(payload) - allowed)
    if illegal:
        raise AuthoringPermissionError(
            f"a {what} may not state {illegal}; the fields it may state are "
            f"{sorted(allowed)}.  Topology belongs to the template an expert owns, so "
            "a subsystem or assembly that changes it is not an instance of its template"
        )


def _write(path: Path, payload: Mapping[str, Any]) -> Revision:
    """Write one document, refusing to overwrite a file the project already has."""
    if path.exists():
        raise AuthoringPermissionError(
            f"{path} already exists; every editing call creates a new document or is a "
            "no-op, so nothing is overwritten by accident"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(dict(payload), indent=2, sort_keys=True)
    path.write_text(text, encoding="utf-8")
    return Revision(path, hashlib.sha256(text.encode("utf-8")).hexdigest())


@dataclass(frozen=True)
class ExpertAuthoring:
    """
    The expert surface: creating and changing templates.

    A template written here is read back through the loader before the call
    returns, so the API cannot produce a document the rest of the system would
    refuse -- the failure lands on the call that caused it, with the file name and
    the offending field.
    """

    root: Path

    def create_template(self, name: str, **declarations: Any) -> Revision:
        """
        Write one template.

        Every argument is a template field: ``functional_role``,
        ``allowed_placement_roles``, ``bodies``, ``hardpoints``, ``joints``,
        ``elements``, ``property_slots``, ``ports``, ``outputs``.
        """
        payload = self._payload(name, declarations)
        revision = _write(self.root / f"{name}.tpl.json", payload)
        TemplateDocument.load(revision.path)
        return revision

    def replace_template(self, revision: Revision, **declarations: Any) -> Revision:
        """
        Rewrite an existing template, keeping its identity.

        Replacing is the expert's own operation and is deliberately not what a user
        surface can reach: changing a template changes every subsystem built from it.
        """
        name = TemplateDocument.load(revision.path).name
        payload = self._payload(name, declarations)
        text = json.dumps(payload, indent=2, sort_keys=True)
        revision.path.write_text(text, encoding="utf-8")
        loaded = TemplateDocument.load(revision.path)
        return Revision(revision.path, loaded.topology_hash)

    @staticmethod
    def _payload(name: str, declarations: Mapping[str, Any]) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "document": "template",
            "schema_version": SUPPORTED_SCHEMA_VERSION,
            "name": name,
            **declarations,
        }
        _reject("template", payload, EXPERT_FIELDS)
        return payload


@dataclass(frozen=True)
class UserAuthoring:
    """
    The user surface: placing declared points, binding property files, assembling.

    Every method funnels through one allow-list check, so a caller cannot reach a
    topology field by finding the method that forgot to check: there is one check
    and every write goes through it.
    """

    root: Path

    def create_subsystem(
        self,
        name: str,
        *,
        template: str,
        functional_role: str,
        placement_role: str,
        hardpoints: Mapping[str, Any],
        property_bindings: Mapping[str, str] | None = None,
        parameters: Mapping[str, Any] | None = None,
        **extra: Any,
    ) -> Revision:
        """
        Place one template's declared hardpoints, and bind its slots to files.

        ``extra`` exists so a caller reaching for a topology field gets this module's
        refusal, which names the field and says where topology belongs, instead of a
        Python ``TypeError`` that says only that the keyword is unexpected.
        """
        payload: dict[str, Any] = {
            "document": "subsystem",
            "schema_version": SUPPORTED_SCHEMA_VERSION,
            "name": name,
            "template": template,
            "functional_role": functional_role,
            "placement_role": placement_role,
            "hardpoints": dict(hardpoints),
            "property_bindings": dict(property_bindings or {}),
            **({"parameters": dict(parameters)} if parameters else {}),
            **extra,
        }
        _reject("subsystem", payload, USER_FIELDS)
        revision = _write(self.root / f"{name}.sub.json", payload)
        SubsystemDocument.load(revision.path).effective()
        return revision

    def set_hardpoints(
        self, revision: Revision, hardpoints: Mapping[str, Any]
    ) -> Revision:
        """
        Move declared hardpoints of one subsystem.

        Only coordinates are written.  A name the template does not declare is
        refused by the loader, so this cannot be used to add a point either.
        """
        payload = _read(revision.path)
        payload["hardpoints"] = {**payload["hardpoints"], **dict(hardpoints)}
        _reject("subsystem", payload, USER_FIELDS)
        revision.path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        SubsystemDocument.load(revision.path).effective()
        return Revision(revision.path, _hash_file(revision.path))

    def bind_property(self, revision: Revision, slot: str, reference: str) -> Revision:
        """Repoint one property slot at another property file."""
        payload = _read(revision.path)
        bindings = dict(payload["property_bindings"])
        bindings[str(slot)] = str(reference)
        payload["property_bindings"] = bindings
        _reject("subsystem", payload, USER_FIELDS)
        revision.path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        SubsystemDocument.load(revision.path).effective()
        return Revision(revision.path, _hash_file(revision.path))

    def create_assembly(
        self,
        name: str,
        *,
        assembly_kind: str,
        subsystems: list[Mapping[str, Any]],
        rig: str | None = None,
    ) -> Revision:
        """Combine subsystems into an assembly, optionally bound to a rig."""
        entries = [self._entry(entry) for entry in subsystems]
        payload: dict[str, Any] = {
            "document": "assembly",
            "schema_version": SUPPORTED_SCHEMA_VERSION,
            "name": name,
            "assembly_kind": assembly_kind,
            "subsystems": entries,
            **({"rig": rig} if rig else {}),
        }
        _reject("assembly", payload, USER_ASSEMBLY_FIELDS)
        revision = _write(self.root / f"{name}.asy.json", payload)
        AssemblyDocument.load(revision.path)
        return revision

    def override(
        self,
        revision: Revision,
        ref: str,
        *,
        hardpoints: Mapping[str, Any] | None = None,
        property_bindings: Mapping[str, str] | None = None,
    ) -> Revision:
        """
        Apply an assembly-level override to one referenced subsystem.

        The override lives in this file only: the subsystem document it names is not
        touched, which is what makes "adjust it in the assembly" a different
        operation from "change the subsystem for everyone".
        """
        payload = _read(revision.path)
        overrides: dict[str, Any] = {}
        if hardpoints:
            overrides["hardpoints"] = dict(hardpoints)
        if property_bindings:
            overrides["property_bindings"] = dict(property_bindings)
        _reject("assembly override", overrides, USER_OVERRIDE_FIELDS)
        found = False
        for entry in payload["subsystems"]:
            if str(entry["ref"]) == ref:
                entry["overrides"] = {**entry.get("overrides", {}), **overrides}
                found = True
        if not found:
            raise AuthoringPermissionError(
                f"{revision.path}: no subsystem entry refers to {ref!r}; the assembly "
                f"refers to {[entry['ref'] for entry in payload['subsystems']]}"
            )
        _reject("assembly", payload, USER_ASSEMBLY_FIELDS)
        revision.path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        AssemblyDocument.load(revision.path)
        return Revision(revision.path, _hash_file(revision.path))

    @staticmethod
    def _entry(entry: Mapping[str, Any]) -> dict[str, Any]:
        """Copy one assembly entry, refusing topology in its overrides."""
        _reject("assembly entry", entry, USER_ENTRY_FIELDS)
        overrides = dict(entry.get("overrides", {}))
        _reject("assembly override", overrides, USER_OVERRIDE_FIELDS)
        return {**dict(entry), "overrides": overrides} if overrides else dict(entry)


def _read(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise AuthoringPermissionError(f"{path}: a document must be a JSON object")
    return payload


def _hash_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
