"""One document loading boundary with pinned, actually consumed resources."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from suspension_contracts import (
    ContractError,
    canonical_json_bytes,
    contract_hash,
    parse_json,
    validate_case,
    validate_solve_plan,
)

from ..modeling.resolved import ResolvedModel
from .documents import (
    AssemblyDocument,
    RigDocument,
    SubsystemDocument,
    TemplateDocument,
)
from .errors import AuthoringError
from .properties import ElementPropertyDocument


@dataclass(frozen=True, init=False)
class CaseDocument:
    """Immutable analysis data, validated identically for path and memory input."""

    _payload: bytes

    def __init__(self, payload: Mapping[str, Any]) -> None:
        if not isinstance(payload, Mapping):
            raise AuthoringError("case input must be a document object or path")
        declared = parse_json(canonical_json_bytes(payload))
        try:
            if declared.get("contract") == "multibody-case":
                validate_case(declared)
                required = {"ride_four_post": ("four_post", "corners"),
                    "ride_random_road": ("ride_random_road", "wheels")}
                if declared.get("family") in required:
                    section, rows = required[declared["family"]]
                    if not declared.get(section, {}).get(rows):
                        raise AuthoringError(f"case requires '{section}' section with at least one {rows}")
            else:
                validate_solve_plan(declared)
        except ContractError as error:
            raise AuthoringError(f"case {declared.get('name', '?')!r}: {error}") from error
        object.__setattr__(self, "_payload", canonical_json_bytes(declared))

    def to_payload(self) -> dict[str, Any]:
        """Return detached data rather than mutable aliases to the case."""
        return parse_json(self._payload)

    def save(self, path: str | Path) -> Path:
        """Persist the same analysis data accepted by the common loader."""
        target = Path(path).resolve()
        target.write_bytes(self._payload)
        return target


@dataclass(frozen=True)
class ResourceRecord:
    """A logical resource and the exact version/content selected at load time."""

    reference: str
    kind: str
    content_sha256: str
    source: str
    file_sha256: str = ""
    version: int = 1

    def to_payload(self) -> dict[str, Any]:
        """Return the manifest record suitable for compiled metadata."""
        return {
            "reference": self.reference, "kind": self.kind,
            "content_sha256": self.content_sha256, "source": self.source,
            "file_sha256": self.file_sha256, "version": self.version,
        }


@dataclass(frozen=True)
class LoadedDocuments:
    """A fully bound model and case, detached from their original directories."""

    assembly: AssemblyDocument
    case: CaseDocument
    manifest: tuple[ResourceRecord, ...]

    @property
    def resource_fingerprint(self) -> str:
        """Identify effective resource contents independently of source locations."""
        return contract_hash([
            {"reference": row.reference, "kind": row.kind,
             "content_sha256": row.content_sha256, "version": row.version}
            for row in self.manifest
        ])

    def resolve(self) -> ResolvedModel:
        """Resolve pinned documents with their consumed-resource provenance."""
        from .generic import assemble_generic

        model = assemble_generic(self.assembly).resolved_model()
        graph = model.to_document()
        graph["resource_manifest"] = [row.to_payload() for row in self.manifest if row.kind != "case"]
        return ResolvedModel(graph, model.resource_payload)


class DocumentLoader:
    """Load and pin references; resolution and compilation consume the same bundle."""

    def __init__(self, *, resource_root: str | Path | None = None) -> None:
        self.resource_root = None if resource_root is None else Path(resource_root).resolve()

    def _path(self, value: str | Path) -> Path:
        path = Path(value)
        if not path.is_absolute() and self.resource_root is not None:
            path = self.resource_root / path
        return path.resolve()

    @staticmethod
    def _read(path: Path) -> Mapping[str, Any]:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise AuthoringError(f"{path}: cannot load document: {error}") from error
        if not isinstance(value, dict):
            raise AuthoringError(f"{path}: document root must be an object")
        return value

    def load(
        self,
        assembly: str | Path | AssemblyDocument,
        case: str | Path | Mapping[str, Any] | CaseDocument,
    ) -> LoadedDocuments:
        """Validate both sources once and retain all effective reference bindings."""
        file_input = isinstance(assembly, (str, Path))
        source = AssemblyDocument.load(self._path(assembly)) if file_input else assembly
        if not isinstance(source, AssemblyDocument):
            raise AuthoringError("assembly input must be a path or AssemblyDocument")
        records: dict[tuple[str, str], ResourceRecord] = {}

        def record(reference: str, kind: str, payload: Mapping[str, Any], path: Path | None) -> None:
            identity = (kind, reference)
            content = contract_hash(payload)
            previous = records.get(identity)
            if previous is not None:
                if previous.content_sha256 != content:
                    raise AuthoringError(f"ambiguous {kind} reference {reference!r}")
                return
            file_hash = ""
            if (file_input or kind == "case") and path is not None:
                try:
                    raw = path.read_bytes()
                except OSError as error:
                    raise AuthoringError(f"consumed resource {path}: {error}") from error
                if path.suffix.lower() == ".tir":
                    loaded = ElementPropertyDocument.from_tir_text(raw.decode("ascii", errors="replace"), name=path.stem)
                    actual = contract_hash(loaded.payload)
                else:
                    actual = contract_hash(json.loads(raw.decode("utf-8")))
                if actual != content:
                    raise AuthoringError(f"consumed resource changed while loading: {path}")
                file_hash = hashlib.sha256(raw).hexdigest()
            records[identity] = ResourceRecord(reference, kind, content, str(path) if path else "memory", file_hash)

        record("assembly", "assembly", source.payload, source.path)
        subsystems: dict[str, SubsystemDocument] = {}
        properties: dict[str, Mapping[str, ElementPropertyDocument]] = {}
        for entry in source.entries:
            subsystem = entry.subsystem
            template = subsystem.template
            effective = entry.effective()
            record(entry.ref, "subsystem", subsystem.payload, subsystem.path)
            record(f"{entry.ref}:template", "template", template.payload, template.path)
            laws = {}
            for slot, law in effective.property_bindings.items():
                record(f"{entry.ref}:property:{slot}", "property", law.payload, law.path)
                laws[slot] = ElementPropertyDocument.from_payload(law.to_payload())
            detached = SubsystemDocument.from_payload(
                subsystem.to_payload(), template=TemplateDocument.from_payload(template.to_payload()),
                properties=laws,
            )
            # Effective bindings include entry overrides; all consumers receive
            # the pinned objects instead of re-opening the referenced resources.
            subsystems[entry.ref] = detached
            properties[entry.ref] = laws
        rig = source.rig
        if rig is None and source.payload.get("rig"):
            if source.path is None:
                raise AuthoringError("assembly rig reference needs an explicit bound RigDocument")
            rig = RigDocument.load(source.path.parent / source.payload["rig"])
        if rig is not None:
            record("rig", "rig", rig.payload, rig.path)
            rig = RigDocument.from_payload(rig.to_payload())
        pinned = AssemblyDocument.from_payload(source.to_payload(), subsystems=subsystems, properties=properties, rig=rig)
        if isinstance(case, (str, Path)):
            case_path = self._path(case)
            case_payload = self._read(case_path)
            record("case", "case", case_payload, case_path)
            loaded_case = CaseDocument(case_payload)
        else:
            loaded_case = case if isinstance(case, CaseDocument) else CaseDocument(case)
            record("case", "case", loaded_case.to_payload(), None)
        return LoadedDocuments(pinned, loaded_case, tuple(records[key] for key in sorted(records)))
