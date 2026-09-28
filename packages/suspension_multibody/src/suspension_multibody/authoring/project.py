"""
A project directory: the files a model is made of, and how they refer to each other.

The loader is what turns "a folder of JSON" into "a model with references that
resolve".  Three things it does that reading one file at a time cannot:

* **it knows the kinds.**  A document says what it is in its own ``document`` field,
  so a project can be scanned without a naming convention, and a file whose kind
  disagrees with what it contains cannot be filed under the wrong sort.
* **it resolves references.**  A subsystem's template, an assembly's subsystems and
  its rig are paths relative to the referring file, which is what lets a project be
  moved as a directory.  A reference that does not land is reported with both ends.
* **it refuses a project that cannot be loaded whole.**  A cycle among references,
  two documents of one kind claiming the same name, or a document whose
  ``schema_version`` is not the one this build reads are all reported at load time
  rather than as a confusing failure while assembling.

Nothing here decides what a model *means*: every document is loaded through the
layer that owns its validation, and this module only adds what one document cannot
know -- the other files.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

from .documents import (
    AssemblyDocument,
    AuthoringError,
    RigDocument,
    SimulationAssembly,
    SubsystemDocument,
    TemplateDocument,
)
from .properties import ElementPropertyDocument

__all__ = [
    "DOCUMENT_KINDS",
    "Project",
    "ProjectError",
    "SUPPORTED_SCHEMA_VERSION",
]

#: The ``document`` value each file kind carries, mapped to what it is.
DOCUMENT_KINDS: dict[str, str] = {
    "template": "template",
    "subsystem": "subsystem",
    "assembly": "assembly",
    "rig": "rig",
    "element_properties": "element_properties",
}

#: The contract version this build reads.  A file stating another one is refused
#: rather than parsed on the chance that the fields happen to line up.
SUPPORTED_SCHEMA_VERSION = 1


class ProjectError(AuthoringError):
    """A project directory cannot be loaded as a whole."""


@dataclass(frozen=True)
class DocumentSet:
    """Every document one kind contributes, keyed by the name it declares."""

    kind: str
    documents: Mapping[str, Path]

    def path(self, name: str) -> Path:
        """Return one document's path, naming the unknown one if it is missing."""
        try:
            return self.documents[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.documents)) or "(none)"
            raise ProjectError(
                f"no {self.kind} named {name!r} in this project; it declares {known}"
            ) from exc


@dataclass(frozen=True)
class Project:
    """
    One project directory, with every document it declares and its file hashes.

    ``hashes`` are hashes of the *files* rather than of their canonical payloads:
    they are what a result records to say which bytes produced it, and they change
    when a file changes even if the change is only whitespace.  The canonical
    hashes the documents expose answer the different question of whether the model
    changed, and the two are deliberately separate.
    """

    root: Path
    sets: Mapping[str, DocumentSet]

    @classmethod
    def load(cls, root: str | Path) -> "Project":
        directory = Path(root).resolve()
        if not directory.is_dir():
            raise ProjectError(f"{directory}: not a project directory")
        found: dict[str, dict[str, Path]] = {kind: {} for kind in DOCUMENT_KINDS.values()}
        for path in sorted(directory.rglob("*.json")):
            payload = _read_payload(path)
            kind = _kind_of(path, payload)
            name = str(payload.get("name", "")) or path.stem
            existing = found[kind].get(name)
            if existing is not None:
                raise ProjectError(
                    f"{path}: a {kind} named {name!r} is already declared by "
                    f"{existing}; one project level holds one document per name"
                )
            found[kind][name] = path
        sets = {kind: DocumentSet(kind, dict(documents)) for kind, documents in found.items()}
        project = cls(directory, sets)
        project._check_references()
        return project

    def _check_references(self) -> None:
        """
        Walk every reference, and refuse a project that does not resolve.

        The graph is acyclic by construction for the format this build reads -- a
        subsystem names a template, an assembly names subsystems and a rig, nothing
        names an assembly -- so a cycle means the references disagree with what the
        files say they are, and reporting it is cheaper than discovering it as a
        recursion while resolving.
        """
        edges: dict[Path, list[tuple[Path, str]]] = {}
        for kind in ("subsystem", "assembly"):
            for name, path in self.sets[kind].documents.items():
                payload = _read_payload(path)
                targets: list[tuple[Path, str]] = []
                if kind == "subsystem":
                    targets.append((_relative(path, payload["template"]), "template"))
                else:
                    for entry in payload["subsystems"]:
                        targets.append((_relative(path, entry["ref"]), "subsystem"))
                    if payload.get("rig"):
                        targets.append((_relative(path, payload["rig"]), "rig"))
                for target, expected in targets:
                    if not target.is_file():
                        raise ProjectError(
                            f"{path}: the {name!r} {kind} refers to {target}, which "
                            f"does not exist"
                        )
                    declared = _kind_of(target, _read_payload(target))
                    if declared != expected:
                        raise ProjectError(
                            f"{path}: the {name!r} {kind} refers to {target} as a "
                            f"{expected}, but that file declares itself a {declared}"
                        )
                edges[path] = targets
        self._refuse_cycles(edges)

    @staticmethod
    def _refuse_cycles(edges: Mapping[Path, list[tuple[Path, str]]]) -> None:
        visiting: list[Path] = []
        done: set[Path] = set()

        def walk(node: Path) -> None:
            if node in done:
                return
            if node in visiting:
                cycle = visiting[visiting.index(node) :] + [node]
                raise ProjectError(
                    "these documents refer to each other in a cycle: "
                    + " -> ".join(str(step) for step in cycle)
                )
            visiting.append(node)
            for target, _expected in edges.get(node, ()):
                walk(target)
            visiting.pop()
            done.add(node)

        for node in edges:
            walk(node)

    # -- access -------------------------------------------------------------

    def names(self, kind: str) -> tuple[str, ...]:
        """Return the names one kind declares, in a stable order."""
        return tuple(sorted(self.sets[kind].documents))

    def template(self, name: str) -> TemplateDocument:
        return TemplateDocument.load(self.sets["template"].path(name))

    def subsystem(self, name: str) -> SubsystemDocument:
        return SubsystemDocument.load(self.sets["subsystem"].path(name))

    def assembly(self, name: str) -> AssemblyDocument:
        return AssemblyDocument.load(self.sets["assembly"].path(name))

    def rig(self, name: str) -> RigDocument:
        return RigDocument.load(self.sets["rig"].path(name))

    def properties(self, name: str) -> ElementPropertyDocument:
        return ElementPropertyDocument.load(self.sets["element_properties"].path(name))

    def simulation(self, assembly_name: str) -> SimulationAssembly:
        """Load one assembly with the rig and references it names."""
        return SimulationAssembly.load(self.sets["assembly"].path(assembly_name))

    def hashes(self) -> Mapping[str, str]:
        """
        Return the content hash of every file this project declares.

        A file is hashed as bytes, not as parsed JSON: the point is to record what
        was read, so a whitespace-only change is still a change of input.
        """
        return {
            str(path.relative_to(self.root)).replace("\\", "/"): _content_hash(path)
            for kind in DOCUMENT_KINDS.values()
            for path in self.sets[kind].documents.values()
        }

    def provenance(self, assembly_name: str) -> Mapping[str, Any]:
        """
        Return the hashes a run of one assembly depends on.

        Two views of the same inputs, kept side by side because they answer
        different questions: ``files`` says which bytes were read, and ``model`` says
        whether the effective model changed.  A whitespace edit moves the first and
        not the second, and a coordinate edit moves both.
        """
        simulation = self.simulation(assembly_name)
        return {
            "files": dict(self.hashes()),
            "model": dict(simulation.provenance()),
        }


def _read_payload(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ProjectError(f"{path}: cannot read document: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ProjectError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProjectError(f"{path}: a document must be a JSON object")
    return payload


def _kind_of(path: Path, payload: Mapping[str, Any]) -> str:
    """Return which kind of document ``payload`` declares itself to be."""
    declared = payload.get("document")
    if not isinstance(declared, str) or declared not in DOCUMENT_KINDS:
        raise ProjectError(
            f"{path}: a document must name its kind in 'document'; known kinds are "
            f"{sorted(DOCUMENT_KINDS)}"
        )
    version = payload.get("schema_version")
    if version != SUPPORTED_SCHEMA_VERSION:
        raise ProjectError(
            f"{path}: schema_version must be {SUPPORTED_SCHEMA_VERSION}, found "
            f"{version!r}; this build reads that version only"
        )
    return DOCUMENT_KINDS[declared]


def _relative(path: Path, reference: Any) -> Path:
    """Resolve one reference against the file that states it."""
    return (path.parent / str(reference)).resolve()


def _content_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def iter_document_paths(root: Path) -> Iterable[Path]:
    """Yield every JSON document under ``root``, in a stable order."""
    return sorted(root.rglob("*.json"))
