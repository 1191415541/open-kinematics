"""
Model identity: one stable name for one model.

A result has to be able to say *which model* it came from, and the answer must be
the same tomorrow.  The identity here is computed from the generic documents a
model is made of -- the template's topology, a subsystem's values, an assembly's
role set -- so it is a property of the model rather than of the Python class a
caller happened to build it with.

Two rules make the value usable:

* **stable** -- the same model computed twice gives the same identity, and the
  identity does not depend on where the declaration came from (a file and an
  in-memory assembly of the same model share one identity);
* **sensitive** -- a changed coordinate, a changed law or a changed role set
  changes the identity, so a tuning loop can tell two runs apart.

What it is *not*: a replacement for the recorded ``Provenance.model_hash`` of
existing results.  Those hashes are frozen evidence, and moving them would
invalidate results nobody re-ran.  This is the identity a *new* caller asks for;
the legacy hash stays exactly where it is until the models it describes are gone.
"""

from __future__ import annotations

from typing import Any, Mapping

from ..io import canonical_hash
from .documents import AssemblyDocument, SubsystemDocument, TemplateDocument

__all__ = ["model_identity", "identity_of"]

#: The fields a template's *topology* consists of.  A subsystem that changes a
#: coordinate or repoints a property does not touch these, which is what makes
#: "the topology is unchanged" a checkable statement rather than a claim.
_TOPOLOGY_FIELDS = ("bodies", "hardpoints", "joints", "elements", "property_slots", "ports")


def model_identity(model: Any) -> str:
    """
    Return the identity hash of a generic model object.

    Accepts a template, a subsystem, an assembly, or a mapping that a caller has
    already declared.  The accepted shapes are the *documents*, not the result
    containers: an identity is about what was built, and everything that records
    that lives in the document layer.
    """
    return canonical_hash(_identity_payload(model))


def identity_of(model: Any) -> str:
    """Alias for :func:`model_identity`, for callers that read better that way."""
    return model_identity(model)


def _identity_payload(model: Any) -> Any:
    """Return the declaration an identity is computed from."""
    if isinstance(model, TemplateDocument):
        payload = model.to_payload()
        return {
            "kind": "template",
            "name": payload.get("name"),
            "functional_role": payload.get("functional_role"),
            "allowed_placement_roles": payload.get("allowed_placement_roles"),
            # The topology fields are taken as declared, not expanded: mirroring
            # is an instantiation decision, and an identity that baked it in would
            # report two different names for one written template.
            **{field: payload.get(field, []) for field in _TOPOLOGY_FIELDS},
        }
    if isinstance(model, SubsystemDocument):
        payload = model.to_payload()
        return {
            "kind": "subsystem",
            "name": payload.get("name"),
            "template": model.template.name,
            "topology": model.template.topology_hash,
            "functional_role": payload.get("functional_role"),
            "placement_role": payload.get("placement_role"),
            "hardpoints": payload.get("hardpoints", {}),
            "parameters": payload.get("parameters", {}),
            "property_bindings": payload.get("property_bindings", {}),
            # The *values* of the bound laws, not only the files they name.  A
            # tuning loop changes a stiffness and nothing else, so an identity
            # that stopped at the file name would report two different models as
            # one -- which is the property the identity exists to provide.
            "laws": _laws_fingerprint(model),
        }
    if isinstance(model, AssemblyDocument):
        return {
            "kind": "assembly",
            "name": model.name,
            "assembly_kind": model.assembly_kind,
            "entries": [
                {
                    "ref": entry.ref,
                    "functional_role": entry.functional_role,
                    "placement_role": entry.placement_role,
                    "overrides": dict(entry.overrides),
                    "pairings": dict(entry.pairings),
                    "subsystem": _identity_payload(entry.subsystem),
                    "laws": _laws_fingerprint(entry),
                }
                for entry in model.entries
            ],
            "rig": model.rig.name if model.rig is not None else None,
        }
    if isinstance(model, Mapping):
        # A bare declaration is taken as written: normalising it would mean
        # deciding what a caller *meant*, and two spellings of one model are
        # exactly what an identity is supposed to distinguish.
        return {"kind": "declaration", "payload": dict(model)}
    raise TypeError(
        "model_identity takes a TemplateDocument, SubsystemDocument, "
        f"AssemblyDocument or a mapping; got {type(model).__name__}"
    )


def _laws_fingerprint(source: Any) -> dict[str, str]:
    """
    Return the effective values of the laws a document or entry resolves.

    Resolving is what makes the answer the same whichever route built the model:
    a file-backed subsystem reads its laws and an in-memory one uses the laws it
    carries, and both report the same value hash.  When a law cannot be resolved
    at all -- an in-memory document with neither preloaded laws nor a path -- the
    fingerprint is empty, and the identity then covers the declaration only.
    """
    try:
        effective = source.effective()
    except Exception:
        return {}
    return {
        name: document.effective_values_hash
        for name, document in sorted(effective.property_bindings.items())
    }
