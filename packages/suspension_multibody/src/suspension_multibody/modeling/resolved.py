"""Immutable SI model graph, independent of authoring and simulation routes."""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np
from suspension_contracts import (
    canonical_json_bytes,
    contract_hash,
    parse_json,
    validate_resolved_model,
    validate_solve_plan,
)

from .primitives.spatial import SE3


@dataclass(frozen=True, init=False)
class ResolvedModel:
    """One validated entity graph; encoded ownership prevents mutable aliases."""

    _document: bytes
    resource_payload: bytes

    def __init__(self, document: Mapping[str, Any], resource_payload: bytes = b"") -> None:
        validate_resolved_model(document)
        if not isinstance(resource_payload, bytes):
            raise TypeError("resolved model resource payload must be bytes")
        object.__setattr__(self, "_document", canonical_json_bytes(document))
        object.__setattr__(self, "resource_payload", resource_payload)

    @property
    def name(self) -> str:
        """Return the model's display name."""
        return self.to_document()["name"]

    def to_document(self) -> dict[str, Any]:
        """Return detached SI data; edits cannot alter the resolved graph."""
        return parse_json(self._document)

    @property
    def fingerprint(self) -> str:
        """Identify effective physical data, separately from source locations."""
        document = self.to_document()
        for key in ("name", "provenance", "resource_manifest"):
            document.pop(key, None)
        for key in ("bodies", "frames", "joints", "elements", "tires", "ports", "coordinates", "signals", "measurements", "couplers"):
            if key in document:
                document[key] = sorted(document[key], key=lambda row: row["name"])
        if "gauges" in document:
            document["gauges"] = sorted(document["gauges"], key=lambda row: row["body"])
        if self.resource_payload:
            document["resource_payload_sha256"] = hashlib.sha256(self.resource_payload).hexdigest()
        return contract_hash(document)

    @property
    def provenance_fingerprint(self) -> str:
        """Identify both physical data and its recorded source resources."""
        return contract_hash({"document": self.to_document(), "resource_payload_sha256": hashlib.sha256(self.resource_payload).hexdigest()})

    def subgraph_fingerprint(self, body_ids: tuple[str, ...]) -> str:
        """Identify a component's physical structure independently of placement."""
        document = self.to_document()
        bodies = {row["name"]: row for row in document["bodies"]}
        if any(name not in bodies for name in body_ids):
            raise ValueError("subgraph references an unknown body")
        selected = set(body_ids)
        if not selected:
            raise ValueError("subgraph must select at least one body")
        anchor = bodies[min(selected)]
        anchor_pose = SE3(np.asarray(anchor.get("position", (0., 0., 0.))), np.asarray(anchor.get("quaternion", (1., 0., 0., 0.))))
        rows = []
        for name in sorted(selected):
            row = dict(bodies[name])
            pose = SE3(np.asarray(row.get("position", (0., 0., 0.))), np.asarray(row.get("quaternion", (1., 0., 0., 0.))))
            relative = anchor_pose.inverse().compose(pose)
            row["position"] = np.round(relative.translation, 12).tolist()
            row["quaternion"] = np.round(relative.quaternion, 12).tolist()
            rows.append(row)
        # Frames and joints are already body-local; only ownership decides scope.
        local = {
            "bodies": rows,
            "frames": sorted((row for row in document["frames"] if row["body"] in selected), key=lambda row: row["name"]),
            "joints": sorted((row for row in document["joints"] if row["body_a"] in selected and row["body_b"] in selected), key=lambda row: row["name"]),
            "elements": sorted((row for row in document["elements"] if row.get("body_a") in selected and row.get("body_b") in selected), key=lambda row: row["name"]),
            "tires": sorted((row for row in document.get("tires", ()) if row["body"] in selected), key=lambda row: row["name"]),
        }
        joint_ids = {row["name"] for row in local["joints"]}
        local["coordinates"] = sorted((row for row in document.get("coordinates", ()) if row["source_joint_id"] in joint_ids), key=lambda row: row["name"])
        def canonical(value: Any) -> Any:
            if isinstance(value, float):
                rounded = round(value, 12)
                return 0.0 if rounded == 0 else rounded
            if isinstance(value, dict):
                return {key: canonical(item) for key, item in value.items()}
            if isinstance(value, list):
                return [canonical(item) for item in value]
            return value

        return contract_hash(canonical(local))


@dataclass(frozen=True, init=False)
class ResolvedSolvePlan:
    """Validated run facts without a copy of the physical model."""

    _document: bytes
    input_payload: bytes

    def __init__(self, document: Mapping[str, Any], input_payload: bytes = b"") -> None:
        validate_solve_plan(document)
        if not isinstance(input_payload, bytes):
            raise TypeError("solve plan input payload must be bytes")
        object.__setattr__(self, "_document", canonical_json_bytes(document))
        object.__setattr__(self, "input_payload", input_payload)

    def to_document(self) -> dict[str, Any]:
        """Return detached study, input, boundary and output data."""
        return parse_json(self._document)
