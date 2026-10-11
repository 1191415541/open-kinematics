"""Entity queries over immutable native channels; no case or assembly dispatch."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..characteristics import (
    camber_angle,
    frame_position,
    legacy_roll_center_envelope_values,
    legacy_roll_center_primitives,
    toe_angle,
)
from ..modeling.primitives.spatial import quaternion_to_matrix
from ..modeling.resolved import ResolvedModel
from .element_wrench import _declared_element_names, decode_element_wrench
from .raw import RawContractResult, _readonly, _readonly_array


@dataclass(frozen=True)
class WrenchChannel:
    """World wrench about the receiving body's COM, with its action point."""

    entity_id: str
    body_id: str
    force: np.ndarray
    moment: np.ndarray
    point: np.ndarray
    power: np.ndarray
    frame_id: str = "world"
    moment_reference: str = "body_com"
    units: tuple[str, str, str, str] = ("N", "Nm", "m", "W")


@dataclass(frozen=True)
class Measurement:
    name: str
    values: np.ndarray
    units: str
    frame_id: str


@dataclass(frozen=True)
class ContactEvent:
    time_s: float
    element_id: str
    transition: int


@dataclass(frozen=True)
class ResultEnvelope:
    """One result surface identified by stable IDs and channel versions."""

    raw: RawContractResult
    model: ResolvedModel
    schema_version: int = 1

    def __post_init__(self) -> None:
        if self.raw.document.get("contract_version") not in (1, 2):
            raise ValueError("unsupported native result contract version")
        if "constraint_reaction" in self.raw.blocks:
            if self.raw.metadata.get("constraint_channel_version") != 1:
                raise ValueError("unsupported constraint channel version")
            count = sum(row.get("reaction_block", "constraint_reaction") == "constraint_reaction"
                for row in self.raw.metadata.get("constraints", ()))
            if self.raw.block("constraint_reaction").shape != (len(self.raw.states), count, 2, 10):
                raise ValueError("constraint channel shape disagrees with its identity manifest")
        if "coupler_reaction" in self.raw.blocks:
            count = sum(row.get("reaction_block") == "coupler_reaction"
                for row in self.raw.metadata.get("constraints", ()))
            if self.raw.block("coupler_reaction").shape != (len(self.raw.states), count, 4, 10):
                raise ValueError("coupler channel shape disagrees with its identity manifest")

    @property
    def status(self) -> str:
        return self.raw.status

    @property
    def times_s(self) -> np.ndarray:
        return self.raw.times_s

    @property
    def body_ids(self) -> tuple[str, ...]:
        return self.raw.body_names

    @property
    def tire_ids(self) -> tuple[str, ...]:
        return self.raw.tire_names

    @property
    def constraint_ids(self) -> tuple[str, ...]:
        return tuple(row["id"] for row in self.raw.metadata.get("constraints", ()))

    @property
    def diagnostics(self) -> np.ndarray | None:
        return self.raw.diagnostics

    @property
    def performance(self) -> Any:
        return self.raw.performance

    @property
    def failure_evidence(self) -> Mapping[str, Any]:
        return self.raw.failure_evidence

    @property
    def partial_evidence(self) -> Mapping[str, Any]:
        return self.raw.partial_evidence

    @property
    def named_blocks(self) -> Mapping[str, np.ndarray]:
        return self.raw.blocks

    @property
    def energy(self) -> np.ndarray:
        return self.raw.block("energy")

    @property
    def cases(self) -> tuple[Mapping[str, Any], ...]:
        return self.raw.cases

    @property
    def contact_events(self) -> tuple[ContactEvent, ...]:
        return tuple(ContactEvent(float(row[0]), self.raw.tire_names[int(row[1])], int(row[2]))
            for row in self.raw.blocks.get("contact_events", ()))

    def case_samples(self, case_index: int) -> slice:
        row = self.cases[case_index]
        start = int(row["sample_offset"])
        return slice(start, start+int(row["sample_count"]))

    def case_body_state(self, case_index: int = 0, sample_index: int = -1) -> np.ndarray:
        return _readonly_array(self.raw.case_body_state(case_index, sample_index))

    def case_residuals(self, case_index: int = 0) -> tuple[float, float, float]:
        return self.raw.case_residuals(case_index)

    @property
    def model_fingerprint(self) -> str:
        return self.model.fingerprint

    def body_state(self, body_id: str) -> np.ndarray:
        if body_id not in self.body_ids:
            raise KeyError(f"unknown body {body_id!r}")
        return self.raw.body_state(body_id)

    def frame_pose(self, frame_id: str) -> np.ndarray:
        """Body-local SE3 composed with sampled world poses, in metres."""
        count = len(self.raw.states)
        result = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
        if frame_id == "world":
            return _readonly_array(result)
        frames = {row["name"]: row for row in self.model.to_document()["frames"]}
        if frame_id not in frames:
            raise KeyError(f"unknown frame {frame_id!r}")
        frame = frames[frame_id]
        if frame["body"] == "ground":
            result[:, :3, :3] = quaternion_to_matrix(np.asarray(frame["quaternion"]))
            result[:, :3, 3] = frame["point"]
        else:
            states = self.body_state(frame["body"])
            for sample, state in enumerate(states):
                rotation = quaternion_to_matrix(state[3:7])
                result[sample, :3, :3] = rotation @ quaternion_to_matrix(np.asarray(frame["quaternion"]))
                result[sample, :3, 3] = state[:3]+rotation @ np.asarray(frame["point"])
        return _readonly_array(result)

    def constraint_multiplier(self, constraint_id: str) -> Measurement:
        if constraint_id not in self.constraint_ids:
            raise KeyError(f"unknown constraint {constraint_id!r}")
        index = self.constraint_ids.index(constraint_id)
        row = self.raw.metadata["constraints"][index]
        start, count = row["row_offset"], row["row_count"]
        values = self.raw.block("constraint_multiplier")[:, start:start+count]
        return Measurement(constraint_id, values, ",".join(row["multiplier_units"]), "world")

    def constraint_wrench(self, constraint_id: str, *, end: str) -> WrenchChannel:
        if constraint_id not in self.constraint_ids:
            raise KeyError(f"unknown constraint {constraint_id!r}")
        index = self.constraint_ids.index(constraint_id)
        channel = self.raw.metadata["constraints"][index]
        ends = channel.get("ends", ("a", "b"))
        if end not in ends:
            raise ValueError(f"constraint end must be one of {tuple(ends)}")
        end_index = list(ends).index(end)
        row = self.raw.block(channel.get("reaction_block", "constraint_reaction"))[:, channel.get("reaction_index", index), end_index]
        body = channel["bodies"][end_index] if "bodies" in channel else channel[f"body_{end}"]
        return WrenchChannel(constraint_id, body, row[:, :3], row[:, 3:6], row[:, 6:9], row[:, 9])

    def tire_state(self, tire_id: str) -> np.ndarray:
        return self.raw.tire_state(tire_id)

    def element_state(self, element_id: str) -> np.ndarray:
        """Read a native law's deformation, force and rate ledger by its ID."""
        if element_id in self.raw.tire_names:
            return self.tire_state(element_id)
        document = self.raw.model_document or {}
        elements = {row["name"]: row for row in document.get("elements", ())}
        if element_id not in elements:
            raise KeyError(f"unknown element {element_id!r}")
        kind = elements[element_id]["type"]
        blocks = {"spring": "spring_output", "damper": "damper_output", "bump_stop": "bump_stop_output",
            "bushing": "bushing_output", "anti_roll_bar": "anti_roll_output", "steering_actuator": "steering_output"}
        if kind not in blocks:
            raise KeyError(f"element {element_id!r} has no separate state ledger")
        names = tuple(row["name"] for row in document["elements"] if row["type"] == kind)
        return self.raw.block(blocks[kind])[:, names.index(element_id)]

    def law_state(self, element_id: str) -> Mapping[str, np.ndarray]:
        """Read the tire's committed brush state recorded by native."""
        values = self.tire_state(element_id)
        columns = {"brush_longitudinal_m": 10, "brush_lateral_m": 11,
            "contact_body_longitudinal_m": 15, "contact_body_longitudinal_rate_m_per_s": 16,
            "contact_body_lateral_m": 17, "contact_body_lateral_rate_m_per_s": 18,
            "contact_body_yaw_rad": 19, "contact_body_yaw_rate_rad_per_s": 20,
            "turn_slip_phi_c_rad_per_m": 21, "turn_slip_phi_f2_rad_per_m": 22,
            "turn_slip_phi_1_rad_per_m": 23, "turn_slip_phi_2_rad_per_m": 24}
        return _readonly({name: values[:, column] for name, column in columns.items()})

    def element_wrench(self, element_id: str, *, body_id: str) -> WrenchChannel:
        declared = _declared_element_names(self.raw)
        # Actuators and ordinary anti-roll elements also have explicit names.
        elements = (self.raw.model_document or {}).get("elements", ())
        declared = dict(declared)
        for code, kind in ((3, "anti_roll_bar"), (4, "steering_actuator")):
            declared[code] = tuple(row["name"] for row in elements if row["type"] == kind)
        targets = {(code, index) for code, names in declared.items()
            for index, name in enumerate(names) if name == element_id}
        if not targets:
            raise KeyError(f"unknown element {element_id!r}")
        body = self.body_ids.index(body_id)
        count = len(self.raw.states)
        force, moment, point = (np.full((count, 3), np.nan) for _ in range(3))
        for record in decode_element_wrench(self.raw):
            if (record.type_code, record.element_index) in targets and record.body == body:
                force[record.sample] = np.nan_to_num(record.force)
                moment[record.sample] = np.nan_to_num(record.moment)
                point[record.sample] = record.point
        state = self.body_state(body_id)
        power = np.einsum("ij,ij->i", force, state[:, 7:10])+np.einsum("ij,ij->i", moment, state[:, 10:13])
        return WrenchChannel(element_id, body_id, *(_readonly_array(value) for value in (force, moment, point, power)))

    def measure(self, name: str) -> Measurement:
        declarations = {row["name"]: row for row in self.model.to_document().get("measurements", ())}
        if name not in declarations:
            raise KeyError(f"unknown measurement {name!r}")
        row = declarations[name]
        kind = row["type"]
        reference = row.get("reference", "world")
        if kind == "roll_center":
            values, units = self._roll_center(row), "m"
        elif kind == "tire_load":
            values, units = self.tire_state(row["element"])[:, 4], "N"
        elif kind in ("toe", "camber", "frame_position"):
            pose, basis = self.frame_pose(row["frame"]), self.frame_pose(reference)
            # R1: the formulas live in `characteristics.measurements`; this method
            # keeps the query, the declaration lookup and the unit check.
            if kind == "frame_position":
                values = frame_position(pose, basis)
                units = "m"
            else:
                axis = np.asarray(row["axis"], dtype=float)
                sign = row.get("sign", 1)
                values = (
                    toe_angle(pose, basis, axis, sign=sign)
                    if kind == "toe"
                    else camber_angle(pose, basis, axis, sign=sign)
                )
                units = "rad"
        else:
            raise ValueError(f"unsupported measurement primitive {kind!r}")
        if row["units"] != units:
            raise ValueError(f"measurement {name!r} requires units {units!r}")
        return Measurement(name, _readonly_array(values), units, reference)

    def characteristic(self, name: str) -> Any:
        """
        Answer one declared measurement as a characteristic request.

        This is the query the design's §4 describes: it reads channels this
        result already holds, attaches a sample identity to every value, and
        refuses to solve.  Delegated to ``results.characteristics`` so the
        envelope keeps its existing ``measure`` contract untouched.
        """
        from .characteristics import measurement_query

        return measurement_query(self, name)

    def _roll_center(self, declaration: Mapping[str, Any]) -> np.ndarray:
        # The aggregation *is* the legacy definition, so it lives beside that
        # definition rather than here; this method keeps only the duty of handing
        # the primitive readings over in the envelope's own shape.
        return legacy_roll_center_envelope_values(self._roll_center_primitives(declaration))

    def _roll_center_primitives(self, declaration: Mapping[str, Any]) -> np.ndarray:
        """
        Extract the legacy per-sample readings from this envelope's own data.

        The assembly itself lives in ``characteristics.centers``; this method only
        gathers what the envelope already holds -- the native constraint rows, the
        body states and the declared frame positions -- and hands them over as
        plain arrays.  That keeps the formula in one place while leaving the
        envelope in charge of reading its own result, and it is why
        ``characteristics`` needs no import of the results layer.
        """
        jacobians = np.asarray(self.raw.block("constraint_jacobian"), dtype=float)
        free = tuple(self.raw.metadata["free_bodies"])
        frames = {row["name"]: row for row in self.model.to_document()["frames"]}
        reference = self.frame_pose(declaration.get("reference", "world"))
        selected = {row["id"]: row for row in self.raw.metadata["constraints"]}
        indices = [
            index
            for name in declaration["constraints"]
            for index in range(
                selected[name]["row_offset"],
                selected[name]["row_offset"] + selected[name]["row_count"],
            )
        ]
        contacts = declaration["contact_frames"]
        drives = declaration["drive_frames"]
        bodies = []
        for contact, drive in zip(contacts, drives):
            body = frames[drive]["body"]
            if frames[contact]["body"] != body or body not in free:
                raise ValueError(
                    "roll-center contact and drive must belong to the same free body"
                )
            bodies.append(body)
        samples = jacobians.shape[0]
        states = np.stack([np.asarray(self.body_state(body)[:samples]) for body in bodies], axis=1)
        contact_points = np.stack(
            [np.asarray(self.frame_pose(name)[:samples, :3, 3]) for name in contacts], axis=1
        )
        drive_points = np.stack(
            [np.asarray(self.frame_pose(name)[:samples, :3, 3]) for name in drives], axis=1
        )
        return legacy_roll_center_primitives(
            jacobians=jacobians,
            selected_rows=np.asarray(indices, dtype=int),
            body_offsets=np.asarray([6 * free.index(body) for body in bodies], dtype=int),
            reference_rotations=np.asarray(reference[:samples, :3, :3], dtype=float),
            reference_origins=np.asarray(reference[:samples, :3, 3], dtype=float),
            body_positions=states[:, :, :3],
            drive_points=drive_points,
            contact_points=contact_points,
            reference_body_offset=self._held_reference_offset(declaration, frames, free),
        )

    def _held_reference_offset(
        self,
        declaration: Mapping[str, Any],
        frames: Mapping[str, Any],
        free: tuple[str, ...],
    ) -> int | None:
        """
        Return the free-body offset whose six coordinates the legacy drive held.

        A declared reference that is itself a free body was pinned by the legacy
        assembly; the world reference pins nothing, which is why this returns
        ``None`` rather than a zero offset.
        """
        reference_id = declaration.get("reference", "world")
        if reference_id == "world":
            return None
        body = frames[reference_id]["body"]
        if body not in free:
            return None
        return 6 * free.index(body)
