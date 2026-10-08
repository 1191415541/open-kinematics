"""Export frozen K/C report fields from explicit result frame IDs."""

from __future__ import annotations

import numpy as np

from ..modeling.primitives.spatial import quaternion_to_matrix
from ..results.envelope import ResultEnvelope
from .geometry import _wheel_geometry


def frame_fields(pose: np.ndarray, *, side: str) -> dict[str, float]:
    """Project a wheel frame into the frozen report's millimetres and degrees."""
    geometry = _wheel_geometry(pose[:3, 3]*1000, pose[:3, :3], np.zeros(3), side=side)
    label = "left" if side == "L" else "right"
    return {f"{label}_wheel_center_{axis}_mm": float(value) for axis, value in zip("xyz", geometry.center)} | {
        f"{label}_camber_deg": geometry.camber_deg, f"{label}_toe_deg": geometry.toe_deg}


def initial_frame_pose(result: ResultEnvelope, frame_id: str) -> np.ndarray:
    """Compose the declared neutral body and marker poses."""
    graph = result.model.to_document()
    frame = next(row for row in graph["frames"] if row["name"] == frame_id)
    body = next(row for row in graph["bodies"] if row["name"] == frame["body"])
    rotation = quaternion_to_matrix(np.asarray(body["quaternion"]))
    pose = np.eye(4)
    pose[:3, :3] = rotation@quaternion_to_matrix(np.asarray(frame["quaternion"]))
    pose[:3, 3] = np.asarray(body["position"])+rotation@np.asarray(frame["point"])
    return pose


def k_records(result: ResultEnvelope, *, frames: dict[str, str], wheel_values: tuple[float, ...], rack_values: tuple[float, ...]) -> list[dict[str, object]]:
    """Report a declared wheel/rack sweep by stable frame IDs."""
    poses = {side: result.frame_pose(frame) for side, frame in frames.items()}
    records = []
    for index, case in enumerate(result.cases):
        wheel, rack = wheel_values[index//len(rack_values)], rack_values[index % len(rack_values)]
        identity = f"k-w{wheel:+.0f}-r{rack:+.0f}"
        if case["name"] != identity:
            raise ValueError("native K grid order differs from the declared input")
        last = int(case["sample_offset"])+int(case["sample_count"])-1
        record = {"case_id": identity, "wheel_travel_mm": wheel, "rack_displacement_mm": rack}
        for side, values in poses.items():
            record.update(frame_fields(values[last], side=side))
        records.append(record)
    return records


def c_records(result: ResultEnvelope, *, frames: dict[str, str], paths: tuple[str, ...], levels: int, maximum: float) -> list[dict[str, object]]:
    """Report load-path deformation against the declared neutral frame."""
    from scipy.spatial.transform import Rotation

    poses = {side: result.frame_pose(frame) for side, frame in frames.items()}
    reference = {side: initial_frame_pose(result, frame) for side, frame in frames.items()}
    neutral = {side: frame_fields(pose, side=side) for side, pose in reference.items()}
    records = []
    for index, case in enumerate(result.cases):
        axis, step = paths[index//levels], index % levels
        level = maximum if step == levels-1 else -maximum+step*(2*maximum/(levels-1))
        identity = f"c-{axis}-{level:+.2f}"
        if case["name"] != identity:
            raise ValueError("native C grid order differs from the declared input")
        load = [0.]*6
        load[("fx", "fy", "fz", "mx", "my", "mz").index(axis)] = level
        record = {"case_id": identity, "path": axis, "level": level, "side_mode": "single", "load_left": load, "load_right": [0.]*6}
        last = int(case["sample_offset"])+int(case["sample_count"])-1
        metrics, differences = {}, {}
        for side, values in poses.items():
            label = "left" if side == "L" else "right"
            pose, origin = values[last], reference[side]
            metrics[label] = frame_fields(pose, side=side)
            rotation = origin[:3, :3]@Rotation.from_matrix(origin[:3, :3].T@pose[:3, :3]).as_rotvec()
            record["deformation_"+label] = np.concatenate(((pose[:3, 3]-origin[:3, 3])*1000, rotation)).tolist()
            differences.update({key: value-neutral[side][key] for key, value in metrics[label].items()})
        record.update(metrics=metrics, c_minus_k=differences)
        records.append(record)
    return records
