"""比较 Adams 与 Native PAC2002 接触斑坐标和世界力分量."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from suspension_multibody.adams.time_domain import (
    AdamsResultChannel,
    parse_adams_result_history,
)
from suspension_multibody.api import validate
from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
from suspension_multibody.schema import VehicleDynamicCase
from suspension_multibody.schema.vehicle import VehicleDeclaration

WHEELS = (
    ("front_left", "til", "front", "front_spindle_L"),
    ("front_right", "tir", "front", "front_spindle_R"),
    ("rear_left", "til", "rear", "rear_spindle_L"),
    ("rear_right", "tir", "rear", "rear_spindle_R"),
)


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON 根节点必须是对象: {path}")
    return payload


def _rotation(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
    )


def _channels() -> dict[str, AdamsResultChannel]:
    result: dict[str, AdamsResultChannel] = {}
    for wheel, prefix, axle, _ in WHEELS:
        entity = f"{prefix}_wheel_contact_patch"
        for axis in ("x", "y", "z"):
            result[f"{wheel}.position_{axis}"] = AdamsResultChannel(
                entity, f"{axis}_{axle}"
            )
        for component in ("lon", "lat", "ver"):
            for axis in ("x", "y", "z"):
                result[f"{wheel}.{component}_{axis}"] = AdamsResultChannel(
                    entity, f"{component}_{axis}_{axle}"
                )
    return result


def diagnose(adams_result: Path, native_artifact: Path) -> dict[str, Any]:
    """对比 Adams 与 native 的 PAC2002 接触坐标系."""
    manifest = _read_json(native_artifact / "manifest.json")
    model = VehicleDeclaration.model_validate(manifest["model"])
    case = VehicleDynamicCase.model_validate(manifest["case"])
    compiled = validate(*migrate_v1_vehicle_case(case.model_copy(update={"vehicle": model})))
    native_tires = {row["name"].rsplit(".", 1)[-1]: row for row in compiled.model_document["tires"]}
    arrays = np.load(native_artifact / str(manifest["arrays_file"]))
    body_names = [str(value) for value in arrays["body_names"]]
    tire_names = [str(value) for value in arrays["tire_names"]]
    states = np.asarray(arrays["states"], dtype=float)
    tire_output = np.asarray(arrays["tire_output"], dtype=float)
    history = parse_adams_result_history(adams_result, _channels())

    result: dict[str, Any] = {"time_s": float(arrays["times_s"][0]), "wheels": {}}
    for wheel, _, _, _ in WHEELS:
        tire = native_tires[wheel]
        parameters = tire["parameters"]
        frame_body = parameters["frame_body"]
        def body_index_of(name):
            return body_names.index(name) if name in body_names else body_names.index(name.split(".json.")[-1].removeprefix("body."))
        body_index = body_index_of(frame_body)
        tire_index = tire_names.index(tire["name"] if tire["name"] in tire_names else wheel)
        forward = _rotation(states[0, body_index, 3:7]) @ np.asarray(
            parameters["forward_axis"], dtype=float
        )
        forward[2] = 0.0
        forward /= np.linalg.norm(forward)
        lateral = np.cross(np.array((0.0, 0.0, 1.0)), forward)
        native_vectors = np.asarray(
            (
                forward * tire_output[0, tire_index, 5],
                lateral * tire_output[0, tire_index, 6],
                (0.0, 0.0, tire_output[0, tire_index, 4]),
            )
        )
        adams_vectors = np.asarray(
            [
                [
                    history.channels[f"{wheel}.{component}_{axis}"][0]
                    for axis in ("x", "y", "z")
                ]
                for component in ("lon", "lat", "ver")
            ],
            dtype=float,
        )
        adams_position = np.asarray(
            [history.channels[f"{wheel}.position_{axis}"][0] for axis in ("x", "y", "z")],
            dtype=float,
        ) * 1.0e-3
        force_body_index = body_index_of(tire["body"])
        force_body_rotation = _rotation(states[0, force_body_index, 3:7])
        center = states[0, force_body_index, :3] + force_body_rotation @ np.asarray(
            tire["center"], dtype=float
        )
        loaded_radius = parameters["unloaded_radius"] - tire_output[0, tire_index, 2]
        native_position = center + np.array((0.0, 0.0, -loaded_radius))
        spin = _rotation(states[0, body_index, 3:7]) @ np.asarray(
            parameters["spin_axis"], dtype=float
        )
        spin /= np.linalg.norm(spin)
        radial_down = -(np.array((0.0, 0.0, 1.0)) - spin * spin[2])
        radial_down /= np.linalg.norm(radial_down)
        inclined_position = center + radial_down * loaded_radius
        result["wheels"][wheel] = {
            "native_forward_world": forward.tolist(),
            "adams_contact_position_m": adams_position.tolist(),
            "native_contact_position_m": native_position.tolist(),
            "contact_position_error_m": (native_position - adams_position).tolist(),
            "inclined_contact_position_m": inclined_position.tolist(),
            "inclined_contact_position_error_m": (
                inclined_position - adams_position
            ).tolist(),
            "adams_force_vectors_n": adams_vectors.tolist(),
            "native_force_vectors_n": native_vectors.tolist(),
            "force_vector_error_n": (native_vectors - adams_vectors).tolist(),
            "net_force_error_n": np.sum(native_vectors - adams_vectors, axis=0).tolist(),
        }
    return result


def main() -> int:
    """运行 PAC2002 接触坐标系诊断."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adams-result", type=Path, required=True)
    parser.add_argument("--native-artifact", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = diagnose(args.adams_result.resolve(), args.native_artifact.resolve())
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
