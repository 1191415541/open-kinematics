"""Compare document submissions with pinned evidence, without a legacy producer."""

from __future__ import annotations

import hashlib
import json
from copy import deepcopy
from pathlib import Path

import numpy as np
from suspension_contracts import unpack_container

from suspension_multibody.authoring import DocumentLoader, migrate_v1_vehicle_case
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.primitives import quaternion_to_matrix
from suspension_multibody.modeling.primitives.elements import (
    BushingElement,
    _curve_integral,
    _curve_value,
)
from suspension_multibody.modeling.primitives.spatial import (
    quaternion_conjugate,
    quaternion_multiply,
)
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan

PHYSICAL_CASES = frozenset({"steering", "nondefault road and initial state", "bushing force curves"})
_DIAGNOSTIC_FIELDS = (
    "accepted", "internal_steps", "rejected_attempts", "newton_iterations",
    "minimum_accepted_step_s", "maximum_accepted_step_s", "last_accepted_step_s",
    "position_residual", "velocity_residual", "dynamics_residual", "active_contacts",
    "contact_events", "local_error_ratio", "energy_residual", "failure_code",
    "pinned_null_directions",
)
_DIAGNOSTIC_INTEGERS = frozenset({
    "internal_steps", "rejected_attempts", "newton_iterations", "active_contacts",
    "contact_events", "failure_code", "pinned_null_directions",
})


def digest(value: bytes) -> str:
    """Identify immutable input and evidence bytes."""
    return hashlib.sha256(value).hexdigest()


def _array_digest(array: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(array, dtype=np.float64).tobytes()).hexdigest()


def _diagnostic_digest(array: np.ndarray) -> str:
    digest_value = hashlib.sha256()
    for column, field in enumerate(_DIAGNOSTIC_FIELDS):
        values = array[:, column]
        if field == "accepted":
            values = values.astype(bool)
        elif field in _DIAGNOSTIC_INTEGERS:
            values = values.astype(int)
        digest_value.update(field.encode("utf-8"))
        digest_value.update(str(values.dtype).encode("ascii"))
        digest_value.update(repr(tuple(int(extent) for extent in values.shape)).encode("ascii"))
        digest_value.update(values.tobytes())
    return digest_value.hexdigest()


def _verify_reference_arrays(arrays, expected, name: str) -> None:
    for field, wanted in expected.items():
        if field == "diagnostics":
            actual = _diagnostic_digest(np.asarray(arrays["diagnostics"]))
        else:
            if field not in arrays:
                raise ValueError(f"vehicle reference {name!r} is missing channel {field!r}")
            actual = _array_digest(np.asarray(arrays[field]))
        if actual != wanted:
            raise ValueError(f"vehicle reference {name!r} channel {field!r} differs from frozen hash")


def reference_case(directory: Path, name: str, baseline: Path):
    """Load pinned inputs and verify outputs against every original digest."""
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest["schema_version"] != 1 or manifest["baseline_sha256"] != digest(baseline.read_bytes()):
        raise ValueError("vehicle reference does not identify the original frozen baseline")
    entry = manifest["cases"][name]
    prefix = entry["prefix"]
    required = {prefix+suffix for suffix in (".model.json", ".case.json", ".model.bin", ".case.bin", ".npz")}
    if set(entry["files"]) != required:
        raise ValueError("vehicle reference file manifest is incomplete")
    for filename, expected in entry["files"].items():
        if digest((directory / filename).read_bytes()) != expected:
            raise ValueError(f"vehicle reference file differs: {filename}")
    model = json.loads((directory / (prefix+".model.json")).read_text(encoding="utf-8"))
    case = json.loads((directory / (prefix+".case.json")).read_text(encoding="utf-8"))
    with np.load(directory / (prefix+".npz"), allow_pickle=False) as stored:
        arrays = {key: stored[key] for key in stored.files}
    expected = json.loads(baseline.read_text(encoding="utf-8"))["cases"][name]
    _verify_reference_arrays(arrays, expected, name)
    return model, case, (directory / (prefix+".model.bin")).read_bytes(), (directory / (prefix+".case.bin")).read_bytes(), arrays, entry


def compile_evidence(case, layout):
    """Compile ordinary documents in the frozen entity order."""
    assembly, plan = migrate_v1_vehicle_case(case)
    loaded = DocumentLoader().load(assembly, plan)
    model = loaded.resolve()
    graph = model.to_document()
    # Entity order affects floating-point elimination. Only permute full rows;
    # retain every declared entity and parameter, including the carrier frame.
    for section, names in (("bodies", layout["bodies"]), ("joints", layout["constraints"])):
        order = {name: index for index, name in enumerate(names)}
        if len(order) != len(graph[section]) or set(order) != {row["name"] for row in graph[section]}:
            raise ValueError(f"frozen {section} identity manifest disagrees with the document")
        graph[section].sort(key=lambda row: order.get(row["name"], len(order)))
    return compile_resolved(ResolvedModel(graph, model.resource_payload), ResolvedSolvePlan(plan.to_payload()))


def _id_maps(old, layout):
    def mapping(rows, names, kind):
        source = [row["name"] for row in rows]
        if len(source) != len(names) or len(set(source)) != len(source) or len(set(names)) != len(names):
            raise ValueError(f"vehicle reference {kind} identity layout is not one-to-one")
        result = dict(zip(source, names))
        if set(result) != set(source) or set(result.values()) != set(names):
            raise ValueError(f"vehicle reference {kind} identity layout is incomplete")
        return result

    maps = {"body": mapping(old["bodies"], layout["bodies"], "body"),
        "joint": mapping(old["joints"], layout["constraints"], "joint"),
        "tire": mapping(old["tires"], layout["ledgers"]["tire"], "tire"),
        "element": {}, "actuator": {}}
    for kind, names in layout["ledgers"].items():
        if kind == "tire":
            continue
        rows = [row for row in old["elements"] if row["type"] == kind]
        maps["element"].update(mapping(rows, names, kind))
        if kind == "steering_actuator":
            maps["actuator"].update((row["target"], name.rsplit(".", 1)[0]+"."+row["target"])
                for row, name in zip(rows, names))
    return maps


def _element_checks(result, compiled, layout, equality, failures):
    bushing_energy = np.zeros(len(result.times_s))

    def point_state(name, point):
        state = result.body_state(name)
        offsets = np.array([quaternion_to_matrix(sample[3:7])@np.asarray(point) for sample in state])
        return state[:, :3]+offsets, state[:, 7:10]+np.cross(state[:, 10:13], offsets)

    for row in compiled.model_document["elements"]:
        parameters = row["parameters"]
        if row["type"] == "steering_actuator":
            if parameters["type"] != "translation":
                failures.append({"check": "steering_law", "reason": "approved cases require translation actuators"})
                continue
            values = result.element_state(row["name"])
            pa, va = point_state(parameters["body"], parameters["point_local"])
            pb, vb = point_state(parameters["reaction_body"], parameters["reaction_point_local"])
            axes = np.array([quaternion_to_matrix(sample[3:7])@np.asarray(parameters["axis_local"])
                             for sample in result.body_state(parameters["reaction_body"])])
            axes /= np.linalg.norm(axes, axis=1)[:, None]
            coordinate, rate = np.sum(axes*(pa-pb), axis=1), np.sum(axes*(va-vb), axis=1)
            inputs = {item["role"]: np.asarray(item["values"]) for item in compiled.metadata["solve_plan"]["inputs"]
                      if item.get("actuator") == row["target"]}
            target, target_rate = inputs["steering_target"], inputs["steering_rate"]
            force = parameters["stiffness"]*(target-coordinate)+parameters["damping"]*(target_rate-rate)
            equality(row["name"]+":coordinate_m", values[:, 0], coordinate, np.sum(np.abs(pa)+np.abs(pb), axis=1))
            equality(row["name"]+":rate_m_per_s", values[:, 1], rate, np.sum(np.abs(va)+np.abs(vb), axis=1))
            equality(row["name"]+":target_m", values[:, 2], target, np.abs(target))
            equality(row["name"]+":force_N", values[:, 3], force,
                     parameters["stiffness"]*(np.abs(target)+np.abs(coordinate))
                     +parameters["damping"]*(np.abs(target_rate)+np.abs(rate)))
        elif row["type"] == "bushing":
            law = BushingElement(row["name"], row["body_a"], row["body_b"],
                stiffness=np.asarray(parameters["stiffness"]), damping=np.asarray(parameters["damping"]),
                preload=np.asarray(parameters["preload"]), force_curves=parameters.get("force_curves", ()),
                force_curve_interpolation=parameters.get("force_curve_interpolation", "piecewise_linear"),
                rotation_coordinates=parameters.get("rotation_coordinates", "rotation_vector"))
            if law.force_curve_interpolation != "piecewise_linear":
                failures.append({"check": "bushing_law", "reason": "approved curve case requires piecewise-linear interpolation"})
                continue
            pa, va = point_state(row["body_a"], parameters["point_a"])
            pb, vb = point_state(row["body_b"], parameters["point_b"])
            actual = result.element_state(row["name"])
            expected_deformation, expected_wrench = [], []
            for index, (a, b) in enumerate(zip(result.body_state(row["body_a"]), result.body_state(row["body_b"]))):
                qa = quaternion_multiply(a[3:7], np.asarray(parameters["frame_a_quaternion"]))
                qb = quaternion_multiply(b[3:7], np.asarray(parameters["frame_b_quaternion"]))
                rotation = quaternion_to_matrix(qa)
                relative = quaternion_multiply(quaternion_conjugate(qa), qb)
                relative = quaternion_multiply(quaternion_conjugate(np.asarray(parameters.get("reference_quaternion", [1, 0, 0, 0]))), relative)
                translation = rotation.T@(pb[index]-pa[index])
                deformation = np.concatenate((translation-np.asarray(parameters.get("reference_translation", [0, 0, 0])),
                                               law.rotational_deformation(relative)))
                rate = np.concatenate((rotation.T@(vb[index]-va[index])-np.cross(rotation.T@a[10:13], translation),
                                       law.rotational_rate(relative, rotation.T@(b[10:13]-a[10:13]))))
                elastic = law.stiffness@deformation
                energy = .5*deformation@elastic
                for axis, curve in enumerate(law.force_curves):
                    if curve:
                        energy += _curve_integral(curve, deformation[axis])-.5*deformation[axis]*elastic[axis]
                        elastic[axis] = _curve_value(curve, deformation[axis])
                bushing_energy[index] += energy-law.preload@deformation
                expected_deformation.append(deformation)
                expected_wrench.append(law.preload-elastic-law.damping@rate)
            deformation, wrench = np.asarray(expected_deformation), np.asarray(expected_wrench)
            equality(row["name"]+":deformation_m_rad", actual[:, :6], deformation, np.abs(deformation)+1)
            equality(row["name"]+":wrench_N_Nm", actual[:, 6:], wrench, np.abs(wrench)+1)
        else:
            failures.append({"check": "force_law_coverage", "element": row["name"], "type": row["type"]})
    equality("bushing_storage_J", result.energy[:, 17], bushing_energy, np.abs(bushing_energy))


def physical_checks(result, compiled, layout) -> dict[str, object]:
    """
    Check invariant physics for an approved contact-frame difference.

    These checks are derived from the native contract and solver tolerances. They
    do not use the old/new result delta as an acceptance threshold.
    """
    failures: list[dict[str, object]] = []
    arrays = document_arrays(result, layout)
    for key, array in arrays.items():
        if not np.isfinite(np.asarray(array, dtype=float)).all():
            failures.append({"check": "finite", "channel": key})
    states = np.asarray(result.raw.states, dtype=float)
    quaternion_norm = np.linalg.norm(states[:, :, 3:7], axis=2)
    quat_error = float(np.max(np.abs(quaternion_norm - 1.0))) if quaternion_norm.size else 0.0
    if quat_error > 256 * np.finfo(float).eps:
        failures.append({"check": "quaternion_norm", "max_error": quat_error,
                         "limit": 256 * np.finfo(float).eps})
    diagnostics = np.asarray(result.raw.block("diagnostics"), dtype=float)[:len(result.times_s)]
    solver = compiled.case_document["solver"]
    residual_limits = (float(solver["position_tolerance_m"]),
                       float(solver["velocity_tolerance_m_per_s"]),
                       float(solver["dynamics_tolerance"]))
    for column, (label, limit) in enumerate(zip(("position", "velocity", "dynamics"), residual_limits), start=7):
        observed = float(np.nanmax(np.abs(diagnostics[:, column])))
        if observed > limit:
            failures.append({"check": f"{label}_residual", "observed": observed, "limit": limit})
    if np.any(diagnostics[:, 0] < 0.5) or np.any(diagnostics[:, 14] != 0):
        failures.append({"check": "solver_acceptance", "failure_code": diagnostics[:, 14].tolist()})
    metrics = {}

    def equality(label, actual, expected, scale, *, operations=1024):
        error = np.abs(np.asarray(actual)-np.asarray(expected))
        bound = operations*np.finfo(float).eps*np.maximum(1.0, scale)
        metrics[label] = {"max_error": float(np.max(error, initial=0.0)),
                          "max_limit": float(np.max(bound)),
                          "max_error_ratio": float(np.max(error/bound, initial=0.0))}
        if np.any(error > bound):
            failures.append({"check": label, **metrics[label]})

    bodies = {row["name"]: row for row in compiled.model_document["bodies"]}
    for name, body in bodies.items():
        initial = np.array([*body["position"], *body["quaternion"],
                            *body.get("velocity", [0, 0, 0]), *body.get("omega", [0, 0, 0])])
        equality(name+":initial_state", result.body_state(name)[0, :13], initial, np.abs(initial))
    kinetic = np.zeros(len(result.times_s))
    kinetic_scale = np.zeros(len(result.times_s))
    for name, body in bodies.items():
        if body.get("fixed", False):
            continue
        mass, inertia = body["mass"], np.asarray(body["inertia"], dtype=float)
        for tire in compiled.model_document.get("tires", ()):
            if tire["body"] == name:
                mass += tire.get("mass", 0.0)
                inertia = inertia+np.asarray(tire.get("inertia", np.zeros((3, 3))))
        state = result.body_state(name)
        translation = .5*mass*np.sum(state[:, 7:10]**2, axis=1)
        angular = []
        for sample in state:
            rotation = quaternion_to_matrix(sample[3:7])
            omega = sample[10:13]
            angular.append(.5*omega@(rotation@inertia@rotation.T)@omega)
        kinetic += translation+np.asarray(angular)
        kinetic_scale += np.abs(translation)+np.abs(angular)
    equality("kinetic_energy_J", result.energy[:, 0], kinetic, kinetic_scale,
             operations=256*len(bodies))
    equality("mechanical_energy_J", result.energy[:, 2], result.energy[:, 0]+result.energy[:, 1],
             np.abs(result.energy[:, 0])+np.abs(result.energy[:, 1]))
    tire_violations = []
    normal_energy, brush_energy = np.zeros(len(result.times_s)), np.zeros(len(result.times_s))
    road = compiled.case_document.get("inputs", {}).get("road", {"kind": "plane", "parameters": {}})
    if road["kind"] != "plane":
        failures.append({"check": "independent_contact_geometry", "reason": "approved cases require a plane road"})
    height = road.get("parameters", {}).get("origin_z", 0.0)
    for row in compiled.model_document.get("tires", ()):
        values = np.asarray(result.tire_state(row["name"]), dtype=float)
        normal_force = values[:, 4]
        penetration = values[:, 2]
        active = values[:, 0]
        scale = max(1.0, float(np.max(np.abs(normal_force), initial=0.0)))
        limit = 256 * np.finfo(float).eps * scale
        if np.any(normal_force < -limit):
            tire_violations.append({"tire": row["name"], "field": "normal_force", "limit": limit})
        flag_limit = 256*np.finfo(float).eps
        if np.any((active < -flag_limit) | (active > 1.0 + flag_limit)):
            tire_violations.append({"tire": row["name"], "field": "active_flag", "limit": flag_limit})
        inactive_force = np.abs(normal_force[active <= 0.5])
        if inactive_force.size and np.max(inactive_force) > limit:
            tire_violations.append({"tire": row["name"], "field": "detached_force", "limit": limit})
        penetration_limit = 256*np.finfo(float).eps*max(1., row["parameters"]["unloaded_radius"])
        if np.any(penetration < -penetration_limit):
            tire_violations.append({"tire": row["name"], "field": "penetration", "limit_m": penetration_limit})
        parameters = row["parameters"]
        if row["model"] != "native_brush" or "deflection_curve" in parameters:
            failures.append({"check": "independent_vertical_law", "tire": row["name"],
                             "reason": "approved cases require the linear brush vertical law"})
            continue
        owner = parameters.get("frame_body", row["body"])
        point = np.asarray(parameters.get("frame_center_local", parameters.get("center_local", [0, 0, 0])))
        center, center_velocity = [], []
        for sample in result.body_state(owner):
            offset = quaternion_to_matrix(sample[3:7])@point
            center.append(sample[:3]+offset)
            center_velocity.append(sample[7:10]+np.cross(sample[10:13], offset))
        center, center_velocity = np.asarray(center), np.asarray(center_velocity)
        delta = parameters["unloaded_radius"]+height-center[:, 2]
        delta_rate = -center_velocity[:, 2]
        stiffness, damping = parameters["vertical_stiffness"], parameters["vertical_damping"]
        expected_force = np.where(delta > 0, np.maximum(0, stiffness*delta+damping*delta_rate), 0)
        equality(row["name"]+":penetration_m", penetration, np.maximum(0, delta),
                 np.abs(center[:, 2])+abs(height)+parameters["unloaded_radius"])
        equality(row["name"]+":normal_force_N", normal_force, expected_force,
                 abs(stiffness)*(np.abs(center[:, 2])+abs(height)+parameters["unloaded_radius"])
                 +abs(damping)*np.abs(delta_rate))
        equality(row["name"]+":normal_velocity_m_per_s", values[:, 3], -delta_rate, np.abs(delta_rate))
        equality(row["name"]+":gap_m", values[:, 1], -delta, np.abs(center[:, 2])+abs(height)+parameters["unloaded_radius"])
        active_expected = (expected_force > 0).astype(float)
        equality(row["name"]+":active", active, active_expected, 1)
        kx, ky = parameters["longitudinal_brush_stiffness"], parameters["lateral_brush_stiffness"]
        mux, muy = parameters["longitudinal_friction_coefficient"], parameters["lateral_friction_coefficient"]
        fx, fy = -kx*values[:, 10], -ky*values[:, 11]
        utilization = np.hypot(np.divide(fx, mux*expected_force, out=np.zeros_like(fx), where=expected_force > 0),
                               np.divide(fy, muy*expected_force, out=np.zeros_like(fy), where=expected_force > 0))
        projection = np.maximum(1, utilization)
        fx, fy = np.where(expected_force > 0, fx/projection, 0), np.where(expected_force > 0, fy/projection, 0)
        equality(row["name"]+":longitudinal_force_N", values[:, 5], fx, np.abs(fx))
        equality(row["name"]+":lateral_force_N", values[:, 6], fy, np.abs(fy))
        equality(row["name"]+":friction_utilization", values[:, 9], np.minimum(1, utilization), utilization)
        equality(row["name"]+":brush_only_moments_and_transients", values[:, 12:], 0, 1)
        forward, lateral = [], []
        for sample in result.body_state(owner):
            axis = quaternion_to_matrix(sample[3:7])@np.asarray(parameters["forward_axis_local"])
            axis[2] = 0
            axis /= np.linalg.norm(axis)
            forward.append(axis)
            lateral.append(np.cross([0, 0, 1], axis))
        velocity = center_velocity+np.cross(result.body_state(row["body"])[:, 10:13],
                                            [0, 0, -parameters["unloaded_radius"]])
        equality(row["name"]+":longitudinal_slip_velocity_m_per_s", values[:, 7],
                 np.sum(velocity*np.asarray(forward), axis=1), np.sum(np.abs(velocity), axis=1))
        equality(row["name"]+":lateral_slip_velocity_m_per_s", values[:, 8],
                 np.sum(velocity*np.asarray(lateral), axis=1), np.sum(np.abs(velocity), axis=1))
        normal_energy += np.where(delta > 0, .5*stiffness*delta**2, 0)
        brush_energy += np.where(expected_force > 0,
            .5*kx*(values[:, 10]/projection)**2+.5*ky*(values[:, 11]/projection)**2, 0)
    failures.extend({"check": "tire_contact", **item} for item in tire_violations)
    equality("tire_normal_energy_J", result.energy[:, 19], normal_energy, np.abs(normal_energy))
    equality("tire_brush_energy_J", result.energy[:, 20], brush_energy, np.abs(brush_energy))
    _element_checks(result, compiled, layout, equality, failures)
    balance_errors, balances = [], []
    if set(result.constraint_ids) != set(layout["constraints"]):
        failures.append({"check": "constraint_coverage", "expected_count": len(layout["constraints"]),
                         "actual_count": len(result.constraint_ids)})
    for constraint_id in result.constraint_ids:
        a = result.constraint_wrench(constraint_id, end="a")
        b = result.constraint_wrench(constraint_id, end="b")
        force_scale = max(1.0, float(np.max(np.abs(np.concatenate((a.force, b.force))))))
        # Reaction moments are about each receiving body's COM, not the marker.
        ra = np.zeros_like(a.point) if a.body_id == "ground" else result.body_state(a.body_id)[:, :3]
        rb = np.zeros_like(b.point) if b.body_id == "ground" else result.body_state(b.body_id)[:, :3]
        arms = np.cross(ra, a.force)+np.cross(rb, b.force)
        moment_scale = max(1.0, float(np.max(np.abs(a.moment)+np.abs(b.moment)
                                            +np.abs(np.cross(ra, a.force))+np.abs(np.cross(rb, b.force)))))
        force_error = float(np.max(np.abs(a.force + b.force)))
        moment_error = float(np.max(np.abs(a.moment+b.moment+arms)))
        force_limit = 512 * np.finfo(float).eps * force_scale
        moment_limit = 1024 * np.finfo(float).eps * moment_scale
        balances.append({"constraint": constraint_id, "force_error_N": force_error,
                         "force_limit_N": force_limit, "moment_error_Nm": moment_error,
                         "moment_limit_Nm": moment_limit})
        if force_error > force_limit or moment_error > moment_limit:
            balance_errors.append({"constraint": constraint_id, "force_error": force_error,
                                   "force_limit": force_limit, "moment_error": moment_error,
                                   "moment_limit": moment_limit})
    failures.extend({"check": "constraint_balance", **item} for item in balance_errors)
    energy = result.energy
    equality("potential_storage_J", energy[:, 1], np.sum(energy[:, 14:21], axis=1), np.sum(np.abs(energy[:, 14:21]), axis=1))
    equality("interval_work_J", energy[:, 11], np.sum(energy[:, 4:7], axis=1), np.sum(np.abs(energy[:, 4:7]), axis=1))
    equality("interval_dissipation_J", energy[:, 12], np.sum(energy[:, 7:10], axis=1), np.sum(np.abs(energy[:, 7:10]), axis=1))
    residual = np.concatenate(([0.], np.diff(energy[:, 2])-energy[1:, 11]+energy[1:, 12]))
    equality("interval_energy_residual_J", energy[:, 3], residual,
             np.abs(energy[:, 2])+np.abs(energy[:, 11])+np.abs(energy[:, 12]))
    equality("diagnostic_energy_residual_J", diagnostics[:, 13], energy[:, 3], np.abs(energy[:, 3]))
    return {"passed": not failures, "failures": failures, "equalities": metrics, "constraint_balances": balances,
            "limits": {"residuals": residual_limits, "quaternion": 256 * np.finfo(float).eps}}


def _remap(value, maps, *, key=""):
    if isinstance(value, dict):
        return {field: _remap(item, maps, key=field) for field, item in value.items()}
    if isinstance(value, list):
        return [_remap(item, maps, key=key) for item in value]
    if not isinstance(value, str):
        return value
    kind = "body" if key in {"body", "body_a", "body_b", "frame_body", "reaction_body",
                              "drive_torque_body", "drive_torque_reaction_body"} else key
    if key == "target":
        kind = "actuator"
    return maps.get(kind, {}).get(value, value)


def _tables(document, blob, maps):
    tables = []
    for row in document.get("blobs", ()):
        array = np.frombuffer(blob[row["offset"]:row["offset"]+row["length"]], dtype="<f8").reshape(row["shape"])
        target = _remap({key: row[key] for key in ("role", "body", "tire", "coordinate", "actuator", "quantity") if key in row}, maps)
        tables.append({**target, "shape": list(array.shape), "values": array.tolist()})
    return sorted(tables, key=lambda row: json.dumps({key: value for key, value in row.items() if key != "values"}, sort_keys=True))


def _model(document, blob, maps):
    source = deepcopy(document)
    resources = {row["name"]: np.frombuffer(blob[row["offset"]:row["offset"]+row["length"]], dtype="<f8").reshape(row["shape"]).tolist()
        for row in source.get("blobs", ())}
    for section, kind in (("bodies", "body"), ("joints", "joint"), ("elements", "element"), ("tires", "tire")):
        for row in source[section]:
            row["name"] = maps.get(kind, {}).get(row["name"], row["name"])
            if section == "joints":
                row.setdefault("axis_a", [0, 0, 1])
                row.setdefault("axis_b", [0, 0, 1])
            if section == "tires":
                row.setdefault("mass", 0)
                row.setdefault("inertia", [[0]*3 for _ in range(3)])
            params = row.get("parameters", {})
            if "blob" in params:
                values = resources.pop(params.pop("blob"))
                # ContractModel reads the coefficient vector only for PAC/Fiala.
                if row.get("model") != "native_brush":
                    params["resource_values"] = values
            for field in ("deflection_curve", "bottoming_curve"):
                if field in params:
                    params[field] = resources.pop(params[field])
            if row.get("type") == "bushing":
                params.setdefault("reference_translation", [0, 0, 0])
                params.setdefault("reference_quaternion", [1, 0, 0, 0])
    # Marker declarations are observations unless explicitly used by a load.
    load_markers = set(source.get("body_wrench_markers", ()))
    source["markers"] = [row for row in source.get("markers", ()) if row["name"] in load_markers]
    source.setdefault("road", {"kind": "plane"})
    for field in ("contract", "contract_version", "kind", "name", "blobs", "capabilities"):
        source.pop(field, None)
    result = _remap(source, maps)
    if resources:
        result["unbound_resources"] = resources
    return result


def _case(document, blob, maps):
    source = deepcopy(document)
    for field in ("contract", "contract_version", "kind", "name", "blobs", "outputs"):
        source.pop(field, None)
    return {**_remap(source, maps), "tables": _tables(document, blob, maps)}


def differences(before, after, path=""):
    """Return complete scalar, structural and missing-field differences."""
    found = []
    if isinstance(before, dict) and isinstance(after, dict):
        for key in sorted(before.keys() | after.keys()):
            field = path+"."+key if path else key
            if key not in before or key not in after:
                found.append({"field": field, "old": before.get(key), "new": after.get(key)})
            else:
                found.extend(differences(before[key], after[key], field))
    elif isinstance(before, list) and isinstance(after, list):
        if len(before) != len(after):
            found.append({"field": path, "old_count": len(before), "new_count": len(after)})
        else:
            for index, (left, right) in enumerate(zip(before, after)):
                found.extend(differences(left, right, path+"["+str(index)+"]"))
    elif before != after:
        found.append({"field": path, "old": before, "new": after})
    return found


def input_differences(compiled, old_model, old_case, model_blob, case_blob, layout):
    """Compare submitted physical data after native-default normalization."""
    maps = _id_maps(old_model, layout)
    _, new_model_blob = unpack_container(compiled.model_payload)
    _, new_case_blob = unpack_container(compiled.case_payload)
    before_model = _model(old_model, model_blob, maps)
    after_model = _model(compiled.model_document, new_model_blob, {})
    before_case = _case(old_case, case_blob, maps)
    after_case = _case(compiled.case_document, new_case_blob, {})
    return differences(before_model, after_model), differences(before_case, after_case), {
        "original_model": digest(json.dumps(before_model, sort_keys=True).encode()),
        "current_model": digest(json.dumps(after_model, sort_keys=True).encode()),
        "original_case": digest(json.dumps(before_case, sort_keys=True).encode()),
        "current_case": digest(json.dumps(after_case, sort_keys=True).encode())}


def document_arrays(result, layout):
    """Extract every frozen channel using its stable entity ID."""
    arrays = {"states": np.stack([result.body_state(name) for name in layout["bodies"]], axis=1),
              "energy": np.asarray(result.energy), "times_s": np.asarray(result.times_s)}
    joints = tuple(row["name"] for row in result.raw.model_document["joints"])
    if set(joints) != set(layout["constraints"]):
        raise ValueError("current constraint identities disagree with frozen layout")
    arrays["constraint_wrench"] = result.raw.block("constraint_wrench")[:, [joints.index(name) for name in layout["constraints"]]]
    for kind, block, width in (("spring", "spring_output", 4), ("bushing", "bushing_output", 12),
                               ("anti_roll_bar", "anti_roll_output", 3), ("tire", "tire_output", 41),
                               ("steering_actuator", "steering_output", 4)):
        values = [result.element_state(name) for name in layout["ledgers"][kind]]
        arrays[block] = np.stack(values, axis=1) if values else np.zeros((len(result.times_s), 0, width))
    arrays["diagnostics"] = result.raw.block("diagnostics")[:len(result.times_s)]
    return arrays


def array_digests(arrays):
    """Hash channels using the unchanged original snapshot algorithm."""
    return {field: _diagnostic_digest(np.asarray(array)) if field == "diagnostics" else _array_digest(np.asarray(array))
            for field, array in arrays.items() if field != "times_s"}


def spin_checks(compiled):
    """Verify the declared bearing's carrier and the same initial wheel centre."""
    model = compiled.request.model.to_document()
    bodies = {row["name"]: row for row in compiled.model_document["bodies"]}
    frames = {row["name"]: row for row in model["frames"]}
    joints = {row["name"]: row for row in model["joints"]}
    boundaries = compiled.metadata["solve_plan"]["boundaries"]
    failures, checked = [], []
    for coordinate in model["coordinates"]:
        if coordinate["kind"] != "rotation":
            continue
        joint = joints[coordinate["source_joint_id"]]
        carrier = frames[coordinate["frame_a"]]["body"]
        hub = frames[coordinate["frame_b"]]["body"]
        component = {hub}
        while True:
            expanded = component | {row[end] for row in joints.values() if row["type"] == "fixed"
                and (row["body_a"] in component or row["body_b"] in component) for end in ("body_a", "body_b")}
            if expanded == component:
                break
            component = expanded
        tires = [row for row in compiled.model_document["tires"] if row["body"] in component]
        if not tires:
            failures.append({"coordinate": coordinate["name"], "reason": "declared wheel bearing has no tire"})
        matching = [row for row in boundaries if row["coordinate"] == coordinate["name"]]
        if len(matching) != 1 or matching[0]["mode"] != "free":
            failures.append({"coordinate": coordinate["name"], "reason": "vehicle spin must be explicitly free"})
        if {joint["body_a"], joint["body_b"]} != {carrier, hub}:
            failures.append({"coordinate": coordinate["name"], "reason": "coordinate owners differ from bearing"})
        for tire in tires:
            params = tire["parameters"]
            if params.get("frame_body") != carrier:
                failures.append({"tire": tire["name"], "reason": "contact frame is not the declared carrier"})
                continue
            def center(body_name, point):
                body = bodies[body_name]
                return np.asarray(body["position"])+quaternion_to_matrix(np.asarray(body["quaternion"]))@np.asarray(point)
            contact = center(carrier, params["frame_center_local"])
            wheel = center(tire["body"], params["center_local"])
            limit = 1024*np.finfo(float).eps*max(1., float(np.max(np.abs(contact))), float(np.max(np.abs(wheel))))
            if np.max(np.abs(contact-wheel)) > limit:
                failures.append({"tire": tire["name"], "reason": "carrier centre differs from initial wheel centre", "limit_m": limit})
            checked.append(tire["name"])
    if len(checked) != 4 or len(set(checked)) != 4:
        failures.append({"reason": "approved evidence must cover exactly four distinct wheel spins"})
    return {"passed": not failures, "failures": failures, "tires": checked}


def allowed_input_changes(model_delta, case_delta):
    """Allow only four carrier-reference changes and their local centres."""
    allowed = {f"tires[{index}].parameters.frame_body" for index in range(4)}
    allowed |= {f"tires[{index}].parameters.frame_center_local[{axis}]" for index in range(4) for axis in range(3)}
    rejected = [row for row in model_delta if row["field"] not in allowed]
    rejected.extend({**row, "field": "case."+row["field"]} for row in case_delta)
    return {"passed": not rejected, "unexpected": rejected}


def channel_differences(before, after, layout, model):
    """Report every column and its SI unit; the full arrays are stored alongside."""
    units = {
        "states": ["m"]*3+["1"]*4+["m/s"]*3+["rad/s"]*3+["m/s2"]*3+["rad/s2"]*3,
        "constraint_wrench": ["N"]*3+["Nm"]*3,
        "spring_output": ["m", "m/s", "N", "N"],
        "bushing_output": ["m"]*3+["rad"]*3+["N"]*3+["Nm"]*3,
        "anti_roll_output": ["rad", "rad/s", "Nm"],
        "tire_output": ["1", "m", "m", "m/s", "N", "N", "N", "m/s", "m/s", "1", "m", "m"]
            +["Nm"]*3+["m", "m/s", "m", "m/s", "rad", "rad/s"]+["rad/m"]*4
            +["m/s"]*2+["rad"]*5+["rad/m"]*2+["rad/s"]*4+["m", "1", "1"],
        "energy": ["J"]*13+["1"]+["J"]*7,
        "diagnostics": ["1"]*4+["s"]*3+["m|rad", "m/s|rad/s", "N|Nm"]+["1"]*3+["J"]+["1"]*2,
        "times_s": ["s"],
    }
    elements = {row["name"]: row for row in model["elements"]}
    names = {"states": layout["bodies"], "constraint_wrench": layout["constraints"],
             "energy": ["system"], "diagnostics": ["solver"], "times_s": ["clock"]}
    for kind, block in (("spring", "spring_output"), ("bushing", "bushing_output"),
                        ("anti_roll_bar", "anti_roll_output"), ("tire", "tire_output"),
                        ("steering_actuator", "steering_output")):
        names[block] = layout["ledgers"][kind]
    steering_units = {name: ["rad", "rad/s", "rad", "Nm"] if elements[name]["parameters"]["type"] == "rotation"
                      else ["m", "m/s", "m", "N"] for name in names["steering_output"]}
    rows = []
    if set(before) != set(after):
        raise ValueError("reference and current channel sets differ")
    for key, actual in after.items():
        wanted = np.asarray(before[key])
        if actual.shape != wanted.shape:
            raise ValueError(f"channel {key!r} shape differs from reference")
        if actual.ndim == 1:
            actual, wanted = actual[:, None, None], wanted[:, None, None]
        elif actual.ndim == 2:
            actual, wanted = actual[:, None, :], wanted[:, None, :]
        for index, entity in enumerate(names[key]):
            columns = steering_units[entity] if key == "steering_output" else units[key]
            if len(columns) != actual.shape[-1]:
                raise ValueError(f"channel {key!r} unit schema differs from its width")
            for column, unit in enumerate(columns):
                left, right = wanted[:, index, column], actual[:, index, column]
                delta = right-left
                rows.append({"channel": key, "entity": entity, "column": column, "unit": unit,
                             "max_abs_difference": float(np.max(np.abs(delta), initial=0.0)),
                             "rms_difference": float(np.sqrt(np.mean(delta**2))),
                             "bit_identical": left.tobytes() == right.tobytes()})
    return rows
