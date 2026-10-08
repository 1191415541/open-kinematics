"""Vehicle declarations shared by tests and frozen numerical evidence."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suspension_multibody.authoring import migrate_v1_vehicle
from suspension_multibody.modeling.primitives.spatial import SE3
from suspension_multibody.schema import (
    DynamicSolverSettings,
    InitialBodyState,
    LinearSpring,
    MassSpec,
    Pose,
    Quaternion,
    RigidBodySpec,
    RoadSurfaceSpec,
    SixVector,
    StaticDamper,
    SteeringSystemSpec,
    TimeSignal,
    TireModelSpec,
    Vec3,
    VehicleDynamicCase,
    WheelSpec,
)
from suspension_multibody.schema.model import AxleDeclaration
from suspension_multibody.schema.vehicle import VehicleDeclaration

_BODY_NAMES = (
    "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
    "upright_L", "upright_R", "tie_rod_L", "tie_rod_R",
)


def _axle(name: str, x: float, dampers: tuple[StaticDamper, ...] = ()) -> AxleDeclaration:
    return AxleDeclaration(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500, z=500),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150, y=-500, z=500),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750, z=350),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500, z=100),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150, y=-500, z=100),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750, z=100),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450, z=250),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750, z=250),
            "WHEEL_CENTER": Vec3(x=x, y=-750, z=300),
            "RACK_CENTER": Vec3(x=x, y=0, z=250),
        },
        mass=MassSpec(sprung_mass=600),
        bodies=tuple(
            RigidBodySpec(name=body, mass=100, inertia=((100, 0, 0), (0, 100, 0), (0, 0, 100)))
            for body in _BODY_NAMES
        ),
        dampers=dampers,
    )


def _tire() -> TireModelSpec:
    return TireModelSpec(
        kind="native_brush", unloaded_radius=300, maximum_compression=250,
        vertical_stiffness=200, cornering_stiffness=80_000,
        longitudinal_stiffness=120_000, relaxation_length=300,
    )


def _vehicle(
    *, front_dampers: tuple[StaticDamper, ...] = (), rear_dampers: tuple[StaticDamper, ...] = (),
) -> VehicleDeclaration:
    return VehicleDeclaration(
        chassis=RigidBodySpec(
            name="chassis", mass=1200,
            inertia=((1_000_000, 0, 0), (0, 1_200_000, 0), (0, 0, 1_500_000)),
        ),
        front_axle=_axle("front", 1400, front_dampers),
        rear_axle=_axle("rear", -1400, rear_dampers).model_copy(update={"rack_fixed_to_chassis": True}),
        wheels=tuple(
            WheelSpec(name=name, body=f"wheel_{name}", center_local=Vec3(), mass=20, axial_inertia=2, tire=_tire())
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16, rack_damping=0),
    )


def _positioned_axle(axle: AxleDeclaration) -> AxleDeclaration:
    def point(name: str, side: str) -> Vec3:
        value = axle.hardpoints[name]
        return value if side == "L" else value.mirrored_y()

    def mean(names: tuple[str, ...], side: str = "L") -> np.ndarray:
        return np.mean(np.asarray([point(name, side).as_array() for name in names]), axis=0)

    origins: dict[str, np.ndarray] = {"rack": mean(("RACK_CENTER",))}
    for side in ("L", "R"):
        origins.update({
            f"upper_arm_{side}": mean(("UPPER_INBOARD_FRONT", "UPPER_INBOARD_REAR", "UPPER_OUTBOARD"), side),
            f"lower_arm_{side}": mean(("LOWER_INBOARD_FRONT", "LOWER_INBOARD_REAR", "LOWER_OUTBOARD"), side),
            f"upright_{side}": mean(("UPPER_OUTBOARD", "LOWER_OUTBOARD", "WHEEL_CENTER"), side),
            f"tie_rod_{side}": mean(("TIE_ROD_INBOARD", "TIE_ROD_OUTBOARD"), side),
        })
    bodies = tuple(
        body.model_copy(update={"pose": body.pose.model_copy(update={"translation": Vec3(
            x=float(origins[body.name][0]), y=float(origins[body.name][1]), z=float(origins[body.name][2]),
        )})})
        for body in axle.bodies
    )
    return axle.model_copy(update={"bodies": bodies})


def _positioned_vehicle(model: VehicleDeclaration) -> VehicleDeclaration:
    return model.model_copy(update={
        "front_axle": _positioned_axle(model.front_axle), "rear_axle": _positioned_axle(model.rear_axle),
    })


def _with_ride_springs(model: VehicleDeclaration) -> VehicleDeclaration:
    front, rear = model.front_axle, model.rear_axle
    front_spring = LinearSpring(
        name="ride_spring", body_a="chassis", body_b="lower_arm",
        point_a=front.hardpoints["LOWER_INBOARD_FRONT"], point_b=front.hardpoints["LOWER_OUTBOARD"],
        stiffness=100.0, free_length=450.0,
    )
    rear_spring = front_spring.model_copy(update={
        "point_a": rear.hardpoints["LOWER_INBOARD_FRONT"], "point_b": rear.hardpoints["LOWER_OUTBOARD"],
    })
    return model.model_copy(update={
        "front_axle": front.model_copy(update={"springs": (front_spring,)}),
        "rear_axle": rear.model_copy(update={"springs": (rear_spring,)}),
    })


def _case(
    model: VehicleDeclaration, *, name: str = "native-vehicle", brake: float = 0.0,
    wheel_speeds: tuple[tuple[str, float], ...] = (), steering: TimeSignal | None = None,
) -> VehicleDynamicCase:
    return VehicleDynamicCase(
        name=name, vehicle=model,
        solver=DynamicSolverSettings(
            end_time=0.001, step_size=0.001, internal_step_size=0.001, min_internal_step_size=0.001,
            adaptive_substepping=False, integrator="generalized_alpha", gravity=Vec3(x=0, y=0, z=0),
        ),
        road=RoadSurfaceSpec(kind="plane"), steering_input=steering or TimeSignal(constant=0.0),
        brake_input=TimeSignal(constant=brake), initial_wheel_speeds=wheel_speeds,
    )


def _pac2002_model(
    *, combined: bool, parameter_source: str = "user", extra_coefficients: dict[str, float] | None = None,
) -> VehicleDeclaration:
    base = _positioned_vehicle(_vehicle())
    coefficients = {
        "FNOMIN": 4_850.0, "PCX1": 1.65, "PDX1": 1.0, "PKX1": 22.3,
        "PCY1": 1.3, "PDY1": 1.0, "PKY1": -21.9, "RBX1": 10.0, "RBX2": 0.0,
        "RCX1": 1.2, "REX1": 0.2, "RBY1": 8.0, "RBY2": 0.0, "RBY3": 0.0,
        "RCY1": 1.1, "REY1": 0.1,
    }
    if not combined:
        coefficients.update(RBX1=0.0, RBY1=0.0)
    if extra_coefficients:
        coefficients.update(extra_coefficients)
    tire = _tire().model_copy(update={
        "kind": "pac2002", "parameter_source": parameter_source, "pac2002_coefficients": coefficients,
    })
    return base.model_copy(update={"wheels": tuple(wheel.model_copy(update={"tire": tire}) for wheel in base.wheels)})


def _uniform_velocity_initial_states(
    model: VehicleDeclaration, *, vx_mm_s: float = 10_000.0, vy_mm_s: float = 5_000.0,
) -> tuple[InitialBodyState, ...]:
    document = migrate_v1_vehicle(model, mode="K")
    bodies = {
        entry.ref + "." + row["name"]: (entry.ref, row)
        for entry in document.entries for row in entry.subsystem.template.payload["bodies"]
    }
    layout = Path(__file__).resolve().parents[1] / "data/vehicle_dynamics_baseline/entity_layout.json"
    order = json.loads(layout.read_text(encoding="utf-8"))["cases"]["default"]["bodies"]
    if set(order) != set(bodies):
        raise ValueError("vehicle fixture body declarations disagree with the frozen identity layout")
    states = []
    for identifier in order:
        ref, row = bodies[identifier]
        name = ref.split("_", 1)[0] + "_" + row["name"] if ref.startswith(("front_", "rear_")) else row["name"]
        # Keep engineering-unit body origins, rather than SI centre-of-mass states.
        pose = SE3(np.asarray(row.get("position", [0, 0, 0]), dtype=float),
            np.asarray(row.get("quaternion", [1, 0, 0, 0]), dtype=float))
        states.append(InitialBodyState(
            body=name,
            pose=Pose(translation=Vec3(x=float(pose.translation[0]), y=float(pose.translation[1]), z=float(pose.translation[2])),
                rotation=Quaternion(w=float(pose.quaternion[0]), x=float(pose.quaternion[1]),
                    y=float(pose.quaternion[2]), z=float(pose.quaternion[3]))),
            velocity=SixVector(fx=vx_mm_s, fy=vy_mm_s),
        ))
    return tuple(states)
