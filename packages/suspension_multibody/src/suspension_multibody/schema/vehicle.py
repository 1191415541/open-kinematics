"""Full-vehicle multibody model and time-domain case schemas."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .common import (
    CoordinateSystem,
    Pose,
    Quaternion,
    StrictModel,
    UnitSystem,
    Vec3,
)
from .dynamic import DynamicSolverSettings, InitialBodyState, TimeSignal, TireModelSpec
from .model import FrontAxleModel, RigidBodySpec

# The road surface is shared with the axle model, so it lives in its own module and is
# re-exported here: callers that already import it from the vehicle schema keep working.
from .road import RoadSurfaceSpec


class AerodynamicDragSpec(StrictModel):
    """Quadratic aerodynamic drag applied to the chassis."""

    air_density: float = Field(gt=0)
    drag_coefficient: float = Field(ge=0)
    frontal_area: float = Field(gt=0)
    application_point: Vec3 = Field(default_factory=Vec3)
    forward_axis: Vec3 = Vec3(x=1.0, y=0.0, z=0.0)

    @model_validator(mode="after")
    def _valid_forward_axis(self) -> AerodynamicDragSpec:
        axis = self.forward_axis.as_array()
        if not math.isfinite(float(axis @ axis)) or float(axis @ axis) <= 1e-12:
            raise ValueError("aerodynamic forward_axis must be non-zero")
        return self


class WheelSpec(StrictModel):
    """One wheel-end rotational body and its tire parameters."""

    #: The wheel's own name.  Deliberately not a four-corner ``Literal``: which wheel
    #: ends exist is the assembly's declaration, and a three-axle truck names six.  The
    #: *four-corner* requirement of a full vehicle is stated once, in
    #: :meth:`VehicleModel._topology`, so widening this field does not relax it -- an
    #: existing vehicle model still has to name exactly the four corners it always did.
    name: str = Field(min_length=1)
    body: str
    center_local: Vec3
    steering_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    spin_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    forward_axis: Vec3 | None = None
    pose: Pose = Field(default_factory=Pose)
    inertia: tuple[tuple[float, ...], ...] | None = None
    mount_body: str | None = None
    mount_joint_kind: Literal["revolute", "fixed"] = "revolute"
    # Optional driveline actuator mapping. The axis is local to
    # ``drive_torque_body``; without it, drive torque keeps the historical
    # wheel-body application used by generic vehicle models.
    drive_torque_body: str | None = None
    drive_torque_reaction_body: str | None = None
    drive_torque_axis_local: Vec3 | None = None
    # 仅用于静态配平的被动转轴；动态积分仍保留完整轮端刚体。
    static_rotation_axis_local: Vec3 | None = None
    mass: float = Field(default=0.0, ge=0)
    #: The part of `mass` that belongs to the *tire* rather than the wheel-end body
    #: (decision D2, requirement 10).  A tire used to carry no mass of its own: the
    #: wheel-end body owned the whole wheel.  Declaring a share here moves it, and
    #: the solver sums what the tire declares back into its carrying body, so the
    #: dynamics are unchanged and only the ownership moves.
    #:
    #: Zero -- the default, and the value every existing model has -- means the body
    #: still owns the whole wheel, so no recorded baseline moves.
    #:
    #: Excluded from `model_dump` on purpose: `api.py` hashes the dump into
    #: `Provenance.model_hash` and `io/artifacts.py` hashes that again into the
    #: artifact manifest, so a dump-visible field would change every recorded
    #: full-vehicle result.
    tire_mass: float = Field(default=0.0, ge=0, exclude=True)
    axial_inertia: float = Field(default=1.0, gt=0)
    tire: TireModelSpec = Field(default_factory=TireModelSpec)
    driven: bool = False
    braked: bool = True

    @field_validator("inertia", mode="before")
    @classmethod
    def _inertia_shape(
        cls, value: object
    ) -> tuple[tuple[float, ...], ...] | None:
        if value is None:
            return None
        rows = tuple(tuple(float(item) for item in row) for row in value)  # type: ignore[union-attr]
        if len(rows) != 3 or any(len(row) != 3 for row in rows):
            raise ValueError("wheel inertia must be a 3x3 matrix")
        if any(not math.isfinite(item) for row in rows for item in row):
            raise ValueError("wheel inertia must contain finite values")
        return rows

    @model_validator(mode="after")
    def _vectors_and_inertia(self) -> WheelSpec:
        for name, vector in (
            ("steering_axis", self.steering_axis),
            ("spin_axis", self.spin_axis),
        ):
            values = vector.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError(f"{name} must be a non-zero finite vector")
        if self.forward_axis is not None:
            forward = self.forward_axis.as_array()
            if not math.isfinite(float(forward @ forward)) or float(forward @ forward) <= 1e-12:
                raise ValueError("forward_axis must be a non-zero finite vector")
            spin = self.spin_axis.as_array()
            cross = (
                (spin[1] * forward[2] - spin[2] * forward[1]) ** 2
                + (spin[2] * forward[0] - spin[0] * forward[2]) ** 2
                + (spin[0] * forward[1] - spin[1] * forward[0]) ** 2
            )
            if float(cross) <= 1e-12 * float(spin @ spin) * float(forward @ forward):
                raise ValueError("forward_axis must not be parallel to spin_axis")
        if self.static_rotation_axis_local is not None:
            values = self.static_rotation_axis_local.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError("static_rotation_axis_local must be non-zero")
        if (self.drive_torque_body is None) != (
            self.drive_torque_axis_local is None
        ):
            raise ValueError(
                "drive_torque_body and drive_torque_axis_local must be provided together"
            )
        if self.drive_torque_reaction_body is not None:
            if self.drive_torque_body is None:
                raise ValueError(
                    "drive_torque_reaction_body requires drive_torque_body"
                )
            if self.drive_torque_reaction_body == self.drive_torque_body:
                raise ValueError("drive torque body and reaction body must differ")
        if self.drive_torque_axis_local is not None:
            values = self.drive_torque_axis_local.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError("drive_torque_axis_local must be non-zero")
        return self


class SteeringSystemSpec(StrictModel):
    """Rack-and-pinion steering actuator boundary."""

    rack_body: str = "rack"
    ratio: float = Field(gt=0)
    max_rack_displacement: float = Field(default=100.0, gt=0)
    input: Literal["rack_displacement", "steering_wheel_angle"] = "rack_displacement"
    rack_displacement_per_steering_wheel_angle: float | None = Field(default=None, gt=0)
    rack_stiffness: float = Field(default=20_000.0, gt=0)
    rack_damping: float = Field(default=500.0, ge=0)
    max_steering_angle: float = Field(default=math.pi, gt=0)
    actuator_mode: Literal[
        "rack_translation",
        "prescribed_rotation",
        "prescribed_translation",
    ] = "rack_translation"
    actuator_body: str | None = None
    actuator_reaction_body: str | None = None
    actuator_axis_local: Vec3 = Vec3(x=0.0, y=0.0, z=1.0)
    actuator_reference_rotation: Quaternion = Field(default_factory=Quaternion)
    #: The name this channel is addressed by (subtask p2-06).
    #:
    #: A channel is a steering rack the preparation layer actuates, and it needs a
    #: name of its own: the constraint row, the contract element and the recorded
    #: output are all keyed by it.  ``"front_rack"`` is what the single
    #: compatibility channel has always been called.
    channel_name: str = Field(default="front_rack", min_length=1, exclude=True)
    #: The placement this channel steers, as the assembly names it.
    #:
    #: The *declaration* rather than a reading of a name: a channel that does not
    #: say which axle it steers would have its rack resolved by guessing, and a
    #: two-channel vehicle would drive the same rack twice.
    placement: str = Field(default="front", min_length=1, exclude=True)
    #: Whether this channel takes part in a run (subtask p2-06).
    #:
    #: A declared-but-disabled channel keeps its geometry and ratio on file while
    #: staying out of the assembled run, which is how a vehicle is compared with
    #: and without its rear steer.
    enabled: bool = Field(default=True, exclude=True)


class SteeringChannelSpec(SteeringSystemSpec):
    """One additional steering channel, beyond the compatibility primary."""

    channel_name: str = Field(min_length=1, exclude=True)
    placement: str = Field(min_length=1, exclude=True)

class JointCoordinateCouplerSpec(StrictModel):
    """Linear relation between two ideal-joint coordinates."""

    name: str
    joint_a: str
    coordinate_a: Literal["rotation", "translation"]
    scale_a: float
    joint_b: str
    coordinate_b: Literal["rotation", "translation"]
    scale_b: float

    @model_validator(mode="after")
    def _valid_relation(self) -> JointCoordinateCouplerSpec:
        if self.joint_a == self.joint_b:
            raise ValueError("a coordinate coupler requires two different joints")
        if not math.isfinite(self.scale_a) or not math.isfinite(self.scale_b):
            raise ValueError("coordinate coupler scales must be finite")
        if abs(self.scale_a) <= 1e-12 or abs(self.scale_b) <= 1e-12:
            raise ValueError("coordinate coupler scales must be non-zero")
        return self


class DrivelineSpec(StrictModel):
    """
    Simplified but torque-based brake and drive actuator contract.

    Two brake/drive descriptions live here, and they are not alternatives:

    * `maximum_brake_torque`, `front_brake_bias`, `maximum_drive_torque` and
      `drive_split` are the *lumped* limits the Python-side torque builder
      (`preparation/vehicle_dynamic.py`, `_build_wheel_torque_signals`) has
      always used; a wheel's brake torque is `maximum_brake_torque * share *
      brake_input`, so one number caps every wheel;
    * `brake_mu`, `piston_area`, `effective_piston_radius` and
      `max_brake_value` are the *per-parameter* torque subset decision D10 takes
      from the Adams simple brake.  They let the same torque be checked parameter
      by parameter against the frozen Adams document, which the lumped form
      cannot express.

    `max_brake_value` is the `.adm`'s `0.1` demand scaling; the effective piston
    radius there is 145.0 at the front and 130.0 at the rear, and one field
    cannot hold both -- a recorded deviation, not an oversight.

    The four new fields are excluded from `model_dump`: `api.py` hashes
    `model.model_dump(mode="json")` into `Provenance.model_hash`, so a
    dump-visible field would change every existing full-vehicle model hash and
    invalidate the recorded baselines.  Exclusion is what keeps them usable --
    declared, typed, validated, and invisible to the hash.
    """

    driven_wheels: tuple[str, ...] = ()
    maximum_drive_torque: float = Field(default=0.0, ge=0)
    maximum_brake_torque: float = Field(default=10_000.0, ge=0)
    front_brake_bias: float = Field(default=0.6, ge=0, le=1)
    drive_split: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    #: Friction coefficient of the simplified brake (Adams simple brake `mu`).
    brake_mu: float = Field(default=0.4, gt=0, exclude=True)
    #: Piston area of the simplified brake (Adams `piston_area`).
    piston_area: float = Field(default=2500.0, gt=0, exclude=True)
    #: Effective piston radius of the simplified brake (Adams front value 145.0).
    effective_piston_radius: float = Field(default=145.0, gt=0, exclude=True)
    #: Demand scaling of the simplified brake (the `.adm`'s `0.1`).
    max_brake_value: float = Field(default=0.1, gt=0, le=1, exclude=True)
    #: Which driver demand the wheels' torque elements follow (subtask p2-09).
    #:
    #: ``"none"`` is the default and means exactly what every existing model
    #: says: the brake and drive reach the solver through the per-wheel torque
    #: tables the preparation layer has always built.  A model that states a
    #: demand asks for the other mechanism instead -- a ``rotational_torque``
    #: element per declared wheel end, whose magnitude the kernel reads from the
    #: case's normalized ``brake_pressure``/``throttle_demand`` signal at every
    #: step.  The two are alternatives rather than a combination: a wheel is
    #: stated in Newton-metres or as a fraction, and the kernel refuses a case
    #: that states both for one wheel's channel.
    #:
    #: Excluded from `model_dump` for the same reason the four fields above are:
    #: `api.py` hashes the dump into `Provenance.model_hash`, so a dump-visible
    #: field would move every existing model hash and with it the recorded
    #: baselines.  Exclusion is what keeps the switch usable.
    torque_demand: Literal["none", "drive", "brake", "both"] = Field(
        default="none", exclude=True
    )

    @model_validator(mode="after")
    def _validate_distribution(self) -> DrivelineSpec:
        if any(name not in {"front_left", "front_right", "rear_left", "rear_right"} for name in self.driven_wheels):
            raise ValueError("driven_wheels contains an unknown wheel")
        if len(set(self.driven_wheels)) != len(self.driven_wheels):
            raise ValueError("driven_wheels must be unique")
        if any(value < 0 or not math.isfinite(value) for value in self.drive_split):
            raise ValueError("drive_split must contain finite non-negative values")
        if self.maximum_drive_torque > 0 and sum(self.drive_split) <= 0:
            raise ValueError("drive_split is required when drive torque is enabled")
        if sum(self.drive_split) > 0 and abs(sum(self.drive_split) - 1.0) > 1e-9:
            raise ValueError("drive_split must sum to one")
        return self


class VehicleModel(StrictModel):
    """Explicit four-corner full-vehicle multibody model."""

    schema_version: int = Field(default=1, ge=1)
    name: str = "full_vehicle"
    units: UnitSystem = UnitSystem.ENGINEERING
    coordinate_system: CoordinateSystem = CoordinateSystem.VEHICLE
    chassis: RigidBodySpec
    front_axle: FrontAxleModel
    rear_axle: FrontAxleModel
    wheels: tuple[WheelSpec, ...]
    steering: SteeringSystemSpec
    #: The additional steering channels (subtask p2-06).
    #:
    #: ``steering`` stays exactly what it was -- the mandatory compatibility
    #: primary channel -- and this tuple carries the rest, so a one-channel
    #: declaration dumps and hashes byte for byte as it always did.  Excluded
    #: from ``model_dump`` for the same reason ``torque_demand`` is: ``api.py``
    #: hashes that dump into ``Provenance.model_hash``.
    steering_channels: tuple[SteeringChannelSpec, ...] = Field(
        default=(), exclude=True
    )
    #: Which allocation law turns one driver input into the per-channel angles.
    #:
    #: ``direct`` is the historical behaviour and the default: every channel is
    #: driven by the case's own steering signal, and a one-channel vehicle is
    #: therefore bit-identical to what it was before channels existed.
    allocation_law: Literal[
        "direct", "ackermann", "four_wheel_steer", "multi_axle_follow"
    ] = Field(default="direct", exclude=True)
    driveline: DrivelineSpec = Field(default_factory=DrivelineSpec)
    coordinate_couplers: tuple[JointCoordinateCouplerSpec, ...] = ()
    aerodynamic_drag: AerodynamicDragSpec | None = None

    @model_validator(mode="after")
    def _topology(self) -> VehicleModel:
        names = tuple(wheel.name for wheel in self.wheels)
        required = {"front_left", "front_right", "rear_left", "rear_right"}
        if set(names) != required or len(names) != 4:
            raise ValueError("vehicle must define exactly one wheel for each of four corners")
        bodies = [self.chassis.name, *(wheel.body for wheel in self.wheels)]
        if len(set(bodies)) != len(bodies):
            raise ValueError("chassis and wheel body names must be unique")
        if any(name not in names for name in self.driveline.driven_wheels):
            raise ValueError("driveline references an undefined wheel")
        required_bodies = {
            "rack",
            "upper_arm_L",
            "upper_arm_R",
            "lower_arm_L",
            "lower_arm_R",
            "upright_L",
            "upright_R",
            "tie_rod_L",
            "tie_rod_R",
        }
        for axle_name, axle in (("front", self.front_axle), ("rear", self.rear_axle)):
            specs = {body.name: body for body in axle.bodies}
            if len(specs) != len(axle.bodies):
                raise ValueError(f"{axle_name} axle body names must be unique")
            if axle.topology == "symmetric_proxy":
                missing = sorted(required_bodies - set(specs))
                if missing:
                    raise ValueError(
                        f"{axle_name} axle requires positive mass specs for: {', '.join(missing)}"
                    )
                if any(specs[name].mass <= 0.0 for name in required_bodies):
                    raise ValueError(
                        f"{axle_name} axle body mass specs must be positive"
                    )
            elif not specs:
                raise ValueError(f"{axle_name} explicit axle has no body specs")
            joint_names = [joint.name for joint in axle.joints]
            if len(joint_names) != len(set(joint_names)):
                raise ValueError(f"{axle_name} explicit joint names must be unique")
            known = {self.chassis.name, *specs}
            for joint in axle.joints:
                if joint.body_a not in known or joint.body_b not in known:
                    raise ValueError(
                        f"{axle_name} joint {joint.name!r} references an undefined body"
                    )
        runtime_joint_names = {
            *(f"front_{joint.name}" for joint in self.front_axle.joints),
            *(f"rear_{joint.name}" for joint in self.rear_axle.joints),
        }
        coupler_names = [coupler.name for coupler in self.coordinate_couplers]
        if len(coupler_names) != len(set(coupler_names)):
            raise ValueError("coordinate coupler names must be unique")
        for coupler in self.coordinate_couplers:
            for joint_name in (coupler.joint_a, coupler.joint_b):
                if joint_name not in runtime_joint_names:
                    raise ValueError(
                        f"coordinate coupler {coupler.name!r} references an undefined joint"
                    )
        self._check_steering_channels()
        return self

    def _check_steering_channels(self) -> None:
        """
        Refuse a channel set whose own declarations disagree (subtask p2-06).

        Three questions are asked, and all three are answerable from the
        declared names alone: whether a channel may be addressed at all, whether
        two channels claim the same identity, and whether a channel states an
        actuator whose two ends are the same body (a couple against itself, which
        is not a load path).

        Deliberately *not* asked: which channel steers which axle.  ``placement``
        is the declaration that answers it, so an ordering rule such as "the first
        channel is the steered one" would be a second answer that could disagree
        with the first.
        """
        channels: tuple[SteeringSystemSpec, ...] = (
            self.steering,
            *self.steering_channels,
        )
        if not self.steering.channel_name.strip():
            raise ValueError("the primary steering channel needs a channel_name")
        if not self.steering.placement.strip():
            raise ValueError("the primary steering channel needs a placement")
        names = [channel.channel_name.strip() for channel in channels]
        repeated = _first_repeated(names)
        if repeated is not None:
            raise ValueError(f"steering channel name {repeated!r} is declared twice")
        placements = [channel.placement.strip() for channel in channels]
        repeated_placement = _first_repeated(placements)
        if repeated_placement is not None:
            raise ValueError(
                f"steering channel placement {repeated_placement!r} is declared twice"
            )
        for channel in channels:
            for label, body in (
                ("rack_body", channel.rack_body),
                ("actuator_body", channel.actuator_body),
                ("actuator_reaction_body", channel.actuator_reaction_body),
            ):
                if body is not None and not body.strip():
                    raise ValueError(
                        f"steering channel {channel.channel_name!r} states an empty {label}"
                    )
            if (
                channel.actuator_body is not None
                and channel.actuator_body == channel.actuator_reaction_body
            ):
                raise ValueError(
                    f"steering channel {channel.channel_name!r} actuates "
                    f"{channel.actuator_body!r} against itself"
                )


def _first_repeated(values: list[str]) -> str | None:
    """Return the first value that appears twice, or ``None``."""
    seen: set[str] = set()
    for value in values:
        if value in seen:
            return value
        seen.add(value)
    return None


class VehicleDynamicCase(StrictModel):
    """Time-domain case for the full-vehicle multibody solver."""

    name: str = "full_vehicle_dynamic"
    solver: DynamicSolverSettings
    vehicle: VehicleModel
    road: RoadSurfaceSpec = Field(default_factory=RoadSurfaceSpec)
    steering_input: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    brake_input: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    drive_input: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    # These optional signals are direct per-wheel torque overrides in the
    # VehicleModel torque units.  They are intentionally separate from the
    # normalized global drive/brake commands.
    wheel_drive_torque: tuple[tuple[str, TimeSignal], ...] = ()
    wheel_brake_torque: tuple[tuple[str, TimeSignal], ...] = ()
    initial_wheel_speeds: tuple[tuple[str, float], ...] = ()
    initial_states: tuple[InitialBodyState, ...] = ()
    static_equilibrium: bool = False
    # ``auto`` retains ideal inboard joints when no physical bushing data is
    # present, and selects compliant C mode when the model supplies bushings.
    # Explicit K/C is retained for reproducible comparisons and is validated
    # by the native adapter instead of silently dropping elements.
    suspension_mode: Literal["auto", "K", "C"] = "auto"
    initial_forward_speed_mps: float = Field(default=0.0, ge=0)
    # 速度大小与车辆坐标轴方向分开表达；Adams 源模型可能沿 -X 行驶。
    initial_velocity_sign: Literal[-1, 1] = 1

    @model_validator(mode="after")
    def _initial_wheel_speeds(self) -> VehicleDynamicCase:
        names = {wheel.name for wheel in self.vehicle.wheels}
        provided = [name for name, _ in self.initial_wheel_speeds]
        if any(name not in names for name in provided):
            raise ValueError("initial_wheel_speeds references an undefined wheel")
        if len(provided) != len(set(provided)):
            raise ValueError("initial_wheel_speeds must be unique")
        if any(not math.isfinite(value) for _, value in self.initial_wheel_speeds):
            raise ValueError("initial wheel speeds must be finite")
        state_bodies = [item.body for item in self.initial_states]
        if len(state_bodies) != len(set(state_bodies)):
            raise ValueError("initial states must contain unique bodies")
        known_bodies = {self.vehicle.chassis.name}
        known_bodies.update(body.name for body in self.vehicle.front_axle.bodies)
        known_bodies.update(body.name for body in self.vehicle.rear_axle.bodies)
        known_bodies.update(
            f"front_{body.name}" for body in self.vehicle.front_axle.bodies
        )
        known_bodies.update(
            f"rear_{body.name}" for body in self.vehicle.rear_axle.bodies
        )
        known_bodies.update(wheel.body for wheel in self.vehicle.wheels)
        if any(name not in known_bodies for name in state_bodies):
            raise ValueError("initial states reference an undefined body")
        if not math.isfinite(self.initial_forward_speed_mps):
            raise ValueError("initial_forward_speed_mps must be finite")
        return self

    @model_validator(mode="after")
    def _direct_wheel_torque_signals(self) -> VehicleDynamicCase:
        names = {wheel.name for wheel in self.vehicle.wheels}

        def validate_signals(
            entries: tuple[tuple[str, TimeSignal], ...],
            label: str,
            nonnegative: bool,
        ) -> None:
            entry_names = [name for name, _ in entries]
            if any(name not in names for name in entry_names):
                raise ValueError(f"{label} references an undefined wheel")
            if len(entry_names) != len(set(entry_names)):
                raise ValueError(f"{label} must contain unique wheels")
            for name, signal in entries:
                values = (
                    (signal.constant,)
                    if signal.constant is not None
                    else signal.values
                )
                if nonnegative and any(value < 0.0 for value in values):
                    raise ValueError(f"{label}[{name!r}] must be non-negative")

        validate_signals(self.wheel_drive_torque, "wheel_drive_torque", False)
        validate_signals(self.wheel_brake_torque, "wheel_brake_torque", True)

        def is_zero(signal: TimeSignal) -> bool:
            values = (
                (signal.constant,)
                if signal.constant is not None
                else signal.values
            )
            return all(abs(value) <= 1e-12 for value in values)

        if self.wheel_drive_torque and not is_zero(self.drive_input):
            raise ValueError(
                "wheel_drive_torque cannot be combined with nonzero drive_input"
            )
        if self.wheel_brake_torque and not is_zero(self.brake_input):
            raise ValueError(
                "wheel_brake_torque cannot be combined with nonzero brake_input"
            )
        return self
