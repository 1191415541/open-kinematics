"""A file-authored axle, written out once and used by the solver tests."""

from __future__ import annotations

import json
import math
from pathlib import Path

#: The ten hardpoint roles a double-wishbone axle has, with the body that owns
#: each and the label it carries.  Written as data because the template declares
#: them and the subsystem places them; neither invents a name.
ROLES: dict[str, tuple[str, str]] = {
    "upper_front": ("upper_arm_L", "inner_front"),
    "upper_rear": ("upper_arm_L", "inner_rear"),
    "upper_outer": ("upper_arm_L", "outer"),
    "lower_front": ("lower_arm_L", "inner_front"),
    "lower_rear": ("lower_arm_L", "inner_rear"),
    "lower_outer": ("lower_arm_L", "outer"),
    "tie_inner": ("tie_rod_L", "inner"),
    "tie_outer": ("tie_rod_L", "outer"),
    "wheel_center": ("upright_L", "center"),
    "rack_center": ("rack", "center"),
}
#: Left-side coordinates, in the model's engineering units.
#:
#: The numbers are the ones the repository's own axle fixture uses, and that is a
#: correctness requirement rather than a convenience: a set of points that all
#: shared one (x, y) line would leave the upright's attachments collinear, and the
#: kernel reports such a model as a rank-deficient Jacobian rather than as a bad
#: coordinate choice.  Reusing a known-solvable set keeps the file route's
#: arithmetic comparable with the Python route's.
COORDINATES: dict[str, tuple[float, float, float]] = {
    "upper_front": (-100.0, -500.0, 400.0),
    "upper_rear": (100.0, -500.0, 400.0),
    "upper_outer": (0.0, -700.0, 450.0),
    "lower_front": (-120.0, -500.0, 150.0),
    "lower_rear": (120.0, -500.0, 150.0),
    "lower_outer": (0.0, -700.0, 150.0),
    "tie_inner": (100.0, -400.0, 250.0),
    "tie_outer": (50.0, -700.0, 250.0),
    "wheel_center": (0.0, -700.0, 300.0),
    "rack_center": (0.0, 0.0, 250.0),
}


def write_axle_project(root: Path, *, spring_stiffness: float = 45.0) -> dict[str, Path]:
    """
    Write a complete, minimal project: one template, its property files, a
    subsystem, an assembly and a rig.

    Everything a run needs is a file, which is the point of the exercise: the
    returned paths are the whole input to the file driven route.
    """
    # A caller may name a subdirectory of the test's temporary tree, so the
    # directory is the fixture's to create rather than the caller's to remember.
    root.mkdir(parents=True, exist_ok=True)
    template = {
        "document": "template",
        "schema_version": 1,
        "name": "file_double_wishbone",
        "functional_role": "suspension",
        "allowed_placement_roles": ["front", "rear"],
        "bodies": [
            {"name": "chassis", "fixed": True},
            {"name": "rack"},
            {"name": "upper_arm_L", "mass": 12.0},
            {"name": "lower_arm_L", "mass": 15.0},
            {"name": "upright_L", "mass": 20.0},
            {"name": "tie_rod_L", "mass": 1.5},
        ],
        "hardpoints": [
            {"name": role, "owner": owner, "label": label}
            for role, (owner, label) in ROLES.items()
        ],
        "joints": [
            # One joint per point, and the axis of a revolute is named by the
            # *other* inboard hardpoint rather than by a second endpoint: the arm
            # pivots on the line from its front point to its rear one.
            {
                "name": "upper_pivot",
                "type": "revolute",
                "body_a": "chassis",
                "body_b": "upper_arm_L",
                "point_a": "upper_front",
                "axis_reference": "upper_rear",
            },
            {
                "name": "lower_pivot",
                "type": "revolute",
                "body_a": "chassis",
                "body_b": "lower_arm_L",
                "point_a": "lower_front",
                "axis_reference": "lower_rear",
            },
            {
                "name": "upper_ball",
                "type": "spherical",
                "body_a": "upper_arm_L",
                "body_b": "upright_L",
                "point_a": "upper_outer",
            },
            {
                "name": "lower_ball",
                "type": "spherical",
                "body_a": "lower_arm_L",
                "body_b": "upright_L",
                "point_a": "lower_outer",
            },
            {
                "name": "tie_inboard",
                "type": "spherical",
                "body_a": "tie_rod_L",
                "body_b": "rack",
                "point_a": "tie_inner",
            },
            {
                "name": "tie_outboard",
                "type": "spherical",
                "body_a": "tie_rod_L",
                "body_b": "upright_L",
                "point_a": "tie_outer",
            },
            {
                "name": "rack_guide",
                "type": "prismatic",
                "body_a": "rack",
                "body_b": "chassis",
                "point_a": "rack_center",
            },
        ],
        "elements": [
            {
                "name": "spring",
                "type": "spring",
                "body_a": "chassis",
                "body_b": "lower_arm_L",
                "point_a": "upper_front",
                "point_b": "lower_outer",
                "property_slot": "spring",
            }
        ],
        "property_slots": [
            {
                "name": "spring",
                "element_type": "spring",
                "allowed_models": ["linear", "piecewise"],
                "required": True,
            }
        ],
        "ports": [
            {"name": "wheel_centre", "role": "wheel_centre", "owner": "upright_L"},
            {"name": "chassis_reference", "role": "chassis_reference", "owner": "chassis"},
        ],
    }
    spring = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "axle_spring",
        "element_type": "spring",
        "model": "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"stiffness": spring_stiffness, "free_length": 250.0},
    }
    subsystem = {
        "document": "subsystem",
        "schema_version": 1,
        "name": "front_suspension",
        "template": "suspension.tpl.json",
        "functional_role": "suspension",
        "placement_role": "front",
        "hardpoints": {role: list(point) for role, point in COORDINATES.items()},
        "property_bindings": {"spring": "spring.json"},
    }
    chassis_template = {
        "document": "template",
        "schema_version": 1,
        "name": "file_chassis",
        "functional_role": "chassis",
        "allowed_placement_roles": ["any"],
        "bodies": [{"name": "chassis", "fixed": True}],
        "hardpoints": [{"name": "chassis_reference", "owner": "chassis"}],
        "joints": [],
        "elements": [],
        "property_slots": [],
        "ports": [
            {"name": "chassis_reference", "role": "chassis_reference", "owner": "chassis"}
        ],
    }
    chassis_subsystem = {
        "document": "subsystem",
        "schema_version": 1,
        "name": "chassis",
        "template": "chassis.tpl.json",
        "functional_role": "chassis",
        "placement_role": "any",
        "hardpoints": {"chassis_reference": [0.0, 0.0, 0.0]},
        "property_bindings": {},
    }
    assembly = {
        "document": "assembly",
        "schema_version": 1,
        "name": "front_axle",
        "assembly_kind": "suspension_axle",
        "subsystems": [
            {
                "ref": "front.sub.json",
                "functional_role": "suspension",
                "placement_role": "front",
            },
            {
                "ref": "chassis.sub.json",
                "functional_role": "chassis",
                "placement_role": "any",
            },
        ],
        "rig": "rig.json",
    }
    rig = {
        "document": "rig",
        "schema_version": 1,
        "name": "kc_bench",
        "supported_assembly_kinds": ["suspension_axle"],
        "required_ports": ["wheel_centre"],
        "measurements": ["wheel_travel", "camber", "toe"],
    }

    written = {
        "template": root / "suspension.tpl.json",
        "spring": root / "spring.json",
        "subsystem": root / "front.sub.json",
        "chassis_template": root / "chassis.tpl.json",
        "chassis_subsystem": root / "chassis.sub.json",
        "assembly": root / "axle.asy.json",
        "rig": root / "rig.json",
    }
    for key, payload in (
        ("template", template),
        ("spring", spring),
        ("subsystem", subsystem),
        ("chassis_template", chassis_template),
        ("chassis_subsystem", chassis_subsystem),
        ("assembly", assembly),
        ("rig", rig),
    ):
        written[key].write_text(json.dumps(payload), encoding="utf-8")
    return written


def write_builtin_axle_project(root: Path) -> dict[str, Path]:
    """
    Write the built-in double wishbone out as a file project.

    The template is *exported* from the registered built-in rather than transcribed
    by hand, so this fixture cannot drift from the template it claims to describe:
    whatever the export loses shows up as a difference in the assembled model, which
    is what the round-trip test asserts.
    """
    from suspension_multibody.templates import DOUBLE_WISHBONE
    from suspension_multibody.templates.export import template_document_from

    template = template_document_from(DOUBLE_WISHBONE)
    spring = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "builtin_spring",
        "element_type": "spring",
        "model": "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"stiffness": 45.0, "free_length": 250.0},
    }
    damper = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "builtin_damper",
        "element_type": "damper",
        "model": "linear",
        "units": {"force": "N", "length": "mm", "time": "s"},
        "parameters": {"viscous_damping": 12.0},
    }
    subsystem = {
        "document": "subsystem",
        "schema_version": 1,
        "name": "builtin_front",
        "template": "suspension.tpl.json",
        "functional_role": "suspension",
        "placement_role": "front",
        # Only the roles the exported template declares: the rack centre belongs to
        # the steering template now (requirement 1 -- the suspension owns the tie rods
        # and the rack does not), and a subsystem may not place a hardpoint its own
        # template does not declare.
        "hardpoints": {
            role: list(point)
            for role, point in COORDINATES.items()
            if role in {row["name"] for row in template["hardpoints"]}
        },
        "property_bindings": {"spring": "spring.json", "damper": "damper.json"},
    }
    chassis_template = {
        "document": "template",
        "schema_version": 1,
        "name": "builtin_chassis",
        "functional_role": "chassis",
        "allowed_placement_roles": ["any"],
        "bodies": [{"name": "chassis", "fixed": True}],
        "hardpoints": [{"name": "chassis_reference", "owner": "chassis"}],
        "joints": [],
        "elements": [],
        "property_slots": [],
    }
    chassis_subsystem = {
        "document": "subsystem",
        "schema_version": 1,
        "name": "chassis",
        "template": "chassis.tpl.json",
        "functional_role": "chassis",
        "placement_role": "any",
        "hardpoints": {"chassis_reference": [0.0, 0.0, 0.0]},
        "property_bindings": {},
    }
    assembly = {
        "document": "assembly",
        "schema_version": 1,
        "name": "builtin_axle",
        "assembly_kind": "suspension_axle",
        "subsystems": [
            {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
            {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
        ],
        "rig": "rig.json",
    }
    rig = {
        "document": "rig",
        "schema_version": 1,
        "name": "kc_bench",
        "supported_assembly_kinds": ["suspension_axle"],
        "required_ports": [],
        "measurements": ["wheel_travel", "camber", "toe"],
    }
    written = {
        "template": root / "suspension.tpl.json",
        "spring": root / "spring.json",
        "damper": root / "damper.json",
        "subsystem": root / "front.sub.json",
        "chassis_template": root / "chassis.tpl.json",
        "chassis_subsystem": root / "chassis.sub.json",
        "assembly": root / "axle.asy.json",
        "rig": root / "rig.json",
    }
    for key, payload in (
        ("template", template),
        ("spring", spring),
        ("damper", damper),
        ("subsystem", subsystem),
        ("chassis_template", chassis_template),
        ("chassis_subsystem", chassis_subsystem),
        ("assembly", assembly),
        ("rig", rig),
    ):
        written[key].write_text(json.dumps(payload), encoding="utf-8")
    return written


def _role_template(
    role: str,
    name: str,
    *,
    bodies: list[dict[str, object]],
    hardpoints: list[dict[str, object]],
    joints: list[dict[str, object]],
    ports: list[dict[str, object]] = [],
    outputs: list[dict[str, object]] = [],
) -> dict[str, object]:
    """Write one minimal, self-consistent template for a single-instance role."""
    template: dict[str, object] = {
        "document": "template",
        "schema_version": 1,
        "name": name,
        "functional_role": role,
        "allowed_placement_roles": ["any"],
        "bodies": bodies,
        "hardpoints": hardpoints,
        "joints": joints,
        "elements": [],
        "property_slots": [],
    }
    if ports:
        template["ports"] = ports
    if outputs:
        template["outputs"] = outputs
    return template


def _role_subsystem(
    name: str,
    template: str,
    role: str,
    placement: str,
    hardpoints: dict[str, list[float]],
) -> dict[str, object]:
    """Write one subsystem file that places every hardpoint its template declares."""
    return {
        "document": "subsystem",
        "schema_version": 1,
        "name": name,
        "template": template,
        "functional_role": role,
        "placement_role": placement,
        "hardpoints": hardpoints,
        "property_bindings": {},
    }


def write_vehicle_project(root: Path) -> dict[str, Path]:
    """
    Write a complete *vehicle* project: two suspensions and the four other roles.

    The roles the package still implements in Python -- steering, wheel, brake,
    drive and the axle chassis -- are declared here as ordinary template and
    subsystem files, because what has to be shown is that a *user* can write them
    and can build a vehicle out of them.  Nothing here claims the composition
    builds a steering subsystem from these files: what the vehicle document decides
    is which of the six roles the vehicle carries, and that is the question this
    fixture exists to ask.
    """
    written = write_axle_project(root)
    # The rack carries a mass because a *vehicle* model requires one: its
    # validator asks every body a `symmetric_proxy` axle names to have a positive
    # mass, and a rack that weighs nothing is a body the equations of motion
    # cannot move.  The axle fixture leaves it at zero because the K/C readings
    # it is used for prescribe the rack rather than integrate it.
    template = json.loads(written["template"].read_text(encoding="utf-8"))
    for row in template["bodies"]:
        if row["name"] == "rack":
            row["mass"] = 8.0
    written["template"].write_text(json.dumps(template), encoding="utf-8")
    front = json.loads(written["subsystem"].read_text(encoding="utf-8"))

    # The rear suspension is the front one placed at the other end, which is what
    # the placement roles exist for: one template, two instances.
    rear = dict(front)
    rear["name"] = "rear_suspension"
    rear["placement_role"] = "rear"

    steering_template = _role_template(
        "steering",
        "file_steering",
        # The rack, its support, and the points the *suspension's* tie rods attach
        # to: a steering template owns no tie rod (requirement 1 -- the suspension
        # does), and the rack-side mount is what it declares for them.
        bodies=[
            {"name": "rack"},
            {"name": "support", "fixed": True},
        ],
        hardpoints=[
            {"name": "rack_center", "owner": "rack", "label": "center"},
            {"name": "tie_inner", "owner": "rack", "label": "tie_L"},
        ],
        joints=[
            {
                "name": "rack_guide",
                "type": "prismatic",
                "body_a": "rack",
                "body_b": "support",
                "point_a": "rack_center",
            },
        ],
        ports=[{"name": "rack_center", "role": "rack_center", "owner": "rack"}],
        outputs=[{"name": "rack_displacement", "unit": "mm"}],
    )
    steering_subsystem = _role_subsystem(
        "steering",
        "steering.tpl.json",
        "steering",
        "any",
        {
            "rack_center": [0.0, 0.0, 250.0],
            "tie_inner": [100.0, -400.0, 250.0],
        },
    )

    wheel_template = _role_template(
        "wheel",
        "file_wheel",
        bodies=[
            {"name": "wheel_L", "mass": 22.0},
            {"name": "wheel_carrier", "fixed": True},
        ],
        hardpoints=[
            {"name": "wheel_center", "owner": "wheel_L", "label": "center"},
            {"name": "spin_axis", "owner": "wheel_L", "label": "spin"},
        ],
        joints=[
            {
                "name": "wheel_spin",
                "type": "revolute",
                "body_a": "wheel_carrier",
                "body_b": "wheel_L",
                "point_a": "wheel_center",
            }
        ],
        ports=[{"name": "wheel_centre", "role": "wheel_centre", "owner": "wheel_L"}],
        outputs=[{"name": "wheel_center_pose", "unit": "mm"}],
    )
    wheel_subsystem = _role_subsystem(
        "wheel",
        "wheel.tpl.json",
        "wheel",
        "any",
        {"wheel_center": [0.0, -700.0, 300.0], "spin_axis": [0.0, -700.0, 300.0]},
    )

    # Brake and drive own no bodies in the package's own simplified form, but a
    # template's hardpoints have to be owned by a declared body, so each carries
    # the smallest honest one: the part the torque acts through.
    brake_template = _role_template(
        "brake",
        "file_brake",
        bodies=[{"name": "caliper_L", "mass": 3.0}],
        hardpoints=[
            {"name": "wheel_center", "owner": "caliper_L", "label": "center"},
            {"name": "spin_axis", "owner": "caliper_L", "label": "spin"},
        ],
        joints=[],
        outputs=[{"name": "brake_torque", "unit": "N*mm"}],
    )
    brake_subsystem = _role_subsystem(
        "brake",
        "brake.tpl.json",
        "brake",
        "any",
        {"wheel_center": [0.0, -700.0, 300.0], "spin_axis": [0.0, -700.0, 300.0]},
    )
    drive_template = _role_template(
        "drive",
        "file_drive",
        bodies=[{"name": "half_shaft_L", "mass": 4.0}],
        hardpoints=[
            {"name": "wheel_center", "owner": "half_shaft_L", "label": "center"},
            {"name": "spin_axis", "owner": "half_shaft_L", "label": "spin"},
        ],
        joints=[],
        outputs=[{"name": "drive_torque", "unit": "N*mm"}],
    )
    drive_subsystem = _role_subsystem(
        "drive",
        "drive.tpl.json",
        "drive",
        "any",
        {"wheel_center": [0.0, -700.0, 300.0], "spin_axis": [0.0, -700.0, 300.0]},
    )

    # The vehicle-level numbers, in the section the assembly schema calls
    # `vehicle`.  They are the ones no subsystem owns: the chassis body, the four
    # wheel ends, the steering system and the driveline.  The two axles are
    # deliberately absent -- they are the suspension subsystems above, and stating
    # them twice would be two descriptions of one axle.
    vehicle = {
        "chassis": {
            "name": "chassis",
            "mass": 1400.0,
            "center_of_mass": [0.0, 0.0, 500.0],
            "inertia": [
                [600.0, 0.0, 0.0],
                [0.0, 2400.0, 0.0],
                [0.0, 0.0, 2600.0],
            ],
        },
        "wheels": [
            {
                "name": corner,
                "body": f"wheel_{corner}",
                "center_local": [0.0, y, 300.0],
                "mass": 22.0,
                "axial_inertia": 1.2,
                # The kind is named because the vehicle solver supports a fixed
                # set of tire models: a wheel whose tire kind is unstated gets the
                # schema's default, which that solver refuses.
                "tire": {
                    "kind": "fiala",
                    "vertical_stiffness": 200.0,
                    "unloaded_radius": 300.0,
                },
            }
            for corner, y in (
                ("front_left", -700.0),
                ("front_right", 700.0),
                ("rear_left", -700.0),
                ("rear_right", 700.0),
            )
        ],
        "steering": {"rack_body": "rack", "ratio": 1.0},
        "driveline": {
            "driven_wheels": ["rear_left", "rear_right"],
            "maximum_drive_torque": 2000.0,
            "drive_split": [0.0, 0.0, 0.5, 0.5],
        },
    }
    assembly = {
        "document": "assembly",
        "schema_version": 1,
        "name": "full_vehicle",
        "assembly_kind": "full_vehicle",
        "subsystems": [
            {
                "ref": "front.sub.json",
                "functional_role": "suspension",
                "placement_role": "front",
            },
            {
                "ref": "rear.sub.json",
                "functional_role": "suspension",
                "placement_role": "rear",
            },
            {
                "ref": "chassis.sub.json",
                "functional_role": "chassis",
                "placement_role": "any",
            },
            {
                "ref": "wheel.sub.json",
                "functional_role": "wheel",
                "placement_role": "any",
            },
            {
                "ref": "steering.sub.json",
                "functional_role": "steering",
                "placement_role": "any",
            },
            {
                "ref": "brake.sub.json",
                "functional_role": "brake",
                "placement_role": "any",
            },
            {
                "ref": "drive.sub.json",
                "functional_role": "drive",
                "placement_role": "any",
            },
        ],
        "rig": "vehicle_rig.json",
        "vehicle": vehicle,
    }
    rig = {
        "document": "rig",
        "schema_version": 1,
        "name": "vehicle_kc",
        "supported_assembly_kinds": ["suspension_axle", "full_vehicle"],
        "required_ports": [],
        "bench": "vehicle_kc",
        "measurements": ["wheel_load", "rig_frame_pose"],
    }
    written.update(
        {
            "rear_subsystem": root / "rear.sub.json",
            "steering_template": root / "steering.tpl.json",
            "steering_subsystem": root / "steering.sub.json",
            "wheel_template": root / "wheel.tpl.json",
            "wheel_subsystem": root / "wheel.sub.json",
            "brake_template": root / "brake.tpl.json",
            "brake_subsystem": root / "brake.sub.json",
            "drive_template": root / "drive.tpl.json",
            "drive_subsystem": root / "drive.sub.json",
            "vehicle_assembly": root / "vehicle.asy.json",
            "vehicle_rig": root / "vehicle_rig.json",
        }
    )
    for key, payload in (
        ("rear_subsystem", rear),
        ("steering_template", steering_template),
        ("steering_subsystem", steering_subsystem),
        ("wheel_template", wheel_template),
        ("wheel_subsystem", wheel_subsystem),
        ("brake_template", brake_template),
        ("brake_subsystem", brake_subsystem),
        ("drive_template", drive_template),
        ("drive_subsystem", drive_subsystem),
        ("vehicle_assembly", assembly),
        ("vehicle_rig", rig),
    ):
        written[key].write_text(json.dumps(payload), encoding="utf-8")
    return written


#: The slot the C-ready fixture binds its mount bushing to.  Deliberately *not*
#: named ``bushing``: a slot's name is the author's, and the route that feeds a
#: mount has to find the slot from the element declaration rather than from the
#: built-in template's spelling.  Were the name load-bearing, this fixture would
#: build a zero-stiffness mechanism instead of a compliant axle.
MOUNT_SLOT = "mount"
#: The same, for the tire's law.
TIRE_SLOT = "tire"
#: The six-axis mount the C snapshot baselines were solved with: stiff in
#: translation, stiffer in rotation.  Stated here rather than imported so the
#: fixture cannot drift from what the file is supposed to describe.
MOUNT_MATRIX: tuple[tuple[float, ...], ...] = tuple(
    tuple(
        10_000.0 if row == column and row < 3
        else 10_000_000.0 if row == column
        else 0.0
        for column in range(6)
    )
    for row in range(6)
)
#: The tire's two numbers, in the same units the property files declare.
TIRE_STIFFNESS = 200.0
TIRE_RADIUS = 300.0


def write_c_ready_axle_project(root: Path) -> dict[str, Path]:
    """
    Write the axle project in the shape the C reading can actually solve.

    Three things the K reading does not need, and that the minimal project
    therefore does not carry, are what a C run needs:

    * the mount bushings, because C mode turns the four inboard joints into
      compliance.  The file states the whole six-axis table: the rotational
      diagonals a single number cannot express are the difference between a
      compliant axle and a mechanism;
    * a tire at the wheel centre, because that is the vertical support the pad
      reading loads, and without one the load has nothing to push against;
    * the loaded marker's *label*, because the case addresses the load by label,
      and a hardpoint labelled after its role is a marker with another name.

    The spring is assembled at its free length, and that is a property of the
    reading rather than a convenience of this fixture: the pad reading's static
    trim gives up on any spring whose assembled length differs from its free one,
    and it does so *identically* for the built-in axle.  Measured, the built-in
    benchmark with the same mounts, tire and spring reports the same success at
    zero preload and the same failure -- `iterations=2`, `force_residual=0.290426`,
    `position_residual=0.000140` at 5 mm -- as this project does.  So the numbers
    below are the reading's, not the file route's, and the route is what the
    equality demonstrates.
    """
    written = write_axle_project(root)
    template = json.loads(written["template"].read_text(encoding="utf-8"))
    subsystem = json.loads(written["subsystem"].read_text(encoding="utf-8"))
    spring = json.loads(written["spring"].read_text(encoding="utf-8"))

    for row in template["hardpoints"]:
        if row["name"] == "wheel_center":
            row["label"] = "wheel_center"

    for role in ("upper_front", "upper_rear", "lower_front", "lower_rear"):
        template["elements"].append(
            {
                "name": f"mount_{role}",
                "type": "bushing",
                "body_a": "chassis",
                "body_b": ROLES[role][0],
                "point_a": role,
                "property_slot": MOUNT_SLOT,
            }
        )
    template["elements"].append(
        {
            "name": "tire",
            "type": "tire",
            "body_a": "upright_L",
            "body_b": "upright_L",
            "point_a": "wheel_center",
            "property_slot": TIRE_SLOT,
        }
    )
    template["property_slots"].extend(
        [
            {
                "name": MOUNT_SLOT,
                "element_type": "bushing",
                "allowed_models": ["linear"],
                "required": True,
            },
            {
                "name": TIRE_SLOT,
                "element_type": "tire",
                "allowed_models": ["linear"],
                "required": True,
            },
        ]
    )
    spring["parameters"]["free_length"] = math.dist(
        COORDINATES["upper_front"], COORDINATES["lower_outer"]
    )
    subsystem["property_bindings"][MOUNT_SLOT] = "mount.json"
    subsystem["property_bindings"][TIRE_SLOT] = "tire.json"

    mount = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "axle_mount",
        "element_type": "bushing",
        "model": "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {},
        "matrix": {"name": "stiffness", "rows": [list(row) for row in MOUNT_MATRIX]},
    }
    tire = {
        "document": "element_properties",
        "schema_version": 1,
        "name": "axle_tire",
        "element_type": "tire",
        "model": "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"stiffness": TIRE_STIFFNESS, "unloaded_radius": TIRE_RADIUS},
    }

    written["template"].write_text(json.dumps(template), encoding="utf-8")
    written["subsystem"].write_text(json.dumps(subsystem), encoding="utf-8")
    written["spring"].write_text(json.dumps(spring), encoding="utf-8")
    written["mount"] = root / "mount.json"
    written["tire"] = root / "tire.json"
    written["mount"].write_text(json.dumps(mount), encoding="utf-8")
    written["tire"].write_text(json.dumps(tire), encoding="utf-8")
    return written
