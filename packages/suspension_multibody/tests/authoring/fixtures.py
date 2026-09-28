"""A file-authored axle, written out once and used by the solver tests."""

from __future__ import annotations

import json
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
    from suspension_multibody.authoring.solver import template_document_from
    from suspension_multibody.templates import DOUBLE_WISHBONE

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
        "hardpoints": {role: list(point) for role, point in COORDINATES.items()},
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
