"""Ordinary template data, independent of assembly purpose and runtime classes."""

from __future__ import annotations

from typing import Any

from ..authoring.documents import TemplateDocument


def _document(name: str, role: str, **rows: Any) -> dict[str, Any]:
    return {
        "document": "template", "schema_version": 1, "name": name,
        "functional_role": role, "allowed_placement_roles": ["any", "front", "rear"],
        "symmetry": "asymmetric", "units": {"length": "m"},
        "bodies": [], "hardpoints": [], "joints": [], "elements": [],
        "property_slots": [], **rows,
    }


_SUSPENSION = _document(
    "double_wishbone_generic", "suspension", symmetry="mirrored_xz",
    bodies=[
        {"name": name, "mass_slot": name + "_mass", "inertia_slot": name + "_inertia"}
        for name in ("upper_arm", "lower_arm", "upright", "tie_rod")
    ],
    hardpoints=[{"name": point, "owner": owner} for point, owner in (
        ("upper_front", "upper_arm"), ("upper_rear", "upper_arm"),
        ("upper_outer", "upper_arm"), ("lower_front", "lower_arm"),
        ("lower_rear", "lower_arm"), ("lower_outer", "lower_arm"),
        ("tie_inner", "tie_rod"), ("tie_outer", "tie_rod"),
        ("wheel_center", "upright"), ("spring_lower", "lower_arm"),
        ("spring_upper", ""),
    )],
    joints=[
        {"name": arm + "_pivot", "type": "revolute", "body_a": "@support",
         "body_b": arm + "_arm", "point_a": arm + "_front",
         "point_b": arm + "_front", "axis_reference": arm + "_rear", "modes": ["K"]}
        for arm in ("upper", "lower")
    ] + [
        {"name": arm + "_outer", "type": "spherical", "body_a": arm + "_arm",
         "body_b": "upright", "point_a": arm + "_outer", "point_b": arm + "_outer"}
        for arm in ("upper", "lower")
    ] + [
        {"name": "tie_inner", "type": "spherical", "body_a": "@rack",
         "body_b": "tie_rod", "point_a": "tie_inner", "point_b": "tie_inner"},
        {"name": "tie_outer", "type": "spherical", "body_a": "tie_rod",
         "body_b": "upright", "point_a": "tie_outer", "point_b": "tie_outer"},
    ],
    elements=[
        {"name": arm + "_bushing_" + end, "type": "bushing", "body_a": "@support",
         "body_b": arm + "_arm", "point_a": arm + "_" + end,
         "property_slot": "mount_bushing", "modes": ["C"]}
        for arm in ("upper", "lower") for end in ("front", "rear")
    ] + [{"name": kind, "type": kind, "body_a": "@support", "body_b": "lower_arm",
          "point_a": "spring_upper", "point_b": "spring_lower", "property_slot": kind}
         for kind in ("spring", "damper")],
    property_slots=[
        {"name": name + "_" + kind, "element_type": kind, "required": True}
        for name in ("upper_arm", "lower_arm", "upright", "tie_rod")
        for kind in ("mass", "inertia")
    ] + [{"name": "mount_bushing", "element_type": "bushing", "required": False, "default": 0}]
    + [{"name": kind, "element_type": kind, "required": True} for kind in ("spring", "damper")],
    markers=[{"name": "carrier", "owner": "upright", "point": "wheel_center"}],
    ports=[
        {"name": "carrier", "role": "wheel_carrier", "marker": "carrier",
         "cardinality": "many", "capabilities": ["mount", "contact_frame", "torque_reaction"]},
        {"name": "arb_mount", "role": "arb_mount", "owner": "lower_arm", "point": "lower_outer"},
    ],
    needs=[{"role": role, "count": 1, "required": True} for role in ("support", "rack")],
)

_STEERING = _document(
    "steering_rack_generic", "steering",
    configurations=["guided", "fixed"], default_configuration="guided",
    bodies=[{"name": name, "mass_slot": name + "_mass", "inertia_slot": name + "_inertia"}
            for name in ("rack", "housing")],
    hardpoints=[{"name": "center", "owner": "rack"}, {"name": "housing_center", "owner": "housing"}],
    joints=[
        {"name": "guide", "type": "prismatic", "body_a": "housing", "body_b": "rack",
         "point_a": "housing_center", "point_b": "center", "axis": [0, 1, 0], "configurations": ["guided"]},
        {"name": "rack_lock", "type": "fixed", "body_a": "housing", "body_b": "rack",
         "point_a": "housing_center", "point_b": "center", "configurations": ["fixed"]},
        {"name": "housing_mount", "type": "fixed", "body_a": "@support", "body_b": "housing",
         "point_a": "housing_center", "point_b": "housing_center"},
    ],
    ports=[{"name": "rack", "role": "rack", "owner": "rack", "point": "center", "cardinality": "many"}],
    needs=[{"role": "support", "count": 1, "required": True}],
    property_slots=[{"name": name + "_" + kind, "element_type": kind, "required": True}
                    for name in ("rack", "housing") for kind in ("mass", "inertia")],
)

_BODY = _document(
    "rigid_body_generic", "chassis",
    bodies=[{"name": "body", "mass_slot": "mass", "inertia_slot": "inertia"}],
    hardpoints=[{"name": "reference", "owner": "body"}],
    ports=[{"name": "support", "role": "support", "owner": "body", "point": "reference", "cardinality": "many"}],
    property_slots=[{"name": kind, "element_type": kind, "required": True} for kind in ("mass", "inertia")],
)

_WHEEL = _document(
    "wheel_generic", "wheel",
    bodies=[{"name": "wheel", "mass_slot": "mass", "inertia_slot": "inertia", "position_point": "center"}],
    hardpoints=[{"name": "center", "owner": "wheel"}],
    markers=[{"name": "center", "owner": "wheel", "point": "center"},
             {"name": "spin", "owner": "wheel", "point": "center"}],
    joints=[{"name": "bearing", "type": "revolute", "body_a": "@carrier", "body_b": "wheel",
             "point_a": "@carrier", "point_b": "center", "axis": [0, 1, 0]}],
    coordinates=[{"name": "spin", "joint": "bearing", "kind": "rotation", "reference": 0}],
    ports=[{"name": "hub", "role": "wheel_hub", "marker": "center", "cardinality": "many"},
           {"name": "spin", "role": "wheel_spin", "kind": "spin", "coordinate": "spin", "owner": "wheel", "cardinality": "many"}],
    needs=[
        {"name": "carrier", "role": "wheel_carrier", "count": 1, "required": True,
         "requires_capabilities": ["mount", "contact_frame"]},
        {"role": "road", "count": 1, "required": True},
    ],
    property_slots=[{"name": kind, "element_type": kind, "required": True} for kind in ("mass", "inertia")]
    + [{"name": "tire", "element_type": "tire", "required": True,
        "allowed_models": ["pac2002", "fiala", "native_brush"]}],
    tires=[{"name": "tire", "body": {"name": "wheel", "center_marker": "center"},
            "model_slot": "tire", "mount_port": "hub", "spin_marker": "spin",
            "contact_frame": {"port": "@carrier"}, "road_port": "@road"}],
)

_DRIVE = _document(
    "drive_generic", "drive",
    needs=[{"name": name, "role": role, "count": 1, "required": True}
           for name, role in (("action", "wheel_hub"), ("reaction", "wheel_carrier"))],
    elements=[{"name": "drive", "type": "torque", "action": "@action", "reaction": "@reaction",
               "reference": "@reaction", "axis": [0, 1, 0],
               "function": "max_torque * gear_ratio * efficiency * share * input",
               "bindings": {
                   "max_torque": {"source": "property", "slot": "max_torque", "unit": "Nm"},
                   "gear_ratio": {"source": "property", "slot": "gear_ratio", "unit": "1"},
                   "efficiency": {"source": "property", "slot": "efficiency", "unit": "1"},
                   "share": {"source": "property", "slot": "share", "unit": "1"},
                   "input": {"source": "signal", "signal": "drive_input", "unit": "1"},
               }}],
    property_slots=[{"name": name, "element_type": "generic", "required": False, "default": value}
                    for name, value in (("max_torque", 0), ("gear_ratio", 1), ("efficiency", 1), ("share", 1))],
)

_BRAKE = _document(
    "brake_generic", "brake", needs=_DRIVE["needs"],
    elements=[{"name": "brake", "type": "rotational_torque", "body_a": "@reaction", "body_b": "@action",
               "point_a": "@reaction", "point_b": "@action", "property_slot": "amplitude",
               "parameters": {"axis_a": [0, 1, 0]},
               "parameter_expressions": {key: {
                   "function": "2 * piston_area * share * input * demand_scale * friction_coeff * effective_radius",
                   "unit": "N*mm",
                   "slots": {name: name for name in ("piston_area", "share", "input", "demand_scale", "friction_coeff", "effective_radius")}}
                   for key in ("stiffness", "max_torque")}}],
    property_slots=[{"name": name, "element_type": "generic", "required": False, "default": value}
                    for name, value in (("piston_area", 2500), ("effective_radius", 145), ("friction_coeff", .4),
                                        ("share", 1), ("input", 1), ("demand_scale", .1), ("amplitude", 0))],
)

_ANTI_ROLL = _document(
    "anti_roll_bar_generic", "anti_roll_bar",
    bodies=[{"name": name, "mass_slot": name + "_mass", "inertia_slot": name + "_inertia"}
            for name in ("bar_left", "bar_right", "link_left", "link_right")],
    hardpoints=[{"name": name, "owner": owner} for name, owner in
                (("pivot_left", "bar_left"), ("pivot_right", "bar_right"),
                 ("link_left", "link_left"), ("link_right", "link_right"))],
    joints=[{"name": "pivot_" + side, "type": "revolute", "body_a": "@support", "body_b": "bar_" + side,
             "point_a": "pivot_" + side, "point_b": "pivot_" + side, "axis": [0, 1, 0]}
            for side in ("left", "right")]
    + [{"name": "link_" + side, "type": "spherical", "body_a": "bar_" + side, "body_b": "link_" + side,
        "point_a": "link_" + side, "point_b": "link_" + side} for side in ("left", "right")],
    markers=[{"name": name, "owner": "bar_" + side, "point": "pivot_" + side}
             for name, side in (("left", "left"), ("right", "right"))],
    elements=[{"name": "torsion", "type": "anti_roll_bar", "body_a": "bar_left", "body_b": "bar_right",
               "point_a": "link_left", "point_b": "link_right", "property_slot": "rate",
               "parameters": {"axis_a": [0, 1, 0]}}],
    ports=[{"name": "droplink_" + side, "role": "arb_mount", "owner": "link_" + side, "point": "link_" + side,
            "labels": [label]} for side, label in (("left", "L"), ("right", "R"))],
    needs=[{"role": "support", "count": 1, "required": True}],
    property_slots=[{"name": name + "_" + kind, "element_type": kind, "required": True}
                    for name in ("bar_left", "bar_right", "link_left", "link_right") for kind in ("mass", "inertia")]
    + [{"name": "rate", "element_type": "generic", "required": False, "default": 0}],
)

_TEMPLATES = {"suspension": _SUSPENSION, "steering": _STEERING, "body": _BODY,
              "wheel": _WHEEL, "brake": _BRAKE, "drive": _DRIVE, "anti_roll_bar": _ANTI_ROLL}


def generic_template(name: str) -> TemplateDocument:
    """Return editable ordinary data for one named topology."""
    return TemplateDocument.from_payload(_TEMPLATES[name])
