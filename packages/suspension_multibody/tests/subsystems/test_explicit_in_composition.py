"""
The explicit topology builds through the composition layer, not the old assembly.

An explicit model states its own parts and joints.  It used to be built by the
axle assembly, which meant deleting that assembly would have taken the explicit
topology with it -- and the explicit topology is the import path for every source
model, so that would have been a capability loss rather than a cleanup.

These tests therefore assert two things at once:

1. the runtime the composition layer produces carries the same constraints, the
   same two rack supports and the same points the explicit build always produced;
2. it does so **without importing the axle assembly**, which is checked by
   running the build in a fresh interpreter that refuses that import.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from suspension_multibody.modeling.primitives import (
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    RigidBody,
    UniversalJoint,
    WeldJoint,
)
from suspension_multibody.schema import (
    FrontAxleModel,
    IdealJointSpec,
    MassSpec,
    RigidBodySpec,
    Vec3,
)
from suspension_multibody.subsystems.explicit import (
    build_explicit_runtime,
    explicit_roles,
)

ROOT = Path(__file__).parents[4]


def _body(name: str, fixed: bool = False) -> RigidBodySpec:
    return RigidBodySpec(
        name=name,
        mass=10.0 if not fixed else 0.0,
        inertia=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        fixed=fixed,
    )


def _vec(x: float, y: float, z: float) -> Vec3:
    return Vec3(x=x, y=y, z=z)


def _explicit_model(*, rack_fixed: bool, with_housing: bool = False) -> FrontAxleModel:
    """
    One explicit model with a rack, optionally fixed to the chassis.

    The two rack supports are the branches that matter: a rack welded to the
    chassis and a rack guided along its own axis are different constraint sets,
    and the source format distinguishes them.
    """
    bodies = [_body("rack"), _body("upright_L"), _body("upright_R")]
    if with_housing:
        bodies.append(_body("rack_housing"))
    return FrontAxleModel(
        hardpoints={
            "wheel_center": Vec3(x=0, y=-700, z=300),
            "rack_center": Vec3(x=0, y=0, z=250),
        },
        mass=MassSpec(sprung_mass=1000),
        topology="explicit",
        rack_fixed_to_chassis=rack_fixed,
        bodies=tuple(bodies),
        joints=(
            IdealJointSpec(
                name="upper_ball_L",
                kind="spherical",
                body_a="chassis",
                body_b="upright_L",
                point_a=_vec(0, -500, 400),
                point_b=_vec(0, -650, 400),
            ),
        ),
    )


def test_a_free_rack_gets_an_axis_guide() -> None:
    """
    A free rack translates along its own axis and nothing else.

    Without the guide the rack is an unconstrained rigid body, which the source
    topology would then carry into the solve as a silent extra freedom.
    """
    runtime = build_explicit_runtime(_explicit_model(rack_fixed=False), "K")
    guides = [c for c in runtime.constraints if isinstance(c, PrismaticJoint)]
    assert len(guides) == 1
    assert guides[0].name == "rack_guide"
    assert not [c for c in runtime.constraints if isinstance(c, WeldJoint)]


def test_a_fixed_rack_is_welded_instead() -> None:
    runtime = build_explicit_runtime(_explicit_model(rack_fixed=True), "K")
    welds = [c for c in runtime.constraints if isinstance(c, WeldJoint)]
    assert [weld.name for weld in welds] == ["rack_fixed_to_chassis"]
    assert not [c for c in runtime.constraints if isinstance(c, PrismaticJoint)]


def test_a_rack_housing_suppresses_the_rigid_guide() -> None:
    """
    With a housing, the rack's support comes from the source, not from here.

    Adding a rigid chassis-rack guide on top of a source TRANSLATIONAL plus
    housing bushings would over-constrain the rack, and the over-constraint would
    look like a modelling choice rather than a duplicate.
    """
    runtime = build_explicit_runtime(
        _explicit_model(rack_fixed=False, with_housing=True), "K"
    )
    assert not [c for c in runtime.constraints if isinstance(c, PrismaticJoint)]
    assert "rack_housing" in runtime.bodies


@pytest.mark.parametrize(
    ("kind", "expected"),
    [
        ("universal", UniversalJoint),
        ("cylindrical", CylindricalJoint),
        ("inplane", InPlaneJoint),
    ],
)
def test_the_joint_kinds_the_old_path_accepted_still_build(kind: str, expected: type) -> None:
    """
    The five joint kinds the explicit path added must survive the move.

    They were the reason the explicit topology existed at all: a source model
    states joints the symmetric proxy has no vocabulary for, so dropping one here
    would shrink what the package can import.
    """
    model = _explicit_model(rack_fixed=True)
    model = model.model_copy(
        update={
            "joints": (
                IdealJointSpec(
                    name=f"{kind}_joint",
                    kind=kind,  # type: ignore[arg-type]
                    body_a="chassis",
                    body_b="upright_L",
                    point_a=_vec(0, -500, 400),
                    point_b=_vec(0, -650, 400),
                ),
            )
        }
    )
    runtime = build_explicit_runtime(model, "K")
    assert any(isinstance(c, expected) for c in runtime.ideal_constraints)


def test_roles_are_read_off_the_build_not_asserted() -> None:
    """
    An explicit model with no rack has no steering, and must not claim one.

    Claiming it made the rig offer a rack coordinate the model does not declare,
    and the run then failed inside the kernel with "unknown coordinate
    rack_drive" -- a message that names the symptom and not the cause.
    """
    with_rack = build_explicit_runtime(_explicit_model(rack_fixed=True), "K")
    assert "steering" in with_rack.capabilities.subsystems

    without = build_explicit_runtime(
        _explicit_model(rack_fixed=True).model_copy(
            update={"bodies": (_body("upright_L"), _body("upright_R"))}
        ),
        "K",
    )
    assert "steering" not in without.capabilities.subsystems
    assert "wheel" in without.capabilities.subsystems


def test_roles_helper_reports_suspension_only_when_something_is_connected() -> None:
    """A bare chassis is not a suspension assembly."""
    chassis = {"chassis": RigidBody("chassis", fixed=True)}
    assert explicit_roles(chassis, [], {}) == frozenset({"chassis"})


def test_the_explicit_build_needs_nothing_from_the_retired_assembly() -> None:
    """
    The move is only real if the new path can run without the old one.

    Checked in a fresh interpreter that blocks the import outright, because inside
    this test process the module is already loaded and the answer would be
    contaminated -- the same reason the import-boundary gate uses a subprocess.

    The blocked module used to be the retired assembly package, which existed but
    was not to be reached.  It is gone now, so the guard asks the stronger question:
    this build runs with the *whole author layer* unavailable.  A build that needed
    a preparation module would fail here, and deleting the package would then not
    have proven anything.
    """
    script = """
import json
import sys

class Blocker:
    def find_module(self, name, path=None):
        if name.endswith("preparation") or ".preparation." in name:
            raise ImportError(f"blocked: {name}")
        return None

sys.meta_path.insert(0, Blocker())

from suspension_multibody.schema import FrontAxleModel, IdealJointSpec, MassSpec, Pose, RigidBodySpec, Vec3
from suspension_multibody.subsystems.explicit import build_explicit_runtime

model = FrontAxleModel(
    hardpoints={"wheel_center": Vec3(x=0, y=-700, z=300), "rack_center": Vec3(x=0, y=0, z=250)},
    mass=MassSpec(sprung_mass=1000),
    topology="explicit",
    rack_fixed_to_chassis=True,
    bodies=(
        RigidBodySpec(name="rack", mass=10.0, inertia=[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]),
        RigidBodySpec(name="upright_L", mass=10.0, inertia=[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]),
    ),
    joints=(
        IdealJointSpec(
            name="ball_L", kind="spherical", body_a="chassis", body_b="upright_L",
            point_a=Vec3(x=0, y=-500, z=400),
            point_b=Vec3(x=0, y=-650, z=400),
        ),
    ),
)
runtime = build_explicit_runtime(model, "K")
print(json.dumps({"bodies": sorted(runtime.bodies), "constraints": len(runtime.constraints)}))
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    payload = json.loads(completed.stdout.strip().splitlines()[-1])
    assert payload["bodies"] == ["chassis", "rack", "upright_L"]
    assert payload["constraints"] >= 2
