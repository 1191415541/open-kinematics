"""
Physical rig templates: a bench that really contributes entities.

A :class:`~suspension_multibody.rigs.rig.RigSpec` says what a bench *drives*.  That
is enough to shrink an interface, and not enough to build a model: the previous
state of the architecture stopped there, so "the rig is a subsystem too" was a
statement in a document rather than something the kernel ever saw.  This module
adds the missing half -- the bodies, joints, motions and force elements a bench
actually contributes -- and it does so on the same template footing the subsystems
use, so a bench is not a second kind of thing.

The two shapes are decided by a declared *capability*, never by the name of the
assembly or of the suspension template:

``wheel_supplying``
    A single-axle bench owns the wheel: a carrier body and a tire per side, the
    travel motion that moves them, and the tire force element.  The assembly
    deliberately builds no wheel body (``D9``), so somebody has to, and the bench
    is the party that has a reason to care where the contact patch is.

``vehicle_loading``
    A full-vehicle bench must **not** create wheels: the vehicle already owns
    them.  It contributes only the loading actuators and the road motions that
    perturb the vehicle from outside.  Creating a second wheel would double a
    mass and a force path that the global rules say belong to the assembly.

The same template therefore adapts to both root categories by the branch it takes,
which is what ``G3`` asks for and what a registration-name rename could never
provide.
"""

from __future__ import annotations

from ..modeling.assembly import Assembly
from ..modeling.identity import EntityId
from ..modeling.instance import FragmentProvenance, ModelFragment
from ..modeling.ports import GeometryPort, PortRequirement
from ..templates.model import PartDefinition
from ..templates.ports import PortNeed, need_to_requirement
from .rig import RIGS, RigError, RigSpec

__all__ = [
    "BENCH_CAPABILITIES",
    "BENCH_TEMPLATE_NAME",
    "BenchCapability",
    "bench_capability",
    "build_rig_assembly",
    "build_rig_fragment",
    "rig_ports",
]

#: The template name every bench is built from.  One template, two branches: a
#: bench is a rig, and the difference between a four-post and a K&C bench is its
#: *content*, not its class.
BENCH_TEMPLATE_NAME = "bench"

#: The capabilities a bench template can declare.
#:
#: ``wheel_supplying`` is what the single-axle bench declares and what the global
#: rules require of it; ``vehicle_loading`` is the other branch.  They are
#: capabilities rather than kinds because they decide *what the template emits*,
#: and a template that emitted wheels for a vehicle would be wrong regardless of
#: what it was called.
BenchCapability = str

BENCH_CAPABILITIES: dict[str, BenchCapability] = {
    "wheel_supplying": "the bench owns the wheels and loads the axle through them",
    "vehicle_loading": "the vehicle owns its wheels; the bench only loads it",
}

#: Which capability each shipped bench declares.
#:
#: Derived from ``RigSpec.supplies_wheels`` rather than duplicated, so a bench
#: cannot claim one capability in its drive list and another here.
_BENCH_CAPABILITY_BY_NAME: dict[str, BenchCapability] = {
    name: ("wheel_supplying" if spec.supplies_wheels else "vehicle_loading")
    for name, spec in RIGS.items()
}

#: The sides a symmetric bench builds.
_SIDES: tuple[tuple[str, str], ...] = (("L", "left"), ("R", "right"))


def bench_capability(rig: str | RigSpec) -> BenchCapability:
    """Return the capability a bench declares, naming an unknown one if needed."""
    name = rig if isinstance(rig, str) else rig.name
    try:
        return _BENCH_CAPABILITY_BY_NAME[str(name).strip().lower()]
    except KeyError as exc:
        known = ", ".join(sorted(_BENCH_CAPABILITY_BY_NAME))
        raise RigError(f"unknown bench {name!r}; the registered benches are {known}") from exc


def _wheel_parts() -> tuple[PartDefinition, ...]:
    """Return the wheel bodies a wheel-supplying bench owns, one per side."""
    return tuple(
        PartDefinition(name=f"wheel_carrier_{side}")
        for side, _ in _SIDES
    )


def _loading_parts() -> tuple[PartDefinition, ...]:
    """
    Return the loading furniture a vehicle bench contributes.

    Deliberately *not* wheels: the vehicle owns those.  What a bench adds is the
    actuator frame it pushes through, which is a body the vehicle does not have.
    """
    return (PartDefinition(name="bench_frame", fixed=True),)


def build_rig_fragment(
    rig: str | RigSpec,
    *,
    capabilities: object | None = None,
    mode: str = "K",
    instance: tuple[str, ...] = (),
) -> ModelFragment:
    """
    Build the entities a bench contributes, for one binding.

    ``capabilities`` is the *assembly's* report, and it is what decides the
    branch: a bench that owns wheels emits a wheel body per side, and a bench
    that does not emits none.  Passing the assembly's capabilities rather than
    its name is the whole point -- the branch is a consequence of what the
    assembly can do, so a new assembly type needs no new bench code.

    ``mode`` is accepted for symmetry with the subsystem templates and does not
    change the bench's content: a bench's geometry is the same in K and C.  It is
    passed rather than ignored so a caller cannot forget it and get a silently
    different bench later.
    """
    if mode not in ("K", "C"):
        raise RigError(f"unknown mode {mode!r}; modes are K and C")
    spec = RIGS[_rig_name(rig)] if not isinstance(rig, RigSpec) else rig
    capability = bench_capability(spec)
    supplies_wheels = capability == "wheel_supplying"

    bodies: dict[str, object] = {}
    points: dict[tuple[str, str], object] = {}
    joints: dict[str, object] = {}
    drives: dict[str, object] = {}
    forces: dict[str, object] = {}
    tires: dict[str, object] = {}

    for part in _loading_parts():
        bodies[part.name] = {"name": part.name, "fixed": part.fixed}

    if supplies_wheels:
        for part in _wheel_parts():
            bodies[part.name] = {"name": part.name, "fixed": part.fixed}
            side = part.name.rsplit("_", 1)[-1]
            # The carrier carries the contact point its tire acts at, and the
            # travel motion is what the bench moves instead of the road.
            points[(part.name, "contact")] = {"role": "contact_patch"}
            joints[f"carrier_{side}"] = {
                "kind": "prismatic",
                "body": part.name,
                "point": "contact",
            }
            tires[part.name] = {"role": "tire", "owner": part.name}
            forces[f"tire_force_{side}"] = {
                "kind": "tire",
                "body": part.name,
                "point": "contact",
            }
            drives[f"travel_{side}"] = {
                "kind": "displacement",
                "body": part.name,
                "coordinate": f"wheel_drive_{side}",
            }

    for drive in spec.drives:
        # The bench's own inputs become motions it owns.  A coordinate the
        # assembly was supposed to provide is not one of these: it is a request
        # the interface resolver deals with, and emitting a motion for it here
        # would let a bench drive something the assembly never offered.
        if drive.from_assembly:
            continue
        motion_name = f"motion_{drive.coordinate}"
        drives[motion_name] = {
            "kind": drive.kind,
            "coordinate": drive.coordinate,
            "base": "bench_frame" if "bench_frame" in bodies else None,
        }

    return ModelFragment(
        bodies=bodies,
        points=points,
        joints=joints,
        drives=drives,
        forces=forces,
        tires=tires,
        outputs={name: {"name": name, "source": "rig"} for name in spec.outputs},
        provenance=FragmentProvenance(
            template=BENCH_TEMPLATE_NAME,
            revision=capability,
            instance=instance,
            note=f"bench {spec.name!r}",
        ),
    ).mounted(instance)


def rig_ports(
    rig: str | RigSpec, *, instance: tuple[str, ...] = ()
) -> dict[str, GeometryPort]:
    """
    Return the ports a bench offers.

    A wheel-supplying bench *offers* a wheel centre per side: the assembly needs
    one and cannot build it.  A vehicle-loading bench offers the mounting it can
    be attached by instead, and needs nothing from the vehicle's wheels -- it
    pushes on the body.  Which of the two it is comes from the same capability
    that decided the entities, so the two cannot disagree.
    """
    spec = RIGS[_rig_name(rig)] if not isinstance(rig, RigSpec) else rig
    supplies_wheels = bench_capability(spec) == "wheel_supplying"
    ports: dict[str, GeometryPort] = {}

    if supplies_wheels:
        for side, label in _SIDES:
            name = f"wheel_centre_{side}"
            ports[name] = GeometryPort(
                id=EntityId(instance, name),
                owner=EntityId(instance, f"wheel_carrier_{side}"),
                role="wheel_centre",
                capabilities=frozenset({"wheel", "load"}),
                labels=frozenset({side}),
            )
        return ports

    ports["mount"] = GeometryPort(
        id=EntityId(instance, "mount"),
        owner=EntityId(instance, "bench_frame"),
        role="body_mount",
        capabilities=frozenset({"load"}),
    )
    return ports


def _rig_name(rig: str | RigSpec) -> str:
    """Return a bench's registry key, refusing one that is not registered."""
    name = rig if isinstance(rig, str) else rig.name
    key = str(name).strip().lower()
    if key not in RIGS:
        known = ", ".join(sorted(RIGS))
        raise RigError(f"unknown rig {name!r}; the registered rigs are {known}")
    return key


def build_rig_assembly(
    rig: str | RigSpec,
    *,
    capabilities: object | None = None,
    mode: str = "K",
    instance: tuple[str, ...] | None = None,
) -> Assembly:
    """
    Wrap a bench's fragment as an :class:`Assembly` that can be joined to one.

    ``root_kind`` on the result is left empty on purpose: a bench has no category
    of its own, and the global rules are checked against the *device under test*.
    Giving the bench a kind would let it be checked twice, once as itself.
    """
    spec = RIGS[_rig_name(rig)] if not isinstance(rig, RigSpec) else rig
    path = instance if instance is not None else (spec.name,)
    supplies_wheels = bench_capability(spec) == "wheel_supplying"
    requirements = () if supplies_wheels else (
        PortRequirement(role="wheel_centre", count=1, required=False),
        need_to_requirement(
            PortNeed(
                role="body_mount",
                required=False,
                bound_outputs=tuple(spec.outputs),
                note="a bench that cannot reach the body contributes no measurement",
            )
        ),
    )
    return Assembly(
        name=path[-1],
        fragment=build_rig_fragment(
            spec, capabilities=capabilities, mode=mode, instance=path
        ),
        ports=rig_ports(spec, instance=path),
        requirements=requirements,
        subsystems=frozenset({"rig"}),
        provenance=FragmentProvenance(
            template=BENCH_TEMPLATE_NAME,
            revision=bench_capability(spec),
            instance=path,
        ),
    )
