"""
The runtime face a composition amounts to, and how it differs from the build.

``composition.py`` produces *identity* -- names, point roles, declarations.
``runtime.py`` produces the objects a contract emitter reads.  The two are
deliberately separate, and this module pins both:

1. the runtime face really is the seven faces an emitter asks for, in the order
   the document records them;
2. the difference against the historical build is **listed**, not assumed.  The
   migration is allowed to differ while it is in progress; what it may not do is
   differ silently, so ``diff_against_reference`` is what the task's own
   evidence is built from.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling.primitives import BushingElement
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.runtime import (
    SubsystemRuntime,
    diff_against_reference,
    runtime_from_outputs,
)
from suspension_multibody.subsystems.si_assembly import contributions_for_axle
from suspension_multibody.subsystems.types import AssemblyRequest


def _model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={
            "uca_front": Vec3(x=-100, y=-500, z=400),
            "uca_rear": Vec3(x=100, y=-500, z=400),
            "uca_outer": Vec3(x=0, y=-700, z=450),
            "lca_front": Vec3(x=-120, y=-500, z=150),
            "lca_rear": Vec3(x=120, y=-500, z=150),
            "lca_outer": Vec3(x=0, y=-700, z=150),
            "tierod_inner": Vec3(x=100, y=-400, z=250),
            "tierod_outer": Vec3(x=50, y=-700, z=250),
            "wheel_center": Vec3(x=0, y=-700, z=300),
            "rack_center": Vec3(x=0, y=0, z=250),
        },
        mass=MassSpec(sprung_mass=1000),
    )


def _runtime(mode: str) -> SubsystemRuntime:
    """Build the runtime face from the contributions of one axle."""
    contributions = contributions_for_axle(_model(), request=AssemblyRequest(mode=mode))
    return runtime_from_outputs(
        (contribution.output for contribution in contributions),
        mode=mode,  # type: ignore[arg-type]
        roles=frozenset(contribution.role for contribution in contributions),
    )


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_runtime_carries_both_constraint_columns(mode: str) -> None:
    """
    The K column survives a C build, because a study may ask for either.

    Keeping only the active column is what made the composition unable to answer
    "what would this model's K joints be", which is exactly the question a study
    switch asks.  The ideal column is therefore non-empty in both modes.
    """
    runtime = _runtime(mode)
    assert runtime.constraints, f"{mode} runtime carries no active constraints"
    assert runtime.ideal_constraints, f"{mode} runtime carries no ideal column"


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_runtime_state_is_built_from_the_bodies(mode: str) -> None:
    runtime = _runtime(mode)
    assert runtime.state is not None
    assert set(runtime.state.bodies) == set(runtime.bodies)


def test_the_runtime_exposes_the_faces_a_document_reads() -> None:
    """
    The seven faces plus capabilities, which a rig binds against.

    Asserted as a set so that dropping a face is a failure rather than a
    silently missing block in some emitter three steps away.
    """
    runtime = _runtime("K")
    for face in (
        "bodies",
        "points",
        "hardpoints",
        "constraints",
        "ideal_constraints",
        "bushings",
        "elements",
        "capabilities",
    ):
        assert hasattr(runtime, face), f"runtime is missing the {face!r} face"
    assert runtime.capabilities is not None
    assert "suspension" in runtime.capabilities.subsystems


def test_a_runtime_point_read_is_a_copy() -> None:
    """
    A caller must not be able to edit the model through a point read.

    The historical assembly returned a copy for the same reason; a shared array
    would let one reader change what every later reader sees, and the drift
    never shows up in a result.
    """
    runtime = _runtime("K")
    (body, label), original = next(iter(runtime.points.items()))
    taken = runtime.point(body, label)
    taken[:] = 999.0
    assert not (runtime.points[(body, label)] == 999.0).all()
    assert (runtime.point(body, label) == original).all()


def test_c_mode_reports_its_bushings_separately_from_its_elements() -> None:
    """
    A C assembly answers "which compliance elements" without walking elements.

    K carries none by construction, so the distinction is asserted in both
    directions rather than only where it happens to be interesting.
    """
    k_runtime = _runtime("K")
    c_runtime = _runtime("C")
    assert k_runtime.bushings == ()
    assert all(isinstance(item, BushingElement) for item in c_runtime.bushings)
    assert any(isinstance(item, BushingElement) for item in c_runtime.elements)


def test_element_ids_follow_the_element_order() -> None:
    runtime = _runtime("K")
    names = tuple(getattr(element, "name", "") for element in runtime.elements)
    assert runtime.element_ids == names


def test_the_difference_against_the_historical_build_is_reported() -> None:
    """
    The migration's own evidence: list the difference, do not assume none.

    ``diff_against_reference`` is what the task record is produced from, so it
    has to work in both directions -- it must notice a difference and it must
    report agreement when the two really do agree.
    """
    historical = compose_axle(_model(), "K")
    runtime = _runtime("K")
    difference = diff_against_reference(runtime, historical)

    # The comparison itself must be exercised: a difference object that always
    # claims equality would make every later "no difference" claim worthless.
    assert isinstance(difference.report(), str)
    assert difference.report()

    same = diff_against_reference(runtime, runtime)
    assert same.equal, same.report()


def test_a_missing_face_is_named_rather_than_counted() -> None:
    """
    A bare count would say "three differ"; the report has to say which.

    Built by comparing against a deliberately reduced reference, which is the
    only way to exercise the missing/extra paths without editing real code.
    """

    class Reduced:
        """A reference exposing the seven faces, with nothing in them."""

        bodies: dict[str, object] = {}
        points: dict[tuple[str, str], object] = {}
        constraints: tuple[object, ...] = ()
        ideal_constraints: tuple[object, ...] = ()
        bushings: tuple[object, ...] = ()
        elements: tuple[object, ...] = ()
        hardpoints: dict[str, object] = {}

    runtime = _runtime("K")
    difference = diff_against_reference(runtime, Reduced())
    assert not difference.equal
    # The reference is empty, so everything the runtime carries is "extra".
    # Both directions are reported because a migration can lose an entity as
    # easily as it can add one, and the two need different fixes.
    assert difference.extra["bodies"], "the surplus names were not listed"
    assert difference.missing["bodies"] == ()
    assert "bodies" in difference.report()
