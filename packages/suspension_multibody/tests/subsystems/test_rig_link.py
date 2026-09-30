"""
The bench's wheel joins the solved model, and the solve still asks the same question.

Two things had to be true at once, and the second is the one that is easy to lose:

1. the bench's carrier body and its tire must actually be **in** the model that
   gets solved, or "the assembly has a wheel" is a statement about a fragment
   nobody reads;
2. the prescribed motion must still act on the **suspension**, through the
   upright.  A bench that moved its own carrier while the upright stayed put would
   load the bench instead of the axle -- a model that converges, reports numbers,
   and answers a different question.

The link is a weld between the two, which is what a K&C rig physically is: a rigid
wheel mounted on the upright, with the tire carrying the compliance to the road.
"""

from __future__ import annotations

import pytest

from suspension_multibody.cases.kc_quasi_static.contract import (
    model_document,
    wheel_centre_body,
)
from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    Vec3,
    VerticalTire,
)
from suspension_multibody.subsystems.rig_link import (
    RigLinkError,
    link_wheel_supplying_rig,
    merge_rig_link,
)
from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle
from suspension_multibody.subsystems.types import AssemblyRequest


def _model(*, with_tires: bool = True) -> FrontAxleModel:
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
        tires=(
            (
                VerticalTire(
                    stiffness=200.0,
                    unloaded_radius=300.0,
                    contact_point=Vec3(x=0, y=-700, z=0),
                    local_axis=Vec3(x=0, y=0, z=1),
                ),
            )
            if with_tires
            else ()
        ),
    )


def _linked(rig: str = "kc_quasi_static", *, with_tires: bool = True):
    """
    Return one composed assembly, its runtime, the link, and the merged runtime.

    The composition merges the link itself, so the merged runtime is the one the
    assembly already carries -- ``merge_rig_link`` is applied here only when the
    caller wants it separately, which is why the returned merged value is built
    from the *linked* runtime rather than by merging twice.  Merging twice appends
    each weld again, which is how this helper was wrong once.
    """
    composed = si_assembly_for_axle(
        _model(with_tires=with_tires),
        request=AssemblyRequest(mode="K"),
        rig=rig,
    )
    runtime = composed.assembly.physical
    link = link_wheel_supplying_rig(runtime, composed.rig, mode="K")
    return composed, runtime, link, runtime


def test_the_bench_carriers_reach_the_solved_model() -> None:
    """
    The carriers are in the runtime, and in the document the kernel reads.

    A body in the composition but not in the runtime is not loaded by the solve,
    and the console would look right -- which is the whole reason this link exists.
    """
    _composed, _runtime, link, merged = _linked()
    assert set(link.bodies) == {"wheel_carrier_L", "wheel_carrier_R"}
    assert {"wheel_carrier_L", "wheel_carrier_R"} <= set(merged.bodies)
    document = model_document(merged, name="probe")
    names = {b["name"] for b in document["bodies"]}
    assert {"wheel_carrier_L", "wheel_carrier_R"} <= names


def test_the_carrier_is_welded_to_its_upright() -> None:
    """
    A real constraint row, not just a body sitting in a dictionary.

    Without the weld the carrier is an unconstrained rigid body: the solver pins
    its freedoms rather than reporting an error, so the model silently changes
    instead of refusing.
    """
    _composed, _runtime, link, merged = _linked()
    assert [c.name for c in link.constraints] == [
        "wheel_carrier_L_weld",
        "wheel_carrier_R_weld",
    ]
    for constraint in link.constraints:
        assert constraint.body_a.startswith(("wheel_hub_", "upright_"))
        assert constraint.body_b.startswith("wheel_carrier_")
        assert constraint.body_a in merged.bodies
        assert constraint.body_b in merged.bodies
    document = model_document(merged, name="probe")
    joint_names = {j["name"] for j in document["joints"]}
    assert {"wheel_carrier_L_weld", "wheel_carrier_R_weld"} <= joint_names


def test_the_prescribed_motion_still_acts_on_the_suspension() -> None:
    """
    The drive row names the upright, so the bench loads the axle.

    This is the assertion that keeps "the bench is in the model" from silently
    changing which question the run answers: a carrier that declared the wheel
    centre would take over the driven coordinate, and the sweep would then move the
    bench's own body while the suspension never travelled.
    """
    _composed, runtime, _link, merged = _linked()
    expected_body = "wheel_hub_L" if "wheel_hub_L" in runtime.bodies else "upright_L"
    assert wheel_centre_body(runtime, "L") == expected_body
    assert wheel_centre_body(merged, "L") == expected_body
    document = model_document(merged, name="probe")
    drives = {
        row["name"]: row["body_a"]
        for row in document["joints"]
        if row["name"].startswith("wheel_drive_")
    }
    assert drives == {
        "wheel_drive_L": expected_body,
        "wheel_drive_R": "wheel_hub_R" if "wheel_hub_R" in runtime.bodies else "upright_R",
    }


def test_the_carrier_does_not_declare_a_second_wheel_centre() -> None:
    """
    Two declarations would make the wheel-centre lookup ambiguous, and it refuses.

    The carrier's point at the wheel centre is named `center` for this reason, and
    the reader that resolves the driven coordinate raises on two candidates rather
    than guessing which one was meant.
    """
    _composed, _runtime, _link, merged = _linked()
    carriers = [name for name in merged.bodies if "carrier" in name]
    assert carriers
    for carrier in carriers:
        assert (carrier, "wheel_center") not in merged.points
        assert (carrier, "center") in merged.points
    expected_body = "wheel_hub_L" if "wheel_hub_L" in merged.bodies else "upright_L"
    assert wheel_centre_body(merged, "L") == expected_body


def test_the_tire_stays_on_the_assembly_it_belongs_to() -> None:
    """
    D3, reversed by decision: the bench does **not** re-own the model's tire.

    The old contract said the tire moves to the bench's carrier, on the argument
    that leaving it on the upright would count the same tire twice.  The bench
    supplies the *wheel*, and the wheel end is the assembly's, so the tire is the
    assembly's too: re-owning it was a rewrite of the model under test, which is
    what the non-invasiveness rule forbids.  Its owner, its frame offset and its
    law are therefore exactly what the model put there, and no element belongs to
    a `wheel_carrier_`.
    """
    _composed, runtime, _link, merged = _linked()
    before = [
        (e.name, e.wheel_body, tuple(e.wheel_center_local), e.stiffness, e.unloaded_radius)
        for e in runtime.elements
        if type(e).__name__ == "VerticalTireElement"
    ]
    after = [
        (e.name, e.wheel_body, tuple(e.wheel_center_local), e.stiffness, e.unloaded_radius)
        for e in merged.elements
        if type(e).__name__ == "VerticalTireElement"
    ]
    assert before, "the probe model must carry a tire for this test to mean anything"
    assert after == before
    assert not any(body.startswith("wheel_carrier_") for _, body, *_ in after)

    document = model_document(merged, name="probe", drive_mode="pad")
    tire_bodies = {tire["name"]: tire["body"] for tire in document["tires"]}
    assert tire_bodies, "the tires must survive into the document"
    assert not any(body.startswith("wheel_carrier_") for body in tire_bodies.values())
    # The body the model hung it on is still the one that carries the wheel centre.
    assert all(
        body == wheel_centre_body(merged, body[-1]) for body in tire_bodies.values()
    )


def test_the_weld_ties_the_two_bodies_at_one_physical_place() -> None:
    """
    A weld's two points must resolve to the same world position.

    This is the assertion that three earlier attachment schemes failed, and it is
    the hardest one to notice by reading: the emitter takes each ``point_*`` as a
    **world** coordinate and re-expresses it in its own body's frame
    (``contract.py::_local_point``), so a carrier placed at the wheel centre with
    ``point_b`` given as zero produced the pair ``[0,-700,300]`` and
    ``[0,700,-300]`` -- two numbers describing one place, which the kernel then
    rejected as a rank-deficient or non-converging model.  Comparing the *resolved*
    points is what catches it; comparing the numbers as written would not.
    """
    import numpy as np

    from suspension_multibody.modeling.primitives import SE3

    _composed, _runtime, _link, merged = _linked()
    document = model_document(merged, name="probe")
    bodies = {body["name"]: body for body in document["bodies"]}
    welds = [j for j in document["joints"] if "weld" in j["name"]]
    assert len(welds) == 2, "the probe must carry one weld per side"

    def world(body: dict, point: list[float]) -> np.ndarray:
        pose = SE3(np.asarray(body["position"]), np.asarray(body["quaternion"]))
        return np.asarray(pose.transform_point(np.asarray(point)), dtype=float)

    for weld in welds:
        at_a = world(bodies[weld["body_a"]], weld["point_a"])
        at_b = world(bodies[weld["body_b"]], weld["point_b"])
        assert np.allclose(at_a, at_b, atol=1e-9), (
            f"{weld['name']} resolves to {at_a} and {at_b}: the two bodies are not "
            "welded at one place"
        )


def test_the_weld_removes_no_freedom_the_model_had() -> None:
    """
    The attachment adds six rows and six freedoms per side, and no redundancy.

    A carrier declared ``fixed`` was tried and is wrong: ground is a body too, so
    welding a grounded carrier to a free upright grounds the suspension.  Measured,
    that produced 64 constraint rows over 54 columns in K -- ten redundant rows --
    and the kernel refused it.  Counting rows against columns is what makes the
    difference visible without running the solver.
    """
    from suspension_multibody.subsystems.entry import compose_axle

    rows_by_type = {"spherical": 3, "revolute": 5, "prismatic": 5, "fixed": 6,
                    "driven_translation": 1, "driven_rotation": 1}

    def counts(assembly) -> tuple[int, int]:
        document = model_document(assembly, name="probe", drive_wheels=True)
        free = sum(1 for body in document["bodies"] if not body["fixed"])
        rows = sum(rows_by_type.get(j["type"], 0) for j in document["joints"])
        return rows, 6 * free

    historical = compose_axle(_model(), "K")
    _composed, _runtime, _link, merged = _linked()
    historical_rows, historical_columns = counts(historical)
    rows, columns = counts(merged)

    assert rows == historical_rows + 12, "each weld contributes its six rows"
    assert columns == historical_columns + 12, "each carrier contributes six freedoms"
    assert rows <= columns, (
        f"{rows} constraint rows over {columns} columns: the model is over-constrained"
    )
    # And the historical model is not itself redundant, so the comparison is fair.
    assert historical_rows <= historical_columns


def test_a_loading_bench_contributes_no_link() -> None:
    """
    A vehicle bench owns no wheel, so there is nothing to attach.

    Inventing an attachment would put a body in the model that the bench never
    declared, which is the same mistake in the other direction.
    """
    composed = si_assembly_for_axle(
        _model(), request=AssemblyRequest(mode="K"), rig="vehicle_dynamic"
    )
    runtime = composed.assembly.physical
    link = link_wheel_supplying_rig(runtime, composed.rig, mode="K")
    assert link.is_empty
    assert merge_rig_link(runtime, link) is runtime


def test_a_side_with_no_wheel_centre_gets_no_carrier() -> None:
    """
    A body connected to nothing would be a freedom the solver has to pin.

    So a bench whose wheel cannot reach a wheel centre on that side leaves the
    carrier out of the model instead of floating it in.  Exercised as a unit here
    by removing the points: the upright comes from the *suspension* role, so an
    assembly can legitimately have the body without the point, and the link must
    then skip that side rather than attach to a guess.
    """
    composed = si_assembly_for_axle(
        _model(), request=AssemblyRequest(mode="K"), rig="kc_quasi_static"
    )
    runtime = composed.assembly.physical
    from dataclasses import replace

    without = replace(
        runtime,
        points={
            key: value
            for key, value in runtime.points.items()
            if key[1] != "wheel_center"
        },
    )
    link = link_wheel_supplying_rig(without, composed.rig, mode="K")
    assert link.is_empty


def test_an_ambiguous_wheel_centre_is_refused() -> None:
    """
    Two wheel-carrying bodies on one side is ambiguous, and guessing is worse.

    The link raises rather than picking one, because attaching a bench's wheel to
    the wrong body produces a model that solves and is wrong.
    """
    composed = si_assembly_for_axle(
        _model(), request=AssemblyRequest(mode="K"), rig="kc_quasi_static"
    )
    runtime = composed.assembly.physical
    from dataclasses import replace

    carrier_pt = (
        ("wheel_hub_L", "wheel_center")
        if ("wheel_hub_L", "wheel_center") in runtime.points
        else ("upright_L", "wheel_center")
    )
    duplicated = replace(
        runtime,
        points={
            **runtime.points,
            ("upper_arm_L", "wheel_center"): runtime.points[carrier_pt],
        },
    )
    with pytest.raises(RigLinkError, match="more than one wheel centre"):
        link_wheel_supplying_rig(duplicated, composed.rig, mode="K")


def test_an_unknown_mode_is_refused() -> None:
    composed = si_assembly_for_axle(
        _model(), request=AssemblyRequest(mode="K"), rig="kc_quasi_static"
    )
    with pytest.raises(RigLinkError, match="unknown mode"):
        link_wheel_supplying_rig(
            composed.assembly.physical, composed.rig, mode="X"
        )
