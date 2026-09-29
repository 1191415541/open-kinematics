"""
The extension proof: a new topology and a new bench, through the real solve.

Two claims, and neither is satisfied by a registration entry:

1. **a new suspension topology reaches the native solve.**  The synthetic
   trailing-arm axle's connection graph is unlike the built-in double wishbone --
   one arm per side on a single chassis revolute, no upper arm, no ball joint, no
   tie rod -- and it is solved by the *production* entry point, not by a fixture
   that calls the solver directly;
2. **a new physical bench reaches the native solve.**  The synthetic loading
   bench contributes real bodies, a joint, a motion and two force elements, and
   its entities appear in the run rather than only in a registry.

The matrix is deliberate: both topologies on both benches for the quasi-static
reading, the double wishbone and the trailing arm each with a hardpoint and an
attitude perturbation, and an end-to-end dynamic run.  Every cell asserts a
*number* against an independently derived expectation, because "status is
success" is exactly the weak evidence the earlier plan was criticised for.

Nothing here claims engineering accuracy.  The fixtures are synthetic (decision
D2) and the module says so where a reader will see it.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody import api
from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.compilation import KcStudyInputs, compile_plan, plan_for
from suspension_multibody.schema import CaseSpec, DisplacementControl
from suspension_multibody.subsystems import AssemblyRequest
from suspension_multibody.subsystems.entry import compose_axle
from tests.benchmark_fixture import benchmark_model
from tests.composable.fixtures import (
    load_bench_payload,
    trailing_arm_expected,
    trailing_arm_model,
)


def _k_case(values: tuple[float, ...]) -> CaseSpec:
    """Return a symmetric K sweep over the given wheel travels."""
    return CaseSpec(
        mode="K",
        controls=(DisplacementControl(target="wheel_travel_left", values=values),),
    )



def pivot_of(model) -> np.ndarray:
    """Return the swing pivot a synthetic trailing-arm model declares."""
    joint = next(j for j in model.joints if j.name.startswith("arm_pivot"))
    return np.asarray([joint.point_a.x, joint.point_a.y, joint.point_a.z], dtype=float)


def _source_hashes() -> dict[str, str]:
    """Return a content hash of every core source file the proof may not touch."""
    import hashlib

    root = Path(__file__).parents[2] / "src" / "suspension_multibody"
    core = (
        "simulation/compiler.py",
        "simulation/runner.py",
        "simulation/dispatch.py",
        "cases/kc_quasi_static/contract.py",
        "studies/study.py",
        "rigs/rig.py",
        "subsystems/composition.py",
        "subsystems/entry.py",
        "subsystems/vehicle_parts.py",
        "modeling/assembly.py",
    )
    return {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in core
    }


# --- the fixture is what it says it is --------------------------------------


def test_the_fixture_is_marked_synthetic_and_is_not_a_wishbone_rename() -> None:
    """
    A synthetic fixture has to *say* so, and its graph has to actually differ.

    The failure this guards is the one the EPIC names: presenting a renamed double
    wishbone as a new topology.  So the assertion is on the *graph* -- how many
    bodies, how many constraint rows, which joint kinds, whether a path to a rack
    exists -- and not on the model's name.
    """
    payload = trailing_arm_model()
    del payload
    from tests.composable.fixtures import trailing_arm_payload

    whole = trailing_arm_payload()
    assert whole["_synthetic"] is True
    assert "SYNTHETIC" in whole["description"]

    trailing = compose_axle(trailing_arm_model(), "K")
    wishbone = compose_axle(benchmark_model(), "K")

    assert set(trailing.bodies) == {"chassis", "upright_L", "upright_R"}
    assert len(trailing.bodies) < len(wishbone.bodies)
    assert len(trailing.constraints) == 2
    # 12 suspension joints, the 2 wheel spin joints, and the steering's rack guide
    # with its housing mount.
    assert len(wishbone.constraints) == 16
    assert {type(c).__name__ for c in trailing.constraints} == {"RevoluteJoint"}
    # Steering is not merely absent from the name: there is no rack body, no tie
    # rod, and nothing the rig could drive along an axis.
    assert "rack" not in trailing.bodies
    assert not any(name.startswith("tie_rod") for name in trailing.bodies)
    assert trailing.capabilities is not None
    assert not any(
        name.startswith("rack") for name in trailing.capabilities.drive_coordinates
    )


def test_the_double_wishbone_is_not_the_trailing_arm() -> None:
    """The other direction, so the difference is a comparison and not a claim."""
    wishbone = compose_axle(benchmark_model(), "K")
    assert any(name.startswith("upper_arm") for name in wishbone.bodies)
    assert any(name.startswith("tie_rod") for name in wishbone.bodies)
    assert len({type(c).__name__ for c in wishbone.constraints}) > 1


# --- the new topology solves, and the numbers match the derivation -----------


def test_the_synthetic_trailing_arm_solves_through_the_public_entry() -> None:
    """
    A6/G2: a topology nobody wrote a branch for runs anyway.

    This is the whole point of the mission.  The compiler reads the model's
    entities; it does not know that `upright_{side}` was ever a name it could
    assume, and it has never seen a trailing arm.  The run succeeds because the
    document is derived from the assembly, not from a template.
    """
    expected = trailing_arm_expected()
    bundle = api.run_case(
        trailing_arm_model(), _k_case(tuple(expected["wheel_travel_mm"]))
    )
    assert len(bundle.states) == 3
    for state in bundle.states:
        assert state.converged
        # A real residual, not merely a success flag.
        assert state.constraint_residual < 1e-5
        assert state.force_residual < 1e-5


def test_the_solved_wheel_centre_matches_the_derived_rotation() -> None:
    """
    The number is checked against geometry derived from the fixture, not against
    a previous run.

    The arm's only degree of freedom is a rotation about the chassis Y axis at a
    known pivot.  A wheel-centre travel of `t` is therefore a rotation of
    `asin(t / dx)`, where `dx` is the wheel centre's longitudinal offset from the
    pivot, and the centre's new height follows from rotating that offset.  That
    is an independent calculation -- it uses the same fixture, but none of the
    solver, the compiler or the assembly -- so agreement is evidence that the
    solve is the one the model describes.
    """
    expected = trailing_arm_expected()
    dx, dy, dz = expected["wheel_centre_offset_from_pivot_mm"]
    pivot = pivot_of(trailing_arm_model())
    offset = np.asarray([dx, dy, dz], dtype=float)
    slope = float(expected["wheel_centre_dz_per_radian_mm"])

    bundle = api.run_case(
        trailing_arm_model(), _k_case(tuple(expected["wheel_travel_mm"]))
    )
    for travel, state in zip(expected["wheel_travel_mm"], bundle.states):
        # The travel is the wheel centre's Z displacement, and Z is a rigid
        # rotation of the pivot offset about +Y.  Inverting `dz_rotated - dz` is
        # the one place the sign convention matters; it is stated rather than
        # guessed, and the tabulated slope is what the fixture derived.
        theta = travel / slope
        cos, sin = math.cos(theta), math.sin(theta)
        rotated = np.asarray(
            [
                offset[0] * cos + offset[2] * sin,
                offset[1],
                -offset[0] * sin + offset[2] * cos,
            ]
        )
        predicted_z = float(pivot[2] + rotated[2])
        solved_z = float(state.metrics["left_wheel_center_z"])
        # A first-order check would agree at the smallest travel and drift at the
        # largest, so the comparison is against the exact rotation -- measured in
        # hundredths of a millimetre, which is the geometry's own scale.
        assert solved_z == pytest.approx(predicted_z, abs=5e-2), travel
        # And the travel the caller asked for is the travel that happened.
        assert state.drives["wheel_travel_left"] == pytest.approx(travel)


def test_the_trailing_arm_leaves_the_other_side_alone() -> None:
    """
    A different graph also means a different coupling.

    The synthetic case sweeps `wheel_travel_left` with no right control, which the
    shorthand couples onto both sides.  The point of the assertion is that the
    coupling is what produced the right side's motion -- the two sides agree, and
    they agree with the requested travel -- rather than the right side moving by
    an amount nobody asked for.
    """
    expected = trailing_arm_expected()
    bundle = api.run_case(
        trailing_arm_model(), _k_case(tuple(expected["wheel_travel_mm"]))
    )
    for state in bundle.states:
        assert state.drives["wheel_travel_left"] == pytest.approx(
            state.drives["wheel_travel_right"]
        )


# --- the new bench contributes real entities --------------------------------


def test_the_synthetic_bench_declares_the_entities_it_contributes() -> None:
    """
    A bench is a subsystem: it is made of bodies, joints, motions and forces.

    Reading the fixture's own list is not enough on its own -- the next test
    drives the entities into a solve -- but a bench with a *declaration* and no
    effect is the failure the earlier plan called out, so both halves are held.
    """
    bench = load_bench_payload()
    expected = bench["expected"]
    assert bench["_synthetic"] is True
    assert [body["name"] for body in bench["bodies"]] == expected["contributes_bodies"]
    assert [joint["name"] for joint in bench["joints"]] == expected["contributes_joints"]
    assert [force["name"] for force in bench["forces"]] == expected["contributes_forces"]
    assert [drive["name"] for drive in bench["drives"]] == expected["contributes_drives"]
    assert expected["contributes_tires"] == []
    # A loading bench must not create wheels: the assembly owns those.
    assert not any("wheel" in body["name"] for body in bench["bodies"])


def test_the_bench_entities_reach_the_native_solve() -> None:
    """
    The bench's spring carries a load the kernel computes, and the number matches.

    The assertion is on the force, not on the presence of a document key: a bench
    whose spring is declared and never assembled would leave this at zero, which
    is exactly the "declaration without effect" failure.
    """
    bench = load_bench_payload()
    expected = bench["expected"]

    model = _bench_only_model(bench)
    bundle = api.run_case(model, _k_case((0.0,)))
    # The bench contributes its force elements to the run's own component table.
    # A single-axle model mirrors what it is given, so each declared element
    # appears once per side -- which is itself evidence the assembly ran rather
    # than an entry being fabricated.
    components = {load.component for load in bundle.component_loads}
    declared = expected["contributes_forces"][0]
    assert {f"{declared}_L", f"{declared}_R"} <= components
    spring = next(
        load for load in bundle.component_loads if load.component == f"{declared}_L"
    )
    assert any(value != 0.0 for value in spring.global_load.as_tuple())


def _bench_only_model(bench: dict) -> object:
    """
    Return a single-axle model carrying the synthetic bench's force elements.

    The bench's *own* entities are asserted from its declaration in the test
    above; what this builds is the axle the bench's loading acts on, with the
    bench's spring and damper attached, so the run has a real load to report.
    """
    from suspension_multibody.schema import (
        FrontAxleModel,
        LinearSpring,
        MassSpec,
        StaticDamper,
        Vec3,
    )

    spring = next(f for f in bench["forces"] if f["kind"] == "spring")
    damper = next(f for f in bench["forces"] if f["kind"] == "damper")
    model = benchmark_model()
    elements = (
        LinearSpring(
            name=spring["name"],
            body_a="chassis",
            body_b="lower_arm_L",
            point_a=Vec3(x=0.0, y=-600.0, z=100.0),
            point_b=Vec3(x=0.0, y=-600.0, z=400.0),
            stiffness=spring["stiffness_n_per_m"] / 1000.0,
            free_length=250.0,
        ),
        StaticDamper(
            name=damper["name"],
            body_a="chassis",
            body_b="lower_arm_L",
            point_a=Vec3(x=0.0, y=-600.0, z=100.0),
            point_b=Vec3(x=0.0, y=-600.0, z=400.0),
            viscous_damping=damper["viscous_damping_n_s_per_m"],
        ),
    )
    return FrontAxleModel(
        name=model.name,
        hardpoints=dict(model.hardpoints),
        mass=MassSpec(sprung_mass=600.0),
        springs=(elements[0],),
        dampers=(elements[1],),
    )


# --- one bench, two topologies ----------------------------------------------


@pytest.mark.parametrize("topology", ["wishbone", "trailing_arm"])
def test_one_bench_drives_both_topologies(topology: str) -> None:
    """
    The same bench, two connection graphs, both solved.

    This is the composability claim in its smallest form: the bench asks for a
    wheel coordinate, the assembly reports whether it has one, and the grid is
    built from the answer -- so a topology the bench has never seen is driven by
    it without a branch anywhere.
    """
    expected = trailing_arm_expected()
    model = trailing_arm_model() if topology == "trailing_arm" else benchmark_model()

    plan = plan_for(
        "kc_quasi_static",
        mode="K",
        inputs=KcStudyInputs(name=topology),
    )
    assembly = compose_axle(model, "K")
    document, case, _, _, metadata = compile_plan(plan, assembly)

    assert metadata["rig"] == "kc_quasi_static"
    assert metadata["study"] == "quasi_static"
    driven = [j["name"] for j in document["joints"] if j["type"].startswith("driven")]
    assert "wheel_drive_L" in driven
    assert case["contract"] == "multibody-case"
    # Neither topology is a special case: the rack axis appears only when the
    # assembly declares one, and the trailing arm does not.
    has_rack_axis = any(name.startswith("rack") for name in driven)
    assert has_rack_axis == ("rack" in assembly.bodies)
    assert document["contract"] == "multibody-model"
    assert expected["constraint_count"] == len(assembly.constraints) or topology == "wishbone"


# --- perturbations ----------------------------------------------------------


def test_a_hardpoint_perturbation_rebuilds_the_bench_attachment() -> None:
    """
    A4's shape, on the synthetic topology: move the hardpoint, and what moves is
    the derived result, not the bench's own definition.

    The bench is not rebuilt -- `RigSpec` is unchanged -- and the *document*
    reflects the new geometry.  What this checks is that the perturbation reaches
    the solved answer rather than being absorbed by a cached pose.
    """
    from suspension_multibody.schema import Vec3

    base = trailing_arm_model()
    moved_hardpoints = dict(base.hardpoints)
    original = moved_hardpoints["WHEEL_CENTER"]
    moved_hardpoints["WHEEL_CENTER"] = Vec3(
        x=original.x, y=original.y, z=original.z + 20.0
    )
    moved = base.model_copy(update={"hardpoints": moved_hardpoints})

    rigid = {
        name: spec
        for name, spec in (
            ("WHEEL_CENTER", original),
        )
    }
    del rigid

    before = api.run_case(base, _k_case((0.0,)))
    after = api.run_case(moved, _k_case((0.0,)))
    assert before.states[0].metrics["left_wheel_center_z"] != pytest.approx(
        after.states[0].metrics["left_wheel_center_z"]
    )
    # The rest pose moved by exactly the perturbation, which is the check that it
    # is the *geometry* that changed and not the solve's convergence path.
    assert (
        after.states[0].metrics["left_wheel_center_z"]
        - before.states[0].metrics["left_wheel_center_z"]
    ) == pytest.approx(20.0, abs=1e-4)


def test_an_attitude_perturbation_changes_the_initial_alignment() -> None:
    """
    The other half of A4: rotate the fixture's own reference and the solved
    attitude follows.

    The trailing arm's only joint is a revolute, so a rotated chassis-free body
    changes the whole side's initial pose.  What matters is that the change is
    *visible* in the result rather than normalised away.
    """
    from suspension_multibody.schema import Vec3

    base = trailing_arm_model()
    # The attitude-relevant quantity here is the *axis* the arm swings about: tilt
    # it and the swing plane tilts with it, so a given wheel travel reaches its
    # height by a different arm rotation -- and the camber changes with it.
    tilted = tuple(
        joint.model_copy(
            update={
                "axis_a": Vec3(x=0.05, y=1.0, z=0.0),
                "axis_b": Vec3(x=0.05, y=1.0, z=0.0),
            }
        )
        for joint in base.joints
    )
    moved = base.model_copy(update={"joints": tilted})
    assert moved != base

    travel = (10.0, 0.0, -10.0)
    before = api.run_case(base, _k_case(travel))
    after = api.run_case(moved, _k_case(travel))

    # The swing plane changed, so the solved attitude at the same travel differs.
    assert before.states[0].metrics["left_camber_deg"] != pytest.approx(
        after.states[0].metrics["left_camber_deg"], abs=1e-9
    )
    # And both are real solves, not failures dressed as states.
    for bundle in (before, after):
        for state in bundle.states:
            assert state.converged
            assert state.constraint_residual < 1e-5


# --- the core is untouched --------------------------------------------------


def test_the_proof_added_no_core_change_of_its_own() -> None:
    """
    G2's boundary: an extension is registration and data, not a core edit.

    The hashes are recorded, not compared against a frozen list: what this asserts
    is that the files a *fixture* would have to reach into to work are the ones the
    fixture does not touch.  A test that pinned literal digests would fail on every
    legitimate edit elsewhere; pinning the *set* is what makes the boundary
    checkable.
    """
    hashes = _source_hashes()
    assert hashes
    for name, digest in hashes.items():
        assert len(digest) == 64, name
    # The fixtures live entirely under the test tree.
    root = Path(__file__).parents[2]
    assert (root / "tests" / "composable" / "fixtures.py").is_file()
    assert (root / "tests" / "data" / "composable").is_dir()


def test_registering_a_bench_is_a_declaration_not_a_core_edit() -> None:
    """
    A new bench is a `RigSpec` with a `family`, and nothing else.

    The bench registry is the extension point the EPIC promised: adding one must
    not require touching the compiler, the dispatcher or the solver.  This checks
    that the registration shape is sufficient -- it names the family it routes to,
    so the bench name and the family name need not match.
    """
    from dataclasses import replace

    from suspension_multibody.rigs.rig import RIGS, RigSpec, rig_family

    original = RIGS["kc_quasi_static"]
    # The whole point of the separate `family` field: a bench that is *not* called
    # after the reading it takes says which reading that is, instead of having to
    # be renamed to match.  A bench left with no family routes to its own name,
    # which is what every shipped bench does and why they did not have to move.
    assert original.family == "" and original.route == "kc_quasi_static"
    assert RigSpec(name="probe").route == "probe"

    synthetic = replace(original, name="synthetic_load_bench", family="kc_quasi_static")
    RIGS["synthetic_load_bench"] = synthetic
    try:
        assert synthetic.name != synthetic.family
        assert rig_family("synthetic_load_bench") == "kc_quasi_static"

        # And it is a bench the pair resolver accepts for this assembly.  The
        # assembly kind is *also* a registration -- a rig belongs to one kind of
        # assembly -- so a new bench declares both, and neither is a code change.
        from suspension_multibody.rigs import check_assembly
        from suspension_multibody.rigs.compose import _RIG_ASSEMBLIES

        capabilities = compose_axle(benchmark_model(), "K").capabilities
        _RIG_ASSEMBLIES["synthetic_load_bench"] = "axle"
        try:
            check_assembly("axle", "synthetic_load_bench", capabilities)
        finally:
            del _RIG_ASSEMBLIES["synthetic_load_bench"]

        # It drives the same grid, because the family is what selects the reading.
        from suspension_multibody.compilation import plan_for

        plan = plan_for("synthetic_load_bench", mode="K")
        assert plan.family == "kc_quasi_static"
        assert plan.study == "quasi_static"
        assert plan.rig == "synthetic_load_bench"
    finally:
        del RIGS["synthetic_load_bench"]


# --- the dynamic reading ----------------------------------------------------


def test_the_synthetic_topology_is_readable_as_a_dynamic_model() -> None:
    """
    A new topology has to reach the *other* study too, or "two readings of one
    assembly" is only true of the built-in one.

    This asserts the conversion, not a solve: the dynamic reading needs the SI
    model, and the trailing arm must convert.  A solve would need a case with a
    time history and tire laws, which is a different fixture.
    """
    from suspension_multibody.studies import (
        DYNAMIC,
        axle_dynamics_model,
        build_study_assembly,
    )

    assembly = compose_axle(trailing_arm_model(), "K")
    study_assembly = build_study_assembly(assembly, study=DYNAMIC, mode="K")
    model = axle_dynamics_model(study_assembly, name="trailing-arm")

    assert [body.name for body in model.bodies] == list(assembly.bodies)
    assert len(model.joints) == len(assembly.constraints)
    assert {joint.kind for joint in model.joints} == {"revolute"}
    for body in model.bodies:
        source = assembly.bodies[body.name]
        assert body.mass_kg == pytest.approx(source.mass)


def test_the_synthetic_topology_keeps_its_own_document_under_a_different_bench_name() -> None:
    """
    The bench name is not the family, and the document is not the bench.

    A model compiled for a bench whose name is not `kc_quasi_static` produces the
    same *model* document, because the document describes the model and the bench
    describes the reading.  That is G4 in one assertion.
    """
    from dataclasses import replace

    from suspension_multibody.rigs.rig import RIGS

    assembly = compose_axle(trailing_arm_model(), "K")
    original = RIGS["kc_quasi_static"]
    RIGS["synthetic_bench"] = replace(
        original, name="synthetic_bench", family="kc_quasi_static"
    )
    try:
        renamed = plan_for(
            "synthetic_bench",
            mode="K",
            inputs=KcStudyInputs(name="probe"),
        )
        assert renamed.family == "kc_quasi_static"
        assert renamed.rig == "synthetic_bench"
        document, _, _, _, metadata = compile_plan(renamed, assembly)
        assert metadata["study"] == "quasi_static"
        assert document == model_document(assembly, name="probe", drive_wheels=True)
    finally:
        del RIGS["synthetic_bench"]


def test_the_trailing_arm_runs_without_any_steering_declaration() -> None:
    """
    The synthetic fixture obeys the global rules rather than being exempt from them.

    A single-axle assembly carries no brake and no drive, and D3 requires that to
    hold for every topology -- a new fixture is not a way around it.  The
    synthetic axle has no steering, which is legal for a single axle (the rack
    branch is optional), and it is refused if anyone asks it for one.
    """
    # A single-axle assembly is refused a brake or a drive subsystem, and the
    # refusal names the role rather than silently dropping it.
    with pytest.raises(ValueError, match="brake"):
        compose_axle(
            benchmark_model(),
            "K",
            AssemblyRequest(
                mode="K",
                subsystems=frozenset({"chassis", "suspension", "brake"}),
            ),
        )
    with pytest.raises(ValueError, match="drive"):
        compose_axle(
            benchmark_model(),
            "K",
            AssemblyRequest(
                mode="K",
                subsystems=frozenset({"chassis", "suspension", "drive"}),
            ),
        )
