"""
The rig and the study, as wired into the running product.

The three layers exist as declarations in ``rigs/``, ``studies/`` and
``templates/``; what these tests hold is that the *product* goes through them.
A declaration that nothing calls is a diagram, and the failure it hides is not a
crash -- it is a run that quietly does something else.  So every test here drives
a real entry point and asks what actually happened.

The bridge tests are the ones with a history: the element walk used to select by
name and drop everything else, so an assembly carrying springs produced a dynamic
model without them.  A missing spring is invisible in the result -- the run still
converges -- which is exactly why it needs an assertion rather than a review.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.preparation.assembly.types import RigidBody
from suspension_multibody.preparation.kc_quasi_static import assembly_for
from suspension_multibody.rigs import resolve_combination
from suspension_multibody.schema import (
    AntiRollBar,
    BumpStop,
    FrontAxleModel,
    LinearSpring,
    MassSpec,
    RigidBodySpec,
    StaticDamper,
    Vec3,
)
from suspension_multibody.simulation import SimulationRequest
from suspension_multibody.studies import (
    DYNAMIC,
    QUASI_STATIC,
    BridgeError,
    axle_dynamics_model,
    build_study_assembly,
)

_BODIES = (
    "rack",
    "upper_arm_L",
    "lower_arm_L",
    "upright_L",
    "tie_rod_L",
    "upper_arm_R",
    "lower_arm_R",
    "upright_R",
    "tie_rod_R",
)

_HARDPOINTS = {
    "UPPER_INBOARD_FRONT": Vec3(x=0, y=-500, z=500),
    "UPPER_INBOARD_REAR": Vec3(x=150, y=-500, z=500),
    "UPPER_OUTBOARD": Vec3(x=0, y=-750, z=350),
    "LOWER_INBOARD_FRONT": Vec3(x=0, y=-500, z=100),
    "LOWER_INBOARD_REAR": Vec3(x=150, y=-500, z=100),
    "LOWER_OUTBOARD": Vec3(x=0, y=-750, z=100),
    "TIE_ROD_INBOARD": Vec3(x=0, y=-450, z=250),
    "TIE_ROD_OUTBOARD": Vec3(x=0, y=-750, z=250),
    "WHEEL_CENTER": Vec3(x=0, y=-750, z=300),
    "RACK_CENTER": Vec3(x=0, y=0, z=250),
}


def _model(**overrides) -> FrontAxleModel:
    """Return a massed double wishbone axle, optionally carrying elements."""
    fields = {
        "name": "wired",
        "hardpoints": dict(_HARDPOINTS),
        "mass": MassSpec(sprung_mass=600),
        "bodies": tuple(
            RigidBodySpec(
                name=name,
                mass=100.0,
                inertia=((100.0, 0, 0), (0, 100.0, 0), (0, 0, 100.0)),
            )
            for name in _BODIES
        ),
    }
    fields.update(overrides)
    return FrontAxleModel(**fields)


# --- the request routes on the rig -----------------------------------------


def test_naming_only_the_rig_fills_in_the_family() -> None:
    """The rig is the routing dimension, so it alone must resolve a request."""
    request = SimulationRequest(assembly="axle", rig="kc_quasi_static")
    assert request.family == "kc_quasi_static"
    assert request.rig == "kc_quasi_static"
    assert request.kind == "kc_quasi_static"


def test_naming_only_the_family_still_resolves() -> None:
    """Every existing caller passes a family, and must keep working."""
    request = SimulationRequest(assembly="axle", family="axle_dynamic")
    assert request.rig == "axle_dynamic"
    assert request.family == "axle_dynamic"


def test_a_rig_that_disagrees_with_the_family_is_refused() -> None:
    """Both name the same bench, so a mismatch is a caller error, not a route."""
    with pytest.raises(ValueError, match="must agree"):
        SimulationRequest(
            assembly="axle", rig="kc_quasi_static", family="axle_dynamic"
        )


def test_a_request_with_neither_a_rig_nor_a_family_is_refused() -> None:
    with pytest.raises(ValueError, match="needs a family or a rig"):
        SimulationRequest(assembly="axle")


def test_a_registered_elsewhere_family_is_left_to_its_registry() -> None:
    """
    The request does not police the registries.

    A compiler or preparation may be registered by a caller -- a test, or a
    downstream product -- so refusing an unknown family here would make the
    registries unextendable.  Its own lookup is where the error belongs.
    """
    request = SimulationRequest(assembly="axle", family="probe")
    assert request.rig == "probe"


def test_a_family_typo_is_caught_by_the_registry_that_owns_it() -> None:
    """
    A misspelled family must be discoverable, not silently accepted.

    The request deliberately does not police family names -- the registries are
    extensible -- so the error surfaces one layer down, at the lookup.  What
    matters is that it *does* surface and names what is available; a fallback
    added there later would turn this back into a silent wrong run.
    """
    from suspension_multibody.simulation import compile_request, prepare_request

    request = SimulationRequest(assembly="axle", family="kc_quasi_staticx")
    with pytest.raises(KeyError, match="kc_quasi_staticx"):
        prepare_request(request)

    # A document request skips preparation and goes straight to compilation,
    # which is the other registry that must refuse it by name.
    document_request = SimulationRequest(
        assembly="axle",
        family="kc_quasi_staticx",
        model={"contract": "multibody-model"},
        case={"contract": "multibody-case"},
    )
    with pytest.raises(KeyError, match="kc_quasi_staticx"):
        compile_request(document_request)


# --- the bench shrinks to the assembly it is given --------------------------


def test_the_drivable_set_is_what_the_bench_asks_for() -> None:
    """
    The set is the bench's declaration shrunk to the assembly, `rack_drive`
    included.

    `rack_neutral` is deliberately absent: it is a coordinate the case layer can
    hold, not one this way of running the bench drives, and the old reading that
    returned it also returned a coordinate nothing downstream consulted.
    """
    from suspension_multibody.api import _k_drivable_coordinates

    assembly = build_front_axle(_model(), "K")
    assert _k_drivable_coordinates(assembly) == frozenset(
        {"wheel_drive_L", "wheel_drive_R", "rack_drive"}
    )


def test_an_assembly_without_steering_loses_only_the_rack() -> None:
    from suspension_multibody.api import _k_drivable_coordinates
    from suspension_multibody.subsystems import (
        DEFAULT_AXLE_SUBSYSTEMS,
        AssemblyRequest,
    )

    hardpoints = {k: v for k, v in _HARDPOINTS.items() if k != "RACK_CENTER"}
    assembly = build_front_axle(
        _model(hardpoints=hardpoints),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    assert _k_drivable_coordinates(assembly) == frozenset(
        {"wheel_drive_L", "wheel_drive_R"}
    )


def test_the_bench_and_the_drivable_set_agree() -> None:
    """
    One decision, read twice: the bench drops a coordinate exactly when the
    drivable set does not carry it.

    The two are computed by different code paths -- `rigs.compose` and the api's
    reading of it -- so this is the assertion that keeps them one answer.
    """
    from suspension_multibody.api import _k_drivable_coordinates
    from suspension_multibody.subsystems import (
        DEFAULT_AXLE_SUBSYSTEMS,
        AssemblyRequest,
    )

    hardpoints = {k: v for k, v in _HARDPOINTS.items() if k != "RACK_CENTER"}
    assembly = build_front_axle(
        _model(hardpoints=hardpoints),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    composition = resolve_combination("axle", "kc_quasi_static", assembly.capabilities)
    assert "rack_drive" in composition.dropped
    drivable = _k_drivable_coordinates(assembly)
    for drive in composition.drives:
        assert drive.coordinate in drivable
    assert "rack_drive" not in drivable


# --- preparation checks the bench against the assembly ----------------------


def test_a_bench_of_another_kind_is_refused_at_preparation() -> None:
    """
    A four-post bench drives a vehicle, and saying so belongs before the run.

    The refusal has to name the bench: "your axle is missing a part" would send
    the reader to the model instead of to the pair they asked for.
    """
    from suspension_multibody.rigs import CompositionError

    with pytest.raises(CompositionError, match="ride_four_post"):
        assembly_for(_model(), mode="K", rig="ride_four_post")


def test_a_compatible_bench_passes_preparation() -> None:
    assembly = assembly_for(_model(), mode="K", rig="kc_quasi_static")
    assert assembly.mode == "K"


def test_the_assembly_entry_builds_through_the_study_layer() -> None:
    """
    The assembly a K/C run uses must be the one the study layer builds.

    This is what keeps the two readings one construction: an assembly built by
    the family entry and one built by `build_front_axle` directly have to be the
    same thing, or a quasi-static run and a dynamic one would start from
    different models while claiming to share one.
    """
    direct = build_front_axle(_model(), "K")
    via_entry = assembly_for(_model(), mode="K", rig="kc_quasi_static")
    assert list(via_entry.bodies) == list(direct.bodies)
    assert set(via_entry.points) == set(direct.points)
    assert len(via_entry.constraints) == len(direct.constraints)
    assert len(via_entry.elements) == len(direct.elements)
    assert via_entry.mode == direct.mode


def test_the_mode_is_checked_against_the_assembly_it_is_handed() -> None:
    """A C assembly asked for the K reading is refused, not silently re-read."""
    with pytest.raises(ValueError, match="mode"):
        assembly_for(build_front_axle(_model(), "C"), mode="K", rig="kc_quasi_static")


def test_a_public_run_resolves_its_bench(monkeypatch) -> None:
    """
    A K/C run through the public entry point must consult the rig.

    Before this, `api` built the assembly itself and handed the documents it
    authored straight to the runner, so the family preparation -- and with it the
    rig check and the study layer -- was never reached.  The run still succeeded;
    the missing step was invisible in the result, which is why it needs a test
    rather than a review.
    """
    import suspension_multibody.rigs as rigs
    from tests.benchmark_fixture import benchmark_model

    seen: list[tuple[str, str]] = []
    original = rigs.check_assembly

    def record(assembly: str, rig: str, capabilities) -> None:
        seen.append((assembly, rig))
        original(assembly, rig, capabilities)

    monkeypatch.setattr(rigs, "check_assembly", record)

    import suspension_multibody.api as api
    from suspension_multibody.schema import CaseSpec

    with tempfile.TemporaryDirectory() as directory:
        api.run_case(benchmark_model(), CaseSpec(mode="K"), Path(directory))

    assert seen == [("axle", "kc_quasi_static")]


# --- the same assembly feeds both studies ----------------------------------


def test_one_assembly_becomes_a_dynamic_model() -> None:
    """The whole point of the study merge, driven through the product."""
    assembly = build_front_axle(_model(), "K")
    study_assembly = build_study_assembly(assembly, study=DYNAMIC, mode="K")
    model = axle_dynamics_model(study_assembly, name="wired")

    assert [body.name for body in model.bodies] == list(assembly.bodies)
    assert len(model.joints) == len(assembly.constraints)
    by_name = {body.name: body for body in model.bodies}
    for name, body in assembly.bodies.items():
        assert by_name[name].mass_kg == pytest.approx(body.mass)


def test_the_axle_family_accepts_an_assembled_axle() -> None:
    """
    The dynamic family takes either an SI model or an assembled K/C axle.

    This is the route the study merge adds: without it, "the same assembly, two
    readings" would hold inside `studies/` and nowhere a caller could reach.
    """
    from suspension_multibody.preparation.axle_dynamic import _dynamic_model

    assembly = build_front_axle(_model(), "K")
    request = SimulationRequest(
        assembly="axle", family="axle_dynamic", model=assembly, name="wired"
    )
    model = _dynamic_model(assembly, request)
    assert [body.name for body in model.bodies] == list(assembly.bodies)
    assert model.units == "SI"


def test_an_si_model_still_passes_straight_through() -> None:
    """The original input stays the original input."""
    from suspension_multibody.preparation.axle_dynamic import _dynamic_model

    assembly = build_front_axle(_model(), "K")
    request = SimulationRequest(
        assembly="axle", family="axle_dynamic", model=assembly, name="wired"
    )
    model = _dynamic_model(assembly, request)
    assert _dynamic_model(model, request) is model


def test_the_dynamic_family_refuses_a_model_it_cannot_read() -> None:
    from suspension_multibody.preparation.axle_dynamic import _dynamic_model

    request = SimulationRequest(assembly="axle", family="axle_dynamic")
    with pytest.raises(TypeError, match="AxleDynamicsModel"):
        _dynamic_model("not a model", request)


# --- the bridge keeps every element it can, and names the ones it cannot ----


def _spring_pair() -> tuple[LinearSpring, StaticDamper]:
    """Return a spring and a damper on the same two points."""
    point = Vec3(x=0.0, y=-600.0, z=100.0)
    other = Vec3(x=0.0, y=-600.0, z=400.0)
    return (
        LinearSpring(
            name="corner_spring",
            body_a="chassis",
            body_b="lower_arm_L",
            point_a=point,
            point_b=other,
            stiffness=200.0,
            free_length=250.0,
        ),
        StaticDamper(
            name="corner_damper",
            body_a="chassis",
            body_b="lower_arm_L",
            point_a=point,
            point_b=other,
            viscous_damping=12.0,
        ),
    )


def test_a_spring_and_a_damper_reach_the_dynamic_model() -> None:
    """
    An element the assembly carries must not vanish on the way to SI.

    The failure this guards is the quiet one: a dropped spring leaves a model
    that solves, converges and is wrong.  The assertion is on the *count* and on
    the stiffness, because a mapping that emitted an empty entry would satisfy a
    presence check.
    """
    spring, damper = _spring_pair()
    assembly = build_front_axle(_model(springs=(spring,), dampers=(damper,)), "K")
    assert assembly.elements, "the fixture assembled no elements"

    study_assembly = build_study_assembly(assembly, study=DYNAMIC, mode="K")
    model = axle_dynamics_model(study_assembly, name="wired")

    # The axle mirrors the left side, so each declared corner element appears
    # twice -- which is itself the evidence that the mapping ran rather than an
    # entry being fabricated.
    names = {entry.name for entry in model.springs}
    assert names == {
        "corner_spring_L",
        "corner_spring_R",
        "corner_damper_L",
        "corner_damper_R",
    }
    spring_entry = next(e for e in model.springs if e.name == "corner_spring_L")
    damper_entry = next(e for e in model.springs if e.name == "corner_damper_L")
    # N/mm to N/m, and the damper's rate is carried as damping rather than
    # stiffness -- mixing them would be a silent physical error.
    assert spring_entry.stiffness_n_per_m == pytest.approx(200.0 * 1000.0)
    assert spring_entry.compression_damping_n_s_per_m == 0.0
    assert damper_entry.compression_damping_n_s_per_m == pytest.approx(12.0)
    assert damper_entry.stiffness_n_per_m == 0.0


def test_a_preloaded_spring_keeps_its_resting_length() -> None:
    """
    The two schemas state a spring's offset differently, and the mapping must
    reconcile them.

    An assembly spring is `k * (length - reference) + preload`; the SI entry is
    `k * (length - free_length)`.  Reading the reference length straight into
    `free_length` would drop the preload term, which moves the corner's resting
    position -- a change nothing in the run would report.
    """
    point = Vec3(x=0.0, y=-600.0, z=100.0)
    other = Vec3(x=0.0, y=-600.0, z=400.0)
    # 200 N/mm, referenced at 300 mm with 1000 N of preload: the zero-force
    # length is 300 - 1000/200 = 295 mm.
    spring = LinearSpring(
        name="preloaded",
        body_a="chassis",
        body_b="lower_arm_L",
        point_a=point,
        point_b=other,
        stiffness=200.0,
        reference_length=300.0,
        preload=1000.0,
    )
    assembly = build_front_axle(_model(springs=(spring,)), "K")
    study_assembly = build_study_assembly(assembly, study=DYNAMIC, mode="K")
    model = axle_dynamics_model(study_assembly, name="wired")

    entry = next(e for e in model.springs if e.name == "preloaded_L")
    assert entry.free_length_m == pytest.approx(0.295)
    assert entry.stiffness_n_per_m == pytest.approx(200.0 * 1000.0)


def test_a_bump_stop_with_no_dynamic_reading_is_named_not_dropped() -> None:
    """
    The bridge refuses what it cannot express, and says which element.

    Silently omitting it is the one failure a solver cannot report: the run
    still converges.  Naming the element is what makes the gap a task instead of
    a mystery.
    """
    assembly = build_front_axle(
        _model(
            stops=(
                BumpStop(
                    name="stop",
                    body_a="chassis",
                    body_b="lower_arm_L",
                    point_a=Vec3(x=0.0, y=-600.0, z=120.0),
                    point_b=Vec3(x=0.0, y=-600.0, z=380.0),
                    clearance=20.0,
                    stiffness=500.0,
                ),
            ),
        ),
        "K",
    )
    carried = {type(element).__name__ for element in assembly.elements}
    if "BumpStopElement" not in carried:
        pytest.skip("this assembly path does not carry the bump stop element")

    study_assembly = build_study_assembly(assembly, study=DYNAMIC, mode="K")
    with pytest.raises(BridgeError, match="BumpStopElement"):
        axle_dynamics_model(study_assembly, name="wired")


def test_an_anti_roll_bar_with_no_dynamic_reading_is_named_not_dropped() -> None:
    """The same refusal for the other element the schema cannot read."""
    assembly = build_front_axle(
        _model(
            anti_roll_bars=(
                AntiRollBar(
                    name="arb",
                    left_body_mount=Vec3(x=0.0, y=-300.0, z=200.0),
                    right_body_mount=Vec3(x=0.0, y=300.0, z=200.0),
                    left_arm_end=Vec3(x=0.0, y=-700.0, z=200.0),
                    right_arm_end=Vec3(x=0.0, y=700.0, z=200.0),
                    left_link_point=Vec3(x=0.0, y=-700.0, z=100.0),
                    right_link_point=Vec3(x=0.0, y=700.0, z=100.0),
                    torsional_stiffness=1000.0,
                ),
            ),
        ),
        "K",
    )
    carried = {type(element).__name__ for element in assembly.elements}
    if "AntiRollBarElement" not in carried:
        pytest.skip("this assembly path does not carry the anti-roll bar element")

    study_assembly = build_study_assembly(assembly, study=DYNAMIC, mode="K")
    with pytest.raises(BridgeError, match="AntiRollBarElement"):
        axle_dynamics_model(study_assembly, name="wired")


def test_the_quasi_static_reading_still_emits_the_kc_contract() -> None:
    """The other half of the pair: adding a dynamic route changes nothing here."""
    from suspension_multibody.studies import study_model_document

    assembly = build_study_assembly(
        build_front_axle(_model(), "K"), study=QUASI_STATIC
    )
    document = study_model_document(assembly, name="wired")
    assert document["contract"] == "multibody-model"
    assert document["joints"]


def test_a_body_without_mass_has_no_dynamic_reading() -> None:
    """
    A kinematic fixture may declare bodies it never gives inertia to.

    The SI schema needs a real mass, so the conversion stops and names the body
    rather than inventing one.  This drives it through an assembly so the
    refusal is exercised where it actually happens.
    """
    assembly = build_front_axle(_model(), "K")
    massless = dict(assembly.bodies)
    massless["rack"] = RigidBody(
        name="rack",
        pose=assembly.bodies["rack"].pose,
        mass=0.0,
        inertia=assembly.bodies["rack"].inertia,
        center_of_mass=assembly.bodies["rack"].center_of_mass,
    )
    stripped = type(assembly)(
        mode=assembly.mode,
        bodies=massless,
        state=assembly.state,
        points=assembly.points,
        hardpoints=assembly.hardpoints,
        connections=assembly.connections,
        constraints=assembly.constraints,
        ideal_constraints=assembly.ideal_constraints,
        bushings=assembly.bushings,
        elements=assembly.elements,
        capabilities=assembly.capabilities,
    )
    study_assembly = build_study_assembly(stripped, study=DYNAMIC, mode="K")
    with pytest.raises(BridgeError, match="rack"):
        axle_dynamics_model(study_assembly, name="wired")
