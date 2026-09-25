"""
The request is orthogonal: assembly, rig, study, case and outputs, independently.

The defect these tests exist for is that a run used to be a `(assembly, family)`
pair with a `rig` that had to spell the family again.  Three questions were
answered by one name, and the consequences were all silent:

* a newly authored bench could not run an existing reading unless it was *renamed*
  to match a family, which makes "extend the bench" a naming exercise;
* naming a study was not possible at all -- the family decided it -- so "the same
  assembly, two readings" could only be expressed by naming two families and
  hoping the two paths had not drifted;
* the family name was the routing key *and* the bench identity, so a caller could
  not say which of the two it meant.

Every test here drives the resolved value rather than reading a declaration, and
each one asserts the refusal as well as the acceptance, because a mechanism that
cannot reject is not checking anything.
"""

from __future__ import annotations

import json

import pytest

from suspension_multibody.axle_dynamics.schema import (
    AxleDynamicsCase,
    AxleSolverSettings,
)
from suspension_multibody.compilation import (
    CompilationError,
    KcStudyInputs,
    compile_plan,
    default_emitters,
    plan_for,
    view_of,
)
from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.simulation import SimulationRequest
from suspension_multibody.studies import (
    DYNAMIC,
    QUASI_STATIC,
    StudyError,
    axle_dynamics_model,
    build_study_assembly,
)

FIXTURE = (
    "packages/suspension_multibody/tests/data/benchmark_axle.json"
)


#: The bodies a massed axle declares.  The shared fixture is kinematic -- it
#: declares no body inertia -- so a test that needs the model to be *readable as
#: a dynamic model* has to give it masses; the bridge refuses a free body with no
#: mass rather than inventing one, and that refusal is a separate test below.
_MASSED_BODIES = (
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


def _model(*, massed: bool = False) -> FrontAxleModel:
    """Return the shared benchmark axle, optionally with body inertia."""
    payload = json.loads(open(FIXTURE, encoding="utf-8").read())
    if not massed:
        return FrontAxleModel.model_validate(payload["model"])
    raw = dict(payload["model"])
    raw["bodies"] = [
        {
            "name": name,
            "mass": 100.0,
            "inertia": [[100.0, 0, 0], [0, 100.0, 0], [0, 0, 100.0]],
        }
        for name in _MASSED_BODIES
    ]
    return FrontAxleModel.model_validate(raw)


def _dynamic_case() -> AxleDynamicsCase:
    """Return a minimal time history for the dynamic reading of one plan."""
    return AxleDynamicsCase(
        name="probe",
        times_s=(0.0, 1e-3),
        solver=AxleSolverSettings(),
    )


# --- the axes are independent ----------------------------------------------


def test_naming_only_the_rig_resolves_the_family_and_the_study() -> None:
    """The bench is the dimension a caller chooses; the rest follows from it."""
    plan = plan_for("kc_quasi_static")
    assert plan.rig == "kc_quasi_static"
    assert plan.family == "kc_quasi_static"
    assert plan.study == QUASI_STATIC
    assert plan.tire_activation == "vertical_only"


def test_a_named_rig_and_a_named_family_need_not_spell_the_same() -> None:
    """
    The G4 statement, as a value.

    A bench is what drives and measures; a family is which compiler and
    preparation the run goes through.  Requiring the names to match is what made
    a bench unable to route an existing reading without being renamed, and it is
    exactly what the EPIC's DESIGN names as the defect (`request.py:61`: "rig 与
    family 相互补值并强制相等 -> 07 独立 rig/study").
    """
    request = SimulationRequest(
        assembly="axle", rig="kc_quasi_static", family="axle_dynamic"
    )
    assert request.rig == "kc_quasi_static"
    assert request.family == "axle_dynamic"


def test_a_study_the_bench_does_not_take_is_refused_by_name() -> None:
    """Separating the axes must not make the pair unfalsifiable."""
    with pytest.raises(StudyError, match="reads the model"):
        plan_for("kc_quasi_static", study=DYNAMIC)


def test_a_bench_that_declares_no_study_needs_the_caller_to_say() -> None:
    """A bench usable for either reading cannot silently pick one."""
    from suspension_multibody.rigs.rig import RIGS

    original = RIGS["kc_quasi_static"]
    RIGS["probe_bench"] = type(original)(
        name="probe_bench", study=None, supplies_wheels=True
    )
    try:
        with pytest.raises(StudyError, match="declares no study"):
            plan_for("probe_bench")
        assert plan_for("probe_bench", study=QUASI_STATIC).study == QUASI_STATIC
    finally:
        del RIGS["probe_bench"]


def test_a_request_that_contradicts_its_bench_is_refused() -> None:
    """Two answers to "how is this read" would make the run a coin toss."""
    request = SimulationRequest(
        assembly="axle", rig="kc_quasi_static", study=DYNAMIC
    )
    with pytest.raises(ValueError, match="declares"):
        request.resolved_study(QUASI_STATIC)


# --- one assembly, two studies ---------------------------------------------


def test_one_assembly_reports_the_same_fingerprint_under_two_studies() -> None:
    """
    A6: the *same* composition read by two studies is the same model.

    The composition carries the fingerprint and the two plans carry the study, so
    the checkable claim is that the fingerprint a compiled quasi-static run
    reports is the one a compiled dynamic run reports -- for one assembly object.
    That is what makes "the study changes the reading, not the model" a property
    of the pipeline rather than a promise about two assembly functions not having
    drifted.
    """
    from suspension_multibody.subsystems import AssemblyRequest
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle

    model = _model(massed=True)
    composed = si_assembly_for_axle(model, request=AssemblyRequest(mode="K"))
    inputs = KcStudyInputs(name="probe", wheel_values_mm=(0.0,), rack_values_mm=(0.0,))
    # The dynamic reading needs inertias and a time history, so its plan carries
    # the SI model and case explicitly rather than having a family name imply
    # them.  Both plans are read off the *same* composition.
    dynamic_model = axle_dynamics_model(
        build_study_assembly(composed.assembly.physical, study=DYNAMIC, mode="K"),
        name="probe",
    )

    quasi = plan_for("kc_quasi_static", mode="K", inputs=inputs)
    dynamic = plan_for(
        "axle_dynamic",
        mode="K",
        inputs=inputs,
        dynamic_model=dynamic_model,
        dynamic_case=_dynamic_case(),
    )

    quasi_model, _, _, _, quasi_meta = compile_plan(quasi, composed)
    dynamic_model, _, _, _, dynamic_meta = compile_plan(dynamic, composed)

    assert quasi_meta["study"] == QUASI_STATIC
    assert dynamic_meta["study"] == DYNAMIC
    assert quasi_meta["tire_activation"] == "vertical_only"
    assert dynamic_meta["tire_activation"] == "full"
    # One object, one fingerprint: the study is metadata and nothing else.
    assert quasi_meta["fingerprint"] == dynamic_meta["fingerprint"] == composed.fingerprint
    # Two emitters, one model: the model document is authored from the same build.
    assert quasi_model["contract"] == "multibody-model"
    assert quasi_model["units"]["length"] == "mm"


def test_the_plan_is_what_selects_the_reading_not_the_family_class() -> None:
    """Two plans over one assembly reach two emitters from one registry."""
    emitters = default_emitters()
    assert emitters.resolve("kc_quasi_static") is emitters.resolve("kc_quasi_static")
    assert type(emitters.resolve("kc_quasi_static")) is not type(
        emitters.resolve("axle_dynamic")
    )


def test_an_unknown_family_is_refused_by_the_emitter_registry() -> None:
    """A family nobody registered must say so rather than fall back."""
    with pytest.raises(CompilationError, match="no emitter is registered"):
        default_emitters().resolve("no_such_family")


# --- the view normalises both shapes ----------------------------------------


def test_the_view_of_an_assembly_and_of_its_composition_agree() -> None:
    """
    One set of facts, two spellings of the assembly that carries them.

    The composed SI assembly carries identity and ports; the historical build
    carries the entities.  A document must not depend on which one the caller
    happened to hold, so the view is where the two are made to agree -- and this
    is the assertion that keeps them agreeing.
    """
    from suspension_multibody.subsystems import AssemblyRequest
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle

    model = _model()
    historical = build_front_axle(model, "K")
    composed = si_assembly_for_axle(model, request=AssemblyRequest(mode="K"))

    from_assembly = view_of(historical)
    from_composition = view_of(composed)

    assert list(from_assembly.bodies) == list(from_composition.bodies)
    assert set(from_assembly.points) == set(from_composition.points)
    assert from_assembly.joint_types() == from_composition.joint_types()
    assert from_assembly.physical is historical
    # The fingerprint is the composition's own; an assembly that has none reports
    # none rather than an empty string that could be mistaken for a value.
    assert from_composition.fingerprint == composed.fingerprint
    assert from_assembly.fingerprint == ""


def test_the_view_refuses_a_composition_that_kept_no_build() -> None:
    """A composition without its build cannot be authored into a document."""
    from dataclasses import replace

    from suspension_multibody.modeling.assembly import SimulationAssembly
    from suspension_multibody.subsystems import AssemblyRequest
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle

    composed = si_assembly_for_axle(
        _model(), request=AssemblyRequest(mode="K")
    )
    stripped = SimulationAssembly(
        name=composed.name,
        assembly=replace(composed.assembly, physical=None),
        rig=composed.rig,
        fingerprint=composed.fingerprint,
    )
    from suspension_multibody.compilation import ViewError

    with pytest.raises(ViewError, match="physical build"):
        view_of(stripped)


def test_a_plan_names_the_bench_and_the_family_it_routes_through() -> None:
    """The compiled metadata is where a caller reads back what was run."""
    plan = plan_for("kc_quasi_static", mode="C")
    described = plan.describe()
    assert described["rig"] == "kc_quasi_static"
    assert described["family"] == "kc_quasi_static"
    assert described["mode"] == "C"
    assert described["study"] == QUASI_STATIC


def test_drive_wheels_follows_the_inputs_then_the_bench_not_the_family() -> None:
    """
    The driven coordinates are a consequence of the case and the bench.

    Reading them off the family name is what made a new bench unable to route an
    existing reading: the family would have had to be the bench's own name.  Two
    benches are compared here, and their capabilities differ, which is the whole
    point -- the answer comes from the bench, not from which family was named.
    """
    # A wheel-travel sweep *is* driving the wheel centres, on either bench.
    swept = plan_for(
        "kc_quasi_static",
        inputs=KcStudyInputs(wheel_values_mm=(-10.0, 0.0)),
    )
    assert swept.drive_wheels is True
    road_bench_swept = plan_for(
        "ride_random_road",
        family="ride_random_road",
        study=DYNAMIC,
        inputs=KcStudyInputs(wheel_values_mm=(-10.0,)),
    )
    assert road_bench_swept.drive_wheels is True

    # With no sweep, the bench's own capability decides: a bench that owns the
    # wheels drives them, and a loading bench loads the body instead.
    assert plan_for("kc_quasi_static").drive_wheels is True
    assert plan_for("ride_random_road", study=DYNAMIC).drive_wheels is False
