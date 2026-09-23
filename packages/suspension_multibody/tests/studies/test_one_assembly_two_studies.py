"""
One assembly, two readings.

The requirement is that `kc_quasi_static` and `axle_dynamic` differ in how the
model is *read*.  That can only be true if both readings start from the same
object, so the tests here check identity rather than similarity: the same
`FrontAxleAssembly` is handed to both studies, and the documents each study
produces are checked to be derived from that one object.

The last test is the one that keeps the guarantee honest: the bridge refuses an
assembly it cannot read, instead of quietly producing a dynamic model that is
missing something.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    Vec3,
)
from suspension_multibody.studies import (
    DYNAMIC,
    QUASI_STATIC,
    BridgeError,
    axle_dynamics_model,
    build_study_assembly,
    get_study,
    study_model_document,
)

_BODY_NAMES = (
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


def _model(*, inertia: float = 100.0, mass: float = 100.0) -> FrontAxleModel:
    """Return a double wishbone axle that declares body inertia."""
    return FrontAxleModel(
        name="probe",
        hardpoints={
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
        },
        mass=MassSpec(sprung_mass=600),
        bodies=tuple(
            RigidBodySpec(
                name=name,
                mass=mass,
                inertia=((inertia, 0, 0), (0, inertia, 0), (0, 0, inertia)),
            )
            for name in _BODY_NAMES
        ),
    )


def test_both_studies_read_the_same_assembly_object() -> None:
    """
    The point of the whole subtask: one assembly, not two.

    Handing the same object to both studies is only meaningful if the object *is*
    the same, so the check is identity -- `is`, not `==` -- because two equal
    assemblies built twice would defeat the purpose quietly.
    """
    model = _model()
    quasi = build_study_assembly(model, study=QUASI_STATIC)
    dynamic = build_study_assembly(quasi.assembly, study=DYNAMIC)
    assert dynamic.assembly is quasi.assembly
    assert dynamic.study_name == DYNAMIC
    assert quasi.study_name == QUASI_STATIC


def test_the_two_readings_come_from_one_assembly_function() -> None:
    """
    A model and an already-built assembly must reach the same construction.

    If the study entry built its own assembly when handed a model, "one path"
    would hold only for callers who happened to pass an assembly.  Comparing the
    assembly built from a model against the one built from that same model by the
    package's own entry point is what rules that out.
    """
    from suspension_multibody.preparation.assembly import build_front_axle

    model = _model()
    via_study = build_study_assembly(model, study=QUASI_STATIC).assembly
    directly = build_front_axle(model, "K")
    assert list(via_study.bodies) == list(directly.bodies)
    assert set(via_study.points) == set(directly.points)
    for key, point in via_study.points.items():
        assert np.array_equal(np.asarray(point), np.asarray(directly.points[key])), key


def test_the_mode_belongs_to_the_assembly_not_to_the_study() -> None:
    """A compliant axle read quasi-statically is legitimate, so no study implies C."""
    model = _model()
    for study in (QUASI_STATIC, DYNAMIC):
        compliant = build_study_assembly(model, study=study, mode="C")
        assert compliant.mode == "C"
        assert compliant.assembly.mode == "C"


def test_asking_a_study_for_a_mode_the_assembly_does_not_have_is_refused() -> None:
    model = _model()
    rigid = build_study_assembly(model, study=QUASI_STATIC, mode="K")
    with pytest.raises(ValueError, match="mode"):
        build_study_assembly(rigid.assembly, study=DYNAMIC, mode="C")


def test_the_quasi_static_reading_still_emits_the_kc_contract() -> None:
    """The K/C reading is unchanged: same document shape as before the study work."""
    assembly = build_study_assembly(_model(), study=QUASI_STATIC)
    document = study_model_document(assembly, name="probe")
    assert document["contract"] == "multibody-model"
    assert [body["name"] for body in document["bodies"]][0] == "chassis"
    assert "joints" in document and document["joints"]


def test_the_dynamic_reading_is_derived_from_that_same_assembly() -> None:
    """Bodies, joints and masses all come from the assembly the study was handed."""
    assembly = build_study_assembly(_model(), study=DYNAMIC)
    model = axle_dynamics_model(assembly, name="probe")
    assert [body.name for body in model.bodies] == list(assembly.assembly.bodies)
    assert len(model.joints) == len(assembly.assembly.constraints)
    by_name = {body.name: body for body in model.bodies}
    for name, body in assembly.assembly.bodies.items():
        assert by_name[name].mass_kg == pytest.approx(body.mass)
        assert by_name[name].fixed == body.fixed


def test_a_body_with_no_mass_is_refused_rather_than_given_one() -> None:
    """
    A kinematic fixture may declare bodies with no inertia.

    The dynamic schema needs a real mass, so the conversion stops and names the
    body.  Inventing a value would produce a model that solves and means nothing,
    which is exactly the failure a bridge is supposed to prevent.
    """
    model = _model(mass=0.0)
    assembly = build_study_assembly(model, study=DYNAMIC)
    with pytest.raises(BridgeError, match="carries no mass"):
        axle_dynamics_model(assembly, name="probe")


def test_the_bridge_reports_which_body_it_could_not_read() -> None:
    """The message names the body, so the fix is obvious from the failure alone."""
    model = _model(mass=0.0)
    assembly = build_study_assembly(model, study=DYNAMIC)
    with pytest.raises(BridgeError) as error:
        axle_dynamics_model(assembly, name="probe")
    assert "rack" in str(error.value)


def test_a_study_assembly_carries_its_study_for_readers() -> None:
    assembly = build_study_assembly(_model(), study=QUASI_STATIC)
    assert assembly.tire_activation == get_study(QUASI_STATIC).tire_activation
    assert assembly.study.name == QUASI_STATIC
