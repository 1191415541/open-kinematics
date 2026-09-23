"""
Tire mass is a change of *ownership*, so the answer must not move.

Decision D2 makes the tire its own inertia source: part of what the wheel-end body
used to carry is declared on the tire instead.  That is a re-attribution, not new
physics, and the way to say so is to run the same axle twice -- once with the whole
inertia on the body, once with a part of it on the tire -- and require the two runs
to agree.

"Agree" here means *bit for bit*.  The split is chosen so that body+tire equals the
whole exactly in binary floating point, and the effective-inertia sum is built once
at model build time; if any term were dropped, double-counted or reordered, the
dynamics would drift immediately.  A tolerance would hide exactly the mistake this
test exists to catch, so there is none.

The second half is the opposite question: is the declared mass actually *used*?  A
field that reaches `Tire` and nothing else would pass the conservation test while
doing nothing at all, so the same model is run with the tire mass on and off and
the results are required to differ.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts import pack_container

from suspension_multibody.cases import axle_dynamic_model_document
from suspension_multibody.cases.axle_dynamic import case_document
from suspension_multibody.simulation import (
    CompilerRegistry,
    DocumentPairCompiler,
    SimulationRequest,
    run_request,
)

#: The acceptance script owns the axle model the kernel cases are checked against.
_ACCEPTANCE = Path(__file__).resolve().parents[2] / "scripts/run_axle_dynamics_acceptance.py"

#: The case the axle dynamics baselines use.
_CASE = "road_pulse"

#: Binary-exact split of the wheel's inertia.  The body keeps a positive-definite
#: inertia (the model loader requires one) and the tire takes the rest, so the sum
#: is the whole with no rounding anywhere:
#:
#:     2.0 + 20.0 == 22.0        (mass)
#:     0.5 +  0.25 == 0.75       (x and z inertia)
#:     0.5 +  0.5  == 1.0        (y inertia, the spin axis)
BODY_MASS, BODY_INERTIA = 2.0, (0.5, 0.5, 0.5)
TIRE_MASS, TIRE_INERTIA = 20.0, (0.25, 0.5, 0.25)
WHOLE_MASS, WHOLE_INERTIA = 22.0, (0.75, 1.0, 0.75)


def _diagonal(values: tuple[float, float, float]) -> list[list[float]]:
    """Return a diagonal 3x3, in the document's row-major form."""
    return [[values[0], 0.0, 0.0], [0.0, values[1], 0.0], [0.0, 0.0, values[2]]]


@pytest.fixture(scope="module")
def axle() -> dict:
    """Return the emitted axle-dynamics document and a runner for variants."""
    spec = importlib.util.spec_from_file_location("acceptance", _ACCEPTANCE)
    assert spec is not None and spec.loader is not None
    acceptance = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(acceptance)
    model = acceptance.build_axle_model()
    document, _ = axle_dynamic_model_document(model)
    case, case_blob = case_document(model, acceptance.build_case(_CASE))
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("axle", "axle_dynamic"))

    def run(variant: dict) -> object:
        return run_request(
            SimulationRequest(
                assembly="axle",
                family="axle_dynamic",
                model=variant,
                case=case,
                context={"case_payload": pack_container(case, case_blob)},
            ),
            registry=registry,
        ).raw

    return {"document": document, "run": run}


def _with_wheel_mass(document: dict, mass: float, inertia: tuple[float, float, float]) -> dict:
    """Return the document with both wheel bodies carrying `mass`/`inertia`."""
    bodies = [
        {**body, "mass": mass, "inertia": _diagonal(inertia)}
        if body["name"].startswith("wheel_")
        else body
        for body in document["bodies"]
    ]
    return {**document, "bodies": bodies}


def _with_tire_mass(document: dict, mass: float, inertia: tuple[float, float, float]) -> dict:
    """Return the document with every tire declaring its own `mass`/`inertia`."""
    tires = [
        {**tire, "mass": mass, "inertia": _diagonal(inertia)} for tire in document["tires"]
    ]
    return {**document, "tires": tires}


def test_the_declared_split_is_exact() -> None:
    """
    The comparison is only meaningful if the split does not itself lose anything.

    Every value here is a sum of binary fractions, so body + tire equals the whole
    exactly.  A split that rounded would make the two runs differ for a reason that
    has nothing to do with the coupling.
    """
    assert BODY_MASS + TIRE_MASS == WHOLE_MASS
    assert tuple(b + t for b, t in zip(BODY_INERTIA, TIRE_INERTIA)) == WHOLE_INERTIA


def test_moving_inertia_from_the_body_to_the_tire_conserves_the_answer(axle: dict) -> None:
    """
    The mass-conservation assertion, run before any numerical comparison.

    Both runs carry the *same* total wheel inertia; they differ only in who owns
    it.  `body_state` and `energy` must therefore be bit-identical, which is the
    strongest available statement that the change is a re-attribution.
    """
    whole = axle["run"](_with_wheel_mass(axle["document"], WHOLE_MASS, WHOLE_INERTIA))
    split_document = _with_tire_mass(
        _with_wheel_mass(axle["document"], BODY_MASS, BODY_INERTIA),
        TIRE_MASS,
        TIRE_INERTIA,
    )
    split = axle["run"](split_document)

    for block in ("body_state", "energy"):
        before, after = whole.block(block), split.block(block)
        assert np.array_equal(before, after), (
            f"{block} changed when the same inertia moved from the wheel body to the "
            f"tire; max|diff| = {float(np.max(np.abs(before - after))):.3e}"
        )


def test_a_declared_tire_mass_actually_changes_the_answer(axle: dict) -> None:
    """
    Ask whether the declared mass is coupled in, or merely stored.

    The same body inertia is run twice: once with the tires declaring their mass,
    once with the identical model minus that declaration.  If the two agreed, the
    field would be reaching `Tire` and stopping there -- which is what the previous
    step left behind, and which the conservation test alone cannot detect.
    """
    with_mass = axle["run"](
        _with_tire_mass(
            _with_wheel_mass(axle["document"], BODY_MASS, BODY_INERTIA),
            TIRE_MASS,
            TIRE_INERTIA,
        )
    )
    without = axle["run"](
        _with_tire_mass(
            _with_wheel_mass(axle["document"], BODY_MASS, BODY_INERTIA),
            0.0,
            (0.0, 0.0, 0.0),
        )
    )
    before, after = without.block("body_state"), with_mass.block("body_state")
    assert not np.array_equal(before, after), (
        "declaring 20 kg on each tire left the run bit-identical to declaring none, "
        "so the solver is not reading the tire's mass"
    )


def test_a_document_without_tire_mass_is_untouched(axle: dict) -> None:
    """
    The historical path must be exactly what it was.

    Every document written before the field existed declares no tire mass, so the
    effective inertia equals the body's own and the run must be unchanged.  The
    frozen dynamic-hash baseline covers this too, but a failure here says *why*.
    """
    plain = axle["run"](axle["document"])
    zeroed = axle["run"](_with_tire_mass(axle["document"], 0.0, (0.0, 0.0, 0.0)))
    for block in ("body_state", "energy"):
        assert np.array_equal(plain.block(block), zeroed.block(block))
