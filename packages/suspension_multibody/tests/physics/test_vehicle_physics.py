"""Self-consistency checks for full-vehicle loads and roll-center geometry."""

import numpy as np

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.authoring.migration import migrate_v1_vehicle
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.simulation import run_compiled
from suspension_multibody.vehicle.static_loads import compute_static_wheel_loads


def _resolved_vehicle(model):
    return assemble_generic(migrate_v1_vehicle(model)).resolved_model()


def _static_loads(model, **kwargs):
    contacts = {wheel.name: f"{wheel.body}.center" for wheel in model.wheels}
    return compute_static_wheel_loads(_resolved_vehicle(model), contact_frames=contacts,
        gravity=9.81, **kwargs)


def _roll_center_run(model):
    resolved = _resolved_vehicle(model)
    graph = resolved.to_document()
    for body in graph["bodies"]:
        body["fixed"] = body["name"] == "body.chassis"
    graph["gravity"] = [0, 0, 0]
    graph["elements"] = []
    graph["measurements"] = []
    for placement in ("front", "rear"):
        contacts, drives = [], []
        for side in ("L", "R"):
            drive = f"{placement}_wheel.sub.json.wheel_center_{side}"
            frame = next(row for row in graph["frames"] if row["name"] == drive)
            contact = drive + ".contact"
            graph["frames"].append(dict(frame, name=contact, point=[0, 0, -.3]))
            contacts.append(contact)
            drives.append(drive)
        graph["measurements"].append({"name": placement, "type": "roll_center", "units": "m",
            "reference": "world", "constraints": [row["name"] for row in graph["joints"]],
            "contact_frames": contacts, "drive_frames": drives})
    plan = ResolvedSolvePlan({"schema_version": 1, "name": "geometry", "study": "dynamic",
        "protocol": "vehicle_dynamic", "samples": [0, .0001], "boundaries": [],
        "inputs": [], "outputs": [], "solver": {"initialization_mode": "provided_consistent_state"}})
    return run_compiled(compile_resolved(ResolvedModel(graph, resolved.resource_payload), plan)).result


def test_static_wheel_loads_balance_weight_and_moments(full_vehicle_model) -> None:
    result = _static_loads(full_vehicle_model)

    assert result.rank == 3
    # A roundoff-scale residual rather than a physical one: the loads are ~1e4 N, so
    # this bound is 1e-10 of the quantity being balanced.  It is sensitive to the mass
    # distribution the composed runtime carries -- 方式 A's hub and the steering
    # housing are bodies of their own -- so the bound is stated at the scale of the
    # arithmetic rather than at the one a particular fixture happened to hit.
    assert result.residual < 1e-6
    assert all(value > 0.0 for value in result.wheel_loads.values())
    assert np.isclose(
        result.summary.total,
        result.total_mass * 9.81,
        rtol=0.0,
        atol=1e-8,
    )
    assert np.isclose(result.summary.left_side, result.summary.right_side)
    assert np.isclose(result.summary.front_axle, result.summary.rear_axle)


def test_longitudinal_acceleration_transfers_load_rearward(full_vehicle_model) -> None:
    static = _static_loads(full_vehicle_model)
    accelerated = _static_loads(
        full_vehicle_model,
        acceleration=np.array([1.0, 0.0, 0.0]),
    )

    assert accelerated.summary.front_axle < static.summary.front_axle
    assert accelerated.summary.rear_axle > static.summary.rear_axle


def test_positive_lateral_acceleration_transfers_load_to_negative_y_side(
    full_vehicle_model,
) -> None:
    static = _static_loads(full_vehicle_model)
    accelerated = _static_loads(
        full_vehicle_model,
        acceleration=np.array([0.0, 1.0, 0.0]),
    )

    assert accelerated.summary.left_side > static.summary.left_side
    assert accelerated.summary.right_side < static.summary.right_side
    assert accelerated.summary.right_left_delta < 0.0


def test_front_and_rear_roll_centers_are_finite_and_symmetric(full_vehicle_model) -> None:
    """
    Both axles report a finite roll centre on the vehicle centreline.

    The height is solved from the axle's own force-to-generalized-displacement
    derivative matrix (see ``vehicle/roll_centers``); the value asserted here is the
    one the double-wishbone fixture's geometry produces, and it is pinned so a change
    to the construction shows up as a number rather than as a shape that still fits.
    """
    envelope = _roll_center_run(full_vehicle_model)
    centers = {name: envelope.measure(name) for name in ("front", "rear")}
    assert set(centers) == {"front", "rear"}
    for placement, result in centers.items():
        assert np.all(np.isfinite(result.values))
        np.testing.assert_allclose(result.values[:, 0], 0.0, atol=1e-11)
        # The fixture's wishbone arms slope the same way on both sides, so the two
        # force lines meet at the same height the arm-line construction gave: -180 mm
        # below the road plane.
        np.testing.assert_allclose(result.values[:, 1], -.180, atol=1e-7)
        left = envelope.frame_pose(f"{placement}_wheel.sub.json.wheel_center_L.contact")
        right = envelope.frame_pose(f"{placement}_wheel.sub.json.wheel_center_R.contact")
        np.testing.assert_allclose(left[:, 1, 3], -right[:, 1, 3], atol=1e-12)
        assert result.units == "m"


def test_static_wheel_loads_are_the_minimum_norm_solution(full_vehicle_model) -> None:
    """
    The retained Python solver must still return the *minimum norm* load split.

    The four vertical reactions are underdetermined (three balance equations),
    so the algorithm -- not the physics alone -- picks one of a family of valid
    answers.  An equal-front-rear, equal-left-right layout has a symmetric
    minimum norm split; a solver that returned any other balanced solution (or
    an unconstrained least-squares fit) would fail this.  This pins the
    algorithmic choice that 2026-09-22 decision A2 keeps in Python.
    """
    result = _static_loads(full_vehicle_model)

    assert result.rank == 3
    loads = result.wheel_loads
    quarter = result.summary.total / 4.0
    for name, value in loads.items():
        assert np.isclose(value, quarter, rtol=1e-9, atol=1e-8), name
    # The minimum-norm member of the balanced family is the uniform split, so
    # equality with the quarter load is the algorithm's fingerprint: a solver
    # returning any other balanced solution would satisfy the balance checks
    # above but fail here.
