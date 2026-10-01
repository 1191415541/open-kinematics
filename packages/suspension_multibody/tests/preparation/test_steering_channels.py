"""
Distributed steering channels: the schema, and a two-channel vehicle that runs.

Subtask p2-06 widens steering from one system to a declared list of channels.  The
hard requirement is the one the model hash rides on: a one-channel declaration
must dump, hash and drive exactly as it did before channels existed
(`api.py` hashes `model_dump(mode="json")` into `Provenance.model_hash`).  The
other half is that a two-channel vehicle actually assembles and runs -- both
through the preparation, down to the two channels' own steering elements in the
model contract, and through the solver, so the two racks really are driven.

The vehicle here is the file fixture's (`tests/authoring/fixtures.py`), with one
extra `steering_channels` row appended to its `vehicle` section.  Its rear
suspension file leaves `rack_fixed_to_chassis` free, which is what a rear channel
needs: a bolted rack has no guide to prescribe a translation along, and
`_rack_guide_joint` refuses that by name rather than inventing an axis for it.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.authoring import AssemblyDocument, vehicle_model_from
from suspension_multibody.preparation.steering_allocator import (
    FOUR_WHEEL_STEER_HIGH_SPEED_GAIN,
    FOUR_WHEEL_STEER_LOW_SPEED_GAIN,
)


def _two_channel_model(tmp_path: Path, *, law: str = "direct"):
    """Write the file fixture's project with a rear channel, and read it back."""
    from tests.authoring.fixtures import write_vehicle_project

    paths = write_vehicle_project(tmp_path)
    payload = json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    payload["vehicle"]["steering_channels"] = [
        {
            "rack_body": "rack",
            "ratio": 1.0,
            "channel_name": "rear_rack",
            "placement": "rear",
        }
    ]
    paths["vehicle_assembly"].write_text(json.dumps(payload), encoding="utf-8")
    document = AssemblyDocument.load(paths["vehicle_assembly"])
    model = vehicle_model_from(document)
    return model.model_copy(update={"allocation_law": law}), document


def _solver(**overrides):
    from suspension_multibody.schema import DynamicSolverSettings, Vec3

    settings = dict(
        end_time=0.004,
        step_size=0.001,
        internal_step_size=0.001,
        min_internal_step_size=0.001,
        adaptive_substepping=False,
        integrator="generalized_alpha",
        # The vehicle needs a load path to settle into; the fixture's own cases
        # switch gravity on for exactly that reason.
        gravity=Vec3(x=0.0, y=0.0, z=-9806.65),
    )
    settings.update(overrides)
    return DynamicSolverSettings(**settings)


def _case(model, *, steer_rad: float = 0.02, speed_mps: float = 0.0):
    from suspension_multibody.schema import (
        RoadSurfaceSpec,
        TimeSignal,
        VehicleDynamicCase,
    )

    return VehicleDynamicCase(
        name="two-channel",
        vehicle=model,
        solver=_solver(),
        road=RoadSurfaceSpec(kind="plane"),
        steering_input=TimeSignal(
            times=(0.0, 0.004), values=(0.0, steer_rad)
        ),
        brake_input=TimeSignal(constant=0.0),
        initial_forward_speed_mps=speed_mps,
    )


def test_the_two_channel_document_states_both_channels(tmp_path: Path) -> None:
    """The channel a document names is the channel the model carries."""
    from suspension_multibody.schema import SteeringChannelSpec

    model, _document = _two_channel_model(tmp_path)

    assert isinstance(model.steering_channels[0], SteeringChannelSpec)
    assert model.steering_channels[0].channel_name == "rear_rack"
    assert model.steering_channels[0].placement == "rear"
    # The compatibility primary is untouched by the declaration.
    assert model.steering.channel_name == "front_rack"
    assert model.steering.placement == "front"


def test_the_preparation_builds_one_actuator_per_channel(tmp_path: Path) -> None:
    """
    Two channels, stacked: names, and every per-channel array two rows deep.

    The stacking order is the declaration order, and the two sample-major tables
    are laid out channel-minor -- `sample_index * count + channel_index`, the
    layout the kernel's own index expression reads.  A channel-major table would
    parse, and would drive each channel with the other's samples, so the layout is
    asserted rather than assumed.
    """
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

    model, _document = _two_channel_model(tmp_path)
    case = _case(model)
    prepared = prepare_vehicle_run(model, case)

    assert prepared.steering.names == ("front_rack", "rear_rack")
    assert prepared.steering.body.shape == (2,)
    assert prepared.steering.reaction_body.shape == (2,)
    assert prepared.steering.actuator_type.shape == (2,)
    assert prepared.steering.point_local.shape == (2, 3)
    assert prepared.steering.reaction_point_local.shape == (2, 3)
    assert prepared.steering.axis_local.shape == (2, 3)
    assert prepared.steering.reference_quaternion.shape == (2, 4)
    assert prepared.steering.stiffness.shape == (2,)
    assert prepared.steering.damping.shape == (2,)
    assert prepared.steering.output.shape == (len(prepared.times), 2, 4)
    assert prepared.steering.target.shape == (len(prepared.times) * 2,)
    assert prepared.steering.target_rate.shape == (len(prepared.times) * 2,)
    # The two racks are two different bodies: one channel driven twice would be
    # a vehicle whose rear wheels never turn.
    assert prepared.steering.body[0] != prepared.steering.body[1]
    table = prepared.steering.target.reshape(len(prepared.times), 2)
    assert np.allclose(table[:, 0], table[:, 1])


def test_direct_two_channel_run_drives_both_racks(tmp_path: Path) -> None:
    """
    The whole way through: both channels are driven, and both report a travel.

    The vehicles are run through the unified runner on the default preparation,
    which is the path the recorded full-vehicle results take.
    """
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
    from suspension_multibody.simulation import SimulationRequest, run_request

    model, _document = _two_channel_model(tmp_path, law="direct")
    case = _case(model)
    prepared = prepare_vehicle_run(model, case)
    run = run_request(
        SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": prepared},
        )
    ).raw

    assert run.status == "success", dict(run.failure_evidence)
    output = run.block("steering_output")
    assert output.shape[1] == 2
    # The tables are in the model document's units -- metres, because the
    # engineering-unit model is scaled once on the way out -- so the travel the
    # case asked for in millimetres is the same number over 1000.
    expected_m = 0.02 / 1000.0
    table = prepared.steering.target.reshape(len(prepared.times), 2)
    assert np.allclose(table[-1], expected_m)
    for index, name in enumerate(prepared.steering.names):
        final = float(output[-1, index, 2])
        assert abs(final - expected_m) < 1e-12, f"{name} reached {final} m"


@pytest.mark.parametrize(
    "speed,expected_gain",
    [(2.0, FOUR_WHEEL_STEER_LOW_SPEED_GAIN), (20.0, FOUR_WHEEL_STEER_HIGH_SPEED_GAIN)],
    ids=["low_speed_opposite", "high_speed_same_direction"],
)
def test_four_wheel_steer_reaches_the_channels_it_drives(
    tmp_path: Path, speed: float, expected_gain: float
) -> None:
    """
    The same driver input produces opposite rear angles at the two speeds.

    This is the pair the roadmap asks for: 4WS with the rear wheels turning
    *against* the front at low speed and *with* them at high speed, under one
    steering input.  The check is on the per-channel target tables the contract
    carries -- the numbers the kernel actually integrates -- and on the recorded
    allocation, so a schedule that was computed and then discarded cannot pass.
    The speed is the case's declared forward speed, which is the only speed the
    preparation layer has.
    """
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

    model, _document = _two_channel_model(tmp_path, law="four_wheel_steer")
    case = _case(model, steer_rad=0.02, speed_mps=speed)
    prepared = prepare_vehicle_run(model, case)
    table = prepared.steering.target.reshape(len(prepared.times), 2)

    # Radians in, millimetres of rack travel out, then metres in the document.
    scale = 1.0 / 1000.0
    assert table[-1, 0] == 0.02 * scale
    assert table[-1, 1] == expected_gain * 0.02 * scale

    assert [name for name, _series in prepared.steering_allocation] == [
        "front_rack",
        "rear_rack",
    ]
    front, rear = (series[-1] for _name, series in prepared.steering_allocation)
    assert front == 0.02
    assert rear == expected_gain * 0.02
    # The sign is the whole point of the schedule, so it is stated on its own.
    same_sign = (rear > 0.0) == (front > 0.0)
    assert same_sign == (expected_gain > 0.0)


def test_the_direct_law_is_what_a_one_channel_vehicle_gets(
    full_vehicle_model,
) -> None:
    """
    Without a law, nothing about the model's channels changes the driven targets.

    The compatibility hard gate: one channel, `direct` law, and the target table is
    the case's own steering signal through the channel's ratio -- no allocator
    arithmetic anywhere on the path.
    """
    from suspension_multibody.preparation.vehicle_dynamic import (
        _steering_channel_specs,
        prepare_vehicle_run,
    )
    from suspension_multibody.schema import (
        RoadSurfaceSpec,
        TimeSignal,
        VehicleDynamicCase,
    )

    assert full_vehicle_model.allocation_law == "direct"
    assert len(_steering_channel_specs(full_vehicle_model)) == 1

    case = VehicleDynamicCase(
        name="one-channel",
        vehicle=full_vehicle_model,
        solver=_solver(end_time=0.002, step_size=0.001),
        road=RoadSurfaceSpec(kind="plane"),
        steering_input=TimeSignal(times=(0.0, 0.002), values=(0.0, 0.016)),
        brake_input=TimeSignal(constant=0.0),
    )
    prepared = prepare_vehicle_run(full_vehicle_model, case)

    assert prepared.steering.names == ("front_rack",)
    # The input states a rack displacement, so the target *is* the signal -- the
    # ratio converts a steering-wheel angle and is not applied here -- and the
    # model's engineering units are scaled to the metres the document declares.
    column = prepared.steering.target.reshape(len(prepared.times), 1)[:, 0]
    expected = np.asarray(
        [case.steering_input.value_at(float(time)) for time in prepared.times]
    ) / 1000.0
    assert np.allclose(column, expected)
    assert column[-1] == 0.016 / 1000.0
    assert prepared.steering_allocation == ()


def test_the_two_channel_model_document_names_both_steering_elements(tmp_path: Path) -> None:
    """Both channels reach the model contract, each as its own element."""
    from suspension_multibody.cases.vehicle_dynamic import model_document
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

    model, _document = _two_channel_model(tmp_path)
    case = _case(model)
    prepared = prepare_vehicle_run(model, case)
    document, _blob = model_document(model, prepared)

    steering = [entry for entry in document["elements"] if entry["type"] == "steering_actuator"]
    assert [entry["name"] for entry in steering] == [
        "steering_front_rack",
        "steering_rear_rack",
    ]
    assert {entry["target"] for entry in steering} == {"front_rack", "rear_rack"}
    # Two different racks, named by two different bodies.
    bodies = {entry["parameters"]["body"] for entry in steering}
    assert len(bodies) == 2


def test_the_kc_family_asks_for_a_vehicle_without_steering_elements(tmp_path: Path) -> None:
    """
    A K/C sweep drives the rack itself, so it is not handed an actuator for it.

    The document is asked for a vehicle without steering elements rather than
    having them deleted afterwards: the deletion was the old form, and it would
    have to be repeated once per channel now that there are several.
    """
    from suspension_multibody.cases.vehicle_kc import (
        model_document,
        vehicle_model_document,
    )
    from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

    model, _document = _two_channel_model(tmp_path)
    case = _case(model)
    prepared = prepare_vehicle_run(model, case)

    base = vehicle_model_document(model, prepared)
    assert not [entry for entry in base[0]["elements"] if entry["type"] == "steering_actuator"]
    # And the sweep's own rack drive is what drives it instead.
    document, _blob = model_document(base, wheels=(), assembly=prepared.assembly)
    assert "rack_drive" in {joint["name"] for joint in document["joints"]}
