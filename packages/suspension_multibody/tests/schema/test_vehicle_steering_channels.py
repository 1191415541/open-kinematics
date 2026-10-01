"""
The steering-channel declarations, and what they must not change.

Subtask p2-06 keeps `VehicleModel.steering` exactly what it was -- the mandatory
compatibility primary channel -- and adds the *additional* channels beside it as
`steering_channels`, excluded from `model_dump` for the same reason several other
fields are: `api.py` hashes that dump into `Provenance.model_hash`.

The gate is therefore the one the epic states as a hard door: a one-channel
vehicle must dump, hash and key-set exactly as it did before channels existed.
The digest below is that value, measured before the change and recorded here as
the frozen reference.

The second half of the file is the refusals: a channel set whose own declarations
disagree is refused by name, because three of the four conditions are answerable
from the declared names alone and the fourth -- which channel steers which axle --
is exactly what the schema deliberately does *not* guess.
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from suspension_multibody.io import canonical_hash
from suspension_multibody.schema import (
    SteeringChannelSpec,
    SteeringSystemSpec,
    VehicleModel,
)

#: `canonical_hash(model.model_dump(mode="json"))` for the `full_vehicle_model`
#: fixture as it stood before steering channels existed.  Recorded rather than
#: re-derived: it is the value the recorded full-vehicle model hashes carry, and
#: a test that recomputed it would agree with whatever the schema became.
FROZEN_ONE_CHANNEL_HASH = (
    "75cb96f55a7b120eff172b0760c378d016a99e7caeb7320e925ff500d9903fd9"
)


def test_the_model_still_has_twelve_dumped_keys(full_vehicle_model: VehicleModel) -> None:
    """The dump's key set is the model's public shape, and it did not move."""
    payload = full_vehicle_model.model_dump(mode="json")

    assert len(payload) == 12
    assert set(payload) == {
        "schema_version",
        "name",
        "units",
        "coordinate_system",
        "chassis",
        "front_axle",
        "rear_axle",
        "wheels",
        "steering",
        "driveline",
        "coordinate_couplers",
        "aerodynamic_drag",
    }
    assert "steering_channels" not in payload
    assert "allocation_law" not in payload


def test_the_declared_channel_adds_no_key_to_the_dump(
    full_vehicle_model: VehicleModel,
) -> None:
    """
    A second channel is declared, and the hash does not move.

    This is the point of `exclude=True`: the declaration is typed, validated and
    usable, and invisible to the dump `api.py` hashes.
    """
    declared = _with_channels(
        full_vehicle_model,
        steering_channels=(
            SteeringChannelSpec(
                rack_body="rack",
                ratio=16.0,
                channel_name="rear_rack",
                placement="rear",
            ),
        ),
    )

    payload = declared.model_dump(mode="json")
    assert len(payload) == 12
    assert "steering_channels" not in payload
    assert canonical_hash(payload) == FROZEN_ONE_CHANNEL_HASH
    assert canonical_hash(declared.model_dump(mode="json")) == canonical_hash(
        full_vehicle_model.model_dump(mode="json")
    )


def test_the_one_channel_hash_is_the_recorded_value(
    full_vehicle_model: VehicleModel,
) -> None:
    """The frozen reference itself, measured against the fixture."""
    assert (
        canonical_hash(full_vehicle_model.model_dump(mode="json"))
        == FROZEN_ONE_CHANNEL_HASH
    )


def test_steering_remains_a_required_singleton() -> None:
    """`steering` was, and is, the one field a vehicle cannot leave out."""
    assert VehicleModel.model_fields["steering"].is_required()
    assert "steering" in VehicleModel.model_fields
    assert "steering_channels" in VehicleModel.model_fields
    # The additional channels are optional and default to none of them.
    assert not VehicleModel.model_fields["steering_channels"].is_required()


def test_the_primary_channel_is_addressable_and_placed_by_default(
    full_vehicle_model: VehicleModel,
) -> None:
    """The compatibility channel carries the name and placement it always had."""
    assert full_vehicle_model.steering.channel_name == "front_rack"
    assert full_vehicle_model.steering.placement == "front"
    assert full_vehicle_model.steering.enabled is True
    assert full_vehicle_model.steering_channels == ()
    assert full_vehicle_model.allocation_law == "direct"


def test_a_channel_name_is_stated_by_the_additional_channel() -> None:
    """An additional channel cannot be anonymous: the name is how it is addressed."""
    from pydantic import ValidationError

    with pytest.raises(ValidationError):
        SteeringChannelSpec(rack_body="rack", ratio=1.0, placement="rear")
    with pytest.raises(ValidationError):
        SteeringChannelSpec(rack_body="rack", ratio=1.0, channel_name="rear_rack")


def _with_channels(full_vehicle_model: VehicleModel, **updates) -> VehicleModel:
    """
    Return the fixture vehicle with ``updates`` applied, *through validation*.

    `model_copy` is a shallow copy and runs no validator, so it cannot be used to
    ask whether the model's own rules reject a channel set: the answer would
    always be "the object was copied".  Re-validating the fields is what puts the
    question to `_topology`, which is where the rules live.
    """
    return VehicleModel.model_validate(
        {
            **{
                name: getattr(full_vehicle_model, name)
                for name in VehicleModel.model_fields
            },
            **updates,
        }
    )


def test_two_channels_with_one_name_are_refused_by_name(
    full_vehicle_model: VehicleModel,
) -> None:
    """A duplicated name would make the channel unaddressable."""
    with pytest.raises(ValidationError, match="rear_rack"):
        _with_channels(
            full_vehicle_model,
            steering_channels=(
                SteeringChannelSpec(
                    rack_body="rack", ratio=1.0, channel_name="rear_rack", placement="rear"
                ),
                SteeringChannelSpec(
                    rack_body="rack",
                    ratio=1.0,
                    channel_name="rear_rack",
                    placement="third",
                ),
            ),
        )
    # And against the primary's own name.
    with pytest.raises(ValidationError, match="front_rack"):
        _with_channels(
            full_vehicle_model,
            steering_channels=(
                SteeringChannelSpec(
                    rack_body="rack",
                    ratio=1.0,
                    channel_name="front_rack",
                    placement="rear",
                ),
            ),
        )


def test_two_channels_at_one_placement_are_refused_by_name(
    full_vehicle_model: VehicleModel,
) -> None:
    """Two channels at one placement would drive the same axle twice."""
    with pytest.raises(ValidationError, match="rear"):
        _with_channels(
            full_vehicle_model,
            steering_channels=(
                SteeringChannelSpec(
                    rack_body="rack", ratio=1.0, channel_name="rear_rack", placement="rear"
                ),
                SteeringChannelSpec(
                    rack_body="rack",
                    ratio=1.0,
                    channel_name="third_rack",
                    placement="rear",
                ),
            ),
        )


def test_a_blank_primary_name_is_refused(full_vehicle_model: VehicleModel) -> None:
    """A channel addressed by whitespace has no name to be addressed by."""
    with pytest.raises(ValidationError, match="channel_name"):
        _with_channels(
            full_vehicle_model,
            steering=full_vehicle_model.steering.model_copy(
                update={"channel_name": "   "}
            ),
        )


def test_a_blank_primary_placement_is_refused(full_vehicle_model: VehicleModel) -> None:
    """A channel that steers no named axle would have its rack resolved by guessing."""
    with pytest.raises(ValidationError, match="placement"):
        _with_channels(
            full_vehicle_model,
            steering=full_vehicle_model.steering.model_copy(update={"placement": " "}),
        )


def test_an_actuator_against_itself_is_refused(full_vehicle_model: VehicleModel) -> None:
    """
    A channel whose actuator body and reaction body are the same is not a load path.

    This is the same refusal `drive_torque_reaction_body` states for a wheel: a
    couple needs two ends, and one body is not two.
    """
    with pytest.raises(ValidationError, match="against itself"):
        _with_channels(
            full_vehicle_model,
            steering=SteeringSystemSpec(
                rack_body="rack",
                ratio=16.0,
                actuator_body="chassis",
                actuator_reaction_body="chassis",
            ),
        )


def test_an_empty_body_name_is_refused(full_vehicle_model: VehicleModel) -> None:
    """A channel naming an empty rack body names no rack."""
    with pytest.raises(ValidationError, match="rack_body"):
        _with_channels(
            full_vehicle_model,
            steering=full_vehicle_model.steering.model_copy(
                update={"rack_body": "  "}
            ),
        )


def test_the_spec_is_a_public_schema_name() -> None:
    """The additional-channel class is importable from the package's schema."""
    import suspension_multibody.schema as schema

    assert "SteeringChannelSpec" in schema.__all__
    assert schema.SteeringChannelSpec is SteeringChannelSpec


def test_channel_identity_is_not_read_off_a_placement_name(
    full_vehicle_model: VehicleModel,
) -> None:
    """
    Nothing in the model refuses a channel because of the word it places itself at.

    Subtask p2-06 is explicitly *not* allowed to guess a channel's axle from a
    name: `placement` is the declaration.  A channel at an unusual placement is
    therefore accepted by the model and only refused later, by the preparation,
    when it turns out the vehicle does not place an axle there.
    """
    declared = _with_channels(
        full_vehicle_model,
        steering_channels=(
            SteeringChannelSpec(
                rack_body="rack",
                ratio=16.0,
                channel_name="middle_rack",
                placement="middle",
            ),
        ),
    )

    assert declared.steering_channels[0].placement == "middle"
