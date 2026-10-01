"""
The signal bus: reads that equal the result document, and writes that edit a document.

The two halves are tested against a real run rather than a hand-built array,
because the property that matters is that a bus reading **is** the number the
result document carries -- a fixture would only prove the bus agrees with itself.
"""

from __future__ import annotations

import importlib.util
import pathlib

import numpy as np
import pytest

from suspension_multibody.signal_bus import (
    ACTUATOR_CHANNELS,
    MEASUREMENT_CHANNELS,
    BusError,
    SignalBus,
    measurement_channels_without_declaration,
    open_bus,
)

_FIXTURE = pathlib.Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"
_CONTRACT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "cases"
    / "test_vehicle_dynamic_contract.py"
)


def _contract_module():
    spec = importlib.util.spec_from_file_location("vd_contract", _CONTRACT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def run_document():
    """Produce a real run with the wheels spinning, so the reads are not vacuous."""
    contract = _contract_module()
    fixture = contract._fixture()
    model = fixture._positioned_vehicle(fixture._vehicle())
    case = fixture._case(
        model,
        wheel_speeds=tuple(
            (name, 12.0)
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
    )
    raw = contract._run(model, case)
    assert raw.status == "success"
    return raw


def test_the_read_channels_exist_and_are_named(run_document) -> None:
    """
    At least one wheel-speed read and one body-acceleration read are offered.

    Named rather than indexed, which is the whole point: a caller asks for
    ``wheel_speed`` and gets the wheel's own rate.
    """
    names = {channel.name for channel in MEASUREMENT_CHANNELS}
    assert {"wheel_speed", "body_acceleration"} <= names
    bus = open_bus(run_document)
    assert bus.measurement("wheel_speed").unit == "rad/s"
    assert bus.measurement("body_acceleration").unit == "mm/s^2"


def test_a_wheel_speed_read_is_the_result_documents_own_number(run_document) -> None:
    """
    The bus reading equals the body-state block, bit for bit.

    This is the parity the design promises and the reason the bus is a reader
    rather than a computer: both sides are the *same* array, so they cannot drift.
    """
    bus = open_bus(run_document)
    wheel = next(name for name in run_document.body_names if "wheel_hub" in name)
    read = bus.read("wheel_speed", entity=wheel)
    direct = run_document.block("body_state")[
        -1, run_document.body_names.index(wheel), 10:13
    ]
    assert np.array_equal(read, direct)
    # And it is not zero, so the equality is not vacuous.
    assert float(np.abs(read).sum()) > 0.0


def test_a_body_acceleration_read_is_the_result_documents_own_number(run_document) -> None:
    """The same parity for the second read channel, taken from a different body."""
    bus = open_bus(run_document)
    read = bus.read("body_acceleration", entity="chassis")
    direct = run_document.block("body_state")[
        -1, run_document.body_names.index("chassis"), 13:16
    ]
    assert np.array_equal(read, direct)


def test_a_tire_load_read_is_the_tires_own_column(run_document) -> None:
    """A third read channel, on a different block, with the same parity rule."""
    bus = open_bus(run_document)
    tire = run_document.tire_names[0]
    read = bus.read("tire_vertical_load", entity=tire)
    direct = run_document.block("tire_output")[-1, 0, 4]
    assert np.array_equal(read, np.asarray([direct], dtype=float))


def test_naming_no_entity_is_refused_rather_than_guessed(run_document) -> None:
    """
    A read that does not say which entity is refused, and says what it would have
    picked.  A silently-guessed entity is a reading nobody asked for.
    """
    bus = open_bus(run_document)
    with pytest.raises(BusError, match="none was named"):
        bus.read("wheel_speed")


def test_an_unknown_channel_is_refused_by_name(run_document) -> None:
    """The refusal lists what the bus does offer, so the caller can fix the typo."""
    bus = open_bus(run_document)
    with pytest.raises(BusError, match="wheel_speed"):
        bus.read("wheel_speeed")


def test_a_write_edits_a_copy_and_leaves_the_original_alone() -> None:
    """
    Writing returns a new document.

    A run's inputs are its own; a bus that rewrote the caller's mapping would make
    "the document I submitted" and "the document I am describing" the same object.
    """
    document = {"elements": {"damper_L": {"parameters": {"compression_damping": 1.0}}}}
    bus = open_bus(case_document=document)
    written = bus.write("variable_damping_L", 42.0)
    assert written["elements"]["damper_L"]["parameters"]["compression_damping"] == 42.0
    assert document["elements"]["damper_L"]["parameters"]["compression_damping"] == 1.0
    assert written is not document


def test_the_motor_torque_write_lands_where_it_says_it_does() -> None:
    """The second write channel creates the path it names when the document lacks it."""
    bus = open_bus(case_document={})
    written = bus.write("motor_torque_FL", 1234.0)
    assert written == {"body_wrench": {"front_wheel_hub_L": {"moment": 1234.0}}}


def test_a_write_channel_names_a_path_inside_a_document() -> None:
    """Both actuator channels state a non-empty path, so a write is inspectable."""
    for channel in ACTUATOR_CHANNELS:
        assert channel.path, channel.name
        assert all(isinstance(key, str) and key for key in channel.path), channel.name


def test_a_non_finite_command_is_refused() -> None:
    """An actuator command that is not a finite number is a mistake, not a setting."""
    bus = open_bus(case_document={})
    with pytest.raises(BusError, match="finite"):
        bus.write("motor_torque_FL", float("nan"))


def test_a_write_through_a_scalar_is_refused_with_the_path_named() -> None:
    """A path that passes through a non-mapping is refused, naming the key."""
    bus = open_bus(case_document={"elements": 5})
    with pytest.raises(BusError, match="elements"):
        bus.write("variable_damping_L", 1.0)


def test_reading_without_a_run_and_writing_without_a_document_are_refused() -> None:
    """Each half refuses when the bus was opened without it."""
    with pytest.raises(BusError, match="no run"):
        SignalBus().read("wheel_speed")
    with pytest.raises(BusError, match="no document"):
        SignalBus().write("motor_torque_FL", 1.0)


def test_the_declaration_set_is_the_source_of_truth_for_the_channels() -> None:
    """
    Every bus channel states the declaration it corresponds to.

    The declaration set (`outputs/`) is what a run says it produces; the bus is a
    typed reader over it.  A channel with no declaration behind it is a claim
    nobody has made, and this reports rather than hides it.
    """
    for channel in MEASUREMENT_CHANNELS:
        assert channel.declaration, channel.name
    # The report is answerable against a caller-supplied declaration list.
    declared = [channel.declaration for channel in MEASUREMENT_CHANNELS]
    assert measurement_channels_without_declaration(declared) == ()
    assert len(measurement_channels_without_declaration([])) == len(MEASUREMENT_CHANNELS)
