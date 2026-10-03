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


def test_a_damping_write_edits_one_element_and_leaves_the_original_alone() -> None:
    """
    Writing returns a new model document with one element's parameter set.

    ``elements`` is an array, so the entry is found by name; a document that
    carries no such element is refused rather than silently accepted.  A run's
    inputs are its own, so the caller's document is never mutated.
    """
    document = {
        "elements": [
            {
                "name": "damper_L",
                "type": "damper",
                "parameters": {"compression_damping": 1.0, "rebound_damping": 2.0},
            },
            {
                "name": "damper_R",
                "type": "damper",
                "parameters": {"compression_damping": 9.0, "rebound_damping": 9.0},
            },
        ]
    }
    bus = open_bus(model_document=document)
    written, blob = bus.write("variable_damping_L", 42.0)

    assert written is not document
    # Both directions of the damper move together: the solver picks between them
    # by the sign of the relative rate, so a channel that set one would leave the
    # other speaking for half the motion.
    assert written["elements"][0]["parameters"]["compression_damping"] == 42.0
    assert written["elements"][0]["parameters"]["rebound_damping"] == 42.0
    # The other element is untouched: the lookup is by name, not by position.
    assert written["elements"][1]["parameters"]["compression_damping"] == 9.0
    assert document["elements"][0]["parameters"]["compression_damping"] == 1.0
    assert document["elements"][0]["parameters"]["rebound_damping"] == 2.0
    assert blob == b""


def test_the_motor_torque_write_lands_in_the_payload_bytes_it_names() -> None:
    """
    The moment reaches the blob, at the descriptor's own offset.

    The kernel reads a case table by dereferencing its descriptor's ``offset``
    and ``length`` inside the payload, so a value written to the JSON root is a
    value the solver never sees.  This asserts the bytes, which is what the
    reader actually dereferences.
    """
    import numpy as np

    samples = 4
    values = np.zeros((samples, 6), dtype=np.float64)
    document = {
        "blobs": [
            {
                "role": "body_wrench",
                "body": "front_wheel_hub_L",
                "offset": 0,
                "length": values.nbytes,
                "dtype": "float64",
                "shape": [samples, 6],
            }
        ]
    }
    blob = values.tobytes()
    bus = open_bus(case_document=document, case_blob=blob)
    written, patched = bus.write("motor_torque_FL", 1234.0)

    assert patched != blob
    assert written is document
    read_back = np.frombuffer(patched, dtype=np.float64).reshape(samples, 6)
    # Every sample of the slot carries the command, so a table cannot disagree
    # with itself between its early and late rows.
    np.testing.assert_array_equal(read_back[:, 3:6], np.full((samples, 3), 1234.0))
    # The force columns are not touched: this channel is a moment, and writing
    # the whole row would silently zero a force the caller never mentioned.
    np.testing.assert_array_equal(read_back[:, 0:3], np.zeros((samples, 3)))


def test_a_write_channel_states_where_the_solver_reads_it() -> None:
    """Both channels name a real destination: an element parameter or a blob role."""
    for channel in ACTUATOR_CHANNELS:
        assert channel.document in {"model", "case"}, channel.name
        assert channel.element or channel.role, channel.name
        if channel.element:
            assert channel.parameters, channel.name
        if channel.role:
            assert channel.entity, channel.name


def test_a_non_finite_command_is_refused() -> None:
    """An actuator command that is not a finite number is a mistake, not a setting."""
    bus = open_bus(model_document={"elements": []})
    with pytest.raises(BusError, match="finite"):
        bus.write("variable_damping_L", float("nan"))


def test_a_damping_write_names_the_element_it_cannot_find() -> None:
    """A document without the named element is refused by name, not ignored."""
    bus = open_bus(model_document={"elements": [{"name": "damper_R", "parameters": {}}]})
    with pytest.raises(BusError, match="damper_L"):
        bus.write("variable_damping_L", 1.0)


def test_a_torque_write_without_its_blob_is_refused() -> None:
    """A table channel cannot land without the bytes the descriptor points at."""
    document = {
        "blobs": [
            {
                "role": "body_wrench",
                "body": "front_wheel_hub_L",
                "offset": 0,
                "length": 48,
                "dtype": "float64",
                "shape": [1, 6],
            }
        ]
    }
    bus = open_bus(case_document=document)
    with pytest.raises(BusError, match="blob"):
        bus.write("motor_torque_FL", 1.0)


def test_reading_without_a_run_and_writing_without_a_document_are_refused() -> None:
    """Each half refuses when the bus was opened without it."""
    with pytest.raises(BusError, match="no run"):
        SignalBus().read("wheel_speed")
    with pytest.raises(BusError, match="no case document"):
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

# --- (h) the write must move the SOLVED trajectory ---------------------------
#
# `EPIC.md:283` p5-03(d) is explicit: writing a value into an object is not
# evidence, "须有前后两次运行的轨迹对照".  The two tests below submit the same model
# and case twice -- once as built, once through `bus.write` -- and compare a named
# state series.  A bus that writes a JSON key the kernel never dereferences passes
# the dictionary tests above and fails these, which is exactly what happened.

#: The tolerance a state change must exceed.  It is a velocity in m/s and a
#: position in m, so rounding cannot produce it.
_TRAJECTORY_TOLERANCE = 1.0e-6


def _two_body_rig(damping: float | None):
    """One free body hung from ground by one damper: a real relative motion."""
    from suspension_multibody.axle_dynamics.schema import (
        AxleBody,
        AxleDamper,
        AxleDynamicsModel,
    )

    def diagonal(value: float):
        return ((value, 0.0, 0.0), (0.0, value, 0.0), (0.0, 0.0, value))

    ground = AxleBody(
        name="ground", mass_kg=0.0, inertia_kg_m2=diagonal(1.0), fixed=True
    )
    # The name is the one `motor_torque_FL` addresses, so the moment channel has
    # a target to reach on this rig.
    body = AxleBody(
        name="front_wheel_hub_L",
        mass_kg=20.0,
        inertia_kg_m2=diagonal(2.0),
        position_m=(0.0, 0.0, 0.30),
    )
    dampers = (
        (
            AxleDamper(
                name="damper_L",
                body_a="ground",
                body_b="front_wheel_hub_L",
                point_a_m=(0.0, 0.0, 0.60),
                point_b_m=(0.0, 0.0, 0.0),
                compression_damping_n_s_per_m=damping,
                rebound_damping_n_s_per_m=damping,
                extension_sign=1.0,
            ),
        )
        if damping is not None
        else ()
    )
    return AxleDynamicsModel(
        name="bus-write-rig",
        bodies=(ground, body),
        joints=(),
        dampers=dampers,
        tires=(),
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
    )


def _solve_axle(model, case):
    """Submit one axle model and case, returning the wheel's state series."""
    from suspension_contracts import pack_container

    from suspension_multibody.cases.axle_dynamic import case_document, model_document
    from suspension_multibody.simulation import (
        SimulationRequest,
        compile_document_pair,
        run_request,
    )

    model_doc, model_blob = model_document(model)
    case_doc, case_blob = case_document(model, case)
    raw = run_request(
        compile_document_pair(
            SimulationRequest(assembly="axle", family="axle_dynamic"),
            model_document=model_doc,
            case_document=case_doc,
            model_payload=pack_container(model_doc, model_blob),
            case_payload=pack_container(case_doc, case_blob),
        )
    ).raw
    return model_doc, model_blob, case_doc, case_blob, np.asarray(
        raw.body_state("front_wheel_hub_L"), dtype=float
    )


def _rig_case():
    from suspension_multibody.axle_dynamics.schema import (
        AxleDynamicsCase,
        AxleSolverSettings,
    )

    times = np.linspace(0.0, 0.05, 26)
    return AxleDynamicsCase(
        name="bus-write-rig",
        times_s=tuple(float(value) for value in times),
        solver=AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.0005,
            minimum_step_s=0.0001,
            maximum_step_s=0.0005,
        ),
    )


def test_a_damping_write_changes_the_solved_trajectory() -> None:
    """
    Writing the damper coefficient moves the state the solver produces.

    The model is submitted twice: once as authored, once with the coefficient the
    bus wrote into its element's ``parameters``.  A write that landed outside the
    element -- or outside the ``elements`` array -- would leave the two runs
    identical, which is the failure this asserts against.
    """
    from suspension_contracts import pack_container

    from suspension_multibody.simulation import (
        SimulationRequest,
        compile_document_pair,
        run_request,
    )

    model = _two_body_rig(1.0)
    case = _rig_case()
    model_doc, model_blob, case_doc, case_blob, baseline = _solve_axle(model, case)

    bus = open_bus(model_document=model_doc, case_document=case_doc, case_blob=case_blob)
    written, _blob = bus.write("variable_damping_L", 10_000.0)
    assert written is not model_doc

    raw = run_request(
        compile_document_pair(
            SimulationRequest(assembly="axle", family="axle_dynamic"),
            model_document=written,
            case_document=case_doc,
            model_payload=pack_container(written, model_blob),
            case_payload=pack_container(case_doc, case_blob),
        )
    ).raw
    after = np.asarray(raw.body_state("front_wheel_hub_L"), dtype=float)

    delta = float(np.max(np.abs(after - baseline)))
    assert delta > _TRAJECTORY_TOLERANCE, (delta, _TRAJECTORY_TOLERANCE)


def test_a_motor_torque_write_changes_the_solved_trajectory() -> None:
    """
    Writing the wheel moment moves the state the solver produces.

    The moment is a payload table, so the write has to land in the bytes the
    descriptor points at.  This is the counterpart of the damping test for the
    case side of the bus: the document is unchanged and the *blob* is what moved.
    """
    from suspension_contracts import pack_container

    from suspension_multibody.simulation import (
        SimulationRequest,
        compile_document_pair,
        run_request,
    )

    model = _two_body_rig(1.0)
    case = _rig_case()
    model_doc, model_blob, case_doc, case_blob, baseline = _solve_axle(model, case)

    samples = baseline.shape[0]
    moment = 50_000.0
    wrench = np.zeros((samples, 6), dtype=np.float64)
    offset = len(case_blob)
    case_doc = dict(case_doc)
    case_doc["blobs"] = list(case_doc.get("blobs", [])) + [
        {
            "role": "body_wrench",
            "body": "front_wheel_hub_L",
            "offset": offset,
            "length": wrench.nbytes,
            "dtype": "float64",
            "shape": [samples, 6],
        }
    ]
    case_blob = case_blob + wrench.tobytes()

    bus = open_bus(
        model_document=model_doc, case_document=case_doc, case_blob=case_blob
    )
    written_doc, written_blob = bus.write("motor_torque_FL", moment)
    assert written_doc is case_doc
    assert written_blob != case_blob

    raw = run_request(
        compile_document_pair(
            SimulationRequest(assembly="axle", family="axle_dynamic"),
            model_document=model_doc,
            case_document=written_doc,
            model_payload=pack_container(model_doc, model_blob),
            case_payload=pack_container(written_doc, written_blob),
        )
    ).raw
    after = np.asarray(raw.body_state("front_wheel_hub_L"), dtype=float)

    delta = float(np.max(np.abs(after - baseline)))
    assert delta > _TRAJECTORY_TOLERANCE, (delta, _TRAJECTORY_TOLERANCE)
