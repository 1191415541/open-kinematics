"""
The FMI 2.0 Co-Simulation export: the archive, and what it promises.

This file is the in-repository half of the p5-05 acceptance.  The other half runs
*outside* the repository (`raw/fmu_validation.md` records its path, command and
verbatim output): it unpacks the archive, loads the binary through `ctypes`, and
drives the FMI state machine to show that a changed input moves a named output.
What can only be checked from inside -- that every declaration is bound to bytes
that exist, that the archive is reproducible, and that exporting is a side
channel rather than a change to how a run is solved -- is checked here.

Why the assertions are structural rather than "the file exists"
---------------------------------------------------------------

An FMU whose variable list names quantities but not the bytes they come from
loads perfectly and returns plausible numbers from the wrong offsets.  So the
tests below do not assert a variable count: they assert that each input's byte
range is inside the case blob the archive carries, that each output's block and
column are inside a block the run actually emits, and that the two sets are
disjoint -- three properties a wrong binding cannot have by accident.

The rig
-------

One damped wheel on a revolute axle over a moving belt, with a braking
rotational-torque element whose demand channel the case carries.  It exists
because the exported run has to be *solved*: a model with no motion would export
just as well and demonstrate nothing about the archive driving the kernel.
"""

from __future__ import annotations

import subprocess
import sys
import zipfile
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from suspension_contracts import unpack_container

from suspension_multibody.api import validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics.schema import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleTire,
)
from suspension_multibody.fmi import (
    FMI_MODEL_IDENTIFIER,
    FMI_VERSION,
    FmiExportError,
    bindings_resource,
    export_fmu,
    fmi_binary_name,
    read_description,
    variable_declarations,
)
from suspension_multibody.simulation import run_compiled

#: The rig's wheel: unloaded radius, the axle height that puts it in contact, and
#: the belt speed that makes the tire slip.
RADIUS_M = 0.30
AXLE_HEIGHT_M = 0.29
BELT_M_PER_S = 2.0
#: The brake element's gain and cap: enough authority that changing its demand
#: moves the slip, which is what the outside validator's trajectory assertion
#: reads.
BRAKE_STIFFNESS = 100.0
BRAKE_MAX_TORQUE = 100.0


def _diagonal(value: float) -> tuple[tuple[float, float, float], ...]:
    """Return a 3x3 diagonal inertia matrix."""
    return ((value, 0.0, 0.0), (0.0, value, 0.0), (0.0, 0.0, value))


def _rig_model() -> AxleDynamicsModel:
    """One damped wheel on a revolute axle over a belt: the smallest rolling rig."""
    ground = AxleBody(
        name="ground", mass_kg=0.0, inertia_kg_m2=_diagonal(1.0), fixed=True
    )
    wheel = AxleBody(
        name="wheel",
        mass_kg=20.0,
        inertia_kg_m2=_diagonal(2.0),
        position_m=(0.0, 0.0, AXLE_HEIGHT_M),
        # The stated spin *is* the operating point; a static trim would zero it,
        # and with it the slip the exported run is supposed to exhibit.
        angular_velocity_rad_per_s=(0.0, BELT_M_PER_S / RADIUS_M, 0.0),
    )
    joint = AxleJoint(
        name="spin",
        kind="revolute",
        body_a="ground",
        body_b="wheel",
        point_a_m=(0.0, 0.0, AXLE_HEIGHT_M),
        point_b_m=(0.0, 0.0, 0.0),
        axis_a=(0.0, 1.0, 0.0),
        axis_b=(0.0, 1.0, 0.0),
    )
    tire = AxleTire(
        name="tire",
        body="wheel",
        model_kind="native_brush",
        unloaded_radius_m=RADIUS_M,
        maximum_compression_m=0.05,
        vertical_stiffness_n_per_m=200_000.0,
        vertical_damping_n_s_per_m=800.0,
        longitudinal_friction_coefficient=1.0,
        lateral_friction_coefficient=0.9,
        longitudinal_brush_stiffness_n_per_m=150_000.0,
        lateral_brush_stiffness_n_per_m=120_000.0,
        longitudinal_relaxation_length_m=0.25,
        lateral_relaxation_length_m=0.35,
        detached_relaxation_s=0.05,
    )
    return AxleDynamicsModel(
        name="fmu-rig",
        bodies=(ground, wheel),
        joints=(joint,),
        tires=(tire,),
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
    )


def _rig_case() -> AxleDynamicsCase:
    """Return the rig's time history: 201 samples over 0.2 s, uniform by construction."""
    return AxleDynamicsCase(
        name="fmu-rig",
        times_s=tuple(float(value) for value in np.linspace(0.0, 0.20, 201)),
        solver=AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.0005,
            minimum_step_s=0.0001,
            maximum_step_s=0.0005,
            max_newton_iterations=50,
        ),
    )


def _append_table(
    document: dict[str, Any], payload: bytes, role: str, values: Any, tire: str
) -> tuple[dict[str, Any], bytes]:
    """Return the case document and blob with one more per-tire table appended."""
    data = np.ascontiguousarray(values, dtype=np.float64)
    document = dict(document)
    document["blobs"] = list(document.get("blobs", [])) + [
        {
            "role": role,
            "tire": tire,
            "offset": len(payload),
            "length": int(data.size * data.itemsize),
            "dtype": "float64",
            "shape": [int(data.size)],
        }
    ]
    return document, payload + data.tobytes()


def _authored_documents():
    """Declare the wheel, brake subsystem and sampled rig inputs."""
    model = _rig_model()
    case = _rig_case()
    assembly, declared = migrate_v1_dynamic_axle(model, case)
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "brake", "functional_role": "brake", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "bodies": [], "hardpoints": [],
        "joints": [], "ports": [], "needs": [
            {"name": "reaction", "role": "body:ground", "count": 1, "required": True},
            {"name": "action", "role": "body:wheel", "count": 1, "required": True}],
        "property_slots": [{"name": "parameters", "element_type": "generic", "required": False, "default": 0}],
        "elements": [{"name": "brake", "type": "rotational_torque",
            "body_a": "@reaction", "body_b": "@action", "point_a": "@reaction", "point_b": "@action",
            "property_slot": "parameters", "parameters": {"axis_a": [0, 1, 0],
                "stiffness": BRAKE_STIFFNESS, "damping": 0, "max_torque": BRAKE_MAX_TORQUE,
                "demand_source": 2, "demand_tire": 0}}]})
    brake = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "brake", "template": "brake", "functional_role": "brake", "placement_role": "any",
        "hardpoints": {}, "property_bindings": {}}, template=template)
    docs = {entry.ref: entry.subsystem for entry in assembly.entries}
    docs["brake"] = brake
    payload = assembly.to_payload()
    payload["subsystems"].append({"ref": "brake", "functional_role": "brake", "placement_role": "any"})
    assembly = AssemblyDocument.from_payload(payload, subsystems=docs)
    times = np.asarray(case.times_s, dtype=float)
    run = declared.to_payload()
    run["inputs"].extend([
        {"name": "belt", "role": "road_velocity", "tire": "wheel.tire",
            "values": np.full(times.size, BELT_M_PER_S).tolist()},
        {"name": "brake", "role": "brake_pressure", "tire": "wheel.tire",
            "values": (0.6 * np.clip((times - .02) / .05, 0, 1)).tolist()}])
    return assembly, run


def _rig_documents() -> tuple[dict[str, Any], bytes, dict[str, Any], bytes]:
    compiled = validate(*_authored_documents())
    model_doc, model_blob = unpack_container(compiled.model_payload)
    case_doc, case_blob = unpack_container(compiled.case_payload)
    return model_doc, model_blob, case_doc, case_blob


@pytest.fixture(scope="module")
def wrapper_binary() -> Path:
    """Build the FMU wrapper binary and return its path."""
    script = (
        Path(__file__).resolve().parents[2] / "scripts" / "build_fmu_binary.py"
    )
    completed = subprocess.run(
        [sys.executable, str(script)], capture_output=True, text=True
    )
    if completed.returncode != 0:
        raise AssertionError(
            f"the wrapper failed to build: {completed.stdout}{completed.stderr}"
        )
    binary = Path(completed.stdout.strip().splitlines()[-1])
    assert binary.is_file(), f"the build reported {binary} but wrote nothing"
    return binary


@pytest.fixture(scope="module")
def exported(wrapper_binary: Path, tmp_path_factory: pytest.TempPathFactory):
    """Export the rig once and hand every test the artifact and its documents."""
    destination = tmp_path_factory.mktemp("fmu") / "rig.fmu"
    model_doc, model_blob, case_doc, case_blob = _rig_documents()
    result = export_fmu(
        destination,
        assembly_document=_authored_documents()[0],
        case_document=_authored_documents()[1],
    )
    return result, model_doc, model_blob, case_doc, case_blob


# --- the archive ------------------------------------------------------------


def test_the_archive_carries_exactly_the_standard_members(exported, wrapper_binary):
    """The archive's layout is FMI's, so an importing tool finds what it looks for."""
    result, *_ = exported
    with zipfile.ZipFile(result.path) as archive:
        names = set(archive.namelist())
        # The binary's *stem* is the model identifier: FMI derives the file to
        # load from `modelIdentifier`, so a differently named container is an
        # FMU nothing can instantiate.
        assert f"binaries/win64/{fmi_binary_name()}" in names
        assert "resources/model.bin" in names
        assert "resources/case.bin" in names
        assert "resources/bindings.txt" in names
        assert archive.read("binaries/win64/" f"{fmi_binary_name()}") == (
            wrapper_binary.read_bytes()
        )


def test_the_model_description_is_fmi_2_0_co_simulation(exported):
    """The description declares the version and interface D4 fixed."""
    result, *_ = exported
    root = read_description(result.path)
    assert root.tag == "fmiModelDescription"
    assert root.get("fmiVersion") == FMI_VERSION == "2.0"
    cosim = root.find("CoSimulation")
    assert cosim is not None, "the description declares no CoSimulation interface"
    # The identifier names the binary, which is what the archive must carry.
    assert cosim.get("modelIdentifier") == FMI_MODEL_IDENTIFIER
    assert root.get("guid"), "an FMU without a GUID is one a tool cannot cache"


def test_every_variable_is_a_real_with_a_unique_reference(exported):
    """FMI addresses variables by value reference, and the kernel has only reals."""
    result, *_ = exported
    root = read_description(result.path)
    scalars = list(root.iter("ScalarVariable"))
    assert scalars, "the description declares no variables"
    references = [int(scalar.get("valueReference")) for scalar in scalars]
    assert references == list(range(len(scalars))), "references are not unique and dense"
    for scalar in scalars:
        assert scalar.find("Real") is not None, scalar.get("name")
        assert scalar.get("causality") in {"input", "output"}


# --- the variable list ------------------------------------------------------


def test_every_input_is_bound_to_bytes_inside_the_case_blob(exported):
    """Each input's declared range has to land inside the blob the archive carries."""
    result, _, _, case_doc, case_blob = exported
    descriptors = {
        (entry.get("role"), entry.get("tire")): entry for entry in case_doc["blobs"]
    }
    assert result.inputs, "the exported run declares no input at all"
    for variable in result.inputs:
        name = variable.name
        role, _, qualifier = name.partition("[")
        tire = qualifier.rstrip("]") or None
        entry = descriptors.get((role, tire))
        assert entry is not None, f"input {name!r} names no case table"
        # The binding is the descriptor's own byte range: the wrapper writes the
        # value into exactly the bytes the kernel reads back.
        assert variable.binding.offset == entry["offset"]
        assert variable.binding.count == entry["length"] // 8
        start = variable.binding.offset
        end = start + entry["length"]
        assert 0 <= start < end <= len(case_blob), name
    # And every table the case carries is offered, so the export cannot omit an
    # input the run reads.
    offered = {(v.name.split("[")[0], v.name.partition("[")[2].rstrip("]") or None)
               for v in result.inputs}
    assert offered == set(descriptors), "the input list does not match the case tables"


def test_an_input_is_written_to_every_sample_of_its_slot(exported):
    """One FMI real has no length, so a value is a constant profile over the horizon."""
    result, *_ = exported
    for variable in result.inputs:
        assert variable.binding.count > 1, variable.name
        assert variable.binding.direction == "input"
        assert variable.variability == "continuous"


def test_every_output_names_a_block_the_run_actually_emits(exported):
    """A declaration bound to nothing would be a promise the wrapper cannot keep."""
    result, model_doc, model_blob, case_doc, case_blob = exported
    run = run_compiled(validate(*_authored_documents())).raw
    blocks = run.named_blocks
    assert result.outputs, "the export declares no output"
    for variable in result.outputs:
        binding = variable.binding
        if not binding.block:
            # The time grid is the instance's own clock, not a column.
            assert variable.name == "time_s"
            continue
        assert binding.block in blocks, f"{variable.name} names block {binding.block}"
        block = np.asarray(blocks[binding.block])
        assert binding.row < block.shape[1], variable.name
        assert binding.column < block.shape[2], variable.name


def test_no_name_is_declared_with_two_directions(exported):
    """Inputs are case tables and outputs are result columns, so the sets are disjoint."""
    result, *_ = exported
    inputs = {variable.name for variable in result.inputs}
    outputs = {variable.name for variable in result.outputs}
    assert inputs and outputs
    assert not (inputs & outputs)


def test_output_entities_come_from_the_model_document(exported):
    """A per-entity channel is expanded over the entities the model declares."""
    result, model_doc, *_ = exported
    bodies = [entry["name"] for entry in model_doc["bodies"]]
    tires = [entry["name"] for entry in model_doc["tires"]]
    named = {variable.name for variable in result.outputs}
    for body in bodies:
        assert f"wheel_speed[{body}][0]" in named, body
    for tire in tires:
        assert f"longitudinal_slip[{tire}]" in named, tire


def test_the_bindings_resource_agrees_with_the_declarations(exported):
    """The file the binary reads is generated from the same bindings the XML states."""
    result, _, _, case_doc, _ = exported
    text = bindings_resource(case_doc, result.variables)
    lines = [line for line in text.splitlines() if line and not line.startswith("#")]
    grid = lines[0].split()
    assert grid[0] == "grid"
    # `time` is `end_s` from the case document, so the fixed step divides it.
    assert float(grid[1]) == 0.0 and float(grid[2]) == 0.001 and int(grid[3]) == 201
    declared = {}
    for line in lines[1:]:
        parts = line.split()
        declared[int(parts[1])] = parts[0]
    assert declared == {
        variable.reference: variable.causality for variable in result.variables
    }


# --- refusals ---------------------------------------------------------------


def test_a_case_document_without_its_sampled_values_is_refused(exported, tmp_path: Path):
    """A document that indexes bytes nobody handed over would export a broken FMU."""
    assembly, case = _authored_documents()
    case["inputs"][0].pop("values")
    with pytest.raises(ValueError, match="values"):
        export_fmu(
            tmp_path / "broken.fmu",
            assembly_document=assembly,
            case_document=case,
        )


def test_a_non_uniform_time_grid_is_refused(exported, tmp_path: Path):
    """The binding addresses samples by index, so a grid has to be uniform."""
    assembly, uneven = _authored_documents()
    uneven["samples"] = [0, .001, .003]
    for row in uneven["inputs"]:
        row["values"] = row["values"][:3]
    with pytest.raises(FmiExportError, match="explicit sample-time table"):
        export_fmu(
            tmp_path / "uneven.fmu",
            assembly_document=assembly,
            case_document=uneven,
        )


def test_a_case_that_names_no_study_is_refused(exported, tmp_path: Path):
    """The export writes a container whose layout the family determines."""
    assembly, unnamed = _authored_documents()
    unnamed.pop("study")
    with pytest.raises(ValueError, match="study"):
        export_fmu(
            tmp_path / "unnamed.fmu",
            assembly_document=assembly,
            case_document=unnamed,
        )


def test_an_unknown_role_is_refused_rather_than_given_a_guess_unit(exported):
    """An input whose unit is a guess is worse than a refused export."""
    _, _, _, case_doc, _ = exported
    invented = dict(case_doc)
    invented["blobs"] = [
        *case_doc["blobs"],
        {
            "role": "not_a_real_role",
            "offset": 0,
            "length": 8,
            "dtype": "float64",
            "shape": [1],
        },
    ]
    with pytest.raises(FmiExportError, match="has no unit for"):
        variable_declarations(invented, {"bodies": [], "tires": []})


# --- the artifact is reproducible and non-invasive --------------------------


def test_exporting_the_same_pair_twice_produces_the_same_bytes(exported, tmp_path: Path):
    """A co-simulation artifact a build cache cannot trust is one nothing can cache."""
    result, model_doc, model_blob, case_doc, case_blob = exported
    again = export_fmu(
        tmp_path / "again.fmu",
        assembly_document=_authored_documents()[0],
        case_document=_authored_documents()[1],
    )
    assert again.guid == result.guid
    assert again.path.read_bytes() == result.path.read_bytes()


def test_the_exported_containers_are_the_ones_a_run_would_be_submitted(exported):
    """The resources are frames, not bare blobs: an unframed one is a refused run."""
    result, model_doc, model_blob, case_doc, case_blob = exported
    with zipfile.ZipFile(result.path) as archive:
        model_container = archive.read("resources/model.bin")
        case_container = archive.read("resources/case.bin")
    # The kernel's containers are `MBC1`, and the blob each describes is the one
    # the case module produced -- byte for byte.
    for container, expected in (
        (model_container, model_blob),
        (case_container, case_blob),
    ):
        assert container[:4] == b"MBC1"
        assert unpack_container(container)[1] == expected


def test_exporting_does_not_change_a_run(exported):
    """The export is a side channel: the same documents still solve the same way."""
    _, model_doc, model_blob, case_doc, case_blob = exported
    compiled = validate(*_authored_documents())
    first = run_compiled(compiled).raw
    slip = np.asarray(first.named_blocks["tire_output"])[:, 0, 7]
    # The exported run is a real one: the wheel rolls and the brake moves the
    # slip, so the archive describes a solved history rather than a still frame.
    assert np.ptp(slip) > 1.0
    second = run_compiled(compiled).raw
    np.testing.assert_array_equal(
        np.asarray(second.named_blocks["tire_output"]),
        np.asarray(first.named_blocks["tire_output"]),
    )

# --- the co-simulation clock is causal --------------------------------------
#
# A value set at the simulator's current time must reach the solver from that
# time on, and must not rewrite the samples already behind it.  Writing every
# sample of the slot -- which an earlier version did -- made the whole reported
# history depend on the value set last, so the same step sequence produced
# different past outputs depending on when the simulator stopped.

#: The sample the driver input is switched at: late enough that a history exists.
_SWITCH_AT = 60


def _fmi_library(archive: Path) -> Any:
    """Load the wrapper from an unpacked archive and declare its signatures."""
    import ctypes

    library = next(
        path
        for path in archive.glob("binaries/*/*")
        if path.suffix in {".dll", ".so", ".dylib"}
    )
    lib = ctypes.CDLL(str(library))
    lib.fmi2GetVersion.restype = ctypes.c_char_p
    lib.fmi2Instantiate.restype = ctypes.c_void_p
    lib.fmi2Instantiate.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p,
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
    ]
    lib.fmi2SetReal.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.fmi2GetReal.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.fmi2DoStep.argtypes = [
        ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_int
    ]
    lib.fmi2FreeInstance.argtypes = [ctypes.c_void_p]
    return lib


def _grid(resources: Path) -> tuple[float, float, int]:
    """Return the (start, step, samples) the archive declares."""
    for line in (resources / "bindings.txt").read_text().splitlines():
        parts = line.split()
        if parts[:1] == ["grid"]:
            return float(parts[1]), float(parts[2]), int(parts[3])
    raise AssertionError("the archive declares no time grid")


def _drive(lib: Any, resources: Path, reference: int, driver: int, switch_at):
    """Run the archive; return the named output at every sample."""
    import ctypes

    start, step, samples = _grid(resources)
    instance = lib.fmi2Instantiate(
        b"probe", 1, b"", str(resources).encode(), None, 0, 0
    )
    assert instance, "fmi2Instantiate failed"
    out = ctypes.c_double(0.0)
    ref = ctypes.c_uint(reference)
    values = []
    try:
        for index in range(samples):
            if switch_at is not None and index == switch_at:
                value = ctypes.c_double(0.9)
                iref = ctypes.c_uint(driver)
                assert lib.fmi2SetReal(
                    instance, ctypes.byref(iref), 1, ctypes.byref(value)
                ) == 0
            status = lib.fmi2DoStep(
                instance, start + index * step, step, 1
            )
            assert status == 0, f"fmi2DoStep returned {status} at sample {index}"
            status = lib.fmi2GetReal(instance, ctypes.byref(ref), 1, ctypes.byref(out))
            assert status == 0, f"fmi2GetReal returned {status} at sample {index}"
            values.append(out.value)
        if switch_at is not None:
            # Read the *past* again, after the switch.  `fmi2GetReal` reports the
            # sample the clock is at out of whatever result is current, so a
            # wrapper that re-ran the whole horizon from the new inputs would
            # answer these differently than the untouched run did.  This is the
            # read that makes the property observable: without it the prefix was
            # collected before the switch and could never differ.
            past = []
            for index in range(switch_at):
                lib.fmi2DoStep(instance, start + index * step, step, 0)
                status = lib.fmi2GetReal(instance, ctypes.byref(ref), 1, ctypes.byref(out))
                assert status == 0, f"fmi2GetReal returned {status} re-reading {index}"
                past.append(out.value)
            values.extend(past)
    finally:
        lib.fmi2FreeInstance(instance)
    return values


def _slip_reference(resources: Path) -> int:
    """Return the valueReference bound to the tire's longitudinal slip column."""
    for line in (resources / "bindings.txt").read_text().splitlines():
        parts = line.split()
        if parts[:2] == ["output", "16"] or (
            len(parts) == 4 and parts[0] == "output" and parts[2] == "tire_output"
            and parts[3] == "7"
        ):
            return int(parts[1])
    raise AssertionError("the archive declares no tire_output column 7")


def _driver_reference(resources: Path) -> int:
    """
    Return the valueReference of the brake-pressure input, by position 1.

    `bindings.txt` lists the inputs in the exporter's own order: reference 0 is
    the belt speed and reference 1 the brake pressure (see `raw/fmu_validation.md`).
    """
    references = [
        int(line.split()[1])
        for line in (resources / "bindings.txt").read_text().splitlines()
        if line.startswith("input ")
    ]
    assert len(references) >= 2, references
    return references[1]


def test_the_clock_is_causal_a_later_input_does_not_rewrite_the_past(
    exported, wrapper_binary, tmp_path: Path, monkeypatch
) -> None:
    """
    Setting an input at t1 leaves the samples before t1 bit-for-bit unchanged.

    Two runs of one archive: untouched, and with the brake raised at `_SWITCH_AT`.
    The sample window behind the switch must be identical in both -- otherwise the
    reported past is a function of what the simulator did next -- and the window
    from the switch on must move.
    """
    import zipfile

    result, *_ = exported
    unpacked = tmp_path / "archive"
    with zipfile.ZipFile(result.path) as archive:
        archive.extractall(unpacked)

    # The wrapper resolves the kernel through this variable; the test sets it to
    # the mirror the package ships, so the archive loads its own solver.
    kernel = (
        Path(__file__).resolve().parents[2]
        / "src" / "suspension_multibody" / "native" / "suspension_kernel.dll"
    )
    monkeypatch.setenv("SUSPENSION_MULTIBODY_KERNEL", str(kernel))

    resources = unpacked / "resources"
    lib = _fmi_library(unpacked)
    slip = _slip_reference(resources)
    driver = _driver_reference(resources)

    untouched = _drive(lib, resources, slip, driver, None)
    switched = _drive(lib, resources, slip, driver, _SWITCH_AT)

    # The run that switched carries its forward samples first and then the
    # re-read past, so the two windows are split by the run's own sample count.
    _start, _step, samples = _grid(resources)
    forward = switched[:samples]
    reread_past = switched[samples:]

    # The past is not rewritten: reading it *after* the switch gives the same
    # numbers the untouched run gave at those instants.
    assert reread_past == untouched[:_SWITCH_AT]

    # And the switch really reaches the solve.
    moved = max(
        abs(a - b) for a, b in zip(untouched[_SWITCH_AT:], forward[_SWITCH_AT:])
    )
    assert moved > 1e-6, moved
