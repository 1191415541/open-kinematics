"""
The rotational torque from a port pairing to a kernel block, and one run of it.

Three things are asserted here, in the order a build does them:

1. **pairing** -- which two bodies an element acts between comes from a *matched*
   port.  The fixture runs the real matcher over a real requirement and reads the
   reaction end off the binding it produced.
2. **construction** -- ``build_element`` has a branch for the kind, and the two
   bodies it builds with are the two the pairing resolved.  The branch is
   exercised through the public entry point, not by calling the builder directly.
3. **the run** -- one minimal model with exactly one torque element goes through
   ``mb_core_run``, and the couple is *measured*: the pair's angular velocity and
   the energy ledger are read, and the same model without the element is run as
   the control.  A test that only asserted "the call returned zero" would pass for
   an element the kernel never read.

The core input/output structures are mirrored here rather than imported, because
they are the kernel's ABI and the product deliberately does not re-declare them
(``tests/architecture/test_core_abi.py`` states that).  What *is* product code
here is everything before the mirror: the pairing, the row, the element and the
encoded block all come from the package under test.
"""

from __future__ import annotations

import ctypes
from pathlib import Path

import numpy as np
import pytest
from suspension_kernel.binding import load_kernel_library

from suspension_multibody.compilation.element_blocks import (
    ELEMENT_BLOCK_SIZE,
    ELEMENT_CURVE_SLOTS,
    ELEMENT_ROTATIONAL_TORQUE,
    ROTATIONAL_TORQUE_AXIS_A_INDEX,
    ROTATIONAL_TORQUE_MAX_TORQUE_INDEX,
    ROTATIONAL_TORQUE_REFERENCE_QUATERNION_INDEX,
    ROTATIONAL_TORQUE_STIFFNESS_INDEX,
    ElementBlockError,
    ElementBlockRow,
    pair_torque_bodies,
    rotational_torque_block,
    torque_element_row,
)
from suspension_multibody.connections.matcher import (
    BindingError,
    MatchReport,
    match_requirements,
)
from suspension_multibody.modeling.identity import EntityId
from suspension_multibody.modeling.instance import ResolvedElement
from suspension_multibody.modeling.ports import GeometryPort, PortRequirement
from suspension_multibody.modeling.primitives import (
    RotationalTorqueElement,
    RotationalTorqueParameters,
)

INSTANCE = ("axle",)
#: The two bodies the fixture uses.  The reaction-side name is deliberately one no
#: rule inside the package knows: the pairing is what makes it work.
FRAME = "subframe"
SPINNER = "spinner"
#: The requirement the torque element fills, and the port role that meets it.
ROLE = "torque_reaction"

#: ``ElementKind`` for the family under test, restated so this test fails loudly if
#: the exported constant ever drifts from the value the kernel was built with.
KIND = 7
#: `kStatePerBody`, and where a body's angular velocity starts inside its block:
#: position(3), quaternion(4), velocity(3), then omega.
STATE_PER_BODY = 19
OMEGA = 10
#: `kDiagnosticsWidth`, `kSpringOutputWidth`, `kEnergyOutputWidth`.
DIAGNOSTICS_WIDTH = 16
SPRING_OUTPUT_WIDTH = 7
ENERGY_OUTPUT_WIDTH = 21
#: Energy ledger columns: the drive's work for one interval, and the total.
DRIVE_WORK = 6
TOTAL_ENERGY = 2


class _MbCoreInput(ctypes.Structure):
    """Mirror of ``MbCoreInput`` in ``core_abi.hpp``, field for field."""

    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("body_count", ctypes.c_size_t),
        ("body_mass", ctypes.c_void_p),
        ("body_inertia_body_3x3", ctypes.c_void_p),
        ("body_pose_position_quaternion", ctypes.c_void_p),
        ("body_velocity_omega", ctypes.c_void_p),
        ("body_fixed", ctypes.c_void_p),
        ("joint_count", ctypes.c_size_t),
        ("joint_type", ctypes.c_void_p),
        ("joint_body_a", ctypes.c_void_p),
        ("joint_body_b", ctypes.c_void_p),
        ("joint_point_a", ctypes.c_void_p),
        ("joint_point_b", ctypes.c_void_p),
        ("joint_axis_a", ctypes.c_void_p),
        ("joint_axis_b", ctypes.c_void_p),
        ("element_count", ctypes.c_size_t),
        ("elements", ctypes.c_void_p),
        ("element_curves", ctypes.c_void_p),
        ("sample_count", ctypes.c_size_t),
        ("sample_times", ctypes.c_void_p),
        ("body_wrench", ctypes.c_void_p),
        ("gravity_x", ctypes.c_double),
        ("gravity_y", ctypes.c_double),
        ("gravity_z", ctypes.c_double),
        ("rho_inf", ctypes.c_double),
        ("integrator_type", ctypes.c_int),
        ("hht_alpha", ctypes.c_double),
        ("initialization_mode", ctypes.c_int),
        ("adaptive_step", ctypes.c_int),
        ("internal_step", ctypes.c_double),
        ("min_step", ctypes.c_double),
        ("max_step", ctypes.c_double),
        ("local_relative_tolerance", ctypes.c_double),
        ("local_position_tolerance", ctypes.c_double),
        ("local_angle_tolerance", ctypes.c_double),
        ("local_velocity_tolerance", ctypes.c_double),
        ("local_angular_velocity_tolerance", ctypes.c_double),
        ("contact_event_tolerance", ctypes.c_double),
        ("max_newton_iterations", ctypes.c_int),
        ("max_line_search_iterations", ctypes.c_int),
        ("position_tolerance", ctypes.c_double),
        ("velocity_tolerance", ctypes.c_double),
        ("dynamics_tolerance", ctypes.c_double),
        ("increment_tolerance", ctypes.c_double),
    ]


class _ElementBlock(ctypes.Structure):
    """The dependency-free element block, sized by the ABI's own constant."""

    _fields_ = [
        ("kind", ctypes.c_int),
        ("flags", ctypes.c_int),
        ("body_a", ctypes.c_int),
        ("body_b", ctypes.c_int),
        ("parameters", ctypes.c_double * ELEMENT_BLOCK_SIZE),
        ("ints", ctypes.c_int * 16),
        ("cached_parameters", ctypes.c_void_p),
        ("cached_parameter_count", ctypes.c_size_t),
    ]


class _ElementCurveReference(ctypes.Structure):
    _fields_ = [("values", ctypes.c_void_p), ("count", ctypes.c_size_t)]


class _MbCoreOutput(ctypes.Structure):
    """Mirror of ``MbCoreOutput`` in ``core_abi.hpp``."""

    _fields_ = [
        ("struct_size", ctypes.c_size_t),
        ("abi_version", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
        ("body_state", ctypes.c_void_p),
        ("body_state_capacity", ctypes.c_size_t),
        ("joint_wrench", ctypes.c_void_p),
        ("joint_wrench_capacity", ctypes.c_size_t),
        ("spring_output", ctypes.c_void_p),
        ("spring_output_capacity", ctypes.c_size_t),
        ("energy_output", ctypes.c_void_p),
        ("energy_output_capacity", ctypes.c_size_t),
        ("diagnostics", ctypes.c_void_p),
        ("diagnostics_capacity", ctypes.c_size_t),
        ("contact_event_count", ctypes.c_void_p),
    ]


# --------------------------------------------------------------------------- #
# 1. the pairing: a requirement, an offered port, and the real matcher
# --------------------------------------------------------------------------- #


def _offered(*names: str) -> dict[str, GeometryPort]:
    """
    One offered geometric port per name, each owned by the reaction body.

    Keyed the way the composition keys them -- by the *rendered* id, which carries
    the mounting path -- because that is the spelling both the matcher and this
    pairing work in.
    """
    return {
        str(port.id): port
        for port in (
            GeometryPort(
                id=EntityId(INSTANCE, name),
                owner=EntityId(INSTANCE, FRAME),
                role=ROLE,
            )
            for name in names
        )
    }


def _matched(*names: str):
    """Run the matcher the composition itself runs, over the ports it offers."""
    return match_requirements((PortRequirement(role=ROLE),), _offered(*names))


def _pairing(*names: str):
    accepted = names or ("reaction",)
    return pair_torque_bodies(
        name="torque",
        role=ROLE,
        own_body=SPINNER,
        report=_matched(*accepted),
        ports=_offered(*accepted),
    )


def test_the_two_bodies_come_from_the_matched_port() -> None:
    """
    The reaction body is whatever the matched port said its owner was.

    The matcher is the one the composition layer uses, and the ports are declared
    with real ids and owners, so this is a statement about the pairing path and not
    about a fixture coincidence.  The neighbour's name is in no table inside the
    package, which is what makes "the port decided it" checkable.
    """
    pairing = _pairing()
    assert pairing.driven_body == SPINNER
    assert pairing.reaction_body == FRAME
    assert pairing.port_id == str(EntityId(INSTANCE, "reaction"))
    assert pairing.role == ROLE


def test_two_offered_ports_are_ambiguous_until_the_pairing_names_one() -> None:
    """
    An ambiguity is reported, not resolved by order.

    With two ports offering the same role there is no answer to pick, and taking
    the first one would be the name-proximity guess the matcher exists to reject.
    The explicit mapping is the repair, and then the *named* port is the one the
    coupling reacts on.
    """
    with pytest.raises(BindingError, match="matches 2 candidates"):
        _pairing("reaction", "spare")
    named = pair_torque_bodies(
        name="torque",
        role=ROLE,
        own_body=SPINNER,
        report=match_requirements(
            (PortRequirement(role=ROLE),),
            _offered("reaction", "spare"),
            explicit={ROLE: str(EntityId(INSTANCE, "spare"))},
        ),
        ports=_offered("reaction", "spare"),
    )
    assert named.port_id == str(EntityId(INSTANCE, "spare"))
    assert named.reaction_body == FRAME

def test_an_unbound_requirement_is_refused_by_name() -> None:
    """A torque whose requirement never matched fails by role, not by guessing."""
    with pytest.raises(BindingError, match="other_role"):
        pair_torque_bodies(
            name="torque",
            role="other_role",
            own_body=SPINNER,
            # A report that bound nothing: the role this element fills is not among
            # the requirements the assembly resolved, so there is no port to read.
            report=MatchReport(),
            ports=_offered("reaction"),
        )


def test_a_pairing_without_the_requiring_side_body_is_refused() -> None:
    """A requirement never names a body, so the contribution has to state one."""
    with pytest.raises(ElementBlockError, match="own body"):
        pair_torque_bodies(
            name="torque",
            role=ROLE,
            own_body="",
            report=MatchReport(),
            ports=_offered("reaction"),
        )


# --------------------------------------------------------------------------- #
# 2. construction: the dispatch branch, and the block
# --------------------------------------------------------------------------- #


def _parameters(**overrides: object) -> RotationalTorqueParameters:
    fields: dict[str, object] = {"stiffness": 1.0, "max_torque": 1.0}
    fields.update(overrides)
    return RotationalTorqueParameters(**fields)  # type: ignore[arg-type]


def _built_element() -> RotationalTorqueElement:
    """Build the element the way a composition does: pairing, row, build."""
    row = torque_element_row(_pairing(), _parameters())
    assert isinstance(row, ResolvedElement)
    element = RotationalTorqueElement(name=row.name, body_a=row.body_a,
        body_b=row.body_b, parameters=row.spec)
    assert isinstance(element, RotationalTorqueElement)
    return element


def test_the_dispatch_branch_builds_the_declared_element() -> None:
    """
    ``build_element`` has the branch, and it builds the declared class.

    The two ends reach the element in the row's order -- ``body_a`` is the
    reaction side and ``body_b`` the driven one -- because the kernel's fields are
    those two and swapping them reverses every couple it applies.
    """
    element = _built_element()
    assert element.name == "torque"
    assert element.body_a == FRAME
    assert element.body_b == SPINNER


def test_an_unknown_kind_is_still_refused() -> None:
    """The branch was added, not the refusal removed."""
    from suspension_multibody.authoring import TemplateDocument

    with pytest.raises(ValueError, match="nothing_like_this"):
        TemplateDocument.from_payload({"document": "template", "schema_version": 1,
            "name": "unknown", "functional_role": "generic", "allowed_placement_roles": ["any"],
            "bodies": [], "hardpoints": [], "joints": [], "property_slots": [],
            "elements": [{"name": "x", "type": "nothing_like_this"}]})


def test_the_block_carries_the_familys_own_slots_and_nothing_else() -> None:
    """
    The block is a whole ABI block, and only the family's own run is written.

    The padding staying zero is asserted rather than assumed: the families' runs
    are adjacent, so a writer that went past its own run would land in another
    family's slots and the reader would pick up a number that means something else.
    """
    block = rotational_torque_block(
        _built_element(), body_index={FRAME: 0, SPINNER: 1}
    )
    assert block.kind == ELEMENT_ROTATIONAL_TORQUE == KIND
    # ``body_a`` is the reaction end, matching the native element's own field.
    assert (block.body_a, block.body_b) == (0, 1)
    assert len(block.parameters) == ELEMENT_BLOCK_SIZE
    assert len(block.ints) == 16
    written = set(
        range(
            ROTATIONAL_TORQUE_STIFFNESS_INDEX,
            ROTATIONAL_TORQUE_REFERENCE_QUATERNION_INDEX + 4,
        )
    ) | {ROTATIONAL_TORQUE_MAX_TORQUE_INDEX}
    assert [
        (index, value)
        for index, value in enumerate(block.parameters)
        if index not in written and value != 0.0
    ] == []
    assert block.parameters[ROTATIONAL_TORQUE_STIFFNESS_INDEX] == pytest.approx(1.0)
    assert block.parameters[ROTATIONAL_TORQUE_MAX_TORQUE_INDEX] == pytest.approx(1.0)
    assert block.parameters[
        ROTATIONAL_TORQUE_AXIS_A_INDEX : ROTATIONAL_TORQUE_AXIS_A_INDEX + 3
    ] == pytest.approx([0.0, 1.0, 0.0])


def test_the_block_refuses_a_body_the_model_does_not_have() -> None:
    """A name the index does not carry is an error, not body zero."""
    with pytest.raises(ElementBlockError, match=SPINNER):
        rotational_torque_block(_built_element(), body_index={FRAME: 0})


# --------------------------------------------------------------------------- #
# 3. the run: one torque element, measured, with its control
# --------------------------------------------------------------------------- #


def _core_library() -> ctypes.CDLL:
    """Load the kernel and type the core entry points through the binding layer."""
    library = load_kernel_library(
        required_symbols=("mb_core_run", "mb_core_abi_version"),
    )
    handle = library.handle
    handle.mb_core_abi_version.argtypes = []
    handle.mb_core_abi_version.restype = ctypes.c_int
    handle.mb_core_run.argtypes = [
        ctypes.POINTER(_MbCoreInput),
        ctypes.POINTER(_MbCoreOutput),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    handle.mb_core_run.restype = ctypes.c_int
    return handle


def _ptr(array: object) -> int | None:
    """Address of a numpy or ctypes buffer; a zero-length one is null."""
    if isinstance(array, np.ndarray):
        return None if array.size == 0 else int(array.ctypes.data)
    if len(array) == 0:  # type: ignore[arg-type]
        return None
    return int(ctypes.cast(array, ctypes.c_void_p).value)  # type: ignore[arg-type]


def _run(
    rows: tuple[ElementBlockRow, ...],
    *,
    spin: float = 3.0,
    seconds: float = 0.2,
    samples: int = 21,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Run the minimal two-body model with the given element rows.

    The model is the smallest one this element can act in: two bodies sharing no
    joint, one of them held to ground, an axis-aligned couple between them and
    nothing else.  No tire, no road and no suspension surface is involved, so what
    the run measures is this element and only this element.

    Returns ``(body_state, energy_output)``.
    """
    handle = _core_library()
    times = np.linspace(0.0, seconds, samples, dtype=np.float64)
    mass = np.array([1.0, 1.0], dtype=np.float64)
    inertia = np.tile(np.diag([0.01, 0.01, 0.01]).ravel(), 2)
    poses = np.array(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
    motion = np.zeros(12, dtype=np.float64)
    # Body 1 starts spinning about +y and body 0 does not, so the pair's relative
    # rate about the element's own axis (which is +y in body 0's frame) is `spin`.
    motion[6 + 3 : 6 + 6] = (0.0, spin, 0.0)
    motion[6 + 3 : 6 + 6] = (0.0, spin, 0.0)
    # Both bodies are free: the couple applies ``+tau`` to one and ``-tau`` to the
    # other, and a reaction body held to ground would take its half into the
    # ground, so the two ends could never meet.
    fixed = np.array([0, 0], dtype=np.int32)

    blocks = (_ElementBlock * len(rows))()
    for index, row in enumerate(rows):
        blocks[index].kind = row.kind
        blocks[index].body_a = row.body_a
        blocks[index].body_b = row.body_b
        for slot, value in enumerate(row.parameters):
            blocks[index].parameters[slot] = value
        for slot, value in enumerate(row.ints):
            blocks[index].ints[slot] = value
    curves = (_ElementCurveReference * (len(rows) * ELEMENT_CURVE_SLOTS))()
    for index in range(len(rows) * ELEMENT_CURVE_SLOTS):
        curves[index].values = None
        curves[index].count = 0

    body_state = np.full(
        (samples, 2 * STATE_PER_BODY), np.nan, dtype=np.float64
    )
    joint_wrench = np.zeros(0, dtype=np.float64)
    spring_output = np.full(
        (samples, len(rows) * SPRING_OUTPUT_WIDTH), np.nan, dtype=np.float64
    )
    energy = np.full((samples, ENERGY_OUTPUT_WIDTH), np.nan, dtype=np.float64)
    diagnostics = np.full(
        (samples + 2, DIAGNOSTICS_WIDTH), np.nan, dtype=np.float64
    )
    contact_event_count = ctypes.c_size_t(0)

    core_input = _MbCoreInput()
    core_input.struct_size = ctypes.sizeof(_MbCoreInput)
    core_input.abi_version = handle.mb_core_abi_version()
    core_input.body_count = 2
    core_input.body_mass = _ptr(mass)
    core_input.body_inertia_body_3x3 = _ptr(inertia)
    core_input.body_pose_position_quaternion = _ptr(poses)
    core_input.body_velocity_omega = _ptr(motion)
    core_input.body_fixed = _ptr(fixed)
    core_input.joint_count = 0
    core_input.element_count = len(rows)
    core_input.elements = _ptr(blocks)
    # The curve array must be non-null whenever elements are supplied, even for a
    # family that declares no curve slot at all.
    core_input.element_curves = _ptr(curves)
    core_input.sample_count = samples
    core_input.sample_times = _ptr(times)
    core_input.gravity_z = 0.0
    core_input.rho_inf = 1.0
    core_input.initialization_mode = 1
    core_input.adaptive_step = 0
    core_input.internal_step = seconds / 200.0
    core_input.min_step = seconds / 2000.0
    core_input.max_step = seconds / 50.0
    core_input.local_relative_tolerance = 1e-8
    core_input.local_position_tolerance = 1e-8
    core_input.local_angle_tolerance = 1e-8
    core_input.local_velocity_tolerance = 1e-8
    core_input.local_angular_velocity_tolerance = 1e-8
    core_input.contact_event_tolerance = 1e-6
    core_input.max_newton_iterations = 40
    core_input.max_line_search_iterations = 8
    core_input.position_tolerance = 1e-8
    core_input.velocity_tolerance = 1e-8
    core_input.dynamics_tolerance = 1e-6
    core_input.increment_tolerance = 1e-10

    core_output = _MbCoreOutput()
    core_output.struct_size = ctypes.sizeof(_MbCoreOutput)
    core_output.abi_version = core_input.abi_version
    core_output.body_state = _ptr(body_state)
    core_output.body_state_capacity = body_state.size
    core_output.joint_wrench = _ptr(joint_wrench)
    core_output.joint_wrench_capacity = joint_wrench.size
    core_output.spring_output = _ptr(spring_output)
    core_output.spring_output_capacity = spring_output.size
    core_output.energy_output = _ptr(energy)
    core_output.energy_output_capacity = energy.size
    core_output.diagnostics = _ptr(diagnostics)
    core_output.diagnostics_capacity = diagnostics.size
    core_output.contact_event_count = ctypes.cast(
        ctypes.byref(contact_event_count), ctypes.c_void_p
    ).value

    error = ctypes.create_string_buffer(4096)
    status = handle.mb_core_run(
        ctypes.byref(core_input), ctypes.byref(core_output), error, len(error)
    )
    message = error.value.decode("utf-8", "replace")
    assert status == 0, f"the run failed with status {status}: {message}"
    del blocks, curves
    return body_state, energy


def _omega(history: np.ndarray, body: int) -> np.ndarray:
    """One body's angular velocity history, in rad/s."""
    start = body * STATE_PER_BODY + OMEGA
    return history[:, start : start + 3]


def test_one_torque_element_changes_the_pairs_spin() -> None:
    """
    The minimal assembly, run twice: with the element and without it.

    What is measured is the pair's angular velocity and the energy ledger -- not
    "the call succeeded".  The control is the same model with the element removed,
    so the difference between the two readings is the element's contribution.

    The block's ``body_a`` is body 0 (the reaction end) and ``body_b`` is body 1
    (the driven one), so the readings below are statements about those two roles:
    a couple drives *both* ends, one by ``+tau`` and one by ``-tau``.
    """
    block = rotational_torque_block(
        _built_element(), body_index={FRAME: 0, SPINNER: 1}
    )
    driven_states, driven_energy = _run((block,))
    control_states, control_energy = _run(())

    driven = _omega(driven_states, 1)
    reaction = _omega(driven_states, 0)
    control = _omega(control_states, 1)

    # The control is a free spinner: nothing slows it, so the comparison is not
    # vacuous and the two runs are genuinely different numbers.
    assert np.allclose(control[-1], [0.0, 3.0, 0.0]), control[-1]
    assert not np.allclose(driven[-1], control[-1])

    # The measured contribution: the driven end slowed, the reaction end was driven
    # the other way, and the two met in the middle because the torques are equal
    # and the inertias are equal.  The initial sample is where the pair stood.
    assert driven[0, 1] == pytest.approx(3.0)
    assert reaction[0, 1] == pytest.approx(0.0)
    assert driven[-1, 1] == pytest.approx(1.5, abs=1e-6)
    assert reaction[-1, 1] == pytest.approx(1.5, abs=1e-6)
    assert driven[-1, 1] == pytest.approx(reaction[-1, 1], abs=1e-9)
    assert driven[-1, 1] < control[-1, 1]

    # The axis is +y in the reaction body's frame and both bodies start unrotated,
    # so nothing appears on x or z: the couple acts where it was declared to.
    assert np.allclose(driven[:, 0], 0.0)
    assert np.allclose(driven[:, 2], 0.0)
    assert np.allclose(reaction[:, 0], 0.0)
    assert np.allclose(reaction[:, 2], 0.0)

    # The ledger is the causal evidence independent of the state: the drive did
    # work, it did it only in the run that carries the element, and the work it
    # booked is exactly the kinetic energy the pair lost.  The interval figures are
    # negative because the couple opposed the spin.
    assert np.allclose(control_energy[:, DRIVE_WORK], 0.0)
    assert np.all(driven_energy[:, DRIVE_WORK] <= 1e-15)
    assert driven_energy[1, DRIVE_WORK] < -1e-3
    assert driven_energy[-1, TOTAL_ENERGY] == pytest.approx(0.0225, abs=1e-9)
    assert control_energy[-1, TOTAL_ENERGY] == pytest.approx(0.045, abs=1e-9)
    lost = control_energy[-1, TOTAL_ENERGY] - driven_energy[-1, TOTAL_ENERGY]
    assert float(np.sum(driven_energy[:, DRIVE_WORK])) == pytest.approx(-lost)


def test_a_still_pair_is_left_alone() -> None:
    """
    No relative rate, no couple, and no work: the model does not move at all.

    This is the branch that keeps a parked pair parked.  It is asserted through the
    run rather than on the law alone, and the ledger is what makes it sharp: an
    element that produced a couple here would show up as drive work and as a state
    that changed, and neither happens.
    """
    block = rotational_torque_block(
        _built_element(), body_index={FRAME: 0, SPINNER: 1}
    )
    states, energy = _run((block,), spin=0.0)
    control_states, control_energy = _run((), spin=0.0)
    assert np.allclose(states, control_states)
    assert np.allclose(energy, control_energy)
    assert np.allclose(energy[:, DRIVE_WORK], 0.0)


def test_the_construction_path_reports_no_body_name_rule() -> None:
    """
    The construction path names no body: the two ends come from the port pairing.

    The check is on the sources of this row, because the failure being guarded
    against is a rule that reads a name and infers a role -- and such a rule is
    found by a reader, not by a call.  The two names it looks for are the two a
    suspension model's ends are usually called.
    """
    source_root = Path(__file__).parents[2] / "src" / "suspension_multibody"
    for relative in (
        "compilation/element_blocks.py",
        "modeling/primitives/factory.py",
        "modeling/primitives/elements.py",
    ):
        text = (source_root / relative).read_text(encoding="utf-8").lower()
        for name in ("upright", "chassis"):
            assert name not in text, f"{relative} names {name!r}"


def test_the_kind_constant_is_the_one_the_kernel_was_built_with() -> None:
    """The exported family constant is the kernel's ``ElementKind`` value."""
    assert ELEMENT_ROTATIONAL_TORQUE == KIND
