"""
The element-wrench fact channel: decoded facts, and the default path still off.

The channel is optional and off by default, so the two things that can go wrong
are opposite: a decoder that invents a fact (a zero-filled row read as an
applied wrench, an unknown type code treated as "some element") and a default
path that stops being the one it was.  The hand-built blocks below pin the first
-- including the row states native really writes, an untouched row and a row
that was opened without anything applied -- and the two native runs pin the
second: with the switch off the block is absent and the document stays at
``contract_version`` 1, with it on the same case yields real bushing facts.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.results import (
    ELEMENT_WRENCH_BLOCK,
    ELEMENT_WRENCH_SWITCH,
    ELEMENT_WRENCH_WIDTH,
    ElementWrenchRecord,
    decode_element_wrench,
    element_wrench_block,
    element_wrench_enabled,
)
from suspension_multibody.results.element_wrench import (
    ELEMENT_WRENCH_TYPE_NAMES,
    rows_per_element,
)
from suspension_multibody.schema import Bushing6x6, Pose, Vec3
from suspension_multibody.schema.model import AxleDeclaration
from tests.benchmark_fixture import benchmark_model

_NAN = float("nan")


def _row(
    *,
    force: tuple[float, float, float] = (_NAN, _NAN, _NAN),
    moment: tuple[float, float, float] = (_NAN, _NAN, _NAN),
    type_code: float = _NAN,
    point: tuple[float, float, float] = (_NAN, _NAN, _NAN),
    body_a: float = -1.0,
    body_b: float = -1.0,
    body: float = 0.0,
) -> list[float]:
    """Build one 13-column row, leaving unset columns NaN as native does."""
    return [*force, *moment, type_code, *point, body_a, body_b, body]


_SPRING = _row(
    force=(1.5, -2.25, 3.0),
    moment=(0.125, -0.5, 0.0),
    type_code=1.0,
    point=(0.1, 0.2, 0.3),
    body_a=0.0,
    body_b=2.0,
    body=2.0,
)
_BUSHING = _row(
    force=(-10.0, 20.0, -30.0),
    moment=(1.0, 2.0, 3.0),
    type_code=2.0,
    point=(-0.4, 0.5, 0.6),
    body_a=1.0,
    body_b=3.0,
    body=3.0,
)
# A row nothing was added to: every column stays as the block was allocated.
_UNSET = [_NAN] * ELEMENT_WRENCH_WIDTH
# A row that was opened and never added to: the identity columns are written,
# the force and the moment are not.
_OPENED_ONLY = _row(
    type_code=3.0,
    point=(0.25, -0.5, 0.75),
    body_a=0.0,
    body_b=4.0,
    body=4.0,
)
# A genuinely applied zero: the first contribution replaces the NaN, so the row
# is a fact even though every force and moment column is 0.0.
_ZERO = _row(
    force=(0.0, 0.0, 0.0),
    moment=(0.0, 0.0, 0.0),
    type_code=1.0,
    point=(0.7, 0.8, 0.9),
    body_a=0.0,
    body_b=2.0,
    body=2.0,
)


def _hand_block() -> np.ndarray:
    """Build a (2, 4, 13) block carrying one row of each state the channel has."""
    return np.array(
        [
            [_SPRING, _BUSHING, _UNSET, _ZERO],
            [_BUSHING, _OPENED_ONLY, _UNSET, _ZERO],
        ],
        dtype=float,
    )


def test_a_hand_built_block_decodes_only_the_rows_that_carry_a_wrench() -> None:
    records = decode_element_wrench({ELEMENT_WRENCH_BLOCK: _hand_block()})

    assert [record.type_name for record in records] == [
        "spring",
        "bushing",
        "spring",
        "bushing",
        "spring",
    ]
    assert [record.sample for record in records] == [0, 0, 0, 1, 1]
    assert isinstance(records[0], ElementWrenchRecord)

    spring = records[0]
    assert spring.type_code == 1
    assert spring.force == (1.5, -2.25, 3.0)
    assert spring.moment == (0.125, -0.5, 0.0)
    assert spring.point == (0.1, 0.2, 0.3)
    assert (spring.body_a, spring.body_b, spring.body) == (0, 2, 2)

    bushing = records[1]
    assert bushing.type_code == 2
    assert bushing.force == (-10.0, 20.0, -30.0)
    assert bushing.moment == (1.0, 2.0, 3.0)
    assert bushing.point == (-0.4, 0.5, 0.6)
    assert (bushing.body_a, bushing.body_b, bushing.body) == (1, 3, 3)

    # The zero row is a record; the untouched row and the opened-only row are
    # not, which is the whole difference between "applied nothing" and
    # "applied zero".
    zero = records[2]
    assert zero.force == (0.0, 0.0, 0.0)
    assert zero.moment == (0.0, 0.0, 0.0)
    assert zero.point == (0.7, 0.8, 0.9)
    assert all(
        not np.isnan(record.force).any() and not np.isnan(record.moment).any()
        for record in records
    ), "a row the element applied nothing to was decoded as a fact"


def test_a_pure_torque_row_is_a_record_and_keeps_its_nan_force() -> None:
    """
    A drive or a brake applies only a torque; its force columns stay NaN.

    Native's ``add_torque`` writes the moment columns alone, so a row an element
    applied only a torque to has NaN force and a real moment.  Requiring finite
    force would silently drop every drive/brake record, so the "did it apply
    anything" test has to accept a row on either vector.
    """
    torque_only = _row(
        moment=(10.0, -20.0, 30.0),
        type_code=5.0,
        body_a=0.0,
        body_b=-1.0,
        body=2.0,
    )
    block = np.stack([torque_only])[None, :, :]

    records = decode_element_wrench({ELEMENT_WRENCH_BLOCK: block})

    assert len(records) == 1
    record = records[0]
    assert record.type_name == "drive_brake"
    assert record.moment == (10.0, -20.0, 30.0)
    assert all(np.isnan(value) for value in record.force)
def test_rows_per_element_is_the_row_stride_of_the_frozen_layout() -> None:
    assert rows_per_element(1) == 2  # spring: one row per end
    assert rows_per_element(2) == 2  # bushing
    assert rows_per_element(3) == 2  # anti-roll
    assert rows_per_element(4) == 2  # steering
    assert rows_per_element(5) == 4  # drive/brake: drive, reaction, brake, reaction
    assert rows_per_element(6) == 1  # tire: one contact wrench
    assert rows_per_element(7) == 1  # external
    # The two laws split out of the fused spring record apply a wrench on each
    # end, like the spring they came from, so they reserve two rows each.
    assert rows_per_element(8) == 2  # damper
    assert rows_per_element(9) == 2  # bump stop
    with pytest.raises(ValueError, match="99"):
        rows_per_element(99)


def test_the_appended_codes_are_named_and_none_of_them_moved() -> None:
    """
    Every appended code keeps the old ones' meaning and does not renumber them.

    The channel's type codes are frozen: a stored artifact's rows carry them, so
    inserting a code would silently relabel every row after it.  The damper /
    bump-stop split appended its two laws at 8 and 9 for that reason, and
    `spring` now counts only the elastic structure.  The rotational actuator
    follows the same rule at 10 (subtask p2-09) -- the kernel's own code is
    `kElementWrenchRotationalTorque = 10`, and this table is that code's name on
    the reading side.
    """
    assert ELEMENT_WRENCH_TYPE_NAMES[1] == "spring"
    assert ELEMENT_WRENCH_TYPE_NAMES[8] == "damper"
    assert ELEMENT_WRENCH_TYPE_NAMES[9] == "bump_stop"
    assert ELEMENT_WRENCH_TYPE_NAMES[10] == "rotational_torque"
    # Every frozen code keeps its name and its position.
    assert ELEMENT_WRENCH_TYPE_NAMES[11] == "function"
    assert tuple(sorted(ELEMENT_WRENCH_TYPE_NAMES)) == (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11)


def test_a_missing_block_is_the_off_channel_and_not_an_error(monkeypatch) -> None:
    monkeypatch.delenv(ELEMENT_WRENCH_SWITCH, raising=False)
    blocks = {"body_state": np.zeros((1, 1, 19))}

    assert element_wrench_block(blocks) is None
    assert decode_element_wrench(blocks) == ()
    assert element_wrench_enabled() is False

    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "")
    assert element_wrench_enabled() is False
    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "0")
    assert element_wrench_enabled() is False
    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "1")
    assert element_wrench_enabled() is True


def test_a_block_of_the_wrong_shape_is_refused() -> None:
    with pytest.raises(ValueError) as narrow:
        decode_element_wrench({ELEMENT_WRENCH_BLOCK: np.zeros((2, 4, 12))})
    assert str(ELEMENT_WRENCH_WIDTH) in str(narrow.value)
    assert "(2, 4, 12)" in str(narrow.value)

    with pytest.raises(ValueError, match=str(ELEMENT_WRENCH_WIDTH)):
        decode_element_wrench({ELEMENT_WRENCH_BLOCK: np.zeros((4, 13))})


def test_an_unknown_type_code_is_refused() -> None:
    block = np.array(
        [
            [
                _row(
                    force=(1.0, 2.0, 3.0),
                    moment=(0.0, 0.0, 0.0),
                    type_code=99.0,
                    body=0.0,
                )
            ]
        ],
        dtype=float,
    )
    with pytest.raises(ValueError, match="99"):
        decode_element_wrench({ELEMENT_WRENCH_BLOCK: block})


def test_body_names_are_checked_and_never_written_into_the_records() -> None:
    block = {ELEMENT_WRENCH_BLOCK: _hand_block()}
    names = ("chassis", "rack", "upper_arm_L", "lower_arm_L", "upright_L", "tie_rod_L")

    records = decode_element_wrench(block, body_names=names)
    assert len(records) == 5
    assert not hasattr(records[0], "body_name"), "the record named a body itself"

    with pytest.raises(ValueError, match="body_b"):
        decode_element_wrench(block, body_names=("chassis",))

    # A negative index is native's "no such body", not an out-of-range name: a
    # tire's contact wrench and an external source carry one body, not two.
    external = _row(
        force=(1.0, 0.0, 0.0),
        moment=(0.0, 0.0, 0.0),
        type_code=7.0,
        point=(0.0, 0.0, 0.0),
        body_a=2.0,
        body_b=-1.0,
        body=2.0,
    )
    single = decode_element_wrench(
        {ELEMENT_WRENCH_BLOCK: np.array([[external]], dtype=float)}, body_names=names
    )
    assert len(single) == 1
    assert single[0].body_b == -1


def _compliant_model() -> AxleDeclaration:
    """Return the synthetic bushing model: the channel records what laws applied."""
    base = benchmark_model()
    stiffness = tuple(
        tuple(
            10_000.0
            if row == column and row < 3
            else 10_000_000.0
            if row == column
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )

    def mount(name: str) -> Pose:
        point = base.hardpoints[name]
        return Pose(
            translation=Vec3(x=float(point.x), y=float(point.y), z=float(point.z))
        )

    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{index}",
            body_a="chassis",
            body_b=body,
            pose_a=mount(name),
            pose_b=mount(name),
            stiffness=stiffness,
        )
        for body, names in (
            ("upper_arm", ("uca_front", "uca_rear")),
            ("lower_arm", ("lca_front", "lca_rear")),
        )
        for index, name in enumerate(names)
    )
    return base.model_copy(
        update={"bushings": bushings, "name": f"{base.name}_compliant"}
    )


def _run_c():
    """Run one loaded C case on the compliant model through the runner."""
    assembly, case = migrate_v1_kc_case(_compliant_model(), mode="C", paths=("fz",), levels=3, maximum=500)
    payload = case.to_payload()
    payload["excitation"]["c"]["loads"] = [{"fz": 500.0, "fx": 120.0}]
    return simulate(assembly, payload).raw


def test_a_real_run_decodes_bushing_facts_when_the_switch_is_on(monkeypatch) -> None:
    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "1")
    result = _run_c()

    block = element_wrench_block(result)
    assert block is not None
    assert block.shape[2] == ELEMENT_WRENCH_WIDTH
    assert result.document["contract_version"] == 2

    records = decode_element_wrench(result, body_names=result.body_names)
    assert records, "the channel was on and decoded nothing"
    bushings = [record for record in records if record.type_name == "bushing"]
    assert bushings, "a loaded C case recorded no bushing wrench"
    for record in records:
        assert all(np.isfinite(record.force)), "an unset row became a record"
        assert all(np.isfinite(record.moment)), "an unset row became a record"
    assert any(
        any(value != 0.0 for value in record.moment) for record in bushings
    ), "every bushing moment was zero, so the decode may be reading padding"


def test_the_unified_path_explicitly_requests_wrench_facts_without_an_environment_switch(monkeypatch) -> None:
    monkeypatch.delenv(ELEMENT_WRENCH_SWITCH, raising=False)
    result = _run_c()

    assert result.document["contract_version"] == 2
    assert element_wrench_block(result) is not None
    assert decode_element_wrench(result)
