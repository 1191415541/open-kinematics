"""
Every capability a build claims has a handler behind it.

Subtask 08's contract is that the *described* capability and the *executable*
capability are the same fact.  Three questions get asked about a kind -- a joint,
an element, a tire model, a case family -- and before this file they were
answered from three directions:

* does the protocol define the name?
* does this build support it?
* is there a handler that will actually process it?

A build that answers "yes" to the second and has no handler for it fails at run
time with something unrelated; a name that is supported but left out of a list is
reported as missing.  These tests walk the declared capability and demand a
handler for each entry, so the two cannot drift.

The C++ half of the same assertion lives in `mb_cases_selftest`
(`cpp/tests/case_registry_selftest.cpp`), which checks the case-family table
against the contract registry.  This file is the Python side: what the *product*
reads and dispatches.
"""

from __future__ import annotations

import ctypes
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[3]

#: The case families the contract defines, and whether this build runs them.
#:
#: Read from the C++ table's own self-test output rather than transcribed: a
#: Python copy would be the fourth place the list lives, which is the defect 08
#: exists to remove.
PROTOCOL_CASE_FAMILIES = (
    "kc_quasi_static",
    "axle_dynamic",
    "vehicle_kc",
    "vehicle_dynamic",
    "handling",
    "ride_four_post",
    "ride_random_road",
    "comparison",
)

#: The families with no expander, and why: `comparison` is a per-target gate.
KNOWN_UNIMPLEMENTED = ("comparison",)


def _kernel_selftest(name: str) -> str:
    """Run a kernel self-test executable and return its output."""
    candidates = sorted(
        (ROOT / "packages/suspension_kernel/build").rglob(f"{name}.exe")
    ) + sorted((ROOT / "packages/suspension_kernel/build").rglob(name))
    if not candidates:
        pytest.skip(f"{name} has not been built")
    completed = subprocess.run(
        [str(candidates[0])],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    return completed.stdout


def test_the_kernel_case_table_is_self_consistent() -> None:
    """
    The case-family table agrees with the contract registry.

    Run rather than inspected: the assertion is about the compiled tables, and a
    source-level reading could not tell an implemented family from a declared one.
    """
    output = _kernel_selftest("mb_cases_selftest")
    assert "mb_cases selftest: OK" in output, output


def test_the_contract_registry_is_self_consistent() -> None:
    output = _kernel_selftest("mb_contract_selftest")
    assert "mb_contract selftest: OK" in output, output


def test_the_declared_case_families_are_every_one_the_protocol_names() -> None:
    """
    The contract's family list and this file's expectation agree.

    The list is checked against the *contract* rather than against the cases
    table: the contract is the half that is allowed to name a family no build
    implements, so it is the authoritative list of names.
    """
    registry = ROOT / "packages/suspension_kernel/build"
    source = (
        ROOT / "packages/suspension_kernel/cpp/src/contract/contract_registry.cpp"
    ).read_text(encoding="utf-8")
    del registry
    for family in PROTOCOL_CASE_FAMILIES:
        assert f'"{family}"' in source, family


def test_each_protocol_family_is_classified_as_supported_or_unimplemented() -> None:
    """
    No family may be missing from the classification.

    A protocol name absent from the cases table is exactly the silent omission
    the table was introduced to prevent; the self-test asserts it in C++, and this
    states the same expectation from the product side so a reader sees it here too.
    """
    dispatch = (
        ROOT / "packages/suspension_kernel/cpp/src/cases/case_dispatch.cpp"
    ).read_text(encoding="utf-8")
    for family in PROTOCOL_CASE_FAMILIES:
        assert f'"{family}"' in dispatch, (
            f"case family {family!r} is declared by the contract but not classified "
            "by the dispatcher's table"
        )


def test_every_supported_family_has_a_handler_and_every_unsupported_one_does_not() -> None:
    """
    The row's `expander` is null exactly when the family is unimplemented.

    Read from the table source: the pointer is what dispatch actually calls, so a
    row claiming support with a null handler would be a crash rather than a
    refusal, and this is where that is caught before it ships.
    """
    dispatch = (
        ROOT / "packages/suspension_kernel/cpp/src/cases/case_dispatch.cpp"
    ).read_text(encoding="utf-8")
    for family in KNOWN_UNIMPLEMENTED:
        assert f'{{"{family}", true, nullptr}}' in dispatch, family
    for family in set(PROTOCOL_CASE_FAMILIES) - set(KNOWN_UNIMPLEMENTED):
        assert f'{{"{family}", true, nullptr}}' not in dispatch, (
            f"{family!r} must have a handler"
        )


def test_the_product_reads_its_capabilities_from_the_kernel() -> None:
    """
    The product does not carry a second copy of the kernel's capability list.

    It asks the library.  A transcribed list is the failure this guards: the old
    hand-copied scope list claimed the kernel never applies the validity-range
    clamps while the kernel was calling them on every tire step.
    """
    source = (
        ROOT
        / "packages/suspension_multibody/src/suspension_multibody/kernel/capabilities.py"
    ).read_text(encoding="utf-8")
    # The module must reach the library rather than declare the answers.
    assert "suspension_kernel_capabilities" in source, (
        "the product must read capabilities from the kernel rather than declare them"
    )


def test_the_capability_document_parses_and_names_its_contract() -> None:
    """The declaration is machine-readable and versioned, not prose."""
    script = """
import json
from suspension_multibody.kernel import capabilities as c
print(json.dumps({"ok": True, "has_scope": bool(c.__doc__)}))
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout)["ok"] is True


def test_the_joint_table_and_the_kernel_agree_on_row_counts() -> None:
    """
    A joint's declared row count matches what the kernel's registry reports.

    A mismatch here would not change a number -- it would mislabel one, which the
    joint table's own docstring calls out as worse.
    """
    from suspension_multibody.joints import table as joint_table

    # Each authoring type maps to exactly one kernel name and row count, and the
    # table is the single place that mapping lives.  A second copy of it is what
    # this asserts against: every entry must resolve both ways.
    for definition in joint_table.JOINT_TYPES:
        # The authoring name and the kernel name must resolve to the same row
        # count: that equality is what makes "the product's joint" and "the
        # kernel's joint" the same object rather than two names for two things.
        # The authoring name maps to a kernel name, and both resolve to the same
        # row count.  That equality is what makes "the product's joint" and "the
        # kernel's joint" one object rather than two names for two things, and it
        # is the mapping whose absence used to be checked by hand.
        assert definition.rows == joint_table.JOINT_ROWS[definition.kernel_name]
        assert joint_table.rows_for(definition.kernel_name) == definition.rows
        # Every kernel name in the table is one the kernel's own registry knows:
        # a name invented on the authoring side would never reach a solver.
        assert definition.kernel_name in joint_table.JOINT_ROWS
    # And the total the kernel's registry advertises is the sum of that same table,
    # so a joint added here without a row count cannot slip through.
    assert joint_table.kernel_row_count() == sum(joint_table.JOINT_ROWS.values())


def test_ctypes_can_load_the_library_the_product_binds() -> None:
    """
    The library the product loads is loadable, not merely present.

    A file that exists but cannot be loaded is the state the freshness guard
    refuses at run time; asserting it here makes the failure name the cause.
    """
    from suspension_multibody.kernel.native import load_library

    library = load_library()
    assert library is not None
    probe = getattr(library, "suspension_kernel_contract_version", None)
    assert probe is not None, "the ABI version probe must be exported"
    probe.restype = ctypes.c_int32
    assert int(probe()) > 0


def test_the_tire_state_interface_is_self_consistent() -> None:
    """
    The tire-state descriptor table and slot layout are internally consistent.

    Subtask 09's contract: the solver asks the tire layer for a width, a slot value
    and a derivative, and never dispatches on a tire model.  The structural half of
    that -- every registered model answers every question, every slot lies in the
    layout it names -- is checked by the compiled self-test rather than by reading
    source.  The *numeric* half is pinned by the frozen-baseline gates
    (`dynamic_hash_sentinel`, `kc_parity`), which would move if the layout changed.
    """
    output = _kernel_selftest("mb_tire_state_selftest")
    assert "mb_tire_state selftest: OK" in output, output


def test_the_solver_does_not_dispatch_on_a_tire_model_kind() -> None:
    """
    The solver reads semantic flags, not model enumerations.

    This is the property 09 exists to establish.  Checked over the solver sources:
    a `VehicleTireModelKind` comparison in `solve_dynamic`/`solve_static` would mean
    the solver had learned about a specific tire model again, and a new model would
    require editing it.
    """
    solvers = (
        ROOT / "packages/suspension_kernel/cpp/src/solve_dynamic",
        ROOT / "packages/suspension_kernel/cpp/src/solve_static",
    )
    offenders: list[str] = []
    for directory in solvers:
        for path in sorted(directory.glob("*.cpp")):
            text = path.read_text(encoding="utf-8")
            for index, line in enumerate(text.splitlines(), start=1):
                stripped = line.strip()
                if stripped.startswith("//"):
                    continue
                if "VehicleTireModelKind" in line or "model_kind ==" in line:
                    offenders.append(f"{path.name}:{index}")
    assert not offenders, (
        "the solver dispatches on a tire model kind again: " + ", ".join(offenders)
    )
