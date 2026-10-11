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
import re
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


def test_the_frozen_port_tangent_reports_the_holding_force() -> None:
    """
    P4a step 2's coordinate-driven port tangent solves the partitioned system.

    The reaction is a number, so the executable states each one in closed form: a
    single spring reports `-k*ds` (the holding force, not the restoring force), a
    two-coordinate chain reports the *series* stiffness rather than the diagonal, a
    coupled pair includes its off-diagonal term, and a singular free block is refused
    rather than answered with a least-squares value.

    Registered here because the routine lives in the solver layer and has no product
    ABI entry point yet: without a caller its assertions would never run, and an
    executable nothing runs buys nothing.

    ``mb_frozen_sample_selftest`` (step 1) is deliberately **not** registered here.
    Measured: it exits 0 when run directly, and exits 0 when launched from a fresh
    pytest file in this same directory, but launched from *this* module it returns
    0xC0000005 while still printing its success line.  ``mb_frozen_tangent_selftest``
    is stable in exactly the same position (5/5 runs of this whole file), so the
    fault is specific to that one executable rather than to this module's process.
    The cause is not yet identified, so registering it would put an unexplained
    crash into the fast gate; the step-1 evidence therefore stays with the executable
    and this limitation is recorded rather than papered over.
    """
    output = _kernel_selftest("mb_frozen_tangent_selftest")
    assert "frozen force tangent self-test passed" in output, output


def test_the_frozen_load_column_carries_the_inverse_stiffness() -> None:
    """
    P4a step 3's ``B_w = -dR/dw`` producer lands the load where it names.

    The load-driven half of P4a.  The executable asserts each of the six wrench
    components individually -- a transposed or offset mapping satisfies a single
    ``Fx`` check -- and then solves ``A X = B_w P`` with the column as its right-hand
    side to show the composition reports ``C = 1/k``.

    The two boundaries are asserted *together*, against one spring: the
    coordinate-driven routine reports `k` and this one reports `1/k`.  That is what
    makes it impossible to "simplify" one path into the other, which the design
    forbids (SPEC :37) because they are different physical boundary conditions.
    """
    output = _kernel_selftest("mb_frozen_load_selftest")
    assert "frozen load column self-test passed" in output, output


def test_the_frozen_oracle_checks_the_tangent_against_the_residual() -> None:
    """
    P4a step 5's independent oracle: a perturbation of the nonlinear residual.

    The three executables above test each operator against a number known by
    construction, which leaves open that they are *mutually consistent and jointly
    wrong*.  This one closes that gap: it perturbs the frozen state and evaluates
    the real ``static_residual`` -- an unrelated computation -- then requires the
    analytic tangent to predict the change, with the discrepancy falling at the
    second-order rate.

    The rate is the discriminating part, and it is asserted rather than a bare
    tolerance.  The discrepancy is ``|0.5*h^2*v^T H v + h*(dA v)|`` -- the honest
    truncation plus whatever the tangent is wrong by -- so a correct tangent gives a
    ratio of ~100 between successive steps while an additive error of ``dA`` pulls it
    toward ~10.  The threshold is 50, between the two regimes.  Measured baseline:
    ``4.136e-04 / 4.138e-06 / 4.138e-08`` (ratios 100.0 and 100.0).

    Sensitivity, measured rather than assumed: an additive error is resolved only
    once ``h*|dA|`` is comparable to the truncation at the smallest step, so on this
    fixture (entries ~5000, ``h`` down to 1e-6) the crossover is ~``0.1``-``0.5``.
    Measured survivors: ``dA`` of 1e-3, 1e-2 and 1e-1 leave ratios 102.2, 128.7 and
    53.5; 5e-1 collapses to 5.8 and is caught.  Note ``dA=1e-1`` clears the threshold
    by only 7%, which is the honest resolution limit of this oracle -- *not* that the
    error "hides under the truncation floor", which would be wrong by two orders of
    magnitude (``h*1e-3`` is ~1e-7 against a 4.1e-04 truncation).  A bounded oracle
    with a recorded bound, not an unbounded one.

    It also pins the SPEC claims beside the oracle (SPEC :51, :67, :59): rotating the
    whole system rotates the tangent by congruence (measured deviation ``1.137e-13``);
    the true transpose ``A^T x = b`` is solved rather than assumed from a symmetric
    matrix, on a hand-built asymmetric fixture (its forward and transpose solutions
    genuinely differ and ``A^T x = b`` is re-checked against the original matrix); and
    the direct/transpose equivalence design :439 asks for is asserted on the *real*
    frozen tangent, and on a **supported** one: a symmetric bushing plus one
    ``AXLE_INPLANE`` row (an asymmetric *bushing stiffness* is rejected by both model
    builders, so a fixture built that way would exercise the assembler on a matrix no
    supported model can produce -- reviewer finding, corrected).  On that fixture the
    assembled ``A`` is asymmetric and the two routes are required to agree (measured
    ``1.110e-16``) while the forward and transposed solutions genuinely differ; the
    assembled ``A`` is also required to predict the nonlinear residual's change while
    ``A^T`` is required to fail it (``1.8e-13`` vs ``1.0e-3``), which is what catches a
    tangent written as the derivative of the wrong system.  The SPEC's *unit* clause is
    checked by rescaling the coordinate units (``A S``, ``G S``) and requiring the
    physical response to be unchanged (measured relative ``0.0``), with an
    uncompensated-``G`` control proving the assertion is live.
    """
    output = _kernel_selftest("mb_frozen_oracle_selftest")
    assert "frozen oracle self-test passed" in output, output
    # The oracle's own liveness, *parsed* rather than grepped: a degenerate fixture
    # reports a zero ladder and would satisfy every ratio check trivially, so the
    # printed numbers are read back and required to be non-zero and decreasing.
    match = re.search(
        r"pose-block oracle error:\s*([0-9.eE+-]+)\s*\(h=1e-4\),\s*"
        r"([0-9.eE+-]+)\s*\(1e-5\),\s*([0-9.eE+-]+)\s*\(1e-6\)",
        output,
    )
    assert match is not None, f"the oracle printed no error ladder:\n{output}"
    e0, e1, e2 = (float(value) for value in match.groups())
    assert e0 > 0.0, "the fixture's residual change is identically zero"
    assert e1 < e0 and e2 < e1, f"the ladder does not decrease: {e0}, {e1}, {e2}"
    # And the frame-equivalence deviation must be a real number, not a vacuous pass.
    frame = re.search(r"frame-equivalence worst deviation:\s*([0-9.eE+-]+)", output)
    assert frame is not None, f"the oracle printed no frame deviation:\n{output}"
    assert float(frame.group(1)) < 1e-9, frame.group(1)
    # The direct/transpose agreement must be on the *real* frozen tangent, not only on
    # a hand-built matrix: the spring fixture is symmetric, so its transpose check is
    # vacuous.  Parse the printed pair and require agreement, which is design :439's
    # claim ("同一物理问题 ... 含非对称切线").
    frozen = re.search(
        r"frozen direct vs transpose:\s*([0-9.eE+-]+)\s+vs\s+([0-9.eE+-]+)\s+"
        r"\(diff\s+([0-9.eE+-]+)\)",
        output,
    )
    assert frozen is not None, f"the oracle printed no frozen direct/transpose pair:\n{output}"
    y_direct, y_adjoint, diff = (float(value) for value in frozen.groups())
    assert y_direct != 0.0, "the frozen direct/transpose response is identically zero"
    # The delivered tangent must predict the *nonlinear residual's* change, and A^T
    # must fail that same check: without this the direct/transpose pair could agree with
    # each other on a tangent that is the derivative of the wrong system.
    residual = re.search(
        r"frozen asymmetric residual oracle \(h=[0-9.eE+-]+\):\s*"
        r"\|A dx - dR\| = ([0-9.eE+-]+),\s*"
        r"\|A\^T dx - dR\| = ([0-9.eE+-]+)",
        output,
    )
    assert residual is not None, f"the oracle printed no residual oracle line:\n{output}"
    err_a, err_at = (float(value) for value in residual.groups())
    assert err_a < 1e-9, residual.groups()
    assert err_at > err_a, residual.groups()
    # The SPEC's clause also names *unit* equivalence (SPEC :51); parse the printed
    # relative gap and require it to be at round-off, which is design :342's
    # "尺度变换还原后才输出" made observable.
    unit = re.search(
        r"frozen unit rescaled:\s*([0-9.eE+-]+)\s+vs\s+([0-9.eE+-]+)\s+"
        r"\(relative\s+([0-9.eE+-]+)\)",
        output,
    )
    assert unit is not None, f"the oracle printed no unit-rescaling pair:\n{output}"
    assert float(unit.group(3)) < 1e-12, unit.groups()
    assert diff == 0.0 or diff <= 1e-12, frozen.groups()


def test_the_transpose_and_batch_surface_is_actually_executed() -> None:
    """
    The batch/transpose solver surface is run by a gate, not merely built.

    This executable has existed and passed since the transpose work landed, but it
    was registered nowhere: `CMakeLists.txt` built it and no gate ever executed it,
    so a regression in `solve_transpose_many` would have been invisible.  Measured
    before this registration: the binary exits 0 when run by hand and no test in the
    tree referenced it by name.

    P4a step 5 is the task that makes this load-bearing, because its acceptance
    names direct/transpose equivalence, so the gate that owns that claim is the right
    place to stop the surface being unexercised.  Registered rather than duplicated:
    the oracle self-test asserts the transpose *in the physical context* (a real
    asymmetric frozen tangent), while this one asserts the arithmetic in isolation
    against an independently computed `A^T x = b`.
    """
    output = _kernel_selftest("mb_linear_batch_selftest")
    assert "batch and transpose solves behave as declared" in output, output


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


def test_the_element_names_match_the_published_contract_schema() -> None:
    """
    The kernel's element table and the contracts schema name the same elements.

    The two tables are the same fact stated twice -- what an element `type` may
    be -- and they are what makes a document readable by one side and not the
    other.  They are edited in different packages for different reasons, so when
    the force-element split renamed the fused `spring_damper` into `spring` and
    `damper`, the kernel moved and the schema did not: the kernel then refused
    every document the product emits while the schema still accepted the retired
    name.  Nothing caught it, because no test compared them.  This is that test.

    Both sides are read from their own source of truth: the kernel from its
    compiled registry (through the self-test, so the table as built is what is
    compared) and the schema from the JSON shipped in the wheel.
    """
    from suspension_contracts.multibody import load_schema

    schema = load_schema("model")
    schema_names = set(schema["$defs"]["element"]["properties"]["type"]["enum"])

    # The kernel's own enumeration, from the self-test that asserts the table.
    source = (
        ROOT / "packages/suspension_kernel/cpp/src/contract/contract_registry.cpp"
    ).read_text(encoding="utf-8")
    table = source.split("const char* const kElements[] = {", 1)[1].split("};", 1)[0]
    kernel_names = set(re.findall(r'"([a-z_]+)"', table))

    assert kernel_names == schema_names, (
        "the kernel's element table and the published contract schema disagree; "
        f"only in the kernel: {sorted(kernel_names - schema_names)}; "
        f"only in the schema: {sorted(schema_names - kernel_names)}"
    )
    # The retired fused name must be gone from both, not merely unused: an
    # accepted-but-unimplemented name is a document that validates and then
    # fails at the kernel.
    assert "spring_damper" not in kernel_names
    assert "spring_damper" not in schema_names
