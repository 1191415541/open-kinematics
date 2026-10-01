"""
The rotational-torque element: its block is readable, and its couple is right.

p2-02 adds one element family to the kernel -- a pure couple whose magnitude a
driver demand sets and whose direction the real-time relative angular rate
decides.  Two things have to be true of it, and this file is where both are
pinned:

* a block of `ELEMENT_ROTATIONAL_TORQUE` survives `read_element_blocks` and
  reaches `Model::rotational_torques` with the values it carried, so the family
  is reachable from the ABI rather than merely declared in the header;
* the law it applies has the three states the epic names: a stationary pair gets
  no couple at all, a reversed pair gets the opposite sign, and a demand above
  `max_torque` is capped rather than followed.

Why a compiled probe instead of a pure-Python test
--------------------------------------------------

`Model::rotational_torques` and the couple the law accumulates are internal: the
product C ABI exposes no per-element wrench channel for one law, and adding one
to a frozen structure is an ABI release this step must not open.  So the
observable under test is only reachable from C++.  ``fixtures/rotational_torque_probe.cpp``
is that observable: it compiles against the kernel's public headers, links the
static libraries the kernel build already produced, and prints one line per fact
asserted below.  Nothing here re-implements the law -- the probe calls it.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
KERNEL = ROOT / "packages" / "suspension_kernel"
FIXTURE = KERNEL / "tests" / "fixtures" / "rotational_torque_probe.cpp"
INPUT_HEADER = KERNEL / "cpp" / "include" / "mb_input" / "types.hpp"


def _relative(path: Path) -> str:
    """
    Return ``path`` relative to the repository root, as the linker wants it.

    Every *input* goes through this: the LTO plugin opens each archive it is
    handed, and it fails on an absolute path under this repository, whose
    directory name is non-ASCII.  The linker's own output does not go through the
    plugin, so the ``-o`` path may stay absolute.
    """
    return path.relative_to(ROOT).as_posix()

def _kernel_archives(build: Path) -> list[Path]:
    """
    Return the static libraries the *current* CMake configuration builds.

    Globbing ``libmb_*.a`` is not enough.  A build directory that predates the
    module split still holds the libraries the split replaced -- ``libmb_base.a``
    among them -- and those carry copies of symbols the current configuration
    gives to their successors.  Whether the linker sees the duplicate then
    depends on what pulls both archives in, so a probe that needs ``mb_config``
    fails while one that does not links fine.  ``TargetDirectories.txt`` is
    CMake's own record of the targets this directory was configured for and is
    written by every generator, so deriving the list from it links exactly what
    the shared library under test links against.  A directory with no record
    falls back to the glob.
    """
    directories = build / "CMakeFiles" / "TargetDirectories.txt"
    if not directories.is_file():
        return sorted(build.glob("libmb_*.a"))
    archives: list[Path] = []
    for line in directories.read_text(encoding="utf-8").splitlines():
        name = Path(line.strip()).name
        if not name.startswith("mb_") or not name.endswith(".dir"):
            continue
        archive = build / f"lib{name[:-len('.dir')]}.a"
        if archive.is_file():
            archives.append(archive)
    return sorted(archives)


@pytest.fixture(scope="module")
def probe(tmp_path_factory: pytest.TempPathFactory) -> dict[str, list[str]]:
    """Compile the probe once and return its output grouped by line prefix."""
    if not FIXTURE.is_file():
        pytest.skip(f"probe source is missing at {FIXTURE}")
    build = KERNEL / "build" / "Release"
    archives = _kernel_archives(build)
    if not archives:
        pytest.skip(f"kernel static libraries are not built under {build}")

    sys.path.insert(0, str(KERNEL / "src"))
    from suspension_kernel.binding.build import discover_compiler

    executable = (
        tmp_path_factory.mktemp("rotational_torque")
        / "rotational_torque_probe.exe"
    )
    compiled = subprocess.run(
        [
            str(discover_compiler()),
            "-std=c++17",
            "-O2",
            "-flto=auto",
            "-fno-fat-lto-objects",
            "-fopenmp",
            "-I",
            _relative(KERNEL / "cpp" / "include"),
            _relative(FIXTURE),
            "-o",
            str(executable),
            "-Wl,--start-group",
            *[_relative(archive) for archive in archives],
            "-Wl,--end-group",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if compiled.returncode != 0:
        pytest.fail(
            "the rotational-torque probe did not compile:\n" + compiled.stderr
        )

    completed = subprocess.run(
        [str(executable)], capture_output=True, text=True, check=False
    )
    assert completed.returncode == 0, completed.stderr
    grouped: dict[str, list[str]] = {}
    for line in completed.stdout.splitlines():
        group, _, rest = line.partition(" ")
        grouped.setdefault(group, []).append(rest)
    return grouped


def _eval(probe: dict[str, list[str]], label: str) -> tuple[float, float]:
    """Return the couple the law applied to body a and body b for one state."""
    prefix = f"{label} tau_a "
    for line in probe["eval"]:
        if line.startswith(prefix):
            _, _, a, _, b = line.split()
            return float(a), float(b)
    raise AssertionError(f"the probe printed no `eval {label}` line")


def test_the_block_reaches_the_built_model(probe: dict[str, list[str]]) -> None:
    """A block of the new kind is read into `Model::rotational_torques`."""
    assert "ok 1" in probe["reader"], probe["reader"]
    assert "count 1" in probe["reader"], probe["reader"]
    assert "bodies 0 1" in probe["reader"], probe["reader"]


def test_the_reader_keeps_the_values_the_block_carried(
    probe: dict[str, list[str]],
) -> None:
    """
    The family's own parameter slots are the ones read, and the axis is
    normalised the way the anti-roll bar's is.
    """
    assert "stiffness 250" in probe["reader"], probe["reader"]
    assert "max_torque 900" in probe["reader"], probe["reader"]
    # The axis was written as the exact unit vector +y, so normalisation must
    # leave it alone; a reader that read the wrong slots would print zeros here.
    assert "axis 0 1 0" in probe["reader"], probe["reader"]


def test_a_stationary_pair_gets_no_couple(probe: dict[str, list[str]]) -> None:
    """
    A stopped pair is not accelerated by the sign of a rounding error.

    This is the epic's first state.  The rate is exactly zero here, which is
    inside `kEps`, so the law takes its third branch and applies nothing: the
    assertion is that *both* ends are exactly zero, which is what "does not
    produce a reverse acceleration" means.  A law that picked a sign at zero
    would print a full-magnitude couple on both lines.
    """
    tau_a, tau_b = _eval(probe, "still")
    assert tau_a == 0.0, tau_a
    assert tau_b == 0.0, tau_b


def test_the_couple_opposes_a_forward_rate(probe: dict[str, list[str]]) -> None:
    """
    Spinning forward, the pair is slowed: the couple on the spinning body is
    negative along the axis, and the two ends are equal and opposite.
    """
    tau_a, tau_b = _eval(probe, "forward")
    assert tau_b < 0.0, tau_b
    # Equal and opposite: whatever one end receives, the other receives negated.
    assert tau_a == -tau_b, (tau_a, tau_b)
    # And the magnitude is the demand, not the cap, for this state.
    assert tau_b == -400.0, tau_b


def test_the_couple_reverses_with_a_reverse_rate(
    probe: dict[str, list[str]],
) -> None:
    """
    The epic's second state: spinning backwards, the couple flips sign.

    Comparing the two directions rather than asserting an absolute number is
    what makes this a statement about the *sign law*: a law that ignored the
    rate's sign would return the same value in both, and this would fail.
    """
    forward_a, forward_b = _eval(probe, "forward")
    reverse_a, reverse_b = _eval(probe, "reverse")
    assert reverse_b > 0.0, reverse_b
    assert reverse_b == -forward_b, (forward_b, reverse_b)
    assert reverse_a == -reverse_b, (reverse_a, reverse_b)


def test_an_over_demand_couple_saturates_at_the_cap(
    probe: dict[str, list[str]],
) -> None:
    """
    The epic's third state, in this element's spelling: the magnitude is capped.

    The demand here is five times the cap, so the couple must be the cap and
    nothing more.  The probe also prints the same state with a demand below the
    cap, so this assertion is about the cap binding rather than about the law
    dropping its demand: a law that always saturated would fail the uncapped
    comparison, and a law that never saturated would fail the capped one.
    """
    capped_a, capped_b = _eval(probe, "capped")
    uncapped_a, uncapped_b = _eval(probe, "uncapped")
    assert capped_b == -1000.0, capped_b
    assert capped_a == -capped_b, (capped_a, capped_b)
    assert uncapped_b == -400.0, uncapped_b
    # The capped couple is bounded by the cap, and the demand above it made no
    # difference: both are the cap.
    assert abs(capped_b) == 1000.0
    assert abs(capped_b) > abs(uncapped_b)


def test_a_grounded_reaction_lands_on_the_one_body(
    probe: dict[str, list[str]],
) -> None:
    """
    With `body_b` = -1 the reaction has no body, so only `a` receives a couple.

    That is the ground reference case, and the assertion is that the absent end
    stays exactly zero rather than accumulating something.
    """
    tau_a, tau_b = _eval(probe, "grounded")
    assert tau_b == 0.0, tau_b
    assert tau_a < 0.0, tau_a


def test_the_demand_channel_follows_the_driver_signal(
    probe: dict[str, list[str]],
) -> None:
    """
    A block with a demand source reads the case's driver signal, not its own gain.

    The gain is 400 and the driver asks for 0.5, so the couple is 200.  The unit
    demand would return the full 400, which is what makes this a statement about
    *which* number the law read rather than about the law still working.
    """
    driven_a, driven_b = _eval(probe, "driven")
    assert driven_b == -200.0, driven_b
    assert driven_a == -driven_b, (driven_a, driven_b)


def test_a_zero_demand_applies_no_couple(probe: dict[str, list[str]]) -> None:
    """
    A driver who asks for nothing gets nothing, even with the pair spinning.

    This is what makes the demand the *magnitude* rather than a constant: the
    same block, the same rate, and a different demand give a different couple.
    """
    released_a, released_b = _eval(probe, "no_demand")
    assert released_a == 0.0, released_a
    assert released_b == 0.0, released_b
    # And the block without a source under the same state is the old law's: the
    # unit demand is the gain, so the pair is slowed.
    still_a, still_b = _eval(probe, "unit_still")
    assert still_a == 0.0, still_a
    assert still_b == 0.0, still_b


def test_a_locked_wheel_is_still_braked(probe: dict[str, list[str]]) -> None:
    """
    The state the road map's 2.1 section names: a held wheel under full demand.

    The pair has stopped turning (`omega` exactly zero, so the rate branch takes
    its third outcome) while the tire is still slipping at 0.8.  The couple must
    then be the *demanded magnitude*, not zero -- the failure the old path had,
    where ``omega == 0`` meant ``brake torque == 0`` for a wheel a real brake
    would still be holding.  The magnitude compared here is the full demand
    (``400 * 1.0``), so an implementation that quietly scaled by the rate would
    fail it.
    """
    locked_a, locked_b = _eval(probe, "locked")
    assert locked_b == -400.0, locked_b
    assert locked_a == -locked_b, (locked_a, locked_b)
    # The sign follows the slip, so the locked pair is not accelerated by the law
    # picking one direction at a standstill: reversing the slip reverses it.
    reverse_a, reverse_b = _eval(probe, "locked_reverse")
    assert reverse_b == 400.0, reverse_b
    assert reverse_b == -locked_b, (locked_b, reverse_b)
    assert reverse_a == -reverse_b, (reverse_a, reverse_b)


def test_the_demand_channel_does_not_read_the_torque_tables(
    probe: dict[str, list[str]],
) -> None:
    """
    The normalized demand lives in its own buffer, not in the N*m tables.

    This is subtask p2-10's whole point.  The probe gives the same wheel a
    full brake demand of 1.0 *and* a stale 250 N*m in the old brake table: a
    law that still read the N*m buffer would take 250 as the fraction (capped
    at the block's 1000 `max_torque`, so the couple would be -1000), while one
    that reads the demand buffer prints the 400 its gain asks for.  Without
    this assertion, "the two unit systems are separate" would be a comment.
    """
    decoupled_a, decoupled_b = _eval(probe, "decoupled")
    assert decoupled_b == -400.0, decoupled_b
    assert decoupled_a == -decoupled_b, (decoupled_a, decoupled_b)


def test_the_sampler_keeps_the_two_unit_systems_apart(
    probe: dict[str, list[str]],
) -> None:
    """
    `interpolate_input` fills four destinations and mixes none of them.

    The case states wheel torque `10 -> 20`, brake torque `30 -> 40` and, in
    the second reading, wheel demand `0.25 -> 0.75` and brake demand
    `0.5 -> 1.0`.  At `t = 0.5` every one of those is a midpoint, so:

    * with no demand declared the two demand vectors stay zero and the N*m
      ones carry the case's own values (15 and 35);
    * with demands declared the demand vectors carry 0.5 and 0.75 while the
      N*m destinations are *unchanged* -- the normalization never leaks into
      the torque buffers, which is the double-application p2-10 exists to
      prevent.
    """
    lines = probe["isolate"]
    none_line = [line for line in lines if line.startswith("none ")]
    both_line = [line for line in lines if line.startswith("both ")]
    assert len(none_line) == 1, lines
    assert len(both_line) == 1, lines
    assert none_line[0] == (
        "none torque 15 brake_torque 35 wheel_demand 0 brake_demand 0"
    ), none_line[0]
    assert both_line[0] == (
        "both torque 15 brake_torque 35 wheel_demand 0.5 brake_demand 0.75"
    ), both_line[0]


def test_the_reader_keeps_the_demand_slots(probe: dict[str, list[str]]) -> None:
    """
    The two integer slots reach the model rather than being read and dropped.

    The probe writes source 2 (the brake column) and tire 2, and prints what the
    model kept: a reader that skipped the slots would leave the family's defaults
    -- unit demand and no tire -- and a law built on those would apply the gain
    itself.
    """
    assert "demand_source 2 2" in probe["reader"], probe["reader"]


def test_the_family_has_its_own_layout_row() -> None:
    """
    The new kind is in the shared layout table, with its own parameter run.

    The block's shape is what the two entry points share, so a family that had a
    `kind` but no row would be refused as "unknown element kind" -- which is
    correct but not what this row delivers.
    """
    text = INPUT_HEADER.read_text(encoding="utf-8")
    assert "ELEMENT_ROTATIONAL_TORQUE = 7" in text
    assert "{ELEMENT_ROTATIONAL_TORQUE, ELEMENT_ROTATIONAL_TORQUE_STIFFNESS" in text
    # And the appended kind did not move any existing one.
    for name, value in (
        ("ELEMENT_SPRING", 0),
        ("ELEMENT_BUSHING", 1),
        ("ELEMENT_ANTI_ROLL", 2),
        ("ELEMENT_TIRE", 3),
        ("ELEMENT_AERODYNAMIC_DRAG", 4),
        ("ELEMENT_DAMPER", 5),
        ("ELEMENT_BUMP_STOP", 6),
    ):
        assert f"{name} = {value}" in text, name
