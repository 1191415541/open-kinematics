"""
Generate an Adams reference for a PAC2002 USE_MODE variant.

Modes 21-25 use the non-linear (advanced) transient contact-mass model, which
requires the ``[DYNAMIC_COEFFICIENTS]`` / ``[CONTACT_COEFFICIENTS]`` families.
The plain mode-14 tire does not carry them and Adams aborts with
``PAC2002INI: error reading parameter CX``, so the clone starts from a tire that
does (the parking tire ships with ``USE_MODE 25`` and the full coefficient set).

Which modes can be referenced with the built-in ``step_steer`` maneuver, as
observed on Adams 2025.1.1:

===========  ==========================================================
USE_MODE     outcome
===========  ==========================================================
21, 22       trim fails; these modes emit a single force axis (Fx/My and
             Fy/Mx/Mz respectively) and the lateral maneuver cannot trim
23, 24, 25   solves, 501 samples
===========  ==========================================================

Referencing 21 or 22 needs a maneuver that does not demand the missing force
axis, for example a straight-line braking case.

Usage::

    uv run --package suspension-multibody python \
        packages/suspension_multibody/scripts/generate_adams_mode_reference.py \
        --mode 24
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import shutil
import tempfile

import numpy as np

from suspension_multibody.adams import discover_profile
from suspension_multibody.adams.vehicle_handling import (
    run_adams_car_handling_case,
)

# Base tire: the only stock PAC2002 tire carrying the contact-mass coefficients.
LIBRARY = pathlib.Path(
    r"G:\MSC.Software\Adams\2025_1_1\acar\shared_car_database.cdb\tires.tbl"
)
BASE_TIRE_NAME = "pac2002_205_55R16_parking.tir"
BASE_USE_MODE = 25
DEFAULT_OUTPUT_ROOT = pathlib.Path("artifacts/adams-mode-ref")

# Modes the built-in handling maneuver can actually trim with.
HANDLING_REFERENCE_MODES = (23, 24, 25)


def base_tire_path(profile, name: str = BASE_TIRE_NAME) -> pathlib.Path:
    """Return the stock tire used as the coefficient source for clones."""
    if profile.database_path is None:
        raise ValueError("Adams profile database path is unavailable")
    return pathlib.Path(profile.database_path) / "tires.tbl" / name


def patch_use_mode(text: str, use_mode: int, source_name: str) -> str:
    """
    Return ``text`` with its single ``USE_MODE`` assignment set to ``use_mode``.

    Matched by pattern rather than by the parking tire's exact spacing, so any
    stock tire can be used as the base -- which matters because the steady-state and
    linear-transient modes are referenced from the tire the mode-14 case already
    uses, and that tire carries none of the contact-mass coefficients.
    """
    pattern = re.compile(r"^(\s*USE_MODE\s*=\s*)[-\d.]+", re.MULTILINE)
    patched, count = pattern.subn(rf"\g<1>{use_mode}", text)
    if count != 1:
        raise ValueError(
            f"expected exactly one USE_MODE assignment in {source_name}, "
            f"found {count}"
        )
    return patched


def clone_with_mode(
    profile, use_mode: int, base_name: str = BASE_TIRE_NAME
) -> pathlib.Path:
    """Return a temporary tire file identical to the base but with ``use_mode``."""
    source = base_tire_path(profile, base_name)
    if not source.is_file():
        raise ValueError(f"coefficient-carrying base tire is unavailable: {source}")
    text = source.read_text(encoding="ascii", errors="replace")
    patched = patch_use_mode(text, use_mode, source.name)
    scratch = pathlib.Path(tempfile.mkdtemp(prefix="pac2002_mode_"))
    target = scratch / f"pac2002_um{use_mode}.tir"
    target.write_text(patched, encoding="ascii")
    return target


# Coefficients the native scope rejects that modes 23/24 provably do not use, and
# the three it rejects that they do.
#
# The stock tires carrying the contact-mass family also carry the turn-slip /
# parking family, so no shipped tire is inside the native scope at USE_MODE 23 or
# 24.  Measured on Adams 2025.1.1, zeroing these subsets of the blockers gives a
# step-steer result *bit-identical* to the stock mode-24 reference:
#
#   * EP, EP12, BF2, BP1, BP2 -- the turn-slip relaxation set;
#   * PDYP1, PHYP1, PHYP2, PKYP1, QBRP1, QCRP1, QCRP2, QDRP1, QDTP1 -- the spin
#     and parking torque terms.
#
# including both together.  They are therefore inert without turn-slip, and this
# is the set ``--native-scope`` zeroes by default.
#
# IC, KP and CP are the opposite: they are *load bearing* even at mode 24.
# Zeroing all three (and the rest) aborts the solve with
# "NaN value detected in AsMath::step()" (Simulate status=-124); zeroing only KP,
# keeping IC and CP, solves but disagrees with the stock reference by 0.96 in
# lateral acceleration and 3.18 in yaw rate (relative).  So Adams' mode-24 model
# really does exercise the contact body's yaw degree of freedom, and no
# native-acceptable tire can reproduce the ``um23``/``um24`` references: the
# correlation gates for those modes need the yaw DOF implemented first, not a
# comparable tire.
VERIFIED_INERT_AT_MODE_24 = (
    "EP", "EP12", "BF2", "BP1", "BP2",
    "PDYP1", "PHYP1", "PHYP2", "PKYP1",
    "QBRP1", "QCRP1", "QCRP2", "QDRP1", "QDTP1",
)

LOAD_BEARING_AT_MODE_24 = ("IC", "KP", "CP")

# The two families, for bisecting.
YAW_AND_RELAXATION = ("IC", "KP", "CP", "EP", "EP12", "BF2", "BP1", "BP2")
SPIN_AND_PARKING = (
    "PDYP1", "PHYP1", "PHYP2", "PKYP1",
    "QBRP1", "QCRP1", "QCRP2", "QDRP1", "QDTP1",
)


def build_native_scope_tire(
    profile,
    use_mode: int,
    zeroed: tuple[str, ...] = VERIFIED_INERT_AT_MODE_24,
    base_name: str = BASE_TIRE_NAME,
) -> pathlib.Path:
    """
    Build the native-scope clone text and stage it at an ASCII-only path.

    ``_assembly_with_tire`` embeds the tire's absolute path into an Adams subsystem
    file written as ASCII, and this repository lives under a non-ASCII directory,
    so the staged copy has to sit in the system temp directory.  ``generate``
    archives an identical copy under the artifact root for the record and for the
    native side, which reads the file through Python and is not path-limited.
    """
    source = base_tire_path(profile, base_name)
    if not source.is_file():
        raise ValueError(f"coefficient-carrying base tire is unavailable: {source}")
    text = source.read_text(encoding="ascii", errors="replace")
    patched = patch_use_mode(text, use_mode, source.name)
    for name in zeroed:
        pattern = re.compile(rf"^(\s*{name}\s*=\s*)[^\s$]+", re.MULTILINE)
        patched, count = pattern.subn(r"\g<1>0", patched)
        if count != 1:
            raise ValueError(
                f"expected exactly one {name} assignment in {source.name}, "
                f"found {count}"
            )
    scratch = pathlib.Path(tempfile.mkdtemp(prefix="pac2002_native_scope_"))
    target = scratch / f"pac2002_um{use_mode}_native_scope.tir"
    target.write_text(patched, encoding="ascii")
    return target


def load_channels(root: pathlib.Path) -> dict[str, np.ndarray]:
    """Read the written Adams time-history channels."""
    payload = json.loads((root / "adams_time_history.json").read_text("utf-8"))
    return {
        name: np.asarray(values, dtype=float)
        for name, values in payload["channels"].items()
    }


def generate(
    use_mode: int,
    output_root: pathlib.Path,
    *,
    case: str = "step_steer",
    native_scope: bool = False,
    zeroed: tuple[str, ...] = VERIFIED_INERT_AT_MODE_24,
    base_tire: str = BASE_TIRE_NAME,
) -> dict[str, object]:
    """
    Run one maneuver with a cloned tire and report the recorded evidence.

    ``native_scope`` additionally zeroes ``VERIFIED_INERT_AT_MODE_24`` so the native
    kernel can load the same tire.  Note that IC/KP/CP stay non-zero and the native
    scope still rejects them: the measurement behind that is recorded above.

    ``base_tire`` selects the stock tire to clone.  The contact-mass modes need the
    parking tire because it is the only stock tire carrying those coefficients; the
    steady-state and linear-transient modes are referenced from the tire the
    mode-14 case already uses, which carries none of them and is inside the native
    scope as it stands.

    Either way the clone is archived under ``<output_root>/clones`` and its path is
    reported, because the native comparison has to be given exactly this file.
    """
    profile = discover_profile()
    if not profile.available:
        raise RuntimeError(f"Adams/Car is unavailable: {profile.message}")

    suffix = "-native" if native_scope else ""
    # The maneuver is part of the identity: a run that failed left its raw files
    # next to a good reference in the same ``um<mode>`` directory, which is one
    # rename away from destroying it.  The default case keeps the historic name so
    # existing references and gates stay valid.
    case_suffix = "" if case == "step_steer" else f"-{case}"
    root = output_root / f"um{use_mode}{case_suffix}{suffix}"
    tire = (
        build_native_scope_tire(profile, use_mode, zeroed, base_tire)
        if native_scope
        else clone_with_mode(profile, use_mode, base_tire)
    )
    archive_dir = output_root / "clones"
    archive_dir.mkdir(parents=True, exist_ok=True)
    archived = archive_dir / (
        f"{pathlib.Path(base_tire).stem}_um{use_mode}{case_suffix}{suffix}.tir"
    )
    shutil.copy2(tire, archived)
    history = run_adams_car_handling_case(
        profile, case, root, tire_property_file=tire
    )
    execution = json.loads((root / "adams_execution.json").read_text("utf-8"))
    return {
        "use_mode": use_mode,
        "native_scope": native_scope,
        "base_tire": base_tire,
        "zeroed": list(zeroed) if native_scope else [],
        "clone": str(tire),
        "archived_clone": str(archived),
        "output_root": str(root),
        "sample_count": len(history.time),
        "channel_count": len(history.channels),
        "producer": execution.get("producer"),
        "returncode": execution.get("returncode"),
        "analysis_mode": execution.get("analysis_mode"),
    }


def main() -> None:
    """Parse arguments, generate the reference, and print the summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", type=int, required=True)
    parser.add_argument("--output-root", type=pathlib.Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--native-scope",
        action="store_true",
        help=(
            "Zero the coefficients the native kernel rejects so both sides can "
            "load the same tire; writes into um<mode>-native."
        ),
    )
    parser.add_argument(
        "--zero-list",
        default=None,
        help=(
            "Bisect helper: 'yaw' (IC/KP/CP/EP/EP12/BF2/BP1/BP2), 'parking' "
            "(spin and parking torque terms), 'none' (keep every coefficient, for a "
            "reference that exercises a family the native kernel is about to grow), "
            "or a comma-separated coefficient list, zeroed instead of the full inert "
            "set."
        ),
    )
    parser.add_argument(
        "--case",
        default="step_steer",
        help=(
            "Adams maneuver. Defaults to the step steer; the pure-longitudinal "
            "modes (and mode 0) cannot be trimmed by a steering maneuver and need "
            "the straight-line 'acceleration' case."
        ),
    )
    parser.add_argument(
        "--base-tire",
        default=BASE_TIRE_NAME,
        help=(
            "Stock tire to clone. Defaults to the parking tire, which is the only "
            "one carrying the contact-mass coefficients; the steady-state and "
            "linear-transient modes use pac2002_235_60R16.tir, the tire the "
            "mode-14 case already uses."
        ),
    )
    parser.add_argument(
        "--compare-to",
        type=pathlib.Path,
        default=None,
        help="Existing reference directory to diff the channels against.",
    )
    args = parser.parse_args()

    zeroed = VERIFIED_INERT_AT_MODE_24
    if args.zero_list:
        key = args.zero_list.strip().lower()
        groups = {"yaw": YAW_AND_RELAXATION, "parking": SPIN_AND_PARKING}
        if key in ("none", "keep"):
            # A reference that keeps every coefficient: USE_MODE 25 needs the
            # turn-slip/parking families, so zeroing them would delete the very
            # feature the reference is supposed to measure.
            zeroed = ()
        else:
            zeroed = (
                groups[key]
                if key in groups
                else tuple(name.strip().upper() for name in args.zero_list.split(","))
            )
    summary = generate(
        args.mode,
        args.output_root,
        case=args.case,
        native_scope=args.native_scope,
        zeroed=zeroed,
        base_tire=args.base_tire,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))

    if args.compare_to is not None and args.compare_to.is_dir():
        reference = load_channels(args.compare_to)
        current = load_channels(pathlib.Path(str(summary["output_root"])))
        print(f"\n--- vs {args.compare_to} ---")
        for name in sorted(set(reference) & set(current)):
            delta = float(np.max(np.abs(current[name] - reference[name])))
            scale = max(float(np.max(np.abs(reference[name]))), 1.0e-30)
            marker = "  <-- DIFFERS" if delta / scale > 1.0e-9 else "  (identical)"
            print(f"  {name:24s} maxabs={delta:.6e} rel={delta / scale:.3e}{marker}")


if __name__ == "__main__":
    main()
