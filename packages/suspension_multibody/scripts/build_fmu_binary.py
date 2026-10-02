"""
Build the FMI 2.0 Co-Simulation wrapper binary for the FMU export.

The wrapper is plain C and links nothing but the platform's own loader, so the
build is one compiler invocation.  What it needs is a compiler that can produce
a binary of the *host's* architecture: the kernel it loads at run time is x64,
and a 32-bit wrapper loading an x64 kernel would fail in a way that looks like a
broken FMU.  The script therefore refuses a compiler whose target triple says
anything other than 64-bit, and names the ones it looked at.

The product is ignored by git, exactly like the kernel mirror it loads: it is a
build artifact of the source beside it, not something to check in.
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
FMI_DIR = PACKAGE_ROOT / "src" / "suspension_multibody" / "fmi"
SOURCE = FMI_DIR / "fmu_wrapper.c"

#: The binary the FMU carries is named after the co-simulation model identifier,
#: so the name is imported from the exporter rather than restated: FMI derives
#: the file it loads from the ``modelIdentifier`` in ``modelDescription.xml``,
#: and the two drifting apart is an archive no importing tool can instantiate.
sys.path.insert(0, str(PACKAGE_ROOT / "src"))
from suspension_multibody.fmi.export import fmi_binary_name  # noqa: E402

#: Compilers to try, in order.  The first entry is the x86_64 MinGW toolchain
#: this repository's own kernel build uses on the development host; the rest are
#: the ordinary names a machine with a compiler on PATH will answer to.
CANDIDATE_NAMES = ("gcc", "cc", "clang")
#: Known x86_64 MinGW locations.  Windows has no `gcc` on PATH by default, so a
#: machine that builds this project usually has one installed explicitly.
KNOWN_X64_GCC = (
    r"C:\Users\zzy11\AppData\Local\Microsoft\WinGet\Packages"
    r"\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe"
    r"\mingw64\bin\gcc.exe",
)


def _library_name() -> str:
    """
    Return the binary's file name, which FMI ties to the model identifier.

    FMI 2.0 requires the binary's *stem* to be the ``modelIdentifier`` from
    ``modelDescription.xml`` -- an importing tool derives the file it loads from
    that string, so a container named after the source file would be an FMU
    nobody could instantiate.  The exporter owns the name; this reads it rather
    than restating it, because the archive the exporter writes has to carry the
    file this build produced.
    """
    return fmi_binary_name()


def _target_triple(compiler: str) -> str:
    """Return the compiler's target triple, or an empty string if it has none."""
    try:
        completed = subprocess.run(
            [compiler, "-dumpmachine"],
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    if completed.returncode != 0:
        return ""
    return completed.stdout.strip()


def _is_64_bit(triple: str) -> bool:
    """Return whether a target triple names a 64-bit architecture."""
    if not triple:
        return False
    return any(token in triple for token in ("x86_64", "amd64", "aarch64"))


def find_compiler() -> str:
    """
    Return a 64-bit C compiler, refusing by name when there is none.

    The refusal names every candidate it tried and every triple it read, because
    the failure mode this guards against -- a 32-bit compiler producing a
    wrapper that cannot load the 64-bit kernel -- surfaces much later and as a
    different problem.
    """
    checked: list[str] = []
    for name in CANDIDATE_NAMES:
        found = shutil.which(name)
        if found is None:
            continue
        triple = _target_triple(found)
        checked.append(f"{found} ({triple or 'no triple'})")
        if _is_64_bit(triple):
            return found
    for known in KNOWN_X64_GCC:
        path = Path(known)
        if not path.is_file():
            continue
        triple = _target_triple(str(path))
        checked.append(f"{path} ({triple or 'no triple'})")
        if _is_64_bit(triple):
            return str(path)
    raise SystemExit(
        "no 64-bit C compiler was found; the FMU wrapper must match the kernel's "
        f"architecture ({platform.machine()}).  Tried: "
        + ("; ".join(checked) if checked else "nothing on PATH")
        + ".  Install one and re-run, or point PATH at an x86_64 toolchain."
    )


def _staged_build_directory() -> Path | None:
    """
    Return a temporary directory whose path is pure ASCII, or ``None``.

    The staging directory exists because MinGW's ``ld.exe`` opens its output
    with a narrow-character API and decodes that path through the *console code
    page*.  A repository checked out under a non-ASCII path -- this one is --
    therefore fails to link whenever the code page cannot represent those
    characters (measured: ``chcp 936`` links, while ``chcp 1252`` and
    ``chcp 65001`` both report ``cannot open output file ...: No such file or
    directory`` for a directory that exists and is writable).  ``ld`` is the
    only tool in the chain with that limitation; the compiler driver itself is
    unaffected.

    The code page is console-wide state that an earlier process can leave
    behind -- the Adams probe tests set it to 1252 -- so this build cannot
    assume the developer's console is still where it started.

    ``tempfile`` promises nothing about its own location being ASCII (a user
    name is part of the path), so a candidate is *observed* rather than
    trusted, and ``None`` means "no ASCII staging directory is available": the
    caller then links in place, which is what any machine with an ASCII
    checkout does anyway.
    """
    candidates: list[Path] = []
    for variable in ("TEMP", "TMP"):
        configured = os.environ.get(variable)
        if configured:
            candidates.append(Path(configured))
    candidates.append(Path(tempfile.gettempdir()))
    for candidate in candidates:
        try:
            if not candidate.is_dir():
                continue
            staged = Path(tempfile.mkdtemp(prefix="fmu_build_", dir=str(candidate)))
        except OSError:
            continue
        if str(staged).isascii():
            return staged
        shutil.rmtree(staged, ignore_errors=True)
    return None


def _compile(compiler: str, destination: Path) -> subprocess.CompletedProcess[str]:
    """Compile the wrapper to one destination and return the completed process."""
    command = [
        compiler,
        "-shared",
        "-O2",
        "-fvisibility=hidden",
        "-o",
        str(destination),
        str(SOURCE),
    ]
    if sys.platform == "win32":
        # The wrapper resolves the kernel with LoadLibrary at run time, so it
        # must not be linked against an import library for it.
        command.append("-Wl,--enable-auto-image-base")
    return subprocess.run(command, capture_output=True, text=True)


def build(*, output: Path | None = None, keep_source: bool = True) -> Path:
    """
    Compile the wrapper and return the produced library's path.

    ``output`` keeps its meaning exactly: the returned path is where the library
    ends up.  The staging detour changes only where the linker is asked to write
    it first, and only when ``output`` cannot be named in ASCII.
    """
    if not SOURCE.is_file():
        raise SystemExit(f"the wrapper source is missing at {SOURCE}")
    compiler = find_compiler()
    destination = output or (FMI_DIR / _library_name())
    destination.parent.mkdir(parents=True, exist_ok=True)

    # A *falsy* sentinel, never `Path()`: `Path()` is `.`, and a truth test on it
    # is always true, which would hand `.` to the cleanup below.
    staged = _staged_build_directory() if not str(destination).isascii() else None
    try:
        linked = (staged / destination.name) if staged is not None else destination
        completed = _compile(compiler, linked)
        if completed.stderr.strip():
            sys.stderr.write(completed.stderr)
        if completed.returncode != 0:
            raise SystemExit(
                f"the wrapper failed to build with {compiler} (exit "
                f"{completed.returncode})"
            )
        if not linked.is_file():
            raise SystemExit(f"{compiler} reported success but wrote no {linked}")
        if staged is not None:
            # Copy rather than rename: the staging directory can sit on another
            # volume, where a move is a copy followed by a delete anyway.
            shutil.copy2(linked, destination)
    finally:
        if staged is not None:
            shutil.rmtree(staged, ignore_errors=True)
    if not destination.is_file():
        raise SystemExit(f"{compiler} reported success but wrote no {destination}")
    if not keep_source:
        raise SystemExit("--no-keep-source is not supported: the source is tracked")
    return destination


def main(argv: list[str] | None = None) -> int:
    """Build the wrapper binary, or list the compilers this machine offers."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="where to write the library (default: the package's fmi/ directory)",
    )
    parser.add_argument(
        "--list-compilers",
        action="store_true",
        help="print the compilers this script found and exit",
    )
    args = parser.parse_args(argv)
    if args.list_compilers:
        for name in CANDIDATE_NAMES:
            found = shutil.which(name)
            print(f"{name}: {found or '(not on PATH)'}")
        for known in KNOWN_X64_GCC:
            print(f"{known}: {'present' if Path(known).is_file() else '(absent)'}")
        return 0
    produced = build(output=args.output)
    print(produced)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
