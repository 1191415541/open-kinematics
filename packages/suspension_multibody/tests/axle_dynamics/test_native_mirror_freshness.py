"""
The packaged mirror must not be silently loaded when it is stale.

``kernel/native.py`` loads the product package's copy under ``native/``, while the
build that produces the canonical library lives in ``suspension_kernel``.  A build
that refreshed only the canonical copy leaves the mirror behind, and because the
``ctypes`` mirrors move with the source, the symptom is an access violation inside
the kernel rather than a build error.  These tests pin the guard that turns it
back into a build error.

The fixture is fully synthetic: real files in the working tree are never touched,
so a failure here cannot leave the checkout in a doctored state.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from suspension_multibody.kernel import native

#: The guard raises the kernel runtime's error type, which is the one callers of
#: this package already catch: ``axle_dynamics`` re-exports it unchanged.
NativeKernelUnavailableError = native.NativeKernelUnavailableError


@pytest.fixture
def fake_pair(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[Path, Path]:
    """Point both paths at temporary files with *different* content."""
    canonical_dir = tmp_path / "canonical"
    canonical_dir.mkdir()
    mirror = tmp_path / native._library_path().name
    mirror.write_bytes(b"stale stub")
    canonical = canonical_dir / mirror.name
    canonical.write_bytes(b"fresh stub, deliberately a different length")
    monkeypatch.setattr(native, "_library_path", lambda: mirror)
    monkeypatch.setattr(native, "kernel_native_directory", lambda: canonical_dir)
    return mirror, canonical


def test_a_stale_mirror_is_refused_before_it_is_loaded(
    fake_pair: tuple[Path, Path],
) -> None:
    mirror, canonical = fake_pair
    stale = canonical.stat().st_mtime - 60.0
    os.utime(mirror, (stale, stale))
    with pytest.raises(NativeKernelUnavailableError) as captured:
        native._require_fresh_mirror()
    assert "build-axle-native" in str(captured.value)


def test_identical_content_is_not_stale_however_old(
    fake_pair: tuple[Path, Path],
) -> None:
    """
    A rebuild that reproduces the same bytes must not read as staleness.

    This is the common case: the kernel builds reproducibly, so running the
    kernel build alone refreshes the canonical copy's timestamp without changing
    a single byte of the mirror.
    """
    mirror, canonical = fake_pair
    canonical.write_bytes(mirror.read_bytes())
    stale = canonical.stat().st_mtime + 60.0
    os.utime(mirror, (stale - 120.0, stale - 120.0))
    assert canonical.stat().st_mtime > mirror.stat().st_mtime
    native._require_fresh_mirror()


def test_a_fresh_mirror_passes_the_check(fake_pair: tuple[Path, Path]) -> None:
    """The guard must not fire when the mirror is at least as new."""
    mirror, canonical = fake_pair
    newer = canonical.stat().st_mtime + 60.0
    os.utime(mirror, (newer, newer))
    native._require_fresh_mirror()


def test_an_equal_age_is_not_treated_as_stale(
    fake_pair: tuple[Path, Path],
) -> None:
    """
    Both build paths copy with metadata preserved, so equal is the normal state.

    Treating equal as stale would reject every freshly built checkout.
    """
    mirror, canonical = fake_pair
    same = canonical.stat().st_mtime
    os.utime(mirror, (same, same))
    native._require_fresh_mirror()


def test_a_wheel_install_without_a_canonical_copy_is_accepted(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """An installed wheel has no kernel package beside it; that is not stale."""
    mirror = tmp_path / native._library_path().name
    mirror.write_bytes(b"stub")
    monkeypatch.setattr(native, "_library_path", lambda: mirror)
    monkeypatch.setattr(native, "kernel_native_directory", lambda: tmp_path / "absent")
    native._require_fresh_mirror()


def test_the_mirror_and_the_canonical_copy_are_different_files() -> None:
    """
    A guard comparing a path against itself would never fire.

    That is worth asserting because both names come from `_library_path()`.
    """
    assert native._library_path() != native._canonical_kernel_library()


def test_the_mirror_name_matches_the_loaded_stem() -> None:
    """The guard must watch the file the loader actually opens."""
    assert native._canonical_kernel_library().name == native._library_path().name
