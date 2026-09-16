"""
K6 / §8-D16: the two step drivers commit an accepted contact event with one helper.

`run_model` has an adaptive driver and a fixed-step driver.  Both used to carry
their own copy of the same nine-statement commit sequence, and the epic's D16 names
the two places they must stay different: which linearization caches are invalidated,
and whether the next suggested step is derived from the event.

This test keeps the merge merged.  Re-inlining one of the two copies would be an
easy and invisible edit -- the code would still compile and still pass every
physics test, because the sequences agree today; what would be lost is the single
place the invariants are written down.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
def _kernel_source(name: str) -> Path:
    """Return the one kernel source called `name`, wherever the module tree put it."""
    matches = sorted((ROOT / "packages" / "suspension_kernel" / "cpp").rglob(name))
    assert len(matches) == 1, (name, matches)
    return matches[0]


ABI = _kernel_source("kernel_abi.cpp")


def test_the_event_commit_is_defined_once_and_called_from_both_drivers() -> None:
    text = ABI.read_text(encoding="utf-8")
    assert text.count("const auto accept_step_commit = [&](") == 1, (
        "the accepted-event commit is not defined exactly once"
    )
    calls = re.findall(r"^\s*accept_step_commit\($", text, re.MULTILINE)
    assert len(calls) == 2, f"expected two call sites, found {len(calls)}"
    # Each call site follows its own localisation: one per driver.
    assert text.count("if (!localize_contact_event(") == 2


def test_the_two_drivers_still_differ_exactly_where_they_must() -> None:
    """
    The differences D16 names are parameters, not duplicated statements.

    The adaptive driver invalidates both trial caches and derives the next step
    from the event; the fixed-step driver invalidates only its own cache and must
    leave the step size alone, because the fixed-step sequence is recorded in the
    frozen baselines.
    """
    text = ABI.read_text(encoding="utf-8")
    assert "event_step, event_h, &full_step_cache, &half_step_cache,\n" in text, (
        "the adaptive driver no longer invalidates both trial caches"
    )
    assert "event_step, event_h, nullptr, nullptr, &single_step_cache,\n" in text, (
        "the fixed-step driver no longer invalidates its own cache"
    )
    # Exactly one of the two call sites asks for the suggested step to move.
    assert text.count("nullptr, true\n") == 1
    assert text.count("&single_step_cache,\n                    false\n") == 1
    # And the cache invalidation itself lives in the commit, not at the call sites.
    body = text[text.index("const auto accept_step_commit") :]
    body = body[: body.index("\n                };")]
    for cache in ("full_cache", "half_cache", "single_cache"):
        assert f"{cache}->invalidate();" in body, cache
