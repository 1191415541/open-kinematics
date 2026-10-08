"""Wrench observations retain both ends and the moment transport term."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from suspension_multibody.results import ELEMENT_WRENCH_SWITCH, decode_element_wrench
from suspension_multibody.results.element_wrench import _declared_element_names

sys.path.insert(0, str(Path(__file__).parents[1] / "authoring"))
sys.path.insert(0, str(Path(__file__).parents[1] / "physics"))
from test_function_forces import run_function
from test_generic_function_elements import function_subsystem


def test_function_wrench_two_ends_balance_in_world(monkeypatch):
    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "1")
    run = run_function(function_subsystem("wrench"))
    assert _declared_element_names(run.compiled)[11] == ("mechanism.actuation",)
    records = [r for r in decode_element_wrench(run.raw.blocks) if r.type_code == 11]
    assert len(records) == 2 * 11
    bodies = run.raw.blocks["body_state"]
    for a, b in zip(records[::2], records[1::2]):
        np.testing.assert_allclose(np.asarray(a.force) + b.force, 0, atol=1e-12)
        ma = np.asarray(a.moment) + np.cross(bodies[a.sample, a.body, :3], a.force)
        mb = np.asarray(b.moment) + np.cross(bodies[b.sample, b.body, :3], b.force)
        np.testing.assert_allclose(ma + mb, 0, atol=1e-12)
        assert (a.body_a, a.body_b) == (1, 0)
        np.testing.assert_allclose(a.force, [3, 3, 3], atol=1e-12)


def test_wrench_observer_does_not_change_motion(monkeypatch):
    monkeypatch.delenv(ELEMENT_WRENCH_SWITCH, raising=False)
    silent = run_function()
    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "1")
    observed = run_function()
    for key in silent.raw.blocks:
        np.testing.assert_array_equal(
            silent.raw.blocks[key], observed.raw.blocks[key]
        )
