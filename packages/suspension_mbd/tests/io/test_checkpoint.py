"""Checkpoint consistency tests."""

from pathlib import Path

import pytest

from suspension_mbd.io import CheckpointStore


def test_checkpoint_resume_and_hash_guard(tmp_path: Path) -> None:
    store = CheckpointStore(tmp_path / "checkpoint.json")
    store.add("k-0", model_hash="m", case_hash="c", solver_hash="s")
    loaded = store.load(model_hash="m", case_hash="c", solver_hash="s")
    assert loaded.completed_ids == ("k-0",)
    with pytest.raises(ValueError, match="case_hash"):
        store.load(model_hash="m", case_hash="different", solver_hash="s")
