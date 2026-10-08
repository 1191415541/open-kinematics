"""Retired source is forbidden even when only interpreter caches remain."""

import runpy
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/check_composable_release.py"


@pytest.mark.parametrize("name", ["core", "model", "metrics", "analysis", "preparation", "subsystems"])
def test_retired_source_is_rejected_and_orphan_cache_is_ignored(tmp_path, name, monkeypatch):
    probe = runpy.run_path(str(SCRIPT))
    check = probe["_package_has_source"]
    monkeypatch.setitem(check.__globals__, "SOURCE_ROOT", tmp_path)
    directory = tmp_path / name
    (directory / "__pycache__").mkdir(parents=True)
    (directory / "__pycache__/old.cpython-312.pyc").write_bytes(b"cache")
    assert not check(name)
    (directory / "revived.py").write_text("def build(): return {}\n", encoding="utf-8")
    assert check(name)
