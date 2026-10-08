"""The seven retired constructors and the role scanner cannot be reintroduced."""

from __future__ import annotations

from pathlib import Path

import pytest

from .test_legacy_surface_gate import gate


@pytest.mark.parametrize("name", gate.RETIRED_SUBSYSTEMS)
@pytest.mark.parametrize("source", [
    "import suspension_multibody.subsystems.{name} as old\nold.build()",
    "from suspension_multibody.subsystems import {name} as old\nold.build()",
    "from .{name} import build\nbuild()",
    "from . import {name}\n{name}.build()",
    "import importlib\nimportlib.import_module('suspension_multibody.subsystems.{name}')",
    "from importlib import import_module as load\nload('.{name}', package='suspension_multibody.subsystems')",
    "__import__('suspension_multibody.subsystems.{name}')",
    "from importlib import import_module\nOLD = 'suspension_multibody.subsystems.{name}'\nimport_module(OLD)",
])
def test_retired_import_is_fatal_in_both_modes(tmp_path: Path, name: str, source: str) -> None:
    path = tmp_path / "src" / "suspension_multibody" / "subsystems" / "consumer.py"
    path.parent.mkdir(parents=True)
    path.write_text(source.format(name=name), encoding="utf-8")
    findings = gate.scan_file(path, root=tmp_path)
    assert any(row.rule == "retired_subsystem_import" for row in findings)
    for mode in (gate.MODE_MIGRATION, gate.MODE_FINAL):
        assert gate.evaluate(findings, mode=mode)


@pytest.mark.parametrize("name", gate.RETIRED_SUBSYSTEMS)
def test_retired_file_cannot_return_even_without_imports(tmp_path: Path, name: str) -> None:
    path = tmp_path / "src" / "suspension_multibody" / "subsystems" / f"{name}.py"
    path.parent.mkdir(parents=True)
    path.write_text("def build():\n    return {}\n", encoding="utf-8")
    assert gate.present_retired_subsystems(tmp_path) == [str(path)]
    assert gate.main(["--package-root", str(tmp_path), "--check"]) == 1


def test_comments_strings_and_retained_generic_modules_are_accepted(tmp_path: Path) -> None:
    path = tmp_path / "src" / "suspension_multibody" / "authoring" / "consumer.py"
    path.parent.mkdir(parents=True)
    path.write_text(
        "# from .suspension import build\n"
        "EXAMPLE = 'import suspension_multibody.subsystems.brake'\n"
        "from .generic import assemble_generic\n"
        "from other_package import wheel\n", encoding="utf-8",
    )
    assert gate.scan_file(path, root=tmp_path) == []
    assert gate.present_retired_subsystems(tmp_path) == []


@pytest.mark.parametrize("module", gate.RETIRED_MODULES)
@pytest.mark.parametrize("source", ["import suspension_multibody.{module}",
    "import importlib\nimportlib.import_module('suspension_multibody.{module}')"])
def test_all_retired_runtime_modules_are_rejected(tmp_path, module, source):
    path = tmp_path / "src/suspension_multibody/authoring/consumer.py"
    path.parent.mkdir(parents=True)
    path.write_text(source.format(module=module), encoding="utf-8")
    assert gate.evaluate(gate.scan_file(path, root=tmp_path))


def test_production_and_public_exports_have_no_retired_builder_reference() -> None:
    root = Path(__file__).resolve().parents[2]
    assert gate.present_retired_subsystems(root) == []
    assert not [row for row in gate.scan_tree(root)
                if row.rule == "retired_subsystem_import"]
