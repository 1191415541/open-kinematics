"""Distribution and wheel boundary tests."""

from pathlib import Path

ROOT = Path(__file__).parents[4]
MEMBER = ROOT / "packages" / "suspension_multibody"


def test_independent_project_files_exist() -> None:
    assert (MEMBER / "pyproject.toml").is_file()
    assert (MEMBER / "src" / "suspension_multibody" / "__init__.py").is_file()


def test_workspace_members_and_wheel_package_paths_are_separate() -> None:
    root_pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    member_pyproject = (MEMBER / "pyproject.toml").read_text(encoding="utf-8")
    assert '"packages/suspension_kinematics"' in root_pyproject
    assert '"packages/suspension_multibody"' in root_pyproject
    assert 'packages = ["src/suspension_multibody"]' in member_pyproject
    assert (
        "suspension_kinematics"
        not in member_pyproject.split("[project]")[1].split("[dependency-groups]")[0]
    )
