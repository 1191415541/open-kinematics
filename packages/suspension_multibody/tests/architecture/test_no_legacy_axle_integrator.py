from __future__ import annotations

from pathlib import Path


def test_legacy_axle_integrator_files_are_absent() -> None:
    package = Path(__file__).parents[2] / "src" / "suspension_multibody"
    repository_package = Path(__file__).parents[2]
    forbidden = (
        package / "analysis" / "axle_dynamic.py",
        package / "dynamics" / "integrator.py",
        package / "dynamics" / "constrained.py",
        package / "core" / "fast_ball_system.py",
        package / "model" / "_axle_mass.py",
        package / "c_axle_integrator.py",
        repository_package / "csrc" / "axle_fast.c",
        repository_package / "csrc" / "axle_fast.dll",
    )

    assert not [path for path in forbidden if path.exists()]


def test_public_sources_do_not_import_legacy_integrators() -> None:
    package = Path(__file__).parents[2] / "src" / "suspension_multibody"
    forbidden = (
        "AxleDynamicIntegratorSolver",
        "ConstrainedDynamicIntegrator",
        "DynamicIntegrator",
        "ensure_axle_mass",
        "CAxleIntegrator",
        "FastBallSystem",
    )
    violations: list[str] = []
    for path in package.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        for symbol in forbidden:
            if symbol in text:
                violations.append(f"{path.relative_to(package)}:{symbol}")

    assert not violations
