"""The fixed 100-point K performance gate."""

from pathlib import Path

import pytest

from suspension_multibody.analysis.benchmarks import run_k_100_benchmark
from suspension_multibody.io import read_table


@pytest.mark.performance
def test_k_100_full_pipeline_meets_gate(tmp_path: Path) -> None:
    report = run_k_100_benchmark(tmp_path)
    assert report.name == "k-100"
    assert report.state_count == 100
    assert report.converged_count == 100
    assert report.convergence_rate == 1.0
    assert report.max_constraint_residual <= 1e-6
    assert report.max_force_residual <= 1e-6
    assert report.elapsed_seconds < 30.0
    assert report.hardware["python"]
    assert len(read_table(tmp_path / "states.parquet")) == 100
    assert (tmp_path / "manifest.json").is_file()
