"""The fixed 6600-point C performance gate."""

from pathlib import Path

import pytest

from suspension_multibody.analysis.benchmarks import run_c_6600_benchmark
from suspension_multibody.io import read_table


@pytest.mark.performance
def test_c_6600_full_pipeline_meets_gate(tmp_path: Path) -> None:
    report = run_c_6600_benchmark(tmp_path)
    assert report.name == "c-6600"
    assert report.state_count == 6600
    assert report.converged_count == 6600
    assert report.convergence_rate == 1.0
    assert report.max_constraint_residual <= 1e-6
    assert report.max_force_residual <= 1e-6
    assert report.elapsed_seconds < 300.0
    assert len(read_table(tmp_path / "states.parquet")) == 6600
    assert (tmp_path / "manifest.json").is_file()
