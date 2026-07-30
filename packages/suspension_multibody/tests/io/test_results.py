"""Result protocol tests."""

from pathlib import Path

import pyarrow.parquet as pq

from suspension_multibody.io import META_KEY, read_table, write_bundle
from suspension_multibody.schema import Manifest, Provenance, ResultBundle, StateResult


def _bundle() -> ResultBundle:
    return ResultBundle(
        manifest=Manifest(
            run_id="run-1",
            mode="K",
            state_count=1,
            provenance=Provenance(package_version="0.1.0"),
        ),
        states=(
            StateResult(
                state_id="k-0", mode="K", metrics={"camber_deg": 1.0}, converged=True
            ),
        ),
    )


def test_write_and_read_json_parquet_csv(tmp_path: Path) -> None:
    write_bundle(_bundle(), tmp_path)
    assert (tmp_path / "manifest.json").is_file()
    assert read_table(tmp_path / "states.parquet")[0]["state_id"] == "k-0"
    assert read_table(tmp_path / "states.csv")[0]["state_id"] == "k-0"
    assert META_KEY.encode() in pq.read_schema(tmp_path / "states.parquet").metadata
