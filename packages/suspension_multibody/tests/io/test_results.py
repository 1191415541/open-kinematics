"""Result protocol tests for the shared hashing and table readers."""

from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

from suspension_multibody.io import META_KEY, read_table
from suspension_multibody.io.results import canonical_hash


def test_write_and_read_json_parquet_csv(tmp_path: Path) -> None:
    table = pa.Table.from_pylist([{"state_id": "k-0", "converged": True}])
    table = table.replace_schema_metadata({META_KEY.encode(): b'{"schema_version":1}'})
    pq.write_table(table, tmp_path / "states.parquet")
    (tmp_path / "states.csv").write_text("state_id,converged\nk-0,True\n", encoding="utf-8")
    assert read_table(tmp_path / "states.parquet")[0]["state_id"] == "k-0"
    assert read_table(tmp_path / "states.csv")[0]["state_id"] == "k-0"
    assert META_KEY.encode() in pq.read_schema(tmp_path / "states.parquet").metadata


def test_canonical_hash_ignores_key_order_and_tracks_content() -> None:
    assert canonical_hash({"b": 1, "a": (1, 2)}) == canonical_hash(
        {"a": [1, 2], "b": 1}
    )
    assert canonical_hash({"a": 1}) != canonical_hash({"a": 2})
