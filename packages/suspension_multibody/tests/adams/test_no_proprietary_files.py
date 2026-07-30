"""Guard against accidentally committing Adams proprietary assets."""

from pathlib import Path


def test_package_contains_no_adams_assets() -> None:
    package_root = Path(__file__).parents[2] / "src" / "suspension_multibody"
    forbidden = {".cdb", ".vdb", ".tpl", ".sub", ".asy", ".xml", ".spr", ".dpr", ".bus"}
    files = [
        path
        for path in package_root.rglob("*")
        if path.is_file() and path.suffix.lower() in forbidden
    ]
    assert files == []
