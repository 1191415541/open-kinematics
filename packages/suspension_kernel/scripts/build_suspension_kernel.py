"""
Build the suspension kernel shared library.

Thin wrapper so the build can be invoked by path, matching how the other
packages expose their scripts:

    uv run python packages/suspension_kernel/scripts/build_suspension_kernel.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from suspension_kernel.binding.build import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
