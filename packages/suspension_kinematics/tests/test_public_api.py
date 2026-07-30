import importlib
import sys


def test_no_matplotlib_import_on_core():
    # Ensure matplotlib is not pulled in by importing the core package.
    sys.modules.pop("matplotlib", None)

    importlib.invalidate_caches()
    importlib.import_module("suspension_kinematics")  # noqa: F401
    assert "matplotlib" not in sys.modules
