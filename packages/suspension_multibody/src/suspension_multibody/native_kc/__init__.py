"""
Native quasi-static K/C workflow.

The module converts a product front-axle assembly into a native model and runs
the K/C workflows on the kernel, returning the same canonical state records the
Python solvers produced.  See ``ARCHITECTURE.md`` for the boundary this replaces.
"""

from .contract import UNITS, case_document, model_document
from .convert import (
    MM,
    NativeKcError,
    axle_drives,
    collapse_spherical_pairs,
    design_separation,
    quaternion_to_rotation,
    rotation_to_quaternion,
    to_native_model,
)
from .load_paths import LOAD_AXES, LeftRightMode, LoadAxis, LoadPath
from .workflow import (
    AXIS_ORDER,
    compliance_matrix,
    k_reference_pose,
    neutral_k_metrics,
    origin_wrench,
    run_c_paths,
    run_c_paths_contract,
    run_k_grid,
    run_k_grid_contract,
    wheel_center_world,
)

__all__ = [
    "AXIS_ORDER",
    "LOAD_AXES",
    "LeftRightMode",
    "LoadAxis",
    "LoadPath",
    "UNITS",
    "MM",
    "NativeKcError",
    "axle_drives",
    "collapse_spherical_pairs",
    "case_document",
    "compliance_matrix",
    "model_document",
    "design_separation",
    "k_reference_pose",
    "neutral_k_metrics",
    "origin_wrench",
    "quaternion_to_rotation",
    "rotation_to_quaternion",
    "run_c_paths",
    "run_c_paths_contract",
    "run_k_grid",
    "run_k_grid_contract",
    "to_native_model",
    "wheel_center_world",
]
