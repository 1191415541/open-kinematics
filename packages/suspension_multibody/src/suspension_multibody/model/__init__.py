"""Vehicle model assembly API."""

from .front_axle import (
    Connection,
    FrontAxleAssembly,
    build_front_axle,
    mirror_hardpoints,
    side_hardpoints,
)

__all__ = [
    "Connection",
    "FrontAxleAssembly",
    "build_front_axle",
    "mirror_hardpoints",
    "side_hardpoints",
]
