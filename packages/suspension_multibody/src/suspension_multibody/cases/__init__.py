"""
The authoring side of the contract boundary.

A *case* in this package is a declarative description of what to simulate; the
documents produced here are how it reaches the kernel.  Each family gets its own
file because each family has its own vocabulary -- a load-path sweep, a time
history, a four-post sweep, a random road and a steering manoeuvre have nothing
in common beyond the identity fields every document carries.

Nothing here solves anything.  These modules turn the product's in-memory model
and case objects into the versioned documents of ``suspension_contracts``, and
are the only place that knows the unit convention: a document declares the
length unit it is written in, and the kernel honours that declaration rather
than assuming one.
"""

from .axle_dynamic import case_document as axle_dynamic_case_document
from .axle_dynamic import model_document as axle_dynamic_model_document
from .axle_dynamic import run_axle_dynamic_contract
from .handling import SteeringShape, run_handling_contract
from .handling import case_document as handling_case_document
from .handling import steering_signals as handling_steering_signals
from .kc_quasi_static import (
    AXIS_ORDER as kc_axis_order,
)
from .kc_quasi_static import (
    case_document as kc_case_document,
)
from .kc_quasi_static import (
    model_document as kc_model_document,
)
from .ride_four_post import FourPostCorner, run_ride_four_post_contract
from .ride_four_post import case_document as ride_four_post_case_document
from .ride_four_post import corner_signals as ride_four_post_corner_signals
from .ride_random_road import (
    RandomRoadWheel,
    RoadComponent,
    run_ride_random_road_contract,
)
from .ride_random_road import case_document as ride_random_road_case_document
from .ride_random_road import road_signals as ride_random_road_signals
from .vehicle_dynamic import case_document as vehicle_dynamic_case_document
from .vehicle_dynamic import model_document as vehicle_dynamic_model_document
from .vehicle_dynamic import run_vehicle_dynamics_contract
from .vehicle_kc import VehicleKcCorner, run_vehicle_kc_contract
from .vehicle_kc import case_document as vehicle_kc_case_document
from .vehicle_kc import vehicle_model_document as vehicle_kc_model_document

__all__ = [
    "FourPostCorner",
    "RandomRoadWheel",
    "RoadComponent",
    "SteeringShape",
    "VehicleKcCorner",
    "axle_dynamic_case_document",
    "axle_dynamic_model_document",
    "handling_case_document",
    "handling_steering_signals",
    "kc_axis_order",
    "kc_case_document",
    "kc_model_document",
    "ride_four_post_case_document",
    "ride_four_post_corner_signals",
    "ride_random_road_case_document",
    "ride_random_road_signals",
    "run_handling_contract",
    "run_axle_dynamic_contract",
    "run_ride_four_post_contract",
    "run_ride_random_road_contract",
    "run_vehicle_dynamics_contract",
    "run_vehicle_kc_contract",
    "vehicle_dynamic_case_document",
    "vehicle_dynamic_model_document",
    "vehicle_kc_case_document",
    "vehicle_kc_model_document",
]
