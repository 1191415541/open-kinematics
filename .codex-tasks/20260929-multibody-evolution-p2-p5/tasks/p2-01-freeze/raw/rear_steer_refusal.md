# p2-01 (b): rear-steer refusal, both layers

Read-only probe. Both refusals are reproduced from real objects; the repository's canonical vehicle fixture is the positive control.

## Positive control (must NOT refuse)

The canonical fixture bolts the rear rack (`tests/vehicle/test_native_vehicle.py:103 update={"rack_fixed_to_chassis": True}`), so `prepare_vehicle_run` accepts it. Verified in `raw/torque_path_snapshot.json`, which prepares the same fixture successfully.

## Layer 1: preparation (`_validate_steering_topology`)

- **EPIC anchor `preparation/vehicle_dynamic.py:205-211` is STALE**: those lines are now `_select_assembly_mode`. The live anchor is the `_validate_steering_topology` function, called from `prepare_vehicle_run`.
- call site: `preparation/vehicle_dynamic.py:252 _validate_steering_topology(facts)`
- exception: `ValueError`
- message: `native vehicle dynamics actuates the rack of 'front' only; rack_fixed_to_chassis must be true on rear`
- raised at: `File "E:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\preparation\vehicle_dynamic.py", line 233, in _validate_steering_topology`

## Layer 2: authoring (`authoring/vehicle.py`)

- exception: `silent overwrite (no exception)`
- message: `vehicle_model_from forced rear_axle.rack_fixed_to_chassis=True`
- raised at: `authoring/vehicle.py:119 axles["rear"] = ... update={"rack_fixed_to_chassis": True}`

## Which fires first

Layer 2 (authoring) fires first in the document route, because a vehicle model is *built* before it is prepared: `authoring/vehicle.py:119` bolts the rear rack unconditionally, so the document route never presents a free rear rack to layer 1. A caller that builds the `VehicleModel` in Python (no document) reaches layer 1 directly, which is the path reproduced above.

## Consequence for p2-06

Both layers must be released: `authoring/vehicle.py:112-119` (the write) and `_validate_steering_topology` (the check). Releasing only the check would still produce a bolted rear rack from any document, and releasing only the write would leave the check refusing.
