# Suspension Kinematics

Suspension Kinematics owns ideal-joint suspension geometry, design-condition
metrics, sweeps, steering geometry, and optimization for daily design work.

## Language

**Kinematic Design Model**:
The geometry, hardpoints, topology, units, and design configuration owned by
Suspension Kinematics.
_Avoid_: MBD model, analysis model

**Kinematic State**:
One solved ideal-joint configuration for a Kinematic Design Model under a
prescribed geometry drive.
_Avoid_: load state, dynamic state

**Geometry Contract**:
The versioned, solver-independent representation exported for another product
to consume without importing Suspension Kinematics internals.
_Avoid_: shared solver model, internal config
