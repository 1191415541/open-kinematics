# Suspension MBD Equivalence

The suspension MBD context models a symmetric front double-wishbone axle and compares it with independently generated Adams/Car results.

## Language

**Canonical Equivalence Model**:
The complete physical front-axle definition consumed independently by suspension_mbd and Adams/Car.
_Avoid_: demo model, reference copy

**Equivalence Manifest**:
The immutable, hashed record of a Canonical Equivalence Model, boundary conditions, load grid, and required result fields.
_Avoid_: configuration, baseline

**Strict K**:
An ideal-joint, load-free kinematic comparison over a fixed displacement and rack grid.
_Avoid_: geometry smoke test

**Strict C**:
A compliant, loaded comparison over a fixed six-component load grid that evaluates response magnitudes and common component loads.
_Avoid_: symmetry check, compliance smoke test

**Common Entity**:
A body, joint, force element, bushing, metric, or load represented identically by both generated solvers.
_Avoid_: approximately similar component

**C Reference State**:
The unloaded, converged pose at the same prescribed wheel and rack inputs as a strict C state.
_Avoid_: default pose, nominal offset

**Strict C Basis**:
The six wheel-center wrench axes, their unit conventions, application frames, levels, and side mode fixed in an Equivalence Manifest.
_Avoid_: report setting, compliance summary
