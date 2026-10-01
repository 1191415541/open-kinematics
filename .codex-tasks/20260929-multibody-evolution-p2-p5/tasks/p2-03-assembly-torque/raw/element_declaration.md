# p2-03 (a): element declaration, construction dispatch, compilation into kernel input

All commands run from the repository root (`E:\杂件\open-kinematics`), 2026-10-01.
Exit codes and outputs below are the real ones.

## 1. The declaration in `modeling/primitives/`

File: `packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py`

| `file:line` | symbol |
|---|---|
| `modeling/primitives/elements.py:56` | `_RATE_EPSILON` (the rate floor the native law uses, `kEps`) |
| `modeling/primitives/elements.py:633` | `class RotationalTorqueParameters` |
| `modeling/primitives/elements.py:707` | `class RotationalTorqueElement` |

Method / property anchors inside `RotationalTorqueParameters`:
`magnitude` (`:684`), `couple` (`:688`).
Inside `RotationalTorqueElement`: `couple` (`:733`), `axis_world` (`:737`), `evaluate` (`:741`).

Both symbols are re-exported by `modeling/primitives/__init__.py` (import at `:20-21`,
`__all__` at `:64-65`).

### Layering

The declaration imports only `numpy` and its two neighbours in the same package
(`.joints`, `.spatial`). It names no authoring layer. Measured:

```
$ grep -nE "^(from|import) " packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py
20:from __future__ import annotations
21:from dataclasses import dataclass, field
22:from typing import Literal
24:import numpy as np
26:from .joints import RigidBodyState
27:from .spatial import (
```

`tests/architecture/test_import_boundaries.py` is green (exit 0; see `raw/run_log.md`).

### What the declaration states, and why

- `stiffness` — the demand-to-magnitude gain. The native law's `demand` is hardcoded
  to `1.0` (`cpp/src/element/anti_roll.cpp:132`, whose own comment marks the demand
  channel as not wired yet), so `stiffness` *is* the amplitude this step asks for.
- `max_torque` — the cap: `magnitude = min(stiffness, max_torque)`, matching
  `std::min(actuator.stiffness * demand, actuator.max_torque)` at `anti_roll.cpp:133-134`.
- `damping` — carried for the ABI slot; the sign law ignores it, so the declaration
  says so rather than pretending to apply it.
- `axis_a` — the couple's axis in `body_a`'s local frame, stored normalized.
- `reference_quaternion` — kept for the ABI round trip; the law never reads it.

The refusals mirror the native reader's (`cpp/src/assembly/element_reader.cpp:354-390`):
a negative or non-finite gain, cap or damping; an axis with no direction.

## 2. The construction branch in `subsystems/element_build.py`

Dispatch chain, `subsystems/element_build.py:53-76`. The new branch, verbatim:

```python
    if row.kind == "rotational_torque":
        return _rotational_torque(row)
```

- `subsystems/element_build.py:74-75` — the branch itself (appended immediately after the
  `bushing` branch at `:72-73` and before `raise ValueError(f"unsupported element kind {row.kind!r}")` at `:76`).
- `subsystems/element_build.py:195` — `def _rotational_torque(row: ResolvedElement) -> RotationalTorqueElement`.
- `subsystems/element_build.py:29-30` — the two imported names.

`EPIC.md:241 (a)` gives the anchor as `:66-71`; on this tree the chain runs `:62-75`,
so the new branch sits at `:74-75` inside the same segment.

### The tire segment is untouched

`git diff` of the file, whole:

```
$ git diff -- packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py
@@ -27,6 +27,8 @@ from ..modeling.primitives import (
     BumpStopElement,
     BushingElement,
     LinearSpringElement,
+    RotationalTorqueElement,
+    RotationalTorqueParameters,
     StaticDamperElement,
     VerticalTireElement,
 )
@@ -69,6 +71,8 @@ def build_element(row: ResolvedElement) -> object:
         return _tire(row, cast(VerticalTire, row.spec))
     if row.kind == "bushing":
         return _bushing(row)
+    if row.kind == "rotational_torque":
+        return _rotational_torque(row)
     raise ValueError(f"unsupported element kind {row.kind!r}")
@@ -188,6 +192,23 @@ def _bushing(row: ResolvedElement) -> BushingElement:
     )
 
 
+def _rotational_torque(row: ResolvedElement) -> RotationalTorqueElement:
...
```

Three hunks, all additive. `_tire` (`:140-148`) and the `tire` branch (`:70-71`) are
context lines, not changes.

## 3. Compilation into kernel input

**Landing: a new module, `packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py`**
(re-exported from `compilation/__init__.py:37-47` / `:55-63`).

| `file:line` | symbol |
|---|---|
| `compilation/element_blocks.py:65` | `ELEMENT_ROTATIONAL_TORQUE = 7` |
| `compilation/element_blocks.py:71-81` | the parameter slot indices 128/129/130/133/137 |
| `compilation/element_blocks.py:84-88` | `ELEMENT_BLOCK_SIZE = 216`, `ELEMENT_BLOCK_INT_SIZE = 16`, `ELEMENT_CURVE_SLOTS = 8` |
| `compilation/element_blocks.py:93` | `class TorquePairing` |
| `compilation/element_blocks.py:115` | `class ElementBlockRow` |
| `compilation/element_blocks.py:133` | `def pair_torque_bodies(...)` |
| `compilation/element_blocks.py:211` | `def torque_element_row(...)` |
| `compilation/element_blocks.py:231` | `def rotational_torque_block(...)` |

### Why this is the minimal set: two measured facts

**(i) The JSON-document route cannot carry this family.** The three candidate landing
points the SPEC listed — `cases/kc_quasi_static/contract.py:436-446`,
`cases/axle_dynamic.py:188`, `studies/bridge.py:119-126` — all emit *contract document*
entries, and the kernel's document reader refuses the type. Measured, against the
shipped library, using this repository's own assembly and its own model document:

```
$ uv run --no-sync python - <<'PY'   (abridged; the entry is appended to a real model document)
doc = model_document(compose_axle(<probe model>, "K"), name="probe")
case = case_document(assembly, family="kc_quasi_static", name="probe")
for label, entry in (("control (an existing mount)", {..."type": "bushing"...}),
                     ("rotational_torque", {..."type": "rotational_torque"...})):
    run_contract({**doc, "elements": [...doc["elements"], entry]}, case)
PY
bodies: ['upper_arm_L', 'lower_arm_L', 'upright_L', 'tie_rod_L', 'wheel_hub_L', 'upper_arm_R', 'lower_arm_R', 'upright_R', 'tie_rod_R', 'wheel_hub_R', 'ground', 'rack', 'rack_housing']
control (an existing mount): REFUSED -> model document: bushing "probe" has no stiffness or damping
rotational_torque: REFUSED -> model document: element "probe" has unsupported type "rotational_torque"
```

The control entry is refused for **its own** reason — a missing parameter — while the
torque entry is refused **by type name**. So the refusal below is specific to this
family and not a general "the document was malformed" failure. The reader's
fall-through is `cpp/src/cases/contract_model.cpp:830`
(`if (*type_name != "bushing") { ... return fail(...) }`), and the name list it checks
against is `cpp/src/contract/contract_registry.cpp:36-40` (`kElements`), which does not
contain `rotational_torque`. Extending either is kernel territory and belongs to
**p2-02**; this row only consumes what p2-02 landed.

**(ii) The generic element-block surface does carry it.** `read_element_blocks`
(`cpp/src/assembly/element_reader.cpp:354-390`) has the family's branch, and the reader's
one layout table (`kElementLayouts`, `cpp/include/mb_input/types.hpp:343-345`) selects it
by `kind`. Hence `rotational_torque_block` produces exactly what that reader consumes:
`kind = 7`, the two body indices, and the five parameter runs at 128/129/130/133/137 inside
the 216-double block.

Note on scope: `ElementBlockRow` is a tuple of plain numbers, not a ctypes struct.
`ElementBlock` belongs to the kernel's ABI and this repository's convention is that the
generic core structures are mirrored by whoever calls them rather than re-declared by the
product (`tests/architecture/test_core_abi.py:10-13` states that on purpose). What this
module owns is the *content*: which kind, which bodies, which slots.

### The measured fragment

Measured through the shipped library and the encoded block (see `raw/min_assembly_torque.md`
for the full reading):

```
block kind/body_a/body_b: 7 0 1
block stiffness idx128: 1.0 max idx137: 1.0 axis 130..132: (0.0, 1.0, 0.0)
```

and the run consumes it — `mb_core_run` returns status 0 and the couple acts
(`raw/min_assembly_torque.md`). A block whose `kind` the kernel did not know would be
refused with `unknown element kind` (`element_reader.cpp:73-76`), which is what
`tests/architecture/test_core_abi.py:721-746` pins for kind 9999.

## 4. What this row did *not* touch

- `packages/suspension_kernel/**` — untouched (measured: `git status --short` shows only
  p2-02's own kernel changes, none of them introduced here).
- `mb_config/version.hpp` and `kernel/native.py` version constants — untouched.
- `templates/roles.py`, `templates/builtin.py`, `subsystems/brake.py`, `subsystems/drive.py`,
  `preparation/vehicle_dynamic.py`, `cases/vehicle_dynamic.py`, `cases/vehicle_kc.py` — untouched.
- `assembly.schema.json` — untouched.
- `subsystems/types.py::ELEMENT_KINDS` (`:102-109`) — **unchanged, and that is a gap**: it
  does not list `rotational_torque`. Nothing validates `row.kind` against it today
  (measured: no reader of `ELEMENT_KINDS` exists outside `subsystems/__init__.py`'s
  re-export), so the new branch works regardless; but a reader that consults the tuple
  would not see the family. `subsystems/types.py` is outside this row's write scope.
  **Carry forward to p2-04.**
