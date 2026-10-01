# p2-01 (d): F1–F5 anchor re-check

Read-only. Every entry is the *live* line, taken from the working tree on
2026-09-29. Where a line moved, the real line number is given and the EPIC's
number is called out as stale.

## F1 — brake/drive torque is a pre-sampled array, not an element

| what | EPIC says | live | verdict |
|---|---|---|---|
| wheel-name tuple | `:71 _WHEEL_NAMES` | `preparation/vehicle_dynamic.py:73 _WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")` | moved by 2 lines |
| builder | `:1496 _build_wheel_torque_signals` | `preparation/vehicle_dynamic.py:1536 def _build_wheel_torque_signals` | moved by 40 lines |
| call site | `:249` | `preparation/vehicle_dynamic.py:279 wheel_torque, brake_torque = _build_wheel_torque_signals(` | moved by 30 lines |
| brake-bias arithmetic | `:1531/1533` | `:1571` (`driveline.front_brake_bias / len(front_braked)`) and `:1573` (`(1.0 - driveline.front_brake_bias) / len(rear_braked)`) | moved by ~40 lines |
| contract table | `cases/vehicle_dynamic.py:579/582` | `:579 role="wheel_torque"`, `:582 role="brake_torque"` | exact |
| route map `:1506` | stale | a use site, not the definition | confirmed stale |

**Substantive claim holds**: the `ResolvedElement` kinds are spring/damper/
bump_stop/anti_roll/tire/bushing (`subsystems/element_build.py:60-71`); torque is
not among them, and the values reach the kernel as per-sample tables.
`_build_wheel_torque_signals` returns `dict[str, tuple[float, ...]]` per wheel
name, consumed as `role="wheel_torque"` / `role="brake_torque"`.

## F2 — the kernel has no rotational torque element

`packages/suspension_kernel/cpp/include/mb_input/types.hpp:65-72`, live:

```cpp
enum ElementKind {
    ELEMENT_SPRING = 0,
    ELEMENT_BUSHING = 1,
    ELEMENT_ANTI_ROLL = 2,
    ELEMENT_TIRE = 3,
    ELEMENT_AERODYNAMIC_DRAG = 4,
    ELEMENT_DAMPER = 5,
    ELEMENT_BUMP_STOP = 6
};
```

Seven kinds, no torque kind. **Hold.** The in-file comment above the last two
records why they are appended: "They are appended so no existing kind moved, and
`ELEMENT_SPRING` keeps its value".

## F3 — ABI versions

| constant | EPIC says | live file | live Python | verdict |
|---|---|---|---|---|
| axle | `version.hpp:27` = 16 | `mb_config/version.hpp:27 inline constexpr int kAxleKernelAbiVersion = 16;` | `kernel/native.py:33 = 16` | exact |
| vehicle | `version.hpp:34` = 31 | `version.hpp:34 inline constexpr int kVehicleKernelAbiVersion = 31;` | `kernel/native.py:34 = 31` | exact |
| core | `version.hpp:37` = 1 | `version.hpp:37 inline constexpr int kCoreKernelAbiVersion = 1;` | `kernel/native.py:35 = 1` | exact |

**Confirmed stale**: the route map and `docs/axle_dynamics_results.md:11` say
"axle 15; vehicle 30"; the live values are **16 / 31 / 1**. So p2-02 would be
16→17, not 15→16.

## F4 — rear steering is refused in two layers

| EPIC says | live | verdict |
|---|---|---|
| `preparation/vehicle_dynamic.py:205-211 _validate_steering_topology` throws | `:205-211` is now `_select_assembly_mode`'s tail; `_validate_steering_topology` is defined at `:216-234`, called at `:252` | **STALE** |
| message `"... rear_axle.rack_fixed_to_chassis must be true"` | live: `native vehicle dynamics actuates the rack of 'front' only; rack_fixed_to_chassis must be true on rear` | rewritten (names the placement, not a model field) |
| `authoring/vehicle.py:112-119` forces the rear rack | `:119 axles["rear"] = axles["rear"].model_copy(update={"rack_fixed_to_chassis": True})` | hold |
| steering is a singleton | `schema/vehicle.py steering: SteeringSystemSpec` | hold |

Both layers reproduced from real objects in `raw/rear_steer_refusal.md`. The
authoring layer is not a refusal at all — it is a **silent overwrite**, which is
why the document route never presents a free rear rack to layer 1.

## F5 — the preparation still deletes the steering actuator

Asked-for anchor: `cases/vehicle_kc.py:128-136`. Live:

```python
    document["elements"] = [
        element
        for element in document["elements"]
        if element["type"] != "steering_actuator"
    ]
```

The predicate is on `:135`. **Hold — the anchor is exact**, and the reason is in
the comment above it: a K/C sweep drives the rack itself, so leaving the model's
prescribed actuator in place would put two rows on one degree of freedom. Owned
by p2-06.

## Summary of stale anchors (must not be reused)

1. route map `preparation/vehicle_dynamic.py:1506` — stale.
2. "axle 15; vehicle 30" (route map + `docs/axle_dynamics_results.md:11`) — stale; live 16/31/1.
3. EPIC F4's `preparation/vehicle_dynamic.py:205-211` — stale; live `_validate_steering_topology` at `:216-234`, called `:252`.
4. EPIC F1's line numbers are all ~2–40 lines early: the anchors are right, the numbers moved (the file grew during stage one).
