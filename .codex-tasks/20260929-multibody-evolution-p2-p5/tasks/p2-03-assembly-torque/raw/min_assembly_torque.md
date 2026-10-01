# p2-03 (c): one minimal assembly with one torque element, run, and its contribution read

All commands run from the repository root, 2026-10-01. Every number below is a real
reading from `mb_core_run` through the shipped library. Nothing here is derived on paper.

## 1. Where the assembly is, and how it is run

Test file: `packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py`

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py -q -p no:cacheprovider
12 passed in 1.25s
exit=0
```

The two measured runs:

| test | line |
|---|---|
| `test_one_torque_element_changes_the_pairs_spin` | `:534` |
| `test_a_still_pair_is_left_alone` | `:591` |

Both call `_run` (`:402`), which builds an `MbCoreInput`, fills the element block from
`compilation.element_blocks.rotational_torque_block`, and calls `mb_core_run` through
`load_kernel_library`.

### The model

The smallest model this element can act in — **no tire, no road, no suspension surface**:

- two bodies, both free (`fixed = np.array([0, 0])`), mass 1 kg, inertia `diag(0.01)` each;
- no joints at all (`joint_count = 0`);
- one element: the rotational torque, `kind = 7`, `body_a = 0` (the reaction end),
  `body_b = 1` (the driven end);
- body 1 starts spinning about +y at 3 rad/s, body 0 at rest, so the pair's relative rate
  about the element's own axis (which is +y in body 0's frame) starts at 3 rad/s;
- 21 samples over 0.2 s, `rho_inf = 1.0`, `initialization_mode = 1`.

Because the two bodies share no joint, no constraint contributes to the reading: the only
thing the run can show is the element.

## 2. The quantity read, its value, and the causal control

**Quantity 1 — each body's angular velocity**, from `body_state` (`kStatePerBody = 19` per
body; omega at offset 10 of each body block).

```
                                       driven (body 1, omega_y)   reaction (body 0, omega_y)
with the element,  sample 0                     3.0                        0.0
with the element,  sample 20                    1.499999999999999          1.5000000000000002
without the element (control), sample 0         3.0                        0.0
without the element (control), sample 20        3.0                        0.0
```

Raw reading, straight from the library:

```
WITH   driven omega y first/last: 3.0 1.499999999999999
WITH   reaction omega y first/last: 0.0 1.5000000000000002
WITHOUT driven omega y first/last: 3.0 3.0
```

The couple's axis is +y and both bodies start unrotated, so `omega_x` and `omega_z` stay
exactly zero for both bodies at every sample (asserted at `:573-576`).

**Quantity 2 — the drive-work column of the energy ledger** (`energy_output`, column 6,
joules), which is a channel the state does not feed:

```
WITHOUT drive-work col6: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
WITH   drive-work col6: [0.0, -0.019999999999999993, -0.0024999999999999957, 0.0, ...0.0]
WITH   total col2 first/last: 0.045 0.022499999999999985
WITHOUT total col2 first/last: 0.045 0.045
```

### The causal evidence

Three independent statements, all of which a kernel that never read the element would fail:

1. **Removing the element changes the reading.** Same model, same initial state, same
   solver settings; the only difference is whether the block is in the array.
   `omega_driven(last) = 1.499999999999999` with the element, `3.0` without. The control is
   a free spinner, so the difference is this element's contribution and nothing else.
2. **The ledger accounts for it exactly.** The work the drive booked,
   `sum(drive_work) = -0.019999999999999993 + -0.0024999999999999957 = -0.0225 J`, equals
   minus the kinetic energy the pair lost, `0.045 - 0.0225 = 0.0225 J`. Asserted as
   `sum(...) == pytest.approx(-lost)` (`:588`). The bookkeeping is independent of the state
   channel, so it is not the same number read twice.
3. **The sign is physical.** The interval drive-work figures are negative because the couple
   *opposed* the spin: an actuator taking energy out of a pair that is turning, which is what
   the demand sign law is for. A wrong sign would appear as positive work and as the pair
   accelerating apart rather than meeting.

### The shape of the response is the couple's signature

A pure couple applies `+tau` to one end and `-tau` to the other. With equal inertias the
two ends therefore meet in the middle:

```
driven:   3.0 -> 1.5 rad/s
reaction: 0.0 -> 1.5 rad/s
```

Asserted both as "equal to each other" (`:568`) and as "equal to 1.5" (`:566`). A one-sided
torque would leave the reaction end at 0; a force-element mistake would show up as nonzero
`omega_x`/`omega_z`; an unconsumed block would leave both at their starting values.

### The still-pair branch, and why it is not a smoke test

With the same element but `spin = 0.0` (`test_a_still_pair_is_left_alone`, `:591`):

```
still: with-drive-work all zero: True | states equal: True
```

The state history equals the control's exactly and the ledger is all zeros. An element that
produced a full-strength couple of arbitrary sign at zero relative rate would show up in both
channels, so the third branch of the sign law is measured here rather than asserted on the
Python law alone.

## 3. What was measured, not assumed: two rejected designs

Both recorded because the readings above only mean what they say given them.

**(i) A ground-held reaction body does not work, and that is informative.** With
`fixed = [1, 0]` (the reaction end held to ground) the same block produces a *different*
reading — the driven body is brought to a stop and the reaction body stays at zero:

```
omega_b per sample: [3.0, 2.0, 1.0, 2.08e-16, 2.08e-16, ...]
drive work col6:    [0.0, -0.025, -0.015, -0.005, 0.0, ...]
total col2 last:    2.17e-34
```

The angular momentum goes into the ground, which is why the two ends cannot meet. This is the
same model with one flag changed, so it is also a control on the reading itself: the numbers
track the physics, not the fixture.

**(ii) A too-stiff element makes the integrator refuse, not converge quietly.** With
`stiffness = 400, max_torque = 1000` J on the same inertia, `mb_core_run` returns

```
status 5, "time integration failed at t=0.000000 s: Newton solve did not converge"
```

So the gains the test uses (`stiffness = 1.0, max_torque = 1.0` on `0.01 kg m^2`) are the
ones that make the run observable rather than the ones that happen to pass. The fixture
asserts `status == 0` with the kernel's own message in the failure text (`:523`), so a run
that failed would name its reason instead of looking like a wrong number.
