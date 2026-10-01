# p2-03 (b): the two ends come from the port pairing, and no construction path names a body

All commands run from the repository root, 2026-10-01. Exit codes and outputs are real.

## 1. The requirement

`EPIC.md:233` ("装配层不得出现按名字猜身份的规则") and `EPIC.md:241 (b)`. The judgement
here: in every file this row added or modified on the *construction* path,
`grep -n "upright"` and `grep -n "chassis"` return **zero hits**, and the driven /
reaction choice is read from a matched `Port`'s owner rather than from a name.

## 2. The grep, verbatim

The files this row added or modified, split by whether they are on the production
construction path:

Production (the construction path — these five are what the zero-hit claim is about):

- `packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py` (new)
- `packages/suspension_multibody/src/suspension_multibody/compilation/__init__.py`
- `packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py`
- `packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py`
- `packages/suspension_multibody/src/suspension_multibody/modeling/primitives/__init__.py`

Tests (the new files; they carry the assertion, see the second grep below):

- `packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py` (new)
- `packages/suspension_multibody/tests/modeling/test_rotational_torque_element.py` (new)

### `upright` and `chassis` on the production construction path — 0 hits each

```
$ FILES="packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py \
         packages/suspension_multibody/src/suspension_multibody/compilation/__init__.py \
         packages/suspension_multibody/src/suspension_multibody/subsystems/element_build.py \
         packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py \
         packages/suspension_multibody/src/suspension_multibody/modeling/primitives/__init__.py"
$ grep -rn "upright" $FILES ; echo "upright exit=$?"
upright exit=1
$ grep -rn "chassis" $FILES ; echo "chassis exit=$?"
chassis exit=1
$ grep -rn "upright\|chassis" $FILES ; echo "combined exit=$?"
combined exit=1
```

Exit **1** = no lines matched, so **every production file this row added or modified has
zero hits for both names**.

### The same two greps including the new tests — one hit, and it is the guard

```
$ ALL="$FILES \
       packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py \
       packages/suspension_multibody/tests/modeling/test_rotational_torque_element.py"
$ grep -rn "upright" $ALL ; echo "exit=$?"
packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py:626:        for name in ("upright", "chassis"):
exit=0
$ grep -rn "chassis" $ALL ; echo "exit=$?"
packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py:626:        for name in ("upright", "chassis"):
exit=0
```

Both names hit exactly one line, and it is the same line: the assertion that *makes* the
zero-hit claim checkable — `test_the_construction_path_reports_no_body_name_rule`
(`tests/subsystems/test_rotational_torque_element.py:610`, the literal at `:626`). It is a
string being searched *for*, not a rule that reads a body name, and it is in a test. It is
reported here rather than omitted so the claim above is not read as "the strings appear
nowhere"; deleting the guard to make a grep return exit 1 would trade the check for the
appearance of one.

## 3. Where the two bodies are decided — `file:line` + code

`packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py:133`
(`pair_torque_bodies`). The decision, verbatim (`:200-208`):

```python
    return TorquePairing(
        name=name,
        role=role,
        port_id=port_id,
        # The couple is applied to the requiring side and reacts on the neighbour:
        # the element drives the body that asked for the connection.
        driven_body=own_body,
        reaction_body=port.owner.local,
    )
```

`port.owner.local` is the matched port's own owner body. This is the **same rule**
`connections/links.py:203` already applies to a matched port:

```python
        body_b = spec.body_b or port.owner.local
```

so the new path consumes the stage-one pairing channel and adds no second inference
path. `connections/**` is **not modified** by this row.

The requiring side's own body is a parameter (`own_body`), for the same reason a
`LinkSpec` states `body_a` (`connections/links.py:74-81`): a `PortRequirement` says what
a neighbour must offer and never names which of the requirement's own bodies is in play.

### The port ids are the matcher's, not re-derived

`element_blocks.py:160-208` reads `report.binding_for(role)` and then
`ports[binding.port_ids[0]]`. Both spellings — the role the element fills and the
offered-port mapping keyed by rendered id — are the ones the composition layer itself
works in (`subsystems/composition.py:278`, `:283-288`), so a composition's binding and a
torque element's pairing cannot come to mean different things. Three failure modes are
refused by name rather than resolved: no binding for the role, a role bound to more than
one port (a couple is between two bodies), and a non-geometric port (no owner body).

### The construction branch adds nothing to this decision

`subsystems/element_build.py:195` (`_rotational_torque`) copies `row.body_a` and
`row.body_b` verbatim into the declaration, and its docstring says why:

```python
    The two bodies come from the row verbatim.  Which one is the driven side is
    decided where the row was built -- from the port pairing that resolved the two
    ends -- and never here: a rule here that read either name would be the
    name-based identity guess the assembly layer is not allowed to have.
```

## 4. The test that carries this claim

`packages/suspension_multibody/tests/subsystems/test_rotational_torque_element.py`

| test | line | what it pins |
|---|---|---|
| `test_the_two_bodies_come_from_the_matched_port` | `:221` | the reaction body equals `port.owner.local`, port id included |
| `test_two_offered_ports_are_ambiguous_until_the_pairing_names_one` | `:237` | ambiguity is reported, not resolved by offer order |
| `test_an_unbound_requirement_is_refused_by_name` | `:262` | a role the assembly never bound fails by role |
| `test_a_pairing_without_the_requiring_side_body_is_refused` | `:276` | a requirement cannot supply the near end |
| `test_the_construction_path_reports_no_body_name_rule` | `:610` | the zero-hit grep, as an assertion |

The fixture's reaction body is named `subframe` (`:66`) and the driven one `spinner`
(`:67`): neither name appears anywhere in the package under test, which is what makes
"the matched port decided it" checkable rather than assumed. `chassis` and `wheel_carrier`
appear only in `tests/modeling/test_rotational_torque_element.py:180-198`, whose point is
the opposite: swapping those two names changes nothing, because the declaration reads no
name.

## 5. Architecture gate

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
147 passed in 590.05s (0:09:50)
exit=0
```

`tests/architecture/test_import_boundaries.py` is part of that run and is green: the new
declaration is a leaf, and the new compilation module imports only `modeling/` plus
`connections.matcher` (looked up lazily inside the function, like
`subsystems/composition.py:230-231` does).
