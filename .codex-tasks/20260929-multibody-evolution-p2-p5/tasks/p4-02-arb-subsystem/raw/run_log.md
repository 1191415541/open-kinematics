# p4-02 实跑日志

命令均在仓库根目录执行（Windows / Git Bash），每条后附**实测**退出码。

## 1. 角色表与模板

```
$ uv run --no-sync python -c "from suspension_multibody.templates.roles import ROLES, role_names; print(role_names()); print(ROLES['anti_roll_bar'].required_mounts)"
roles: ('anti_roll_bar', 'brake', 'chassis', 'drive', 'steering', 'suspension', 'wheel')
arb mounts: ('chassis_mount_L', 'chassis_mount_R', 'droplink_mount_L', 'droplink_mount_R')
exit=0        # 无 RoleSpecError：import 期硬断言通过

$ uv run --no-sync python -c "from suspension_multibody.templates.builtin import ANTI_ROLL_BAR, ANTI_ROLL_BAR_NAME, BUILTINS; from suspension_multibody.templates import names; ANTI_ROLL_BAR.check_role_contract(); print(ANTI_ROLL_BAR.name, ANTI_ROLL_BAR.role); print(ANTI_ROLL_BAR_NAME in names()); print([t.name for t in BUILTINS])"
anti_roll_bar_simplified anti_roll_bar
True
['double_wishbone', 'steering', 'vehicle_body', 'wheel_on_hub', 'brake_4wdisk_simplified', 'powertrain_simplified', 'anti_roll_bar_simplified']
exit=0
```

## 2. F12 同步点的实测命中数

```
$ for f in authoring/documents.py subsystems/types.py subsystems/capabilities.py subsystems/composition.py connections/policy.py; do
    printf "%-32s %s
" "$f" "$(grep -c anti_roll_bar packages/suspension_multibody/src/suspension_multibody/$f)"; done
authoring/documents.py           2
subsystems/types.py              2
subsystems/capabilities.py       1
subsystems/composition.py        2
connections/policy.py            1

$ for s in template subsystem assembly; do
    printf "%-32s %s
" "$s.schema.json" "$(grep -c anti_roll_bar packages/suspension_contracts/src/suspension_contracts/contracts/$s.schema.json)"; done
template.schema.json             2
subsystem.schema.json            1
assembly.schema.json             1

$ printf "%-32s %s
" templates/roles.py "$(grep -c anti_roll_bar packages/suspension_multibody/src/suspension_multibody/templates/roles.py)"
templates/roles.py               3
$ printf "%-32s %s
" templates/builtin.py "$(grep -c anti_roll_bar packages/suspension_multibody/src/suspension_multibody/templates/builtin.py)"
templates/builtin.py             3
$ printf "%-32s %s
" test_template_model.py "$(grep -c anti_roll_bar packages/suspension_multibody/tests/templates/test_template_model.py)"
test_template_model.py           2
```

## 3. 杆元件与端口实测

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-02-arb-subsystem/raw/arb_rows_probe.py
exit=0        # 输出见 arb_rows_output.txt
```

## 4. 判据命令

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
298 passed in 5.83s
exit=0

$ uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider
32 passed in 0.11s
exit=0

$ bash -c '! grep -rn "upright_L" packages/suspension_multibody/src/suspension_multibody/subsystems/suspension.py'
exit=0

$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0

$ uv run --no-sync ruff check .
All checks passed!
exit=0

$ uv run --no-sync ty check .
All checks passed!
exit=0
```

`combined sha256` 与冻结值一致；`git status --short -- packages/suspension_multibody/tests/data/` 输出为空。

## 5. 本行未跑（如实记账）

- `tests/architecture/` 整个目录、`tests/adams/`（未触及分层/轮胎力律）。
- `just gate-numeric` 的 `case_parity_check.py` 与 `kc_perf_gate.py`：本行不改求解路径，
  数值门三项归 p4-05 阶段收尾与 Epic 收尾统一跑。**不声称已跑**。

## 6. 过程中修正的一处自身缺陷

端口第一次落盘时，`droplink_mount` 的 `owner` 被写成 `torsion_bar`（复制粘贴自
`chassis_mount`），于是 `arb.droplink_bodies()` 返回的是两个扭杆半体而不是两个小吊杆——
`test_the_subsystem_declares_a_bar_half_and_a_droplink_per_side` 立刻失败
（`('torsion_bar_L','torsion_bar_R') != ('droplink_L','droplink_R')`）。
改为 `droplink` 后通过。**这条失败正是「端口声明必须指到真实部件」的检查**，
说明该断言确实在起作用，不是摆设。
