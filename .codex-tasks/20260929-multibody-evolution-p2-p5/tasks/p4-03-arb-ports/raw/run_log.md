# p4-03 实跑日志

命令均在仓库根目录执行（Windows / Git Bash），每条后附**实测**退出码。

## 1. 端口声明与产物实测

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-03-arb-ports/raw/ports_probe.py
（输出见 ports_declared_output.txt）
exit=0
```

## 2. 判据命令

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/connections packages/suspension_multibody/tests/authoring packages/suspension_multibody/tests/rigs packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
494 passed in 15.08s
exit=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems/test_arb_mount_ports.py -q -p no:cacheprovider
6 passed in 1.43s
exit=0

$ uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider
32 passed in 0.10s
exit=0

$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0

$ uv run --no-sync ruff check .
All checks passed!
exit=0

$ uv run --no-sync ty check .
All checks passed!
exit=0
```

`git status --short -- packages/suspension_multibody/tests/data/` 输出为空。

## 3. 过程中修正的两处自身缺陷（诚实记账）

1. **测试初版用了不存在的 API**：先写 `report.bindings.values()`（`bindings` 是
   **元组**，不是 dict），改用 `report.binding_for(role)`；又把 `binding.port_id` 写成
   单数（真实字段是 `port_ids`）。两处都是断言自身写错，测试当场报 `AttributeError`，
   修正后 6 passed。**没有放宽任何断言**——报错的是我的 API 用法，不是被断言的性质。
2. **测试初版残留了几行死代码**（`args`、`del args`、一个恒真的
   `assert ... or True`）。这些是我起草时的残留，已整文件重写清掉。

## 4. 本行未跑（如实记账）

- `just gate-numeric` 的 `case_parity_check.py` 与 `kc_perf_gate.py`：本行不改求解路径，
  数值门三项归 p4-05 阶段收尾与 Epic 收尾统一跑。**不声称已跑**。
- `tests/architecture/` 整目录、`tests/adams/`。
- 未重录任何基线。
