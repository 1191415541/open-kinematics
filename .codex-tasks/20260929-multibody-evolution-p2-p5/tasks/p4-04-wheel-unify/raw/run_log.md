# p4-04 实跑日志

命令均在仓库根目录执行（Windows / Git Bash），每条后附**实测**退出码。

## 1. 独立复验探针

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p4-04-wheel-unify/raw/wheel_unify_probe.py
exit=0        # 输出见 wheel_unify_output.txt
```

## 2. 判据命令

```
$ bash -c '! grep -rn VerticalTireElement packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py'
exit=0

$ grep -c isinstance packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py
0

$ grep -n _FILE_ROLE_TEMPLATES packages/suspension_multibody/src/suspension_multibody/authoring/solver.py
726:_FILE_ROLE_TEMPLATES: tuple[str, ...] = ("steering", "chassis", "wheel")

$ grep -n "wheel_template" packages/suspension_multibody/src/suspension_multibody/subsystems/types.py
174:    wheel_template: object | None = None
287:    "wheel": "wheel_template",

$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/vehicle_assembly packages/suspension_multibody/tests/authoring -q -p no:cacheprovider
279 passed in 13.84s
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0

$ git status --short -- packages/suspension_multibody/tests/data/
（空）
```

## 3. 本行的改动范围（就这些）

唯一的生产代码改动是 `subsystems/assembler.py` 的**两处注释措辞**（`:36` 与 `:94`），
为的是让父行那条「按类名 grep」的机械判据恢复判别力（详见
`stage1_04_reverification.md` 第 2 节）。**没有任何逻辑改动。**

## 4. 本行未做（如实记账）

- **未重做阶段一 04 的任何动作**（文件读取链、类型过滤移除、单轴侧凝结、K/C 受力激活断言）：
  `EPIC.md:75` 与 `:271` 明令禁止。
- **未跑** `tests/architecture` 整目录、`tests/adams`、`tests/cases`。
- **未跑** `just gate-numeric` 的 `case_parity_check.py` 与 `kc_perf_gate.py`：
  归 p4-05 阶段收尾与 Epic 收尾。**不声称已跑**。
- 未重录任何基线。
