# p5-02 run log · 命令、退出码、关键输出

全部命令在仓库根 `E:\杂件\open-kinematics` 实跑。`uv run --no-sync` 为唯一调用方式。
会话 scratch：`C:\Users\zzy11\.pi-desktop\scratch\ba123231-caeb-4061-b2dc-c970cf431e16`。

## A. 静态检查

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| A1 | `uv run --no-sync ruff check <本行三个文件>` | 0 | `All checks passed`（首轮 4 处已修：`api.py` 的 `TYPE_CHECKING` 导入、测试文件 import 排序） |
| A2 | `uv run --no-sync ty check .` | 0 | `All checks passed!` |
| A3 | `uv run --no-sync ruff check .` | 1 | 17 处 finding **全部**落在 `.codex-tasks/**/raw/*.py` 的既有临时探针脚本（p2-01/p3-01 的 `rear_steer_probe.py`、`probe_*.py`），与 p5-02 改动无关；本行没有触碰它们 |

## B. 验收五条（SPEC「Final Validation Command」+ 数值门 + 快速集）

### B1 门禁一：`legacy_surface_gate.py --check`

```
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0

OK: no unregistered Python boundary violation
exit=0
```

### B2 门禁二：`check_composable_release.py --skip-isolation`

```
$ uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
release probe: E:\杂件\open-kinematics
scratch      : C:\Users\zzy11\AppData\Local\Temp\suspension-release-5jb3ci7e

[PASS] migration list: 0 retained legacy imports (0 production, 0 test) and 4 retired packages gone
[PASS] documentation roots: 16 documented modules present, 4 retired ones absent
[PASS] documentation examples: 3 example(s) executed: E-1, E-2, E-3

OK: 3 release checks passed
exit=0
```

（`[PASS] documentation examples` 说明 `composable_extension_examples.md` 的三个 `runnable`
块确实被**执行**并通过；本行没有改那份文档。）

### B3 `pytest tests/api tests/architecture`

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/api packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
........................................................................ [ 37%]
........................................................................ [ 74%]
.................................................                        [100%]
193 passed in 621.82s (0:10:21)
exit=0
```

其中新增文件单独跑：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/api/test_simulate.py -q -p no:cacheprovider
..................                                                       [100%]
18 passed in 2.12s
exit=0
```

`test_public_api_boundary_gate.py` 单独跑（SPEC 判据 (e)）：

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_public_api_boundary_gate.py -q -p no:cacheprovider
.....                                                                    [100%]
5 passed in 3.73s
exit=0
```

### B4 数值门：`dynamic_hash_sentinel.py --check`

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
...
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency',
                    'opposite_phase_road', 'road_pulse', 'road_sine',
                    'road_step_finite_rise', 'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
exit=0
```

（`acceptance exit 1` 与那 9 个失败用例是**基线里就记着**的既有状态；本行实跑与
`tests/data/dynamic_hash_baseline.json` 的
`{"acceptance_exit_code": 1, "artifact_count": 26, "combined_sha256": "fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9"}`
逐字段一致，故哈希未变。）

### B5 快速集 + 另两个包

```
$ uv run --no-sync pytest packages/suspension_multibody/tests -q \
    --ignore=packages/suspension_multibody/tests/adams \
    --ignore=packages/suspension_multibody/tests/cases \
    --ignore=packages/suspension_multibody/tests/architecture -p no:cacheprovider
1108 passed, 1 xfailed in 38.45s
exit=0

$ uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests -q -p no:cacheprovider
73 passed in 15.31s
exit=0
```

`xfailed` 数量与 p5-01 基线一致，无新增 skip / xfail。

## C. 结构门（EPIC 冻结约束「改结构后加跑」）

```
$ uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
module cycles (SCC size>1) : 0
OK: layering matches the recorded baseline
exit=0

$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check --final
mode      : final
findings  : 0

OK: no unregistered Python boundary violation
exit=0
```

## D. 内核提交唯一归属实测

```
$ grep -rn "run_contract" --include=*.py packages/suspension_multibody/src \
      packages/suspension_multibody/tests packages/suspension_multibody/scripts | grep -v __pycache__
src/suspension_multibody/kernel/__init__.py:30                       __all__ = [... "run_contract"]
src/suspension_multibody/kernel/__init__.py:186                      def run_contract(          # 定义
src/suspension_multibody/simulation/backend.py:7                     from ..kernel import ContractRun, run_contract
src/suspension_multibody/simulation/backend.py:24                    return run_contract(        # 唯一调用点
tests/... 命中全部是门禁脚本自身与其负例夹具
```

**生产侧唯一调用点仍是 `simulation/backend.py:24`**，与 p5-01 `gates.md` 的冻结现状一致；
`simulate` 经 `run_request`（`simulation/runner.py`）到达这里，没有新增提交路径。

## E. `__all__` / `_PUBLIC_NAMES` 计数

```
$ uv run --no-sync python -c "import suspension_multibody as m; \
    print(len(m.__all__), len(m._PUBLIC_NAMES), 'simulate' in m.__all__, m._PUBLIC_NAMES['simulate'], 'FrontAxleModel' in m.__all__)"
18 17 True ('.api', 'simulate') True
```

## F. 临时脚本（只写在会话 scratch，未进仓库）

| 脚本（`$PI_SCRATCH_DIR/`） | 用途 |
|---|---|
| `probe_hash.py` / `probe_hash2.py` | `model_hash` 与 `model_dump` 形状对照（改动前后各跑一次） |
| `probe_sim.py` / `probe_sim2.py` | `simulate` 端到端、三种总成拼法、八类拒绝的探路 |
| `e2e_record.py` | 产出 `simulate_entrypoint.md` §4 引用的原始输出 |
| `hash_before.txt` | 本行开工时的改动前哈希 |
| `gates_raw.txt` | `raw/gates.txt` 的原始捕获 |
