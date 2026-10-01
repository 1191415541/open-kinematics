# p3-01 run_log：本行实际执行的命令、退出码、关键输出行

诚实记录。本文件与上面四份证据文件的每处引用一一对应。工作目录除注明外均为仓库根 `E:\杂件\open-kinematics`。

## A. 环境确认

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| A1 | `uv run --no-sync python -c "import suspension_multibody, sys; print('ok', suspension_multibody.__file__)"` | 0 | `ok E:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\__init__.py` |

结论：包可导入，后续所有 python 一律走 `uv run --no-sync`。

## B. 物理测试（SPEC 的 Final Validation Command 的前半段）

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| B1 | `uv run --no-sync pytest packages/suspension_multibody/tests/physics -q -p no:cacheprovider` | **0** | `..... [100%]` / `5 passed in 2.02s` |
| B2 | 同 B1 复跑一次 | **0** | `..... [100%]` / `5 passed in 1.56s` |

结论：`tests/physics` 全绿（5 passed），是阶段三唯一的既有回归网。

## C. 定位检索（只读）

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| C1 | `ls .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/` | 0 | `SPEC.md TODO.csv PROGRESS.md raw/`（`raw/` 起始为空） |
| C2 | `wc -l .codex-tasks/.../EPIC.md` | 0 | `368 .codex-tasks/.../EPIC.md` |
| C3 | `find . -name "roll_centers.py" -o -name "static_loads.py"` | 0 | `./packages/suspension_multibody/src/suspension_multibody/vehicle/{roll_centers,static_loads}.py` |
| C4 | `grep -rn "compute_vehicle_roll_centers" --include=*.py --include=*.md . \| grep -v "^./.codex-tasks"` | 0 | 5 行；生产调用者 0，测试调用者 `tests/physics/test_vehicle_physics.py:5`（导入）/`:56`（调用） |
| C5 | `grep -rn "compute_static_wheel_loads" --include=*.py packages/` | 0 | 生产调用者 1 个：`vehicle/service.py:40` |
| C6 | `grep -rn "full_vehicle_model" packages/suspension_multibody/tests/ --include=*.py` | 0 | 夹具定义在 `tests/conftest.py:26` |
| C7 | `ls -la tests/data/kc_baseline/` + `find . -name dynamic_hash_baseline.json` | 0 | 真实路径在包内：`packages/suspension_multibody/tests/data/...`；`kc_baseline/` 含 `c_states.json k_states.json manifest.json` |
| C8 | `grep -rn "suspension_kind" --include=*.py packages/` | 0 | 模板真源 `templates/builtin.py:405`；registry 键名实测见下方 E2 |
| C9 | `grep -rniE "five.?link\|macpherson\|mcpherson\|twist.?beam\|multilink" --include=*.py --include=*.json ... packages/` | 1 | **零命中**（唯一命中在两份 `.codex-tasks` 旧文档） |
| C10 | `grep -n "def side_hardpoints" -A 40 .../subsystems/geometry.py` | 0 | `side_hardpoints` 在 `geometry.py:102-135`；`lookup_hardpoint` 在 `:138-147` |
| C11 | `grep -n "def check_role_contract" -A 60 .../templates/model.py` | 0 | `check_role_contract` 在 `templates/model.py:343-367`，抛错段 `:356-359` |
| C12 | `grep -rn "trailing" --include=*.py packages/suspension_multibody/tests/` | 0 | 夹具 `tests/data/composable/synthetic_trailing_arm_axle.json` + `tests/composable/fixtures.py` |
| C13 | `grep -rn "vehicle" .../tests/architecture/test_import_boundaries.py` | 0 | `suspension_multibody.vehicle` 在被检查包列表 `:92` |

## D. 基线只读核查（F10）

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| D1 | `cd packages/suspension_multibody/tests/data && grep -rniE 'roll_center\|rollcentre\|roll centre' kc_baseline/ dynamic_hash_baseline.json` | **1** | 输出为空 → **0 命中** |
| D2 | `grep -rniE 'roll_stiffness\|track_change' kc_baseline/ dynamic_hash_baseline.json` | **1** | 输出为空 → **0 命中** |
| D3 | `grep -rl '' kc_baseline/` | 0 | `kc_baseline/c_states.json`、`kc_baseline/k_states.json`、`kc_baseline/manifest.json`（证明 D1/D2 的搜索范围非空） |
| D4 | `uv run --no-sync python - <<'PY' ...`（遍历三个基线的叶子键名 + `dynamic_hash_baseline.json` 的条目形状） | 0 | `k_states.json` list/9；`c_states.json` list/66；叶子键见 `baseline_impact.md` §2；`dynamic_hash_baseline.json` dict，顶层 6 键，`entries` 26 条，条目键 `['arrays_npz_sha256','artifact','completed_samples','manifest_sha256','status']`，status 全 `success` |
| D5 | `cat packages/suspension_multibody/tests/data/kc_baseline/manifest.json` | 0 | 全文 417 B，见 `baseline_impact.md` §2.3 |
| D6 | `grep -rn "roll_stiffness" --include=*.py --include=*.json --include=*.md . \| grep -v "^./.codex-tasks"` | 0 | 产品代码仅 `adams/vehicle_parameters.py:40`；其余 24 个 `artifacts/**` JSON |
| D7 | `grep -rn "track_change" --include=*.py --include=*.json --include=*.md . \| grep -v "^./.codex-tasks"` | 0 | multibody 内仅 `templates/builtin.py:393`、`templates/roles.py:85` |
| D8 | `grep -rln "roll_stiffness" artifacts/ \| wc -l` | 0 | `24` |
| D9 | `uv run --no-sync python -c "...acceptance_statuses..."` | 0 | `total 13 PASSED 4 FAILED 9` |
| D10 | `cd packages/suspension_multibody/tests/data && sha256sum kc_baseline/*.json dynamic_hash_baseline.json`（工作前后各跑一次，两次结果相同） | 0 | 四份 sha256 见 `baseline_impact.md` §0，**前后一致** → 未重录 |

## E. 四种构型构造实测（Goal 2）

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| E1 | `uv run --no-sync python tasks/p3-01-freeze/probe_configs.py`（首版；入口 A 用 `vehicle()` 建 `VehicleModel`） | 0 | 首次暴露 `VehicleModel` 的拓扑校验（见 E4 的 ValidationError） |
| E2 | `uv run --no-sync python tasks/p3-01-freeze/probe_configs.py` | **0** | 双叉臂：`CONSTRUCTED: front center=array([1.14e-13, -180.0])`；5 连杆/麦弗逊/扭梁：`CONSTRUCT REFUSED: ValueError: missing hardpoint for roll-center role upper_front`（`roll_centers.py:78`）。完整输出落 `raw/out_configs.txt` |
| E3 | `uv run --no-sync python tasks/p3-01-freeze/probe_explicit.py` | 0 | 注册期拒绝：`TemplateError: template 'five_link_probe' does not satisfy role 'suspension': missing mount(s) [...]`；`suspension.required_mounts = ('upper_front', ..., 'tie_outer')`；`registered templates: ('brake_4wdisk_simplified', 'double_wishbone', 'powertrain_simplified', 'steering', 'vehicle_body', 'wheel_on_hub')` |
| E4 | `uv run --no-sync python tasks/p3-01-freeze/probe_trailingarm_vehicle.py`（首版，后来修正硬点表） | 1 | `pydantic ValidationError: front axle requires positive mass specs for: lower_arm_L, ...`（`schema/vehicle.py:294-296`） |
| E5 | 同 E4 修正后（用 fixture 校验过的 `hardpoints`） | 0 | `explicit trailing-arm axle topology = explicit bodies = ['upright_L','upright_R']` / `VEHICLE DECLARED` / `REFUSED: ValueError: missing hardpoint for roll-center role upper_front`（`roll_centers.py:78`） |
| E6 | `uv run --no-sync python tasks/p3-01-freeze/probe_axle_rollcenter.py` | 0 | `trailing-arm explicit model declared OK; topology = explicit` / `trailing-arm assembled: bodies = ['chassis','upright_L','upright_R'] constraints = 2` |
| E7 | `uv run --no-sync python tasks/p3-01-freeze/probe_tb.py`（取 compose_axle 的完整调用栈） | 0 | `entry.py:52` → `si_assembly.py:468` → `si_assembly.py:242` → `suspension.py:397` → `types.py:411` → `types.py:407` → `geometry.py:146` `ValueError: missing required front-axle hardpoint for upper_front` |
| E8 | `uv run --no-sync python tasks/p3-01-freeze/probe_explicit5.py` | 0 | `5-link explicit model declared OK; bodies = ['upright_L','upright_R'] joints = 10` / `compose_axle OK: bodies=['chassis','upright_L','upright_R'] constraints = 10` / `VehicleModel accepted` / `roll-centre REFUSED: ...upper_front` |
| E9 | `uv run --no-sync python tasks/p3-01-freeze/probe_three.py` | 0 | `5-link ... constraints=10`；`MacPherson ... constraints=6`；`twist-beam ... constraints=3`（三者都 `ASSEMBLED`） |
| E10 | `uv run --no-sync python tasks/p3-01-freeze/probe_vehicle_route.py`（首版函数签名写错，`TypeError: BaseModel.__init__() takes 1 positional argument`） | 1 | 脚本自身 bug，修正后见 E11 |
| E11 | 同 E10 修正后 | 0 | `5-link: compose_vehicle_runtime OK: bodies=9 constraints=24 total_mass=1380.0`；`twist-beam: compose_vehicle_runtime OK: bodies=9 constraints=8 total_mass=1380.0` |

## F. 断言原文摘录

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| F1 | `grep -c "assert " packages/suspension_multibody/tests/physics/test_vehicle_physics.py` | 0 | `17`（5 个测试函数、17 条 assert） |
| F2 | `grep -n "assert \|^def test" .../test_vehicle_physics.py` | 0 | 逐行清单见 `tests_physics_assertions.md` |

## G. 只读自证

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| G1 | `git status --short`（工作开始后、写证据文件之前） | 0 | 见下方「环境观察」 |
| G2 | `git status --short`（本行结束时） | 0 | 同上；新增的 `??` 只有本行的 `raw/` 与 `probe_*.py` |
| G3 | `git diff --stat -- packages/` | 0 | 7 个 kernel C++ 文件被改（238 insertions, 8 deletions）——**非本行所改**，见下 |
| G4 | `sha256sum kc_baseline/*.json dynamic_hash_baseline.json`（第二次） | 0 | 与 D10 逐字节一致 |

### 环境观察（必须写明，避免误读 G3）

`git diff --stat -- packages/` 在本行执行期间显示 `packages/suspension_kernel/cpp/**` 有 7 个文件被修改。**这不是
**这不是本行改的**：

- 本行全部写入只落在 `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/{raw/,probe_*.py}` 与会话 scratch；`packages/**` 全程只读（`Read`/`Grep`/`Glob` 与只读 python 导入）。
- 本行开始时 `git status --short` 已显示 `packages/suspension_kernel/cpp/include/mb_input/types.hpp` 被改；结束时该集合扩大到 7 个文件（新增 `element_wrench.hpp`、`mb_element/functions.hpp`、`mb_model/types.hpp`、`assembly/element_reader.cpp`、`element/anti_roll.cpp`、`force/external_vector.cpp`）。
- 文件 mtime 实测：`mb_input/types.hpp` 02:31:48、`assembly/element_reader.cpp` 02:37:17、`element/anti_roll.cpp` 02:37:48；本行探测脚本的执行时间在 02:33–02:39 之间，**与本行命令时间线交叉**——即有**另一个并发写入者**（p2-02 方向：内核力矩元/防倾杆 ABI）在同一工作区改 `packages/suspension_kernel/cpp/`。
- 本行的只读探测**未受其影响**：所有探测只依赖 Python 侧 `suspension_multibody`，且 B1/B2 两次 `pytest tests/physics` 结果一致。
- 提醒：这正是 AGENTS.md 第 7 节「不要在会话外并发改同一批文件」警告的模式。本行的 `roll_centers.py` / `static_loads.py` 不在被改集合内，故本行证据未被污染。

## H. 未能验证的条目

| 条目 | 原因 |
|---|---|
| 四种构型各自跑通一次 study/solve | 本行判据只要求构造性盘点；真实求解会触及 `cases/`，超出本行写范围 |
| 麦弗逊滑柱的真实约束（prismatic/cylindrical vs 本行用的 spherical） | 本行只验证「能否构造」，未做工程等价性验证 |
| 5 连杆显式 10 球面副的过约束与数值可解性 | 只验证装配产物生成（`constraints=10`），未跑求解 |
| 扭梁扭杆耦合语义 | 本行用一个 revolute 代替扭杆，仅为通过装配入口的最小声明，未验证物理 |
| `roll_centers` 的既有数值基准（改造前 `center[1]`） | 既有 5 个测试**不含任何数值断言**（见 `tests_physics_assertions.md` 测试 4）；本行实测值 `-180.0` 只是本行脚本输出，**不构成既有基线** |
| `kc_parity_check.py` | 本行**未跑**；`EPIC.md:231` 已说明不带 `--actual-dir` 时恒过、不构成证据 |
| `tests/data/**/sha256.json` 与 `kc_perf_baseline_native.json` | 非本行范围，未读 |

## I. SPEC 的 Final Validation Command（实跑）

命令（逐字照 `SPEC.md:100`）：

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/physics -q && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/raw/config_refusals.md && test -s .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-01-freeze/raw/baseline_impact.md
```

分步实跑结果：

```
config_refusals.md non-empty OK
baseline_impact.md non-empty OK
.....                                                                    [100%]
5 passed in 1.49s
FINAL_EXIT=0
```

退出码 **0**，三条全部满足。

## J. 证据可复现性核验

| # | 命令 | 退出码 | 关键输出 |
|---|---|---|---|
| J1 | `uv run --no-sync python .codex-tasks/.../p3-01-freeze/probe_configs.py > $PI_SCRATCH_DIR/repro.txt && diff $PI_SCRATCH_DIR/repro.txt .codex-tasks/.../p3-01-freeze/raw/out_configs.txt` | 0 | 无差异 → `REPRODUCIBLE: byte-identical`（`raw/out_configs.txt` 是真跑输出，可重复） |

## K. 命令总数

上表 A–J 共 **44** 条命令（含复跑与失败命令，不含 `Read`/`Grep` 工具调用本身之外的纯文件查看）。其中退出码非 0 的 4 条均为**预期**：D1/D2（grep 零命中，正是结论本身）、E4/E10（探测脚本自身首版缺陷，已修正并记录）。
