# p3-04 run_log：本行实际执行的命令与退出码

只记录**真正执行过**的命令。所有命令在仓库根目录执行（`E:/杂件/open-kinematics`），
shell 为 Git Bash。`<scratch>` 指会话临时目录 `C:\Users\zzy11\.pi-desktop\scratch\ba123231-...`。

## 起点状态（改造前）

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 1 | `uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider` | 0 | `93 passed, 1 xfailed in 9.47s`（改造前基线） |
| 2 | `uv run --no-sync python <scratch>/dump_before.py` | 0 | 改造前四轮的 rank/residual/载荷十六进制，落盘 `<scratch>/before_values.txt` |
| 3 | `md5sum packages/.../vehicle/static_loads.py .codex-tasks/.../raw/static_loads_before.py` | 0 | 均 `2b989cfe078bacc5a5969dbb4ba19250`（存档 = 改造前的文件字节） |
| 4 | `git hash-object packages/.../vehicle/static_loads.py` | 0 | `53703aaead4c600e0b0e6ba44892d544c83deb75` |

## 探索（确定接触点来源；probe 脚本在会话 scratch，未落 raw）

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 5 | `uv run --no-sync python <scratch>/probe1.py` | 0 | 四轮：`wheel_centers` 4 键、`total_mass=1472.411361572`、`residual=9.5367431640625e-07`、`rank=3` |
| 6 | `uv run --no-sync python <scratch>/probe2.py` | 1 | 三轴装配成功（6 个轮端、`total_mass=138.617042358`）；单侧 entry 因模板缺 `upright_R` 报 `KeyError` —— 说明「单轮」必须也声明 `sides=("L",)`，且轴模板必须两套部件都在 |
| 7 | `uv run --no-sync python <scratch>/probe3.py` | 1 | 同一处边界；同时确认 chassis 位置可推动总质心 |
| 8 | `uv run --no-sync python <scratch>/probe4.py` | 0 | 用「两套体都在、轮端在中心线」的轴可装配；质心 y 还差 0.627 mm |
| 9 | `uv run --no-sync python <scratch>/probe5.py` | 0 | 该几何下 `wheel_centers` 只有 `single_left` 一项 |
| 10 | `uv run --no-sync python <scratch>/probe6.py` | 0 | 三轴 6 点、单轮 1 点的求解与报错数值 |
| 11 | `uv run --no-sync python <scratch>/probe7.py` | 0 | 单轮两例的异常消息全文 |
| 12 | `uv run --no-sync python <scratch>/probe8.py` | 0 | 单侧 entry 的逐体质量与质心清单（确认 0.627 mm 来自 `rack_housing`，把轴放到中心线即可消掉） |

## 改造后：4 轮逐位比较

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 13 | `git diff --stat -- packages/.../vehicle/static_loads.py` | 0 | `1 file changed, 191 insertions(+), 31 deletions(-)` |
| 14 | `uv run --no-sync python <scratch>/dump_before.py > <scratch>/after_values.txt`，再 `diff` 两份 | 0 | 唯一差异是 `module has _WHEELS: True` → `False`；所有数值行逐字相同 |
| 15 | `uv run --no-sync python .codex-tasks/.../raw/four_wheel_bitwise_probe.py` | 0 | `DIFFERENCES: none`（6 个工况、每个字段的十六进制字节全部相同） |
| 16 | `git show HEAD:.../static_loads.py \| md5sum` + `md5sum .../raw/static_loads_before.py` + `diff` | 0 | 两个 md5 均 `e8ee0b870a9ebde39034fb7b9582081e`；`diff` 无输出 |

## 改造后：pytest 与静态检查（迭代中）

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 17 | `uv run --no-sync pytest packages/.../tests/physics/test_static_loads.py -q -p no:cacheprovider` | 0 | `9 passed in 1.52s` |
| 18 | `uv run --no-sync ruff check packages/.../vehicle/static_loads.py packages/.../tests/physics/test_static_loads.py` | 1 | 4 条 docstring 规则命中（D401 ×3、D202 ×1），已修 |
| 19 | 同上 | 0 | `All checks passed!` |
| 20 | `uv run --no-sync pytest packages/.../tests/physics packages/.../tests/vehicle -q -p no:cacheprovider` | 0 | `102 passed, 1 xfailed in 9.52s` |
| 21 | `uv run --no-sync ruff check packages/.../vehicle/static_loads.py packages/.../tests/physics/test_static_loads.py` | 0 | `All checks passed!` |

## 改造后：架构门与快速集

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 22 | `uv run --no-sync python packages/.../tests/architecture/legacy_surface_gate.py --check` | 0 | `findings: 0`；`OK: no unregistered Python boundary violation` |
| 23 | `uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | 0 | `module cycles (SCC size>1): 0`；`OK: layering matches the recorded baseline` |
| 24 | `uv run --no-sync python packages/.../scripts/check_composable_release.py --skip-isolation` | 0 | `OK: 3 release checks passed` |
| 25 | `uv run --no-sync pytest packages/.../tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q -p no:cacheprovider` | 0 | `1162 passed, 1 xfailed in 39.98s` |
| 26 | `uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_contracts/tests -q -p no:cacheprovider` | 0 | `73 passed in 16.04s` |

## 干扰与重试（如实记录，均为并发写入者造成的中间态）

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 27 | `uv run --no-sync pytest packages/.../tests/physics packages/.../tests/vehicle -q` | 2 | 收集期错误 `NativeKernelUnavailableError: the native kernel mirror ... is older and different from ...`。**不是本行改动导致** —— 同一时刻有另一个写入者在重建 native kernel（镜像 DLL 正被覆盖）。未改任何东西，稍后重跑即恢复 |
| 28 | 同上，30 秒后重跑 | 0 | `102 passed, 1 xfailed in 9.51s` |
| 29 | `uv run --no-sync pytest packages/.../tests/architecture/test_import_boundaries.py -q -p no:cacheprovider` | 1 | `1 failed, 55 passed in 586.90s`：`test_every_ordered_pair_of_entry_points_imports` 报 19 个「`SyntaxError: keyword argument repeated: note`」。**不是本行改动导致** —— 同时刻别的写入者正在改 `subsystems/*.py` 与 `templates/builtin.py`（`find -newermt` 可见），子进程读到了半写完的文件。逐条手工重放那 19 对导入（含 `connections then studies`）退出码全为 0 |
| 30 | 同上，重跑 | **0** | `56 passed in 602.74s` |
| 31 | `uv run --no-sync ruff check .`（早先一轮） | 1 | 除 F4 的两条外还额外报出 `raw/four_wheel_bitwise_probe.py` 的 3 条 `E402`；那是本行自己的脚本，已加 `# noqa: E402` 修掉（见 F6） |

## 最终验证（任务书逐条，实跑）

| # | 命令 | 退出码 | 结果 |
|---|---|---|---|
| F1 | `uv run --no-sync pytest packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle -q -p no:cacheprovider` | **0** | `102 passed, 1 xfailed in 9.37s` |
| F2 | `uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | **0** | 末行 `OK: dynamic output matches the frozen baseline byte-for-byte`；`combined sha256: fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`。该脚本前半段还打印 `solver self-convergence: FAILED` / `adams accuracy: BLOCKED` / 若干 `road_*: FAILED`：这是本行之前既有的、与静态轮荷无关的输出（本行未改任何求解路径）；脚本的退出码由末行 `OK` 决定，为 0 |
| F3 | `bash -c '! grep -rn -e "_WHEELS = (\"front_left\"" packages/suspension_multibody/src/suspension_multibody/vehicle'` | **0** | 零命中 |
| F4 | `uv run --no-sync ruff check .` | **0** | `All checks passed!`（repo 级全绿）。更早一轮该命令退出码为 1，2 个发现均在 `.codex-tasks/.../tasks/p2-07-doc-route/raw/document_route_probe.py:3`（`E401` + `F401`）；那是 p2-07 的文件，本行不碰，**其归属者随后自行修好**，重跑即 0 |
| F5 | `uv run --no-sync ty check .` | **0** | `All checks passed!` |
| F6 | `uv run --no-sync ruff check .codex-tasks/.../tasks/p3-04-static-loads/raw/` | **0** | `All checks passed!` |
| F7 | `uv run --no-sync python .codex-tasks/.../raw/n_point_cases_probe.py` | **0** | 输出见 `raw/n_point_cases_output.txt` |
| F8 | `uv run --no-sync python .codex-tasks/.../raw/four_wheel_bitwise_probe.py` | **0** | `DIFFERENCES: none`；输出见 `raw/four_wheel_bitwise_output.txt` |

## 未做的验证（如实声明）

- **未跑 `tests/architecture/` 全目录**：其中与本行直接相关的
  `tests/architecture/test_import_boundaries.py` 单独跑了两次，第二次 `56 passed`（#30）。
  其余用例按 `AGENTS.md` 第 1 节只在收尾跑；本行没有搬模块、没有改分层。
- **未跑 `tests/adams/` 与 `tests/cases/`**：`AGENTS.md` 第 3 节说 `adams/` 只在改轮胎力律/对标时跑、
  `cases/` 只在改 `cases/` 或 preparation 时跑；本行两者都没动。
- **未跑 `just gate-numeric` 的其余两项**（`case_parity_check.py` 162 s、`kc_perf_gate.py` 9 s）：
  本行只改了 `vehicle/static_loads.py`，不在 `axle_dynamics/`、`kernel/`、`cases/`、轮胎/力元/求解路径上，
  按 `AGENTS.md` 第 2 节不触发；该节第一项 `dynamic_hash_sentinel.py --check` 已跑且退出码 0（F2）。
- **未跑全量 `just test-all`**（约 33 分钟）：`AGENTS.md` 第 3 节把它定位为「提交前或跨子系统时」才跑；
  本次交付是子任务，且本行只触及一个模块。
- **未改任何冻结基线**：`git status` 中无 `tests/data/**`、`dynamic_hash_baseline.json`、
  `kc_perf_baseline_native.json`。
- **未新增 skip/xfail**：`tests/physics` + `tests/vehicle` 的 `1 xfailed` 与改造前一致
  （改造前 #1 也是 `1 xfailed`）。
