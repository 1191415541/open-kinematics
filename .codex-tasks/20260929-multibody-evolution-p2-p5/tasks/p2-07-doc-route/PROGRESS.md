# PROGRESS：p2-07 内核文档路由接受 rotational_torque 元素族

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-07`

## Session Start

- **Date**: 2026-10-01
- **Task name**: p2-07-doc-route
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-07-doc-route/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步，全部 DONE）
- **Environment**: Python 3.12.13 / uv / pytest 8.3.4 / ruff 0.13.3 / ty / Git Bash on Windows

## Context Recovery Block

- **Current milestone**: 完成（5/5）
- **Current status**: DONE
- **Last completed**: 第 5 步——既有族往返与零回归门
- **Current artifact**: `raw/document_route_implementation.md`、`raw/document_route_evidence.md`、
  `raw/existing_family_regression.md`、`raw/run_log.md`，以及三份可复跑的探针脚本
- **Key context**:
  - 依赖 p2-03 已落地（`ELEMENT_ROTATIONAL_TORQUE = 7`、参数槽 128/129/130/133/137、
    `kElementBlockSize = 216`）。本行只补**文档路由**这一处缺口，不改内核的元素律、不改 ABI。
  - 生产动态路径唯一的提交点是 `simulation/backend.py:24` 的 `run_contract`，它提交的是**模型文档**；
    C ABI 路径（`mb_core_run` 的 `ElementBlock`）在 p2-02 就能读本族，只有文档路径不能。
  - **取方案 (iv)**：`ContractModel::read` 解析并保存该族的 `ElementBlock`，文档入口在既有逐族
    `build_model` 之后把这些块交给**既有** `read_element_blocks` 追加。不新增 `AxleInput` 逐族字段
    （避免第二次 ABI 变更），也不放宽 `build_model.cpp:41-58` 的互斥守卫。
  - **参考姿态的语义取值有实测依据**：`element_reader.cpp:375-380` 把「块的四个槽为 0」当作本族的
    「无参考姿态」，所以文档读取器在 `reference_quaternion` 缺省时**写 0 而不是补单位四元数**，
    否则同一份文档经两条路径会得到不同的块。测试里有断言钉这一点。
- **Known issues**:
  - `anti_roll.cpp:128` 把驾驶员需求硬编码为 `1.0`、不读轮胎滑移——这是 G1 的另一层，
    本行只证明「该族经文档可达且影响轨迹」，按 SPEC「已知边界」不声称已解决，归 p2-05 结项前处置。
  - 验收的运行在 `dynamic_hash_sentinel.py --check` 内部打印 `acceptance exit : 1` 与 9 个 `FAILED`
    用例名，那是冻结基线的既有状态（与 p3-01/p2-02 记录的完全一致），脚本自身判定
    `OK: ... byte-for-byte`、退出码 0、combined sha256 逐字符一致。
  - 本行跑过 `ruff check .` 与 `ty check .`，两者退出码均为 0。为达到这一点，
    `ruff --fix` 修掉了 p2-07 自己的 `raw/document_route_probe.py`（首次落盘时的 `import json, sys`）
    与本行的新测试的 import 分组；盘面上其它 `raw/` 探针脚本（p2-01、p3-01 等）未被触碰。

## 交付

1. **`kElements[]` 与 schema enum 同步**：`contract_registry.cpp:36-40` 加名（10 → 11）；
   `cpp/tests/contract_selftest.cpp:144` 断言 `== 11`，`:132-136` 补上 anti-roll 与本族的
   `contract_element_known` 检查，可执行文件输出 `mb_contract selftest: OK (45 checks)`；
   `multibody_model.schema.json:139-145` 的**元素族** enum 加名（joint 的 enum 在 `:80-85`，未动）。
   漏任何一处，`validate_model` 或文档解析都会先于读取分支拒绝。

2. **文档读取**：`contract_model.cpp:830-915` 新增 `rotational_torque` 分支（位于原 fall-through
   之前），键名与 `element_reader.cpp:354-390` 逐一对齐，写进一个 `ElementBlock`
   （`kind = ELEMENT_ROTATIONAL_TORQUE`，槽位 128/129/130/133/137，`ints`/`cached_parameters` 留零，
   力矩类参数乘 `length_scale_`）；存储为 `mb_cases/functions.hpp:431` 的
   `std::vector<ElementBlock> rotational_torques_`，`:280` 给只读访问器。

3. **文档入口追加**：`kernel_contract_run.cpp:445-457` 在 `build_model` 之后对块调用既有
   `read_element_blocks`（曲线数组按 `count * kElementCurveSlots` 给零项），错误经既有 `fail` 路径。
   这些块**不**进 `AxleInput::elements`，所以互斥守卫看到的仍然只有逐族数组。

4. **验收测试**：新增 `packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`
   （7 项，`7 passed`），用 `run_contract` 跑同一模型的两次：含族与移除族。

## 验证

`raw/run_log.md` 逐条列出命令与**实测**退出码；摘要：

| 命令 | 结果 |
|---|---|
| `build_suspension_kernel.py` | exit 0 |
| `build_axle_native.py` | exit 0 |
| `pytest tests/cases/test_rotational_torque_document.py` | 7 passed, exit 0 |
| `pytest packages/suspension_contracts/tests`（独立） | 32 passed, exit 0 |
| `pytest packages/suspension_kernel/tests`（独立） | 41 passed, exit 0 |
| `dynamic_hash_sentinel.py --check` | OK，26 artifacts，sha256 与冻结值一致，exit 0 |
| `git status --short -- packages/suspension_multibody/tests/data/` | 空，exit 0 |
| `ruff check .` | All checks passed，exit 0 |
| `ty check .` | All checks passed，exit 0 |

**两次运行的实测数字**：受力体末态 `omega_y` 含族 `1.499999999999999` vs 对照 `3.0`（差
`1.500000000000001`）；反力体 `1.5000000000000002` vs `0.0`（差 `1.5000000000000002`），且两端相等。
容差 `0.5 rad/s` = 解析解 `1.5 rad/s` 的三分之一，推导与理由见 `raw/document_route_evidence.md` 第 3 节。

**未改 ABI 的自证（注意判据的陷阱）**：`git diff --stat` 对 `mb_config/version.hpp` 与
`kernel/native.py` **不是空输出**——那两行是 **p2-02 的未提交改动**（ABI 16/31 → 17/32 及其
Python 镜像），留在本工作区里。所以本行改用三条互相独立的证据：

1. 全部写操作的目标清单里没有这两个路径；
2. 两文件 mtime 都停在 `2026-10-01 08:19:29`（本行会话开始之前），本行 C++ 改动在 `09:02-09:04`；
3. 两者当前内容仍是 p2-02 的 `17 / 32`（`native.py` 为 `17 / 32 / 1`），sha256 见 `raw/run_log.md` §8。

另：`contract_model.cpp`、`kernel_contract_run.cpp`、`mb_cases/functions.hpp` 的
`git diff -U0` 删除行数为 0。

## Done-When 对照

- [x] 最小文档含 `rotational_torque` 时 `validate_model` 通过且 `run_contract` `status == "success"`
- [x] 移除该族重跑仍 `success`，两次受力体末态角速度差 `1.5 > 0.5` 容差（证明被求值）
- [x] `contract_registry_size(1) == 11`，`contract_selftest` 通过（`OK (45 checks)`）
- [x] 未改 ABI 版本常量、未动 `build_model.cpp` 守卫、未重录任何基线
- [x] 既有族文档往返逐项不变（五族，块名集合与每元素宽度未变）
