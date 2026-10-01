# SPEC：p2-10 内核 normalized torque demand 独立缓冲与旧 N·m 路由隔离

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-10`。
> 形态：`single-full`。来源：`code-reviewer` 裁决 `5e01b75d`（p2-09 开工前的实测缺口）。

## Goal

把 `SUBTASKS.csv` 中 `p2-10` 的 `acceptance_criteria` 拆成下面 6 条，逐条可判定：

1. **独立存储**：`ContractCase` 增 `wheel_demand`/`brake_demand` 两表；`Model` 增两个注册期 `const double*`
   （照 `vehicle_brake_torque` 先例）；`SampleInput` 增两个独立 vector。旧 `wheel_torque`/`brake_torque` 的
   类型、位置与语义（N·m）一字不变。
2. **稳定缓冲与生命周期**：`kernel_contract_run.cpp` 在 case 循环外分配两张 demand 缓冲，地址不变，
   每 case `memcpy` 覆盖；未声明的表显式清零，**不得跨 case 泄漏**。
3. **插值分流**：`interpolate_input` 把两张新表填进两个新目标；旧路径仍由 `in.wheel_torque` 与
   `model.vehicle_brake_torque` 填 `torque`/`brake_torque`。
4. **消费端分流**：`anti_roll.cpp` 标量路径与 `directional.cpp` 的 rotational-torque 路径改读新 demand；
   `drive_brake.cpp` 与 `directional.cpp` 的旧直给路径**只**读旧 N·m 表（有断言）。
5. **不改 ABI**：`AxleInput`/`VehicleInput`/`AxleOutput`/`VehicleOutput`/`ElementBlock`/ABI 入口与版本常量
   均不动；版本仍 `17/32/1` 且 `test_kernel_abi_version_single_source.py` 绿。
6. **零回归**：未声明 demand 的整车路径保持既有 baseline 与 dynamic hash 逐字节。

## 写范围（允许改的路径）

照裁决给出的清单：

- `packages/suspension_kernel/cpp/include/mb_cases/functions.hpp`
- `packages/suspension_kernel/cpp/include/mb_model/types.hpp`
- `packages/suspension_kernel/cpp/src/input/kernel_input.cpp`
- `packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp`
- `packages/suspension_kernel/cpp/src/cases/contract_model.cpp`（本行**未改**：`fill_case` 无需改动，
  因为两张新表由运行器自己持有，不经 `VehicleInput`）
- `packages/suspension_kernel/cpp/src/cases/vehicle_dynamic.cpp`（role 表边界；见「与 p2-09 的分工」）
- `packages/suspension_kernel/cpp/src/element/anti_roll.cpp`
- `packages/suspension_kernel/cpp/src/element/directional.cpp`
- `packages/suspension_kernel/cpp/tests/contract_selftest.cpp`（本行**未改**：`ContractCase` 的两个新字段
  是尾部追加的 `std::vector`，自测不计其数）
- `packages/suspension_kernel/tests/fixtures/rotational_torque_probe.cpp`
- `packages/suspension_kernel/tests/test_rotational_torque.py`
- 本目录 `raw/`

## 禁止触碰

- **ABI 结构**：`cpp/include/mb_input/types.hpp`、`cpp/include/abi/functions.hpp`、
  `mb_config/version.hpp`、`packages/suspension_multibody/src/suspension_multibody/kernel/native.py`。
- **Python 生产代码**：本行是纯内核行，`packages/suspension_multibody/src/**` 不得改
  （装配侧接线归 p2-09）。
- **旧 N·m 路径的语义**：`drive_brake.cpp` 的施加逻辑不得改。
- **`axle_dynamic` 家族 role 表**：归 p2-09 的装配/product 路径接线范围。
- **基线不得重录**；**不得新增 skip/xfail**；不得用 `-k`/`--deselect`。
- 其它子任务目录、`EPIC.md`、`SUBTASKS.csv`（父表由主代理登记）、父 `PROGRESS.md`——不得改。

## 依赖与时机

- `depends_on = p2-08`（需求通道先存在，才有「它读哪张表」的问题）。
- **不依赖 p2-09**（避免 p2-09 ↔ p2-10 循环）；`p2-09.depends_on` 加上本行。
- 上游：p2-02（元素族 + ABI）、p2-08（需求通道与滑移判定）。

## 判据与证据落点

逐条对应 `SUBTASKS.csv` 的 `acceptance_criteria` → `raw/demand_plumbing.md`。

| 判据 | 做什么 | 看什么 | 证据 |
|---|---|---|---|
| (1)(2)(3) 存储/生命周期/插值 | 改五处 C++ 与探针 | 四目标的逐字读数（`isolate none` / `isolate both`） | `raw/demand_plumbing.md` §4.1 |
| (4) 消费端分流 | 探针 `decoupled` | `eval decoupled tau_a 400 tau_b -400`（读旧表会得 −1000） | §4.2 |
| (5) 不改 ABI | 读 `mb_config/version.hpp` 与 `kernel/native.py` | 仍 17/32/1；单一真源门绿 | §2、§3 |
| (6) 零回归 | 哨兵 + case_parity + kc_perf + 快速集 | 逐字节/逐项一致 | §3 |
| 端到端 | 真实整车文档经 `simulation.run_request` 提交 | 归一化 role 成功；同通道混用与越界按名拒绝 | §5 |

## Done-When

- [x] 五个 C++ 文件按独立缓冲改造完成，编译通过（`build_suspension_kernel.py` 退出码 0），
      并跑过 `build_axle_native.py`。
- [x] `packages/suspension_kernel/tests` **47 passed**（起点 45，+2 为本行新增；既有 45 条未删未弱化）。
- [x] 两条新断言可判别「旧表不再被需求通道读取」与「两制在采样层不混」（逐字读数入证据）。
- [x] `dynamic_hash_sentinel --check` 逐字节一致；`case_parity_check.py`（无参数）8 families；
      `kc_perf_gate --check` 在预算内；快速集 1201 passed / 1 xfailed。
- [x] ABI 常量未动（17/32/1）；`tests/architecture` 147 passed；`ruff`/`ty` 全仓 exit 0；
      三个架构门脚本 exit 0。
- [x] `git status --short -- packages/suspension_multibody/tests/data/` 为空。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_kernel/tests -q -p no:cacheprovider && \
uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q && \
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
