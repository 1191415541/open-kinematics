# PROGRESS：p2-06 分布式转向通道与转向分配器

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-06`

## Session Start

- **Date**: 2026-10-02
- **Task name**: p2-06-steering-channels
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-06-steering-channels/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步，全部 DONE）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #6 — 全部 6 步完成，门禁全绿
- **Current status**: DONE
- **Last completed**: `TODO.csv` 6/6 DONE；`raw/` 六份证据齐备
- **Current artifact**: `raw/{steering_restriction_removed,steering_channels,allocator_assertions,vehicle_kc_boundary,two_channel_assembly,run_log}.md`
- **Key context**:
  - 依赖 p2-05（力矩段）已 `DONE`；**p4-04（轮胎拒绝段）在本行之后**（`EPIC.md` 行 217），本行未触碰该段。
  - 本行是阶段二收尾行、G2 的实现行（`EPIC.md` 行 81）。
  - `steering` 保持必填单例（裁决 `1331b13d` 方案 B），附加通道走 `exclude=True` 的 `steering_channels`；`model_dump(mode="json")` 的 12 键 / 11835 字节 / `75cb96f5…03fd9` 三项逐项未变。
  - 角色名集合归 p4-02；本行**未增删任何角色名**，未触碰 `templates/roles.py`、`templates/builtin.py`。
  - 分配器落点 = `preparation/steering_allocator.py`（preparation 层，只 import 标准库，几何量由调用方给）。

## 交付清单

### 新增文件

| 文件 | 行数 |
|---|---|
| `packages/suspension_multibody/src/suspension_multibody/preparation/steering_allocator.py` | 322 |
| `packages/suspension_multibody/tests/preparation/test_steering_allocator.py` | 311（24 用例） |
| `packages/suspension_multibody/tests/preparation/test_steering_channels.py` | 311（8 用例） |
| `packages/suspension_multibody/tests/schema/test_vehicle_steering_channels.py` | 290（14 用例） |

### 修改文件（`git diff --numstat`）

| 文件 | +/- |
|---|---|
| `.../schema/vehicle.py` | +106 / −0 |
| `.../preparation/vehicle_dynamic.py` | +414 / −22 |
| `.../authoring/vehicle.py` | +24 / −8 |
| `.../cases/vehicle_kc.py` | +31 / −11 |
| `.../cases/vehicle_dynamic.py` | +16 / −5 |
| `.../schema/__init__.py` | +2 / −0 |
| `packages/suspension_contracts/.../contracts/assembly.schema.json` | +6 / −0 |
| `packages/suspension_multibody/tests/authoring/test_vehicle_assembly_documents.py` | +35 / −15 |

### 契约 schema 的相邻改动（必要，已登记）

`packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json` 的 `vehicle.properties` 增 `steering_channels` 数组属性：该段 `additionalProperties: false`，不加则新键在加载期被拒。**只加 `steering_channels`，未加 `allocation_law`**（spec §3 把 `vehicle_model_from` 的范围限定为 `steering_channels`；测试用 `model_copy`/`model_validate` 设置 law）。`contracts` 32 用例全绿。

## 验收结果

| 判据 | 命令 | 退出码 | 结果 |
|---|---|---|---|
| 解除两层限制 | `grep -rn "must be true" packages/suspension_multibody/src --include=*.py` | **1** | 零命中 |
| 单通道向后兼容 | `python <scratch>/dump_evidence.py` | **0** | 12 键 / 11835 字节 / hash 三项一致（含 +law、+channel 两种声明） |
| 分配律数值 | `pytest tests/preparation tests/schema/test_vehicle_steering_channels.py -q` | **0** | `46 passed` |
| 两通道 study | `python <scratch>/final_two_channel.py` | **0** | `run.status == "success"`，每通道各有读数 |
| 既有快速集 | `pytest tests/authoring tests/subsystems tests/cases -q` | **0** | `385 passed in 58.37s` |
| 契约包 | `pytest packages/suspension_contracts/tests -q` | **0** | `32 passed` |
| 冻结基线 | `git status --short -- packages/suspension_multibody/tests/data/` | **0** | 输出为空 |
| 数值门 | `dynamic_hash_sentinel.py --check` | **0** | 26 artifacts 逐字节一致 |
| 家族平价 | `case_parity_check.py`（无参数） | **0** | 8 families PASS |
| 性能门 | `kc_perf_gate.py --check` | **0** | 在预算内（k-100 x0.806 / c-66 x0.787） |
| 分层 | `check_module_layering.py --strict --final` | **0** | 无跨聚合、无环 |
| 边界面 | `legacy_surface_gate.py --check` | **0** | findings 0 |
| 架构测试 | `pytest tests/architecture -q` | **0** | `147 passed` |
| 静态检查 | `ruff check .` / `ty check .` | **0** / **0** | `All checks passed!` |

## 未达成（如实登记）

1. **4WS 高速端未跑求解器**。`initial_forward_speed_mps > 0` 时内核返回 `status 7: initial velocity violates velocity constraints`（内核侧既有整车初速限制，拒绝点在 `suspension_kernel/cpp/src/abi/kernel_abi.cpp:276`；**已实测同夹具的单通道模型在 `speed=0.5/2.0` 下报同一错误、`speed=0.0` 为 `success`**，故与转向通道无关，本行不去绕）。高速同向由准备层的 per-channel target 表与 `steering_allocation` 记录背书；低速与 `direct` 两端都跑通到 `run.status == "success"`。原文见 `raw/two_channel_assembly.md` §5。
2. **`ackermann` / `multi_axle_follow` 未在该夹具整车路径上落地**。夹具前后轴 `WHEEL_CENTER.x` 实测相同（均 `0.0`），`wheelbase_mm` 与 `distance_to_reference_mm` 皆为 0，两条律按名拒绝。它们的数值断言由 `tests/preparation/test_steering_allocator.py` 用显式几何量覆盖（`L=2800`、`t=1600`、`Li=4200/5600`）。**未**声称这两条律已在一台两通道整车上跑通。
3. 两通道的 `reaction_body` 具体体名未断言（只断言形状），声明里未给 `actuator_body`/`actuator_reaction_body`，走默认派生路径。spec 未要求该项断言。
4. `.codex-tasks/**` 的行尾：任务书写「是 CRLF」，实测本行 `SPEC.md` / `TODO.csv` / `PROGRESS.md` 与 p2-05 的 `raw/*.md` **均为 LF**（`CRLF=0`）。新写的六份 `raw/*.md` 按 LF 写入，与相邻文件一致；`TODO.csv` 保持 LF 并逐行改写（`CRLF=0`、8 列结构不变）。此为实测口径差异，非未达成项，登记以免被读成违规。

## Final Summary

六步全部完成并交证据。两层后轮转向限制已解除（`--include=*.py` 下 `must be true` 零命中）；`VehicleModel` 保留必填的 `steering` 单例并新增 `exclude=True` 的 `steering_channels` 与 `allocation_law`，单通道 dump 的键集合 / 字节数 / canonical_hash 三项与改造前逐项相同；`preparation/steering_allocator.py` 实现 `direct` / `ackermann` / `four_wheel_steer` / `multi_axle_follow` 四条律并有独立重写公式的数值断言；`cases/vehicle_kc.py` 的准备期删除改为在源头不请求执行器（只增不删，与阶段一 05 同口径、不同 Epic、写范围不相交）；两通道总成装配成功、`run.status == "success"`、每通道各有可读转角。冻结基线未写（`git status` 为空），未新增 skip/xfail，未用 `-k`/`--deselect`，未触碰 `mb_config/version.hpp`、`kernel/native.py` ABI 常量、`templates/roles.py`、`templates/builtin.py`、`subsystems/rig_link.py`、`rigs/**`。

## 证据落点

- `raw/steering_restriction_removed.md` — 两层解除的改前/改后原文 + 两条 grep 的退出码与输出 + 两条既有断言反转（改前 → 改后 → 理由）
- `raw/steering_channels.md` — schema 改前/改后字段形状对照 + 单通道 dump 三项实测对照表
- `raw/allocator_assertions.md` — 四条律公式 + 每条断言的 `测试文件:行` 与断言表达式原文 + 数值对照表
- `raw/vehicle_kc_boundary.md` — `cases/vehicle_kc.py` 改前/改后原文对照 + 与阶段一 05 的关系点名
- `raw/two_channel_assembly.md` — 通道声明片段 + 逐通道实体清单 + study 实跑命令与退出码 + 逐通道转角读数
- `raw/run_log.md` — 每条命令与退出码
