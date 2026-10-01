# SPEC：p2-08 内核 rotational_torque 驾驶员需求通道与滑移判定

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-08`。
> 形态：`single-full`。**本行由 `code-reviewer` 裁决新增**（裁决依据：G1 与 D1 要求按实时驾驶员信号与打滑状态求值，
> 而 p2-02 只交付元素族、`demand` 硬编码 1.0，p2-04 把该通道推回 p2-02，p2-05 的 SPEC 又禁止其触碰内核——缺口无人负责）。

## Goal

1. **需求通道**：`assemble_rotational_torque_forces` 与 `external_force_rotational_torque_directional`
   都接收 `const SampleInput&`，力偶幅值为 `min(stiffness * demand, max_torque)`，
   其中 `demand` 按元素块的整数槽从工况的逐轮胎信号取（0=单位需求、1=`wheel_torque`、2=`brake_torque`）。
2. **滑移判定**：力元读 `State::tire_sx` 的实时纵向滑移；`ω` 在 `kEps` 内而滑移显著时，
   力偶保持**满需求幅值**、方向随滑移——修掉「抱死时输出零制动力矩」。
3. **三态断言**：静止/倒车/抱死各有断言，抱死态的断言必须是「ω≈0 时仍施加满需求力矩」。
4. **两条路径一致**：标量与方向导数路径在同一状态下力偶相同；重新签名后注册点与调用点都补传 `input`。
5. **读取与校验**：整数槽经 `element_reader.cpp` 读入模型；非法源索引与「有源无轮胎」都明确失败并由名字报错。
6. **不改 ABI 版本常量**：保持 17/32/1（两个整数槽取自 `kElementIntBlockSize = 16` 的既有空位），
   `test_kernel_abi_version_single_source.py` 保持绿。

## 写范围（允许改的路径）

- `packages/suspension_kernel/cpp/include/mb_input/types.hpp` 的整数槽枚举
- `packages/suspension_kernel/cpp/include/mb_model/enums.hpp`（需求源枚举）
- `packages/suspension_kernel/cpp/include/mb_model/types.hpp`（`RotationalTorque` 的两个字段）
- `packages/suspension_kernel/cpp/include/mb_element/functions.hpp`（两条签名）
- `packages/suspension_kernel/cpp/src/force/external_vector.cpp`（调用点）
- `packages/suspension_kernel/cpp/src/element/anti_roll.cpp`、`cpp/src/element/directional.cpp`
- `packages/suspension_kernel/cpp/src/assembly/element_reader.cpp`
- `packages/suspension_kernel/tests/fixtures/rotational_torque_probe.cpp`、`packages/suspension_kernel/tests/test_rotational_torque.py`
- `packages/suspension_multibody/src/suspension_multibody/compilation/element_blocks.py`（槽编码）
- `packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py`（参数的两个字段）
- 本目录 `raw/`

## 禁止触碰

- **`mb_config/version.hpp` 与 `kernel/native.py` 的版本常量**（冻结约束：只有 p2-02 可改；本行按 p2-07 先例不改）
- `preparation/vehicle_dynamic.py` 与 `cases/vehicle_dynamic.py`（归 p2-05）
- `templates/roles.py`、`templates/builtin.py`、`subsystems/brake.py`、`subsystems/drive.py`（归 p2-04）
- 既有元素族的任何输入形式与装配顺序
- 不得重录任何基线；不得新增 skip/xfail

## 依赖与时机

- `depends_on = p2-02`（元素族与 ABI 由它交付）
- p2-05 的 `depends_on` 已加上本行（`p2-04;p2-07;p2-08`），p2-06 仍为 `p2-05`

## 判据与证据落点

全部落 `raw/demand_channel.md`（含改动清单、两处力律原文、读取校验原文、探针完整输出、全部命令与退出码、ABI 常量实读）。

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_multibody/tests/architecture -q && \
uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final && \
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```

## Done-When

- [x] 两条路径都接收 `SampleInput`，`demand` 不再硬编码；幅值为 `min(stiffness * demand, max_torque)`
- [x] 需求为零时施加零力偶（`eval no_demand tau_a 0 tau_b 0`）
- [x] 滑移分支：`ω≈0` 而滑移 0.8 时力偶为满需求幅值（`eval locked tau_a 400 tau_b -400`）
- [x] 三态（静止/倒车/抱死）各有断言；既有 8 条断言一条未删未弱化
- [x] 读取端两条校验（非法源、有源无轮胎）
- [x] 版本常量仍 17/32/1；`dynamic_hash_sentinel --check` 逐字节一致
