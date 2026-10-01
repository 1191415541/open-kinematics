# PROGRESS：p2-08 内核 rotational_torque 驾驶员需求通道与滑移判定

- 父 Epic：`20260929-multibody-evolution-p2-p5`
- 形态：`single-full`
- 状态：**DONE**
- 来源：`code-reviewer` 裁决新增（delegation `ea64ca92`）

## 恢复块

1. `任务:` 内核 rotational_torque 族接入驾驶员需求通道并加滑移判定
2. `形态:` single-full
3. `进度:` 5/5 步骤 DONE
4. `当前:` 已关闭。内核两条力律接收 `SampleInput` 并按逐轮胎驾驶员信号求值；抱死态由滑移分支保持满需求力偶
5. `文件:` `packages/suspension_kernel/cpp/{include,src}/**` 共 10 处 + `packages/suspension_multibody/src/suspension_multibody/{compilation/element_blocks.py,modeling/primitives/elements.py}` + 本目录 `raw/demand_channel.md`
6. `下一步:` 无。下游 p2-05（契约表改发归一化需求）与 p5-04（ABS 闭环）按各自 SPEC 开工

## 交付

见 `raw/demand_channel.md`（282 行）：改动清单 14 项、两条力律改后原文、读取端校验、探针完整输出、9 条命令与退出码、ABI 常量实读。

**核心结果**（实跑）：

- `packages/suspension_kernel/tests` → **45 passed**（新增 4 条，既有 8 条未删未弱化）
- 探针 `eval locked tau_a 400 tau_b -400`——`ω` 恰为 0 而滑移 0.8 时力偶是满需求幅值，**不再是零**
- `dynamic_hash_sentinel --check` → combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`，**与 p2-01 起点逐字节一致**
- ABI 常量 **未动**（仍 17/32/1）；`git status --short -- packages/suspension_multibody/tests/data/` 为空

## 边界

未改 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量；未碰 `preparation/vehicle_dynamic.py`、`cases/vehicle_dynamic.py`、
`templates/*`、`subsystems/brake.py`、`subsystems/drive.py`（分属 p2-05 与 p2-04）；未改任何既有元素族的输入形式。
