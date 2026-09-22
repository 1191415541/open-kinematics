# 子任务 05：接管 Python 元件报告事实、静轮荷辅助求解与凝聚等价性登记

## 目标

让 Python 侧的元件载荷报告、静轮荷与焊接体凝聚都建立在 native 事实上，按三步顺序推进，禁止一次积累全任务 diff：

1. 凝聚与模型映射：**（2026-09-21 用户裁决 A1 修订、2026-09-22 裁决 A3 取代）** 生产路径**不凝聚**——焊接体保持独立 body，weld 以 `WeldJoint` → native `kind="fixed"` 关节（6 行）承接；融合实现保留为 `preparation/assembly/vehicle.py` 的 `_fuse_welded_bodies`，由 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复。须有等价性测试与体 ID 映射/回退开关登记；`vehicle_dynamics_baseline/sha256.json` 按新路径的 native 结果经授权重录。原「把计算性焊接体凝聚迁入 mb_assembly」与 A1 的「凝聚保留 Python 作者层」两种口径均**已作废**，理由与证据见 EPIC 的 A3 修订记录与 `raw/a3_condensation_switchover.md`。
2. native 输出与静轮荷：冻结并补齐元件报告通道；**2026-09-22 裁决 A2 修订**——`analysis/vehicle_physics.py:88` 的 `compute_static_wheel_loads` **不迁 native**（native 无静力求解 ABI 入口且 ABI 导出面冻结），保留 Python 最小范数算法本体与 service 调用语义，归属在 `results` 映射层收口，并登记 file:line + 阻断原因 + 解除条件。
3. decoder 接线：元件载荷报告从 native 结果读取，保留用户可见字段、单位、方向、参考点与失败证据。

必须冻结的对照范围：名称/ID、两端、坐标系、作用点、符号、单位、能量、active 状态；覆盖弹簧、阻尼、衬套、防倾杆、限位、垂向轮胎以及 K/C 两模式；身份映射覆盖凝聚后的实体。

现有事实：`elements/elastic.py:253` 的 `evaluate` 含弹簧力律，`api.py:742` 在内核返回的状态上调用 `evaluate_generalized_forces`（因此"Python 无本构"的结论不成立，不能整体搬到 report）；`results/{channels,decoder,timeseries,raw,axle,vehicle,common}.py` 是唯一事实通道解码归属。

## 非目标

- 不删除 Python 旧路径：`elements/elastic.py`、`elements/assembly.py` 的旧调用面保留到 06 切换。
- 不在 report 侧复算本构、不调用 native、不执行 preparation（07 的边界）。
- 不改变现有输入与输出通道顺序与单位；不把未实现的整车 Adams 对标标为通过。
- 缺字段时不得填零；不得用容差掩盖通道级差异。

## 约束

- 必须执行父级 `VALIDATION.md`（01 冻结）中的动态哈希逐位一致门、ABI 七符号门与契约版本兼容门；Python 报告来源切换单独使用冻结的通道级容差验收。
- 新增契约字段只在证明确有输出缺口时采用向后兼容**可选**扩展（默认关闭），并记录版本策略；契约版本允许 1→2，ABI 七符号与 ABI 版本号（15/30/1）不变。默认路径的 artifact 字节不得变化，`dynamic_hash_sentinel` 与 `case_parity_check` 的字节级门必须保持绿；不得为让可选通道生效而重录任何基线。
- 输出接管若揭示原 Python 与 native 力律差异，必须登记差异并阻断切换，在既有物理定义下消除，不得保留第二套力律。
- 顺序三步之间各自构建、各自跑门禁；每步失败不得带入下一步。

## 范围与文件归属

- 可写：multibody 的 results/**、api.py（只准备结果适配，不提前切换06负责的生产调用）、analysis/vehicle_physics.py（**A2 修订：只做归属收口与登记，不迁出算法**）、相关 tests；kernel 的 cpp/include 和 cpp/src 中 assembly/输出及所需契约接线、CMakeLists.txt；contracts 的必要兼容字段与测试。改动限本任务交付。
- 只读：`packages/suspension_kernel/MODULES.md`、`packages/suspension_multibody/src/suspension_multibody/elements/**`（06 才改）、父 `EPIC.md`、`VALIDATION.md`。
- 不写：`report`（07 新建）、`preparation`（06 负责）、父 `EPIC.md`、`SUBTASKS.csv`、`PROGRESS.md`。

## 依赖

- 前置：01（`VALIDATION.md` 通道级容差与数值门）、02（职责边界门禁）、03、04（C++ 侧 `mb_assembly`/`mb_force`/`mb_element`/`mb_tire` 与本构边界已冻结）。
- 后续：06 依赖本任务的通道证据才能切换 `api` 元件报告并删除 Python 本构；07 依赖本任务冻结的 results 解码边界。

## 验收标准

1. 元件报告字段与 native 通道逐项对照表覆盖名称/ID、两端、坐标系、作用点、符号、单位、能量、active 状态，且覆盖弹簧、阻尼、衬套、防倾杆、限位、垂向轮胎与 K/C 两模式。
2. 缺口按「向后兼容可选扩展（默认关闭）」由 C++ 输出补字段解决；`kernel` 与 `contracts` 测试及版本兼容门通过，无字段被填零；默认路径 artifact 字节不变。
3. **（A3 修订）** 生产路径不凝聚且 weld 以 native `kind="fixed"` 关节承接（6 行）；两条路径的**世界系质量性质等价**有实测证据（总质量与质心，全部 8 case）；回退开关 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 有测试；差异（布局 + 求解器残差级数值差）已逐 case 登记，不得表述为逐位相同。
4. **（A2 修订）** 静轮荷：保留 Python 最小范数算法与 service 调用语义（`analysis/vehicle_physics.py:88` → `vehicle/service.py:40` 唯一生产调用者），归属在 `results` 映射层收口，单独有测试；须有 file:line 保留登记与解除条件，**不判为「已迁 native」**。
5. 元件载荷报告经 `results` decoder 从 native 结果读取（可选通道启用时）；默认路径保持现状且两门为绿。通道顺序、单位、方向、参考点与失败证据保持。
6. 冻结的通道级容差验收逐通道通过；动态数组逐位一致、K/C parity、family parity、ABI 七符号门通过；**A3 前**未重录任何基线，**A3 后**唯一经授权重录 `tests/data/vehicle_dynamics_baseline/sha256.json`（8 个 case，按不凝聚路径的 native 结果），其余基线（`dynamic_hash_baseline.json`、`kc_baseline/`、`kc_perf_baseline*.json`）未改动。
7. Python 与 native 的力律差异全部登记。**（A1 修订）**：力元本构的删除以本任务的可选通道证据为前提；通道未启用期间旧 Python 路径保留，**不因此判本任务未达成**（按 EPIC G3 修订）。
8. 无 `report` 目录被提前创建，无 `preparation` 改动。

## 验证协议

```bash
uv run python packages/suspension_multibody/scripts/build_axle_native.py
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
uv run python packages/suspension_multibody/scripts/case_parity_check.py
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
uv run --package suspension-multibody pytest packages/suspension_multibody/tests/results packages/suspension_multibody/tests/physics packages/suspension_multibody/tests/vehicle packages/suspension_multibody/tests/cases packages/suspension_multibody/tests/architecture -q
```

三步顺序固定为"凝聚等价性登记 → native 输出与静轮荷归属收口 → decoder 接线"；每步结束都要同时满足构建成功、通道级容差验收与数值门通过。

每个 TODO 行的验收均包含本 SPEC 验证协议全集及父 VALIDATION.md 冻结门禁：构建并同步DLL、动态字节门、K/C与family parity、ABI七符号/契约版本门以及本任务的kernel/contracts/通道测试。CSV validation_command 为该步专项命令，不能代替全集。未执行全集不得将行置DONE。
