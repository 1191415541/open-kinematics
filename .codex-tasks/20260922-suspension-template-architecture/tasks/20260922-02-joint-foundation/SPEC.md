# 子任务 02：统一副底座并拆除作者层的副类型截断

## 目标

让「任意总成可用任意运动副」成为事实，且只存在一份副定义。

1. 新建 `suspension_multibody/joints/` 包，提供**一份**副定义表：作者层类型名 ↔ 内核契约名 ↔ 约束行数 ↔ 参数需求。表覆盖 8 种真实副（`spherical`/`revolute`/`fixed`/`prismatic`/`universal`/`cylindrical`/`inplane`/`constant_velocity`）与 2 种驱动坐标（`driven_translation`/`driven_rotation`），且两组分属不同命名空间（驱动坐标不是连接约束）。
2. 拆除 `cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 截断（现状只映射 `BallJoint`/`RevoluteJoint`/`PrismaticJoint` 三种，其余抛 `NativeKcError`），改为从统一表取，并补齐 `universal`/`cylindrical`/`convel`/`inplane`/`fixed` 的文档编码分支（含 `axis_a_secondary`/`axis_b_secondary` 与 `constant_velocity_angle_target`）。
3. 把 `constant_velocity → convel` 的改名从两处（`cases/axle_dynamic.py:292`、`cases/vehicle_dynamic.py:79`）收口到统一表的一处。
4. 装配期副可用性验证：某副需要轴而几何不足以定义轴时（轴退化为零向量、缺次轴），在**装配期**报错并点明缺什么，不留给内核拒绝。

## 非目标

- 不改 C++ 内核。`cpp/src/contract/contract_registry.cpp:23-30` 的 10 项副表已完备，本任务只做**镜像一致性测试**。
- 不新增副类型，不改契约文档格式。
- 不改 `symmetric_proxy` 拓扑的副自动生成规则（哪些角色用哪种副仍由现有逻辑决定）；本任务只让「用户显式声明的任意副」能通过编码。
- 不改 `symmetric_proxy`/`explicit` 的语义，不改装配几何。

## 约束

- **现役 3 种副（`spherical`/`revolute`/`prismatic`）的文档编码必须逐字节不变**——这是本步零基线风险的前提，也是验收判据。
- 副表与内核行数表的一致性必须由测试强制：行数来源是 `contract_registry.cpp:23-30` 的镜像，两边漂移是静默的，因此必须有断言把两边钉在一起。
- 驱动坐标（`driven_translation`/`driven_rotation`）虽与副共用文档的 `joints` 数组（`cases/kc_quasi_static/contract.py:50-51` 有注释说明），但**不得混入副表**，否则「8 种副」这个口径会失真。
- 不引入新依赖。
- 父 `EPIC.md` 的 ABI 冻结（七符号 15/30/1/1）与分层 DAG 保持绿。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/joints/**`
  - `packages/suspension_multibody/src/suspension_multibody/cases/kc_quasi_static/contract.py`
  - `packages/suspension_multibody/src/suspension_multibody/cases/axle_dynamic.py`
  - `packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py`
  - `packages/suspension_multibody/src/suspension_multibody/preparation/assembly/front_axle.py`（仅副可用性验证与编码路径，不拆子系统）
  - 新增测试 `packages/suspension_multibody/tests/joints/**`
  - `packages/suspension_multibody/src/suspension_multibody/__init__.py`（仅当需要导出新包时可加，且不得改变现有导出）
- 只读：`packages/suspension_kernel/cpp/src/contract/contract_registry.cpp`、`cpp/include/mb_joint/**`、父 `EPIC.md`、`SUBTASKS.csv`。
- 不写：C++ 内核源码；`preparation/assembly/types.py`（保留给 04）；`templates/`（03）；`outputs/`（07）；父级计划文件（归主代理）。

## 依赖

- 前置：01（冻结基线与现状清单；`joint_inventory.md` 给出截断点的 file:line 证据）。
- 后续：03 的模板要能声明任意副，依赖本任务的统一表；04 拆子系统时副编码已统一。

## 验收标准

1. 副表覆盖 8 种真实副 + 2 种驱动坐标，且副表与内核 `contract_registry.cpp` 的行数由测试逐项断言一致。
2. 任意总成可声明任意副：有测试证明 kc 能发出 `universal`/`cylindrical`/`convel`/`inplane`/`fixed` 五种此前被拒的副，且内核接受并成功运行。
3. 现有 3 种副（spherical/revolute/prismatic）的文档编码逐字节不变（有测试或字节门证明）。
4. `constant_velocity → convel` 改名只存在于一处。
5. 五个负例必须失败且报错信息点名原因：
   - 未知副名；
   - 需要轴的副缺轴；
   - 轴退化为零向量；
   - 副表行数与内核不符（人为篡改镜像后测试须失败）；
   - 驱动坐标混入副表。
6. 父级门禁保持：`check_module_layering.py --strict --final` 退出 0；架构测试通过；`dynamic_hash_sentinel`/`kc_parity`/`case_parity` 三门保持绿（未重录任何基线）。

## 验证协议

每步单独构建并跑门禁，禁止积累全任务 diff 后才验证：

1. 副表落地后：跑新增 `tests/joints`，断言表与内核一致。
2. 拆 kc 截断后：跑 `build_axle_native.py` + `kc_parity_check.py --check` + `case_parity_check.py` + `dynamic_hash_sentinel.py --check`，确认默认路径未变。
3. 收口改名后：跑 `tests/axle_dynamics` 与 `tests/vehicle` 确认编码一致。
4. 加装配验证后：跑负例测试，确认五个负例全部失败。
5. 收尾：`check_module_layering.py --strict --final`、kernel/contracts/multibody 三套 pytest、ruff、ty、`git diff --check`。

本任务**不应导致任何基线重录**；若发现必须重录，立即停止并上报，因为这违反 EPIC 的冻结约束（纯结构阶段要求动态数组逐位一致）。
