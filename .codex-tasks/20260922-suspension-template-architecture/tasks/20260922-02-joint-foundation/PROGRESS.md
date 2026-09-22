- 任务：统一副底座并拆除作者层的副类型截断
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 01（冻结基线与现状清单）未完成，未冻结判据前不得开始。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-02-joint-foundation/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 01 已完成并产出 `raw/baseline_commands.md`（逐条命令与退出码）与 `raw/joint_inventory.md`（8 种副的现状落点与截断点 file:line）。
- 父 `EPIC.md` 的冻结约束有效：ABI 七符号与版本（15/30/1/1）不变；`check_module_layering.py --strict --final` 保持绿；纯结构阶段要求动态数组逐位一致。
- 用户裁决 D4 允许重录基线，**但本子任务不应重录任何基线**；若发现必须重录，停止并上报。

## 本任务的现状事实（制定计划时实测，实施时复核）

- 内核副注册表已完备：`packages/suspension_kernel/cpp/src/contract/contract_registry.cpp:23-30` 声明 10 项——`spherical`3 / `revolute`5 / `fixed`6 / `prismatic`5 / `universal`4 / `cylindrical`4 / `inplane`1 / `convel`4 / `driven_translation`1 / `driven_rotation`1。
- 装配层已支持 8 种副：`preparation/assembly/types.py` 定义 `PointCoincidence`/`BallJoint`/`WeldJoint`/`DistanceConstraint`/`RevoluteJoint`/`UniversalJoint`/`ConstantVelocityJoint`/`CylindricalJoint`/`InPlaneJoint`/`PrismaticJoint`；`_explicit_constraint`（`front_axle.py:271-315`）已能构造全部 8 种。
- schema 已允许 8 种：`schema/model.py:100-109` 的 `IdealJointSpec.kind`；次轴字段 `axis_a_secondary`/`axis_b_secondary` 与 `constant_velocity_angle_target` 在 `:116-119`。
- **截断点唯一**：`cases/kc_quasi_static/contract.py:44-48` 的 `_JOINT_KINDS` 只映射 `BallJoint`→`spherical`、`RevoluteJoint`→`revolute`、`PrismaticJoint`→`prismatic`，其余抛 `NativeKcError`。且编码处（`:110-111`）只有 `RevoluteJoint`/`PrismaticJoint` 会写 `axis_a`/`axis_b`。
- **改名重复两处**：`cases/axle_dynamic.py:292`、`cases/vehicle_dynamic.py:79` 各写了 `"convel" if joint.kind == "constant_velocity" else joint.kind`。
- 驱动坐标与副共用文档 `joints` 数组，`cases/kc_quasi_static/contract.py:50-51` 有注释说明「driven coordinate 是规定的自由度，不是装配意义上的副」。

## 步骤顺序（固定）

按 `TODO.csv` 顺序推进，每步单独跑门禁：

1. 清点（只读）→ 2. 建表 → 3. 一致性测试 → 4. 拆 kc 截断 → 5. 收口改名 → 6. 装配期验证 → 7. 五个负例 → 8. 默认路径逐位不变 + 门禁全集 → 9. 收尾。

第 4 步是**唯一会改变既有行为的步骤**（kc 从「3 种副可用」变为「8 种可用」），但正常输入的默认路径不应改变；第 8 步是这一点的判据。

## 下一步

等 01 完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 02 行的状态由主代理回填。
