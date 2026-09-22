- 任务：建立模板与连接点的双列数据模型
- 形态：single-full（Epic 子任务）
- 进度：0/9 步骤 TODO，尚未实施
- 当前：未开工。前置 02（统一副表）未完成，模板的 joint 列需引用统一表。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-03-template-model/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 02 已完成，`suspension_multibody/joints/` 的统一副表可用，且与内核行数表一致性测试通过。
- `preparation/assembly/types.py` 未被 04 占用（本步与 04 的写范围冲突，须等 04 之前或之后串行）。
- 父 `EPIC.md` 三层结构与 G2/G3 口径有效。

## 本任务的现状事实（制定计划时实测，实施时复核）

现役 K/C 映射（`preparation/assembly/front_axle.py`，`symmetric_proxy` 拓扑）实测结果，是内置模板的数据来源：

| 连接角色 | K 模式 | C 模式 | 代码位置 |
|---|---|---|---|
| 上臂内前（uca inner_front） | `RevoluteJoint`，轴 = upper_front→upper_rear，进 `constraints` + `ideal_constraints` | `BallJoint`（仅 `ideal_constraints`）+ 零刚度占位 `BushingElement` | `:694-740` |
| 上臂内后（uca inner_rear） | **无约束**（轴已由内前的旋转副穿过两点定义） | `BallJoint` + 零刚度占位衬套 | `:717-739` |
| 下臂内前（lca inner_front） | `RevoluteJoint` | `BallJoint` + 占位衬套 | `:759-805` |
| 下臂内后（lca inner_rear） | **无约束** | `BallJoint` + 占位衬套 | `:782-804` |
| 上臂外 → upright | `BallJoint` | **同 K** | `:816-837` |
| 下臂外 → upright | `BallJoint` | **同 K** | `:816-837` |
| 拉杆内 ← rack | `BallJoint` | **同 K** | `:846-848` |
| 拉杆外 → upright | `BallJoint` | **同 K** | `:849-851` |
| 齿条 ← 车身 | `WeldJoint` 或 `PrismaticJoint` | **同 K** | `:881-903` |
| 用户声明的 `model.bushings` | **被忽略** | `BushingElement`（刚度取 `spec.stiffness`） | `:602-622` |

实测计数（`benchmark_axle.json`，`symmetric_proxy`）：K 模式 `constraints=13`、`ideal_constraints=13`、`bushings=0`、文档 `elements=0`；C 模式 `constraints=9`、`ideal_constraints=17`、`bushings=8`、文档 `elements=8`（8 条刚度范数全为 `0.0`）。

**这正是用户裁决「C 模式零刚度占位衬套改用模板默认属性」与「一个点同时建立运动副和衬套」的落点**：模板的 bushing 列要带真实属性引用，取代 `front_axle.py:737`/`:802` 的 `stiffness=np.zeros((6,6))`。

## 命名冲突提醒

`schema/model.py:173` 的 `topology` 已被占用，语义是「怎么描述轴」（`symmetric_proxy`/`explicit`）；本步引入的「模板名/悬架类型」是另一个概念，**不得复用该字段名**。建议用 `template`，并在代码注释写明两者区分：`topology` 说怎么描述，`template` 说描述的是哪种架构。

## 本步的边界（务必守住）

- 只建数据结构与注册表，**不接线到装配**：`build_front_axle` 本步不读模板，因此现有基线不应有任何变化。
- 不写 `preparation/assembly/types.py`（04 的写范围）。
- 不实现属性文件加载（06）与输出求值（07）。

## 下一步

等 02 完成后从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 03 行状态由主代理回填。
