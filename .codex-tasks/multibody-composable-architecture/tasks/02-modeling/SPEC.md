# 02 基础模型、SI 与依赖解环

- 父任务：`.codex-tasks/multibody-composable-architecture/EPIC.md`
- 依赖：01（已 DONE）。Goal：G1;G5。
- 输入规格：`../../TASKS.md` 第 02 节（26-31 行）。
- 真源：状态以 `../../SUBTASKS.csv` 第 3 行为准。

## 目标

1. 把刚体/关节/marker/驱动/力元声明与空间代数归入低层 `modeling/primitives/`，使其**不导入** preparation/subsystems/rigs/simulation/kernel/report。
2. 定义稳定 `EntityId`、只读 `ModelFragment`、`Assembly`/`SimulationAssembly` 与基础 `Port` 值对象；本步不实现具体模板。
3. 明确 SI 单位、局部/世界变换与既有四元数约定；输入输出适配保留原外部单位（输入毫米制由边界适配一次）。
4. 冻结字段与序列化约定，供 03/04/06 消费；旧类型暂时转发仅用于迁移。
5. 交付独立进程任意顺序导入测试与导入边界门。

## 实测现状（本任务修正）

- 真实缺陷：`preparation/assembly/types.py` 是叶子类型模块，但导入它必然执行 `preparation/assembly/__init__.py`，从而拖入 `front_axle → elements → subsystems` 整条链。实测：`import suspension_multibody.preparation.assembly.types` 后 `sys.modules` 含 `elements`、`elements.elastic`、`preparation.assembly.front_axle`、`subsystems.assembly`。
- 该问题**当前被根 `__init__.py` 强制先导 `.api` 所掩盖**：单独 `import suspension_multibody.<sub>` 不会暴露循环；只有「先导入低层模块」的路径才暴露。
- `core/`、`model/`、`metrics/` 源码已被旧任务删除（仅剩 `__pycache__`），git 只跟踪 `analysis/` 与 `elements/`——比 01 gate 扫描得到的「5 个旧包」更窄。
- 空间代数在 `preparation/geometry.py`（355 行，SE3 + 四元数 + 12 个 numba 内核）；关节/刚体声明在 `preparation/assembly/types.py`（278 行）。

## 非目标

- 不实现具体模板（归 03）、端口连接（归 04）、试验台（归 05）。
- 不重写物理逻辑，不改变任何数值结果。
- 不动 `suspension_kinematics`。

## 约束

- 低层不得反向导入 preparation/subsystems/rigs/simulation/kernel/report。
- 不重录任何基线；默认路径数值必须逐位不变。
- 旧导入路径保持可用（迁移期转发），不一次性删除调用方。

## 写范围

- `packages/suspension_multibody/src/suspension_multibody/modeling/`（新增）
- `preparation/assembly/types.py`、`preparation/geometry.py`、`subsystems/types.py`（迁移转发）
- `packages/suspension_multibody/tests/modeling/`、`tests/architecture/test_import_boundaries.py`（新增）
- 本任务目录证据文件

## 验收（对齐 SUBTASKS.csv:3）

主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q`

- 独立子进程按任意顺序导入成功。
- 编译期/运行期引用无循环；基础模型无反向依赖。
- 单位与坐标变换、稳定 ID、重复实体拒绝、不可变输入测试通过。
- 原默认数值回归不变。
