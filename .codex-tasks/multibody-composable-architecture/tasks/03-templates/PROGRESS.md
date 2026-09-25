# 03 模板图生成与实例化

## Recovery

- 任务：`03 统一模板物理图和实例化协议`。形态：single-full。依赖 02（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 4 行；输入规格 `../../TASKS.md` 第 03 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/instantiation packages/suspension_multibody/tests/properties -q`

## 实测现状（进入本任务时）

- `templates/` 已有 1,226 行实现：`model.py`（Template/PartDefinition/ConnectionDefinition/PropertySlot/OutputDeclaration + JSON 往返）、`instantiate.py`（K/C 列激活、属性解析）、`registry.py`、`roles.py`（六 role 契约）、`builtin.py`（双横臂声明）。
- 底座是**纯声明式**：模板声明构件与连接，但没有端口，也没有任何路径让模板真实产出一个模型片段。「注册即证明可扩展」正是 EPIC 点名要补强的弱验收。

## 做了什么

### 端口声明（`templates/ports.py`）

- `PortDeclaration`：端口在**模板本地**名字空间声明（role、owner、kind、capabilities、labels、cardinality、family/units/direction）；不携带坐标——端口位置属于模型而非模板，硬点移动时端口随之移动而模板不变（A4 正是这样测的）。
- `PortNeed`：模板对邻居的需求；`required=False` 标记可整体消失的分支，且**必须**记录 `bound_outputs`。
- `declaration_to_port`：把声明落到某个挂载点，产出 `modeling.ports` 的具体端口；同一模板挂两次得到不同 id。

### 端到端产出（`templates/builders.py`）

- `declared_fragment`：把声明式模板展开为 `ModelFragment`——构件→刚体、连接→点与 joint/bushing 列（复用 `activated_column` 这一条规则）、输出与端口随行。
- `build_fragment`：**两条作者路径的统一入口**。模板带 builder 就调用它，否则走声明展开；两条路径都返回 `ModelFragment`，都经同一校验，并统一附加 `FragmentProvenance`（模板名、修订、实例路径、属性指纹）。
- builder 注册表：模板文档只存 builder 的**名字**，不存 callable；重名注册拒绝，未注册名字点名已知项。

### 校验（`templates/model.py`）

- Template 新增 `ports`/`needs`/`builder` 字段，全部进入 JSON 往返。
- `_check_uniqueness` 扩展：重复端口拒绝；端口 owner 必须引用真实构件；可选需求缺 `bound_outputs` 拒绝（否则收缩会静默孤立输出）。

## 关键实测：模板真的产出实体

```
build_fragment(DOUBLE_WISHBONE, mode="K") -> 10 bodies, 13 joints, 4 forces
build_fragment(DOUBLE_WISHBONE, mode="C") -> 10 bodies,  9 joints, 8 forces
```

这两个数字与 `builtin.py` 模块文档记录的真实约束数一致（K 13、C 9），来自 `build_front_axle` 的实际行为，不是本任务选定的。特别是 C 模式保留 9 个 joint——拉杆端与臂外点没有 bushing 列，因此两模式都是 joint；把「C 激活衬套」读成「C 丢弃关节」会少掉 4 个。

## 门禁设计（`tests/templates/test_builders.py`，13 项）

| 断言 | 说明 |
|---|---|
| 声明模板真实产出实体 | 构件集与声明一致，entity_count > 0 |
| K/C 激活列正确 | (13,4) 与 (9,8)，取自既有求解行为 |
| 模式切换不改几何 | bodies/points 集合相同 |
| 零刚体模板合法 | 简化力元子系统可表达 |
| builder 可循环生成可变构件数 | 0/1/5 构件均成立 |
| **两条作者路径产出同一片段** | 数据与 builder 路径 fragment 相等 |
| builder 返回错误类型被拒 | 必须返回 ModelFragment |
| 未注册 builder 点名已知项 | 错误可诊断 |
| 重名注册被拒 | 不静默覆盖 |
| provenance 指纹与键序无关 | 同值两次构造一致 |
| instance 路径带入片段 | 嵌套寻址前提 |

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/templates tests/instantiation tests/properties -q` | 0 | 主验收 75 passed |
| `pytest tests/templates -q` | 0 | 43 passed（含 13 项新 builder 测试） |
| `pytest tests/modeling tests/architecture/test_import_boundaries.py -q` | 0 | 02 主验收仍 71 passed |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed（修正了 `**dict` 展开导致的 missing-argument） |
| `pytest tests -q`（全量） | 0 | `1135 passed, 1 skipped, 1 xfailed in 1379.48s`；基线 1052/1/1 → 新增 83 项全通过，skip/xfail 未增长 |

## 未覆盖与保留

- 内置模板仍是声明式；本任务提供 builder 路径与注册表，供 11 的新拓扑实证使用。
- 端口**绑定求解**（显式映射优先、歧义拒绝、几何适配）归 04；本任务只提供声明与「能否接受」判定。
- 试验台模板归 05；`rigs/` 语义未动。
- JS 属性文件的加载与解析沿用既有 `properties/load.py`，本任务未改。
