# 04 端口组合、自适应与全局策略

## Recovery

- 任务：`04 实现端口自适应连接与全局规则`。形态：single-full。依赖 03（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 5 行；输入规格 `../../TASKS.md` 第 04 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/connections -q`

## 实测现状（进入本任务时）

- `connections/` 目录**不存在**（本任务新增）。
- D3 全局规则此前**没有单一权威**：以 `_RIG_ASSEMBLIES`（`rigs/compose.py:185`）与 `DEFAULT_AXLE_SUBSYSTEMS`/`DEFAULT_VEHICLE_SUBSYSTEMS`（`subsystems/types.py`）分散表达，可用性矩阵测试单独锁定。规则本身正确，但「非法组合」的判定没有一处能统一回答。
- 端口概念只到 03 的 `PortDeclaration`；没有任何代码做匹配、歧义判定或几何适配。

## 做了什么

### 全局 D3 规则（`connections/policy.py`）

- `AXLE_RULE` / `VEHICLE_RULE` 以**数据**表达：axle 禁 brake/drive、必需 suspension、车轮来自试验台；vehicle 必需 suspension/steering/brake/drive、自有机轮。
- `check_root(kind, roles)` 是唯一入口，**只接受两个参数**；没有 `allow`/`override` 关键字——模板作者想绕过 D3，第一个会找的地方就是那里，所以它的缺席被测试断言。
- 按 **root kind** 查表：整车内的前后轴不是独立单轴仿真，套用单轴规则会要求整车交出车轮。
- 两个方向都查：禁止项存在、必需项缺失，同类错误（「不是它自称的东西」）。

### 端口匹配（`connections/matcher.py`）

固定优先级，逐条实现：

1. **显式映射优先**；映射到不存在的端口或未声明的需求 → 报错，**不静默回退**到推断。
2. 否则按 role + capabilities + labels 筛选；labels 精确相等（L 不绑 R）。
3. 零候选：required → 报错点名；optional → 记录消失，并带走 `bound_outputs`（不留悬空输出）。
4. **多候选 → `AmbiguousBindingError` 并列出候选**，要求补显式映射。按名称相似度或距离悄悄选，正是「看起来转向其实没转」的来源。

`Binding.explicit` 记录该绑定是人为指定还是推断，`MatchReport.trace` 记录每项决策——绑定事后可审计，不必重推。

### 几何适配（`connections/geometry.py`）

- `port_world_pose(owner_pose, port_local)`：端口世界位姿 = 所属刚体位姿 ∘ 局部偏移。局部偏移**常量**，移动全部来自刚体——这是「硬点变化自动重建试验台」能成立的那一行，也是 A4 的判据。
- `solve_mount`：`rig_instance = port_world ∘ installation`；`MountSolution` 保留两个输入，A4 可以独立复核这个乘法而不重建装配。
- `MountSolution.matches` 用**主旋转向量**比对旋转，而不是四元数：`q` 与 `-q` 是同一旋转，四元数比对会把一致的姿态报成不一致。
- `cardinality="none"` 的端口拒绝挂载：它存在是为了被报告，不是为了被连接。

## 门禁设计（`tests/connections/`，37 项）

`test_matcher.py`（13）：显式优先、显式映射错误报错、**歧义拒绝且列出候选**、labels 精确、capabilities 筛选、required 无候选点名、optional 消失并带走输出、optional 找到则保留输出、trace 可审计、count=2 选两个、空需求。

`test_geometry.py`（10）：平移合成、**owner 移动则挂载同量移动**（自适应接口的差分表述）、与手算矩阵结果一致（不用被测代码自证）、旋转安装不是相加、独立算出的位姿被接受/超出容差被拒、`q` 与 `-q` 视为同旋转、不可连接端口被拒、解保留证据。

`test_policy.py`（14）：axle 禁 brake/drive、axle 可有可无 steering、vehicle 必需三项（逐个缺失都拒）、未知 role/kind 点名、**两个 assembly 的默认子系统集各自满足本规则**、`check_root` 签名无可绕过参数、以及用**真实构建器**产出的 axle 复核其 capabilities 不含 brake/drive。

最后一条是刻意的：只有规则测试自己遵守的规则不算规则。

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/connections -q` | 0 | 主验收 37 passed |
| `python scripts/kc_parity_check.py --check` | 0 | 冻结快照容差内，无漂移 |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |
| `pytest tests -q`（全量） | 见下 | 回归无新增失败 |

## 未覆盖与保留

- 本任务的 policy/matcher/geometry 是**独立可用的新层**；把它接到现有 `build_front_axle`/`build_vehicle` 的装配路径上，属于 06（收口子系统与 SI 仿真总成）的写范围。
- 内核约束秩检查不由本层冒充：DESIGN 明确作者层只做引用/单位/所有权/结构检查，完整秩由既有内核 audit 负责。
- 试验台模板化为物理模板归 05。
