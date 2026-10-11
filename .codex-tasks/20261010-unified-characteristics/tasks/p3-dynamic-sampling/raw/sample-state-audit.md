# P3 步骤 1：接受态与输出状态同步时序审计

> SPEC §44（步骤 1）：只读审计状态/乘子/模式/内部变量，核心采样完整性证据明确。
> 证据落点：本文档。
> 记录规则：只记**实测**事实与源码行号；未读的代码不推测；缺口如实登记。
> 本步为**只读审计**，不改任何源码。

## 0. 结论摘要

| 项 | 结论 |
|---|---|
| 接受态提交时机 | 明确，且是唯一入口：`accept_step_commit` λ（`kernel_abi.cpp:411-452`） |
| 提交内容完整性 | **充分**：`state` 整体 move + `constraint_multiplier` + 能量/事件/加速度历史 + 三缓存失效 |
| 输出采样粒度 | **每输出样本一个状态**，非每接受步一个（`:794 copy_state(..., sample_index)`） |
| 接受步被持久化？ | **否**——仅计数（`accepted_steps`/`rejected_attempts` 进 diagnostics） |
| 核心采样完整性 | **可用但不足**：能拿到"输出网格点"的一致状态，**拿不到"每个接受步"的一致状态** |
| 对 P3 SPEC §26/§35 的影响 | `all_output` 可达成；`all_accepted` 需新增采样路径（本步登记为 P3 步骤 2 的落点） |

**关键发现**：原 native 结果**不是**"逐接受步的完整检查点"，而是"逐输出样本的
一致状态快照"。P3 SPEC §28 要求的 `all_accepted`（含一致初始点与**每个接受步**）
在现有输出机制下**无法**从 `body_state` 数组还原——数组长度由 `sample_count` 决定，
与接受步数无关。

## 1. 接受态提交链路（实测行号）

### 1.1 唯一提交入口

`kernel_abi.cpp:411` 定义 `accept_step_commit` λ，`:430-445` 为提交体：

| 行 | 动作 | 审计意见 |
|---|---|---|
| :418-425 | `accumulate_energy_step(...)`，失败则记 `"nonphysical energy rate detected"` 并**提前返回不提交** | 提交前能量校验，失败即不推进 |
| :426-429 | `append_contact_events(...)` | 事件在提交时追加 |
| :430 | `capture_acceleration_history()` | 保存 `previous_acceleration`/`previous_alpha`（:365-366） |
| :431 | `state = std::move(event_step.state)` | **完整状态整体接管**，非逐字段 |
| :432-434 | `current_constraint_multiplier = std::move(...)` | **乘子一并提交** |
| :435-437 | 三个缓存 `invalidate()`（full/half/single） | 步长改变后线性化失效 |
| :438 | `t += event_h` | 时间推进与状态提交**同步** |
| :439 | `accepted = true` | 循环出口 |
| :440-445 | 计数 + 步长统计 | 仅计量，**不保存状态** |

**审计结论**：提交是原子的——状态、乘子、时间、加速度历史、缓存失效在同一次调用内完成，
且**在 `t += event_h` 之后**才 `accepted=true`，故"接受"与"状态已同步"是同一时刻。
这满足 SPEC §34"接受且内部状态/乘子/活动模式同步后、下一积分步前只读提取"的前置条件。

### 1.2 三个调用点

| 调用点 | 场景 | 传入缓存 | 更新建议步长 |
|---|---|---|---|
| `:559` | 事件定位成功（自适应） | 三缓存 | 是 |
| `:661` | 半步/全步一致（自适应） | 三缓存 | 是 |
| `:675` | 固定步长路径 | 单缓存 | **否** |

`:405-410` 的注释明确记录了两个驱动**必须**不同的地方（缓存失效范围、是否改步长），
理由是"固定步序正是冻结基线记录的内容"。**这与 P3 SPEC §39"开启不改变原轨迹"
直接相关**：任何新采样不得触碰 `suggested_step` 或固定步序。

### 1.3 活动模式与内部变量

- 活动轮胎：`output->tire_output[tire*kTireOutputWidth] > 0.5`（`:325`、`:820`）——
  由输出块**推断**，不是独立模式数组；`:822` 统计 `active_contacts` 进 diagnostics。
- 接触事件：`contact_events`（`std::vector<ContactEventRecord>`，`:336`）在提交时追加（`:426`），
  最终写入 `output->contact_event_output`（`:103-104` 校验容量）。
- 内部变量：`previous_acceleration`/`previous_alpha`（`:337-338`）是唯一跨步保留的内部量，
  经 `capture_acceleration_history` 在每次提交时刷新。

**审计意见**：SPEC §28 要的 `mode identity` 当前**没有一等表示**，只能从
`tire_output` 的首字段阈值反推。P3 步骤 2 若需要模式身份，应登记具体字段而非假设其存在。

## 2. 输出采样机制（实测）

### 2.1 采样循环粒度

`:347`：

```cpp
for(std::size_t sample_index=1; sample_index<input->sample_count; ++sample_index){
    const double target = input->sample_times[sample_index];
    double t = input->sample_times[sample_index-1];
```

**每个输出样本区间一个外层迭代**，区间内部循环步进直到 `t` 到达 `target`。
故 `sample_index` 是**输出样本**身份，不是接受步身份。

### 2.2 状态写出位置

| 行 | 内容 |
|---|---|
| :305 | `copy_state(state, model, output->body_state, 0)` —— 初始点写 `sample_index=0` |
| :794 | `copy_state(state, model, output->body_state, sample_index)` —— 成功时写该样本 |
| :797 | `copy_constraint_wrench(..., output->constraint_wrench, sample_index)` |
| :804-829 | 该样本的 diagnostics 块（`d = sample_index * kDiagnosticsWidth`） |
| :746-782 | **失败时**同样写 `sample_index`，但 `diagnostics[d+0]=0.0` 标记失败并记 `failure_code`（`:781`） |

**审计结论**：`body_state` 的长度语义是 **`sample_count` 行**，行 `i` 对应
`sample_times[i]`。区间内被接受的多个步**只有最后一步**的状态存活到写出。
接受步数与拒绝对数只作为标量存进 diagnostics（`:749-750`、`:805-806`），
**逐接受步的状态不在任何输出数组中**。

### 2.3 初始一致点

`sample_index=0` 由 `:305` 写出，其状态来自起点的 trim/一致化。SPEC §26 要求
`sample_kind=initial`——该点在现有机制中**存在且可识别**（索引 0），
这是 `all_accepted` 的第一项，本步确认它不必新造。

## 3. 核心采样完整性的证据与缺口

### 3.1 可用的部分

- **输出网格**：每个 `sample_times[i]` 都有一致状态 + 乘子 + diagnostics，含失败标记与原因。
- **初始一致点**：索引 0。
- **失败样本**：`diagnostics[d+0]=0.0` + `failure_code` 显式标记，非静默。
- **步序统计**：`accepted_steps`、`rejected_attempts`、`min/max/last_accepted_step`、
  `event_count`、`max_iterations`、`local_error_max`（`:749-762`、`:805-829`）。

### 3.2 缺口（必须在 P3 步骤 2 解决，不能以"全点 inconsistent_sample"结束）

| # | 缺口 | 影响 | SPEC 依据 |
|---|---|---|---|
| G1 | **逐接受步状态未持久化** | `all_accepted` 无法从现有输出还原；只知接受步**计数**，不知其状态/时间 | §28、§35 |
| G2 | **无 `mode identity` 一等字段** | 只能从 `tire_output` 阈值反推活动集 | §28 |
| G3 | 拒绝试探步无逐条记录 | 仅有 `rejected_attempts` 标量；SPEC §54 的"拒绝试探步"场景需要身份而不仅是计数 | §54 |
| G4 | `sample_index` 与 `time` 的对应需调用方自持 `sample_times` | 输出块不含时间数组，`raw.times_s` 由 Python 侧按切片拼接（P1 已登记为 `raw.py` 设计问题） | §28 |

**G1 是最关键的**：它决定 P3 不能仅靠"读取现有结果"。SPEC §84 已预警此点
（"当前原生结果不是完整检查点，必须从原生已接受状态取值"），本审计**实测确认该预警成立**。
按 SPEC §84，正确做法是落实采样路径（在 `accept_step_commit` 之后、下一积分步之前
只读提取），**而非**用插值位姿冒充（§36 明令：只有插值位姿时相关请求标
`inconsistent_sample`，不得换到邻近接受态）。

### 3.3 为什么不能在现有循环里"顺手"记录

`accept_step_commit` 在 `kernel_abi.cpp` 内，而 P3 SPEC §19 的修改范围包含
`kernel_abi.cpp` 的 state 提交/正式输出采样。但要注意 §39 的硬约束：
**开启特性不得改变原轨迹/事件/接受拒绝次数**。因此新增提取必须
(a) 在提交**之后**、(b) 纯只读、(c) 不触碰 `suggested_step`/`h`/缓存失效语义、
(d) 无请求时**零额外工作**。这四条构成 P3 步骤 2 的实现契约。

## 4. 对 P3 后续步骤的前置结论

1. **步骤 2（采样选择与身份表）**：`sample_kind=initial` 可直接用索引 0；
   `all_output` 直接映射现有 `body_state` 行；**`all_accepted` 需要新增逐接受步提取路径**，
   这是本步识别的最大工作量。身份表必须独立于 `sample_index`（两者语义不同，
   混用会把"第 3 个输出样本"记成"第 3 个接受步"）。
2. **步骤 3（几何/输入偏导/真实时间导数）**：必须在**接受态**上算，
   故依赖步骤 2 的提取路径先就位；用输出网格点算会让"真实时间导数"退化成
   "输出间隔差商"，恰是 SPEC §37/§38 禁止的。
3. **步骤 4（中心差分）**：端点一侧只有单边可得，SPEC §38 要求写明"端点单侧标识"，
   故数据结构必须容纳单边标记，不能假定总是中心。
4. **步骤 5（状态不变与预算）**：比较基准是 `rejected_attempts`/`event_count`/
   状态 hash 等**已在 diagnostics 中的标量**，本步确认这些标量足以支撑比较。

## 5. 本步未做与本步不声称的

- 未运行任何命令（本步为源码审计）；无退出码可记。
- 未验证 Python 侧 `raw.py` 对上述字段的解码现状——属步骤 2 范围。
- 未声称 G1 的解法已确定；只确认"现有输出机制不足以还原逐接受步状态"这一事实。
- 未改动任何文件。
