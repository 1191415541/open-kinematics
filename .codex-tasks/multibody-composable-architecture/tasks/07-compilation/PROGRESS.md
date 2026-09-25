# 07 正交请求、契约编译与真实 Study

## Recovery

- 任务：`07 解开请求路由并统一契约编译`。形态：single-full。依赖 06（DONE）;08（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 8 行；输入规格 `../../TASKS.md` 第 07 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/simulation packages/suspension_multibody/tests/studies -q`
- 补充门：`kc_parity_check.py --check` 与 `case_parity_check.py`。

## 实测现状（进入本任务时）

- `compilation/` **不存在**。运行语义全部隐含在 family 名与人名类里：7 个 compiler 类各自决定 bench、reading、驱动坐标与轮胎激活。
- `simulation/request.py:61-69`：rig 与 family 相互补值并**强制相等**（`elif family != rig: raise`），`request_kind` 再由 family 派生。DESIGN.md:14 把这行点名为缺陷。
- 驱动坐标由 family 名推断：`api.py:335 _KC_RIG` 硬编码，`planned drive_wheels` 无处表达。
- `studies/bridge.py`（421 行）是 K/C 模型转动态模型的唯一通道；06 记录「生产入口未切换」。
- **GAP-1 实测仍在**：`cases/kc_quasi_static/contract.py:model_document` 输出的键中没有 `tires`；`api.py:743 _tire_compression(case)` 函数体为 `del case; return {"left": 0.0, "right": 0.0}`。

## 做了什么

### 新增 `compilation/`：三层，各管一件事

| 模块 | 管什么 | 关键事实 |
|---|---|---|
| `plan.py` | 运行**是什么** | `SolvePlan` = rig + family + study + mode + 轮胎激活 + 采样 + solver + case 输入 + 输出 |
| `model_view.py` | 模型**是什么** | `ModelView` 把 `FrontAxleAssembly` 与 `SimulationAssembly` 读成同一形状 |
| `compile.py` | 两者**蕴含的文档** | `EmitterRegistry` 按 family 选 emitter；emitter 内不出现 family/template/topology 分支 |

family 名只在这三者之一出现一次，且只作为**选 emitter 的注册键**。

### 请求正交化

- `SimulationRequest` 增 `study`、`outputs`；**取消** rig==family 强制相等与 request_kind 由 family 派生。
- `RigSpec` 增 `family` 字段与 `RigSpec.route`；新增 `rigs.rig.rig_family()`。空的 `family` 仍回落到 rig 名，所以 7 个出厂 bench 与既有调用方行为不变。
- 「rig 与 family 不同」不再被拒；被拒的是**真正的矛盾**——bench 声明了自己的 study，调用方却要另一个。该拒绝发生在 `plan_for`/`request.resolved_study`，措辞点名 bench。
- `drive_wheels` 依次取自：case 自身的 wheel 扫描 → bench 的 `supplies_wheels` 能力。**不读 family 名**。

### SI 图成为可编译对象

- `modeling/assembly.py:Assembly` 增 `physical` 回指：组合层拥有**身份与端口**（指纹、provenance），物理实体属于被组合的那次 build。`ModelView` 各取所属，避免文档与指纹各说一套。
- `subsystems/si_assembly.py` 组合时把 `build_front_axle` 的产物一并带上；`compose_simulation_assembly` 增 `physical` 参数。

### 生产入口切换到同一流水线

- `api._compile_plan_run(...)`：先 `plan_for` 得到计划，再 `compile_plan` 由模型产出文档，最后组装 `CompiledSimulation` 交给既有 `run_compiled`。`_run_k` 与 `_run_c` 都改走它。
- 结果：K/C 生产路径与其余 family 走**同一条编译流水线**，不再是流水线旁边的一条路。

### GAP-1 闭合：准静态垂向轮胎真实进入残差

- `cases/kc_quasi_static/contract.py` 新增 `_tire_entries`：把装配携带的 `VerticalTireElement` 发射为契约 `tires` 条目。这是 D1 的 K/C 一半——准静态态无滑移，轮胎只走垂向分支，但用的是**同一条力律**，不是第二条。
- 垂向分支用不到的系数写成**中性正值**（`_TIRE_NEUTRAL_COEFFICIENTS`：单位摩擦、单位刷子刚度、单位松弛长度），不是零：模型读取器要求这些严格为正，写零是被**拒绝**而非被忽略。`maximum_compression = radius/2`，满足「正且小于半径」。
- `api._tire_compression(run, case_index)` 改为读内核实测的 `tire_output` 第 2 列（`penetration_m`）。原实现回显零，理由是「`CaseSpec` 没有路面高度」——那对**输入**成立，对**答案**不成立。缺轮胎时返回空字典而非零：不存在的测量值不该冒充 0.0。

### 消除 K/C 转动态的生产桥接

- `studies/bridge.axle_dynamics_model` 本身**保留**：它是「已装配的 K/C 轴被推进时间」这一输入路线的唯一实现，删掉会让 Study 合并的初衷不可表达。
- 但实测生产入口模块（api/cli/contract_run/vehicle service/simulation 五模块/preparation.kc_quasi_static）**无一调用它**；唯一调用者是 `preparation/axle_dynamic.py`（输入适配器）。

## 关键实测

| 判据 | 实测值 |
|---|---|
| 轮胎力随垂向刚度线性变化 | k=200 → Fn=8000 N；k=400 → Fn=16000 N；Fn = k·1000·δ |
| 压缩量与刚度无关（驱动量） | 两次运行 `tire_output[:,:,2]` 一致 |
| 声明轮胎不移动冻结动力学 | 有/无轮胎的 `upright_L/R` body_state 在 1e-6 内一致 |
| K/C 冻结快照 | kc_parity 在容差内，无漂移 |
| 8 个 family | case_parity 全 PASS |
| 生产桥接调用点 | 仅 `preparation/axle_dynamic.py` |

## 门禁设计（21 项新增）

`tests/simulation/test_orthogonal_requests.py`（12 项）：axes 独立 / rig≠family 可命名 / bench 不接受的 study 点名拒绝 / 无 study 声明的 bench 必须由调用方指定 / 请求与 bench 矛盾被拒 / 同一装配两 study 同一指纹 / emitter 注册按 family / 未知 family 拒绝 / 两种装配形状的视图一致 / 无 build 的组合被拒 / 计划描述 / drive_wheels 来源。

`tests/simulation/test_quasi_static_tire.py`（9 项）：文档声明装配携带的轮胎 / 轮胎进入残差且力与穿透成正比 / **改变垂向属性改变响应**（GAP-1 的真正验收）/ 无轮胎则无轮胎行（负对照）/ 中性系数完整性 / 压缩上限物理有效 / 结果通道读的是求解值 / 声明不移动冻结运动学。

`tests/simulation/test_no_production_bridge.py`（4 项）：生产入口不达桥接（AST 级性质）/ 桥接只剩输入适配器一个调用者 / 输入适配器仍能转换已装配轴 / 未声明惯量的轴被点名拒绝。

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/simulation tests/studies -q` | 0 | 主验收 **111 passed**（新增 25 项） |
| `python scripts/kc_parity_check.py --check` | 0 | 冻结 K/C 快照容差内，无漂移 |
| `python scripts/case_parity_check.py` | 0 | **8 families 全 PASS** |
| `ruff check .` | 0 | All checks passed |
| `ty check .` | 0 | All checks passed |

## 未覆盖与保留

- 协议的**表达力不足**未出现：现有 `multibody-model.tires` 已足够表达垂向激活，无需新增协议字段或 ABI 符号。这是「先利用现有原生能力」那一句的实测结论，不是假设。
- `_DocumentPairEmitter` 覆盖 vehicle_kc/handling/ride_four_post/ride_random_road/vehicle_dynamic 五个 family：它们的文档由各自 preparation 撰写，本层只做装帧。把它们也纳入编译是 10 与 11 的工作。
- `subsystems/` 的 `SubsystemOutput` 仍未携带 element 行与理想约束列，所以 SI 组合的 `fragment.forces` 为空。这不影响本任务（编译器读的是 `physical`），但 11 的新拓扑夹具若要只靠组合层编译则需要补齐。
- 10 负责结果解码与公共 API 薄化；本任务只把 K/C 结果通道的 `tire_compression` 从回显改为实测。
