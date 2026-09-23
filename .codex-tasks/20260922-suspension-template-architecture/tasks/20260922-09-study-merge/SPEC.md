# 子任务 09：合并 kc_quasi_static 与 axle_dynamic 为准静态/动态 study

## 目标

把 `kc_quasi_static` 与 `axle_dynamic` 两个 family 合并为**同一个仿真**，由 study 选择准静态或动态。

1. **统一装配入口**：同一套模型装配（模板 + 子系统）既能跑准静态也能跑动态，不再有两套模型 schema（`FrontAxleModel` 与 `AxleDynamicsModel` 并存）与两套装配路径。
2. **study 选择**：`study="quasi_static"` 或 `study="dynamic"` 决定时间语义、case 结构、求解器默认与轮胎激活范围。
3. **准静态的轮胎退化**（用户裁决 D1）：准静态 study **接受与动态相同的力律选择**（`fiala`/`pac2002`/`native_brush`），但**只使用其垂向刚度与轮胎尺寸、质量**，相当于模型退化——不新增内核力律。
4. **轮胎质量归属轮胎**（用户裁决 D2）：两 study 共享同一套轮胎定义，质量来自轮胎本身（由 08 落地）。

## 非目标

- **不新增内核轮胎力律**。D1 明确：不在内核加 `vertical_linear`。
- 不改轮胎力律的物理定义与参数含义。
- 不改求解器算法（积分、牛顿、步长控制）。
- 不合并结果对象本身（那是 07 的输出声明与 10 的组合问题）；本步只要求两 study 的结果**可对照**。
- 不删除旧 family 的公开入口（`run_dynamic_case` 等保持可用直到 10 之后再定）。

## 现状事实（制定计划时实测，实施时复核）

KC 与 `axle_dynamic` 的差异**不止"求解器与轮胎"**，实测七项：

| 差异 | kc_quasi_static | axle_dynamic |
|---|---|---|
| 模型 schema | `FrontAxleModel`（mm、`symmetric_proxy`/`explicit`、3 种副可编码） | `AxleDynamicsModel`（m、纯显式、8 种副） |
| 模型文档是否发轮胎 | **不发**（实测 `"tires" in doc` 为 `False`） | 发（`cases/axle_dynamic.py:342-345`） |
| K/C 装配模式 | 有（`drive_wheels`） | **无** |
| case 结构 | 网格 + 常量 wrench | 时程 + 路面 + 谐波 + 逐采样表 |
| 时间网格语义 | 两采样准静态（`api.py:91` 的 `_TIMES_S = (0.0, 1e-3)`），每网格点独立成 case、速率恒为 0 | 真实积分 |
| 结果对象 | `StateResult`/`ResultBundle`，元件力在 Python 侧重算（`api.py:751-791`） | `AxleDynamicsResult`，读内核 7 类输出块（`axle_dynamics/contract_run.py:234-257`） |
| 驱动坐标命名与 case 输入形式 | `wheel_drive_L/R` + `rack_drive`（`cases/kc_quasi_static/contract.py:177-222`） | `AxleDynamicsModel.driven_coordinates` |

**真正共享的只有三样**：`AxleSolverSettings` 与 `kernel/solver.py` 的序列化器、driven coordinate 内核机制、同一个 C++ dispatcher 与求解器。

本步的实质是**消除上表前六项差异**，让"只差求解器与轮胎的激活范围"成立。

## 约束

- **垂向退化不得新增内核力律**。可利用的既有底座：`packages/suspension_kernel/cpp/src/tire/fiala/forces.cpp:20-29` 的 `fiala_elastic_force` 在无 `deflection_curve` 时就是 `tire.k * penetration`；`fiala_vertical_force`（`:31-37`）为 `max(0, elastic + c*rate)`。准静态"只取垂向"通过作者层的参数与激活选择实现。
- **`vertical_linear` 是死名字，不得使用**：它在 `contract_registry.cpp:36` 的名单里，但内核解析表只有 `{native_brush,0},{pac2002,1},{fiala,3}`（`cpp/src/cases/contract_model.cpp:948-950`），枚举 `VehicleTireModelKind`（`cpp/include/mb_model/enums.hpp:61-70`）也只有这 4 项。写它进文档会报 `unknown model`。
- **kc 从「无轮胎」变为「有轮胎（垂向激活）」**，这是模型改变，`kc_baseline/` 与 `kc_perf_baseline*.json` 会失效。重录必须逐项登记（格式见 05 的 `PROGRESS.md` 基线重录台账）。
- **必须先建立「垂向激活等价于原 `VerticalTireElement`」的对照**（`:561-574` 的现役垂向轮胎）再重录，否则无法区分"模型改变"与"回归"。
- 两 study 必须共用**同一个装配入口**：有测试证明同一模型对象既可配准静态 case 也可配动态 case。
- 不引入新依赖。

## 范围与文件归属

- 可写：
  - 新增 `packages/suspension_multibody/src/suspension_multibody/studies/**`（study 定义与分派）
  - `packages/suspension_multibody/src/suspension_multibody/cases/kc_quasi_static/**`、`cases/axle_dynamic.py`（case 层归并为按 study 分派）
  - `packages/suspension_multibody/src/suspension_multibody/preparation/axle_dynamic.py`、`preparation/kc_quasi_static.py`（装配入口统一）
  - `packages/suspension_multibody/src/suspension_multibody/api.py`（`_run_axle_quasi_static` 等入口改走统一入口）
  - `packages/suspension_multibody/src/suspension_multibody/cases/axle_dynamic.py` 的轮胎质量发射落点（审核 B4：08 只做内核侧，轴侧落点归本步；整车侧落点归 11）
  - 轮胎激活范围的选择逻辑（垂向退化）
  - 新增测试 `packages/suspension_multibody/tests/studies/**`
  - `packages/suspension_multibody/tests/data/kc_baseline/**`、`kc_perf_baseline*.json`（重录 + 登记）
- 只读：`schema/**`、`templates/**`、`subsystems/**`、`joints/`、08 的轮胎质量字段。
- 不写：`outputs/`、`report/`（07）；C++ 内核与契约 schema（08 的写范围）；`cases/vehicle_dynamic.py` 与 `preparation/vehicle_dynamic.py` 的整车侧质量落点（11 的写范围）；父级计划文件（归主代理）。

## 依赖

- 前置：06（属性文件，供轮胎/弹性元件属性来源）、08（轮胎质量归属轮胎，本步的轮胎定义依赖它）。
- 后续：10（试验台抽取）、12（终局验收）。

## 验收标准

1. **同一装配入口**：同一模型对象可分别配准静态 study 与动态 study，有测试证明装配路径相同（不是两条独立装配）。
2. **垂向退化的正确性**：准静态 study 下，轮胎只贡献垂向刚度与尺寸、质量——有断言证明侧向/纵向输出为零或未激活，而动态 study 下同参数产生非零侧向/纵向。
3. **力律选择一致可用**：`fiala`/`pac2002`/`native_brush` 三种在准静态与动态 study 下都可选，且准静态只取垂向部分。
4. **两 study 结果可对照**：同一模型、同一轮胎定义下，两 study 的结果对象可通过既有结果层对照（不要求对象同一，但要有可对照的通道）。
5. **轮胎质量来自轮胎**：两 study 都从轮胎读取质量，不依赖 body 上的 `WheelSpec.mass`（08 落地后）。
6. **基线重录有登记**：`kc_baseline/` 与 `kc_perf_baseline*.json` 的重录前后值、导致重录的步骤、判定依据逐项写明；先有垂向等价对照证据再重录。
7. 门禁：`case_parity_check.py` 8 family 重新通过；`dynamic_hash_sentinel` 视影响保持绿或登记；`--strict --final` 保持绿；ruff/ty 通过。

## 验证协议

1. 统一入口落地后：跑「同一装配两 study」测试。
2. 垂向退化落地后：先跑「垂向激活 vs 现役 `VerticalTireElement`」对照，再跑"侧向/纵向零或未激活"断言。
3. 力律选择落地后：三力律 × 两 study 的 6 个组合各跑一次。
4. 重录前：定量记录旧值；重录后：跑 `kc_parity_check.py --check` + `case_parity_check.py` + `dynamic_hash_sentinel.py --check` + 性能门。
5. 收尾：三套 pytest、ruff、ty、`--strict --final`、`git diff --check`。

**若动态 study 的既有行为发生变化（`axle_dynamics_baseline/`、`dynamic_hash_baseline.json` 失效）**，说明本步误改了动态路径——动态侧本步应只改"与准静态共享"的部分，不改变其数值。若确实必须变，须单独裁决并登记。
