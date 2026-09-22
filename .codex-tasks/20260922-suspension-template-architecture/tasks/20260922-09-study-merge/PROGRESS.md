- 任务：合并 kc_quasi_static 与 axle_dynamic 为准静态/动态 study
- 形态：single-full（Epic 子任务）
- 进度：0/10 步骤 TODO，尚未实施
- 当前：未开工。前置 06（属性文件）与 08（轮胎质量归属轮胎）均未完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-09-study-merge/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 08 已完成：轮胎质量归属轮胎，内核 `Tire` 含质量且求解器显式耦合，质量守恒断言通过。
- 06 已完成：属性文件可加载，弹性元件属性来源可用。
- 父 `EPIC.md` 的 D1（准静态只取垂向、不新增力律）与 D2（质量归属轮胎）口径有效。

## 本步对应的用户需求原文

- 「对于 axle_dynamic 与 kc_quasi_static，这两个完全可以合并为一个仿真，可以选择准静态和动态模式」
- 「准静态 接受 fiala/pac2002/native_brush 只是使用其垂直刚度和轮胎尺寸、质量，相当于模型退化」
- 「这两个应该只有求解器和轮胎区别，所以也应该尽可能通用化」

## 本任务的现状事实（制定计划时实测，实施时复核）

**KC 与 axle_dynamic 的差异不止"求解器与轮胎"**，实测七项（详见 `SPEC.md` 的对照表）：

1. 结果对象与输出路径不同：kc 走 `StateResult`/`ResultBundle` 且元件力在 Python 侧重算（`api.py:751-791`）；axle_dynamic 走 `AxleDynamicsResult` + 内核 7 类输出块（`axle_dynamics/contract_run.py:234-257`）。
2. 模型 schema 不同：`FrontAxleModel`（mm、`symmetric_proxy`/`explicit`）vs `AxleDynamicsModel`（m、纯显式）。
3. kc 模型文档**完全不发轮胎**（实测 `"tires" in doc` 为 `False`）；axle_dynamic 发（`cases/axle_dynamic.py:342-345`）。
4. kc 有 K/C（`drive_wheels`），axle_dynamic **无**。
5. case 结构不同：网格 + 常量 wrench vs 时程 + 路面 + 谐波 + 逐采样表。
6. 时间网格语义不同：两采样准静态（`api.py:91` 的 `_TIMES_S = (0.0, 1e-3)`）vs 真实积分。
7. kc 独有的 `markers`/`body_wrench_markers`。

**真正共享的三样**：`AxleSolverSettings` + `kernel/solver.py` 序列化器、driven coordinate 内核机制、同一个 C++ dispatcher 与求解器。

## 三个必须守住的技术边界

1. **禁止新增内核轮胎力律**。D1 裁决是"模型退化"，不是"加一条力律"。可利用的既有底座：`packages/suspension_kernel/cpp/src/tire/fiala/forces.cpp:20-29` 的 `fiala_elastic_force` 在无 `deflection_curve` 时即 `tire.k * penetration`。
2. **禁止使用 `vertical_linear` 这个名字**。它在 `contract_registry.cpp:36` 的 `kTires` 名单里，但内核解析表只有 `{native_brush,0},{pac2002,1},{fiala,3}`（`cpp/src/cases/contract_model.cpp:948-950`），枚举 `VehicleTireModelKind`（`cpp/include/mb_model/enums.hpp:61-70`）也只有 4 项。写它进文档会报 `unknown model`。
3. **先对照、后重录**。kc 从"无轮胎"变为"有轮胎（垂向激活）"是模型改变；必须先建立「垂向激活等价于现役 `VerticalTireElement`（`front_axle.py:561-574`）」的对照，再重录。否则无法区分模型改变与回归。

## 基线重录台账（实施时填写）

| 基线文件 | 是否重录 | 导致重录的步骤 | 重录前值 | 重录后值 | 判定依据 |
|---|---|---|---|---|---|
| `kc_baseline/k_states.json` | 待定 | 步骤 4、7 | 待填 | 待填 | 准静态开始发垂向轮胎 → 模型改变 |
| `kc_baseline/c_states.json` | 待定 | 步骤 4、7 | 待填 | 待填 | 同上 |
| `kc_perf_baseline*.json` | 待定 | 步骤 7 | 待填 | 待填 | 装配改变 → 性能改变 |
| `dynamic_hash_baseline.json`（26 artifact） | **应为否** | — | — | — | 动态侧本步只改与准静态共享的部分，不应变数值 |
| `axle_dynamics_baseline/` | **应为否** | — | — | — | 同上 |
| `vehicle_dynamics_baseline/sha256.json` | **应为否** | — | — | — | 整车侧不属本步范围 |

## 全量套件基线（主代理实测，作为"新增失败为零"的对照）

`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` → `783 passed, 1 skipped, 1 xfailed`（退出 0）。skip 数与此前记录的 47 不同，原因是本机 `artifacts/` 下 Adams 参考工件已就绪，46 项由 skip 转为实跑通过；守卫代码未改动。

## 下一步

等 06 与 08 完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 09 行状态由主代理回填。
