# P4a 步骤6：覆盖与成本验证

- 日期：2026-10-11（Asia/Shanghai）。
- 结果：**步骤6未通过验收，状态记为 FAILED，retry_count=1；P4a 保持 IN_PROGRESS。**
- 原因：四组合与 P0 目标已完整登记，但动态冻结响应只有低层算子和注入式 Python 驱动证据；尚无实际动态样本到 ABI 应答块的执行路径。因此无法验证整车/悬架动态响应覆盖，也无法实测 P4a 特性路径的成本。不得将缺实现标成物理无效或把范围缩小后通过。
- 逐目标台账：[coverage.csv](coverage.csv)。它按 P0 `metric-catalog.csv` 原始场景展开，保留全部场景、指标、响应类型、单位和边界 profile；另列矩阵视图、R8、R9 与成本预算。

## 覆盖判读

- `S-QS` 与 `V-QS` 的既有工作归属 P1/P2：33 项运动学/几何目录场景与 30 项平衡响应目录场景分别链接至原验收证据。本轮没有重跑 Adams 逐项验证，也不据此声称 Adams 兼容。
- `S-DYN` 与 `V-DYN` 中，18 个几何/时间导数场景有 P3 专项证据；另 33 个局部刚度/柔度及派生响应场景只有 P4a 算子级自测/注入式序列证据，端到端状态标为 `P4a-low-level-only`。
- P0 的 `matrix_view_adams_12x12` 在两个动态场景都保留为独立可选项，并单独登记 `pending_reference`；它不是核心具名模式指标的前置能力。当前 ABI 的 `characteristics_response_kinds` 仍为空，因此核心响应和矩阵响应均未跨 ABI 发布，不能把二者混为一个已支持能力。
- R8 当前是阶段性迁移：P2 锁定精确片段，P3 抽取并迁移 7 个 `body_world_inertia` 调用点；全消费者收口仍由 P6 验收。R9 的缓存/共享分解机制已由 P2 验证，P4a Python 驱动也验证了注入切线下逐样本共享；真实原生响应消费者仍缺，故运行时预算待接线后复核。
- P0 要求每样本报告组装/分解、回代、序列化、峰值内存、输出体积及接受步/实际特性产出差异。没有真实 P4a 响应块时，这些特性路径数据不可测；本轮不拿 `kc_perf_gate` 或注入式缓存自测代替。

## 本轮验证

`just` 在本 Windows 环境启动配方时因找不到 `sh` 退出1，故按 [VALIDATION.md](../../../VALIDATION.md) 逐项运行无 `sh` 等价命令。`just build-kernel` 首次退出1同属该启动限制；等价内核构建和 native 镜像构建均成功。

| 检查 | 实际结果 |
|---|---|
| `uv run --no-sync python packages/suspension_kernel/scripts/build_suspension_kernel.py` | exit 0；生成 `suspension_kernel.dll` |
| `uv run --no-sync python packages/suspension_multibody/scripts/build_axle_native.py` | exit 0；刷新 multibody native 镜像 |
| `mb_frozen_sample_selftest.exe` / `mb_frozen_tangent_selftest.exe` / `mb_frozen_load_selftest.exe` / `mb_frozen_oracle_selftest.exe` / `mb_linear_batch_selftest.exe` | 五个自测均 exit 0；解析切线残量误差阶、非对称切线直接/转置、单位缩放、冻态载荷/坐标闭式结果均见命令输出 |
| `dynamic_hash_sentinel.py --check` | exit 0；报告 `dynamic output matches the frozen baseline byte-for-byte`。内部接受报告仍列9个既有不收敛 case，Adams accuracy 为 BLOCKED；不把接受报告非零子状态误作哨兵失败，也未重录基线 |
| `case_parity_check.py` | exit 0；8 个 case family accepted |
| `kc_perf_gate.py` | exit 0；k-100 best 1.1928 s / 基线 1.0230 s（×1.166），c-66 best 1.6784 s / 基线 1.4904 s（×1.126）；均在现行门预算内，未改基线 |
| `ruff check .` / `ty check .` | 均 exit 0；All checks passed |
| 三条架构门 | `legacy_surface_gate`、`check_module_layering --strict --final`、`check_composable_release --skip-isolation` 均 exit 0；无环、无新增分层边 |
| architecture 定向测试 | 5 passed |
| `tests/characteristics` | 447 passed |
| multibody 快速测试集（忽略 adams/architecture/cases） | 2004 passed，1 xfailed；skip/xfail 未增长 |
| kernel / contracts 测试 | 116 passed / 64 passed |

## 当前阻断及下一步

1. 将 `characteristics_response_kinds` 的声明与实际原生求解、结果块同时接通；只改非空 capability 数组会成为空壳能力，禁止这样做。
2. 从动态存储样本构造 P4a 冻结工作点与实际物理切线/载荷列，接入具名轮端到 `body_wrench` 的显式映射、边界 profile、样本身份和 `local_response_v1`；未知力律/内部状态必须拒绝并给出既定状态/原因。
3. 接通结果查询/单位/frame 与逐测量身份/有效性通道后，重测四组合和 P0 锁定的成本项，再重跑 P4a 所需快速门、构建门及数值门。
4. Adams 2024.1 与独立参考数据仍不可用；G1–G6 保持 pending，不能用 native 证据替代 P5/P6 Adams 兼容发布。
