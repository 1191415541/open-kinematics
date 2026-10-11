# P1 验证归档：统一准静态测量与运动学

对应 `SPEC.md` §51 步骤 6 与 §53–59 验收条件。所有命令在仓库根执行，记录**实测结果与退出码**。

## 0 门与基线总表

| 命令 | 结果 | 退出码 |
|---|---|---|
| `ruff check .` | All checks passed | 0 |
| `ty check .` | All checks passed | 0 |
| `legacy_surface_gate.py --check` | findings 0 | 0 |
| `check_module_layering.py --strict --final` | 0 环，与基线一致 | 0 |
| `check_composable_release.py --skip-isolation` | 3 checks passed | 0 |
| `pytest tests/characteristics` | **158 passed** | 0 |
| 快速集（`--ignore=adams/architecture/cases`） | **1715 passed, 1 xfailed** | 0 |
| `packages/suspension_kernel/tests packages/suspension_contracts/tests` | **176 passed** | 0 |
| 专项场景（SPEC §66 五个文件） | **52 passed** | 0 |
| `dynamic_hash_sentinel.py --check` | 冻结基线**逐位**一致 | 0 |
| `case_parity_check.py` | 8 族 accepted | 0 |
| `kc_perf_gate.py` | 在记录预算内（k-100 ×1.059，c-66 ×1.051） | 0 |
| `git diff --check` | 无冲突标记（仅 CRLF 提示） | 0 |

**基线未重录**：`dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`、`kc_baseline/` 均未改动。

**skip/xfail 未增长**：全任务保持 `1 xfailed`（有登记原因），skip 0。

**交付后经独立只读复审**（D2）：发现并修复 2 个真实缺陷（多 case 时间身份、块 dtype 强转），
详见 [raw/query-artifact-evidence.md](query-artifact-evidence.md) §5。

## 1 验收条件逐条对账（SPEC §53–59）

### §55 slider/四杆/双叉臂/空间机构逐准静态样本有实际值或物理无效原因

覆盖文件 `tests/physics/test_roll_centres_by_topology.py`（双叉臂、五连杆、麦弗逊、扭转梁）
与 `tests/vehicle/test_screw_kinematics.py`、`tests/characteristics/test_kinematic_response.py`。
断言的是拓扑无关的性质——有限值、镜像对称、符号随摆臂朝向——
而非从单一 fixture 抄来的数字，因此新增拓扑不会因「没有对应常数」而无法验证。

「不以缺实现记合法无效」由 P0 的样本状态契约兜底：`SAMPLE_STATUSES` 七个状态均需给出物理原因，
未实现的路径报**查询错误**而不是返回某个状态（见 §58）。

### §56 坐标/frame 旋转、镜像、移动参考体、输入偏导与欠驱动审计

- 旋转/镜像/移动参考体：`tests/characteristics/test_values_and_measurements.py`（135→ 现 155 项中的相关用例），
  含本轮新增的 `legacy_wheel_camber`/`legacy_wheel_toe`/`legacy_lateral_axis_y`/`transformed_point` 覆盖。
- 输入偏导：`measurement_derivative` 的 `method ∈ {analytic, transpose, oracle_difference}`，
  **不使用生产差分 J**（SPEC §47 的禁止项），差分仅作独立 oracle。
- 欠驱动审计：`characteristics/motion.py::audit_boundary` 返回
  `rank/nullity/status/reason/residual_norm/redundant_rows/empty_rows`，
  且「全部允许运动均参与中心可辨识性检查」。

### §57 R1–R6 迁移兼容性，R3 真实调用默认与注入两条路径

见 `raw/retirement-evidence.md`。R3 的证据是**真实调用**而非符号存在：
`tests/vehicle/test_screw_kinematics.py::test_the_vehicle_report_really_calls_both_its_paths`
构造 `VehicleDeclaration`，分别以「无 hook」与「有 `instant_center_engine`」调用
`compute_vehicle_roll_centers`，并断言 hook **改变了答案**。
默认与注入路径均无对 `result._roll_center_primitives` 的私有访问。

### §58 查询/原生持久化随 P1 可用；通道缺失报查询错误；查询不调用 native 或资源重建

见 `raw/query-artifact-evidence.md`。三条实测：

1. 缺失通道报错并列出可用通道（`test_a_channel_that_was_never_requested_is_a_query_error`）。
2. 冷读不重解：把 `run_compiled` 换成抛错函数 + 计数器断言 **0**
   （`test_a_cold_read_does_not_invoke_the_kernel`）。
3. 不重建资源：把 `result.model` 换成访问即抛错的对象后应答仍可读
   （`test_the_read_path_does_not_reload_modelling_resources`）。

**诚实边界**：设计 §225 指出 `write_artifact` 的 `formats` 尚未驱动 CSV/Parquet 写表
（实测 `manifest["tables"]` 为空），故「导出」不等于「读取」，导出收口归 P6。
本步**不声称**导出已完成。

### §59 TODO 全部步骤完成行为验收，raw/validation.md 有真实证据

本文件即该证据；`TODO.csv` 六行状态与本文件、`PROGRESS.md` 三处一致。
父 `SUBTASKS.csv` 的 P1 状态在六步全部 DONE 后回填（见下）。

## 2 专项验证场景对账（SPEC §61–66）

| 场景 | 覆盖 |
|---|---|
| slider/四杆/双叉臂/空间机构解析或独立差分响应 | `test_roll_centres_by_topology.py`（4 拓扑 × 有限/对称/比值/符号） |
| 旋转/镜像/单位协变 | `test_values_and_measurements.py` + `test_kinematic_response.py` |
| 欠驱动但目标唯一 / 目标不唯一 / 不相容驱动 | `test_kinematic_response.py`（`audit_boundary` 三态） |
| 接地点高度不齐 / 多轴 / 权重缩放 / 零合力 | `characteristics/centers.py` 的 `_weights`/`_solve_vertical_forces`（`RESULTANT_FLOOR` 判定零合力） |
| 纯平移 / 有限轴退化 / 作用线平行或位于截面 / 自由力偶 | `motion.py` 的 `screw_from_twist` + `ANGULAR_SPEED_FLOOR`；`centers.py` 的 `section_intersection`/`WORK_TOLERANCE` |
| legacy 数值逐项等价 | `raw/retirement-evidence.md` §4/§5 数值对账表 |

SPEC §66 点名的五个文件实测 **52 passed**。

## 3 步骤 6 未新增功能代码

本步只执行门与场景并归档，除本文件与状态回填外**未改动任何 `src`/`tests` 代码**。
因此上表的通过即是步骤 1–5 全部产物的联合验收，而非某个局部。

## 4 已知缺口（不软化）

- **Adams Car 2024.1 未安装**（本机仅 `G:\MSC.Software\Adams\2025_1_1`），
  六组规则 G1–G6 保持 `pending_definition`；按 EPIC §36 不阻断 P1，
  但**严格阻断 P5 与 P6 的 Adams 兼容发布**。
- **R1–R6 生产消费者为 0**（仅测试调用）。这不是 P1 的缺陷，但必须在 P6 收口时
  要么接上真实消费者，要么明确登记为仅测试范围。
- **R7/R8 边界仍是泛化描述**，需在 P2 以精确符号/代码片段钉死。
- **R5 保持 `pending`**：按设计 §576 为测试范围 oracle，无生产消费者。
- `envelope_from_artifact` 重建的文档只带 `contract_version/manifest/status`，
  未还原 `case_identity`/`contract`/`kind`；若后续消费者需要完整原生文档需补。
- **`raw.times_s` 的逐 case 切片语义是既有行为**（逐 case 取 `base[:count]` 后相接，
  实测 `[10,11,20,21,22,23]` + 两个 case → `[10,11,10,11,20]`）。
  本步的查询层已按该语义正确索引，但该语义本身在大时间差的多 case 场景下是否合适
  属 `raw.py` 的既有设计问题，**不在 P1 范围内**，此处登记以免后续被误认为新引入。
- `sample_identities` 的栅格过短护栏（`disagree`）在当前 `raw.times_s` 语义下
  由 `raw` 自己的 `exceeds decoded time grid` 先拦下，故该护栏是**防御性**的，
  测试经 stub 触发（已在测试 docstring 中如实说明可达性）。
