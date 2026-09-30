# 06 证据：对称成为声明（本轮范围）

## 已落地

| 项 | 位置 | 断言 |
|---|---|---|
| 侧成为声明（缺省＝对称对，D5） | `subsystems/types.py::AssemblyRequest.sides` | `tests/subsystems/test_optional_symmetry.py::test_the_default_still_pairs_both_sides` |
| 装配按声明取值 | `si_assembly._sides()`（`side_schema`、`side_bodies`、`side_content`、记录顺序、轮端挂接体）、`element_build.element_rows`（两个循环）、`wheel.sides(context)`、`suspension.side_body_order` | `test_one_declared_side_is_a_topology_not_a_hole` |
| **单轮（单侧悬架）装配并跑通一次 K/C study** | 同上 + `cases/kc_quasi_static::model_document` | `test_a_one_sided_assembly_runs_a_quasi_static_study`（文档只含 `wheel_drive_L`；求解返回有限状态且文档内无 `_R` 体） |
| 镜像为默认、显式 `name__R` 覆盖 | `subsystems/geometry.py::side_hardpoints` | `test_a_declared_right_side_overrides_the_mirror`（`__R` 键不进结果、键集两侧一致、左侧不受影响） |
| 默认路径零变化 | — | `snapshot.py --check` 零差异（七组合） |

## 未落地（阻断项）

1. **文档级 `sides`/`mirror` 声明 + `runtime_template_from` 的不再镜像路径**。这是 Done-When (b)「镜像写法与左右独立文件写法逐项一致」与 Goal 5「单侧文件契约」的兑现行。落点：`authoring/documents.py::TemplateDocument`（新增可选 `sides`/`mirror` 与校验）、`authoring/solver.py::runtime_template_from`（把声明传给 `_mirrored_parts`/`_mirrored_connections`）、`authoring/bridge.py`（给两者加 `mirror: bool = True`）、`template.schema.json`、`authoring/solver.py::assembly_request_for`（把条目声明的侧并成 `AssemblyRequest.sides`）。
2. **三轮整车（两悬架 + 一个单侧）**：需要 `subsystems/assembler.py::AxleEntry` 携带每条目的侧并在 `compose_entries_runtime` 里转发给 `si_assembly_for_axle`（约 3 行 + 用例）。
3. `SIDES` 收口的 grep 判据：`si_assembly._SIDES` 仍作为缺省值保留（D5 要求镜像仍为默认），消费点已全部改走 `_sides(request)`。

## 本轮验证

全量 `pytest packages/suspension_multibody/tests` → 1526 passed / 1 skipped / 1 xfailed；`tests/architecture` 147 passed；contracts+kernel 65 passed；ruff/ty 全过；三个架构门 OK；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致；K/C probe + `--actual-dir` parity OK；`case_parity_check` 8 families；`kc_perf_gate --check` 在预算内；`snapshot.py --check` 零差异。

---

## 收尾（2026-09-29）：上文「未落地」两条均已交付

1. **已交付**：`template.schema.json` 的 `sides`/`mirror`；`documents.TemplateDocument::_check_sides` + `declared_sides`/`mirrors`；`bridge._mirrored_parts(mirror=)`；`solver._mirrored_connections(mirror=)`、`runtime_template_from` 的 `_per_side_roles`、`_sides_from_file`、`_connection_name` 的按侧兜底。
2. **已交付**：`assembler.AxleEntry.sides` + `compose_entries_runtime` 转发；三轮整车用例；另修 `steering.py` 的侧硬编码。

**Done-When (b) 的实测判据**（`tests/authoring/test_symmetry_declaration.py`）：两种写法在模板层（`parts`、`connections` 逐字段）与运行层（体集合、逐体点坐标、约束名与类型）一致。
**登记的残余差异**：写两侧的文件其右侧点 label 带自身 token，镜像路线不带；坐标与约束一致。
