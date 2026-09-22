# 08 步骤 6：删除后扫描复跑与打包内容检查

## 1. 扫描命令与留档

```bash
uv run python .codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-08-delete/legacy_reference_scan.py \
  --report-after --json .codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-08-delete/raw/scan_after.json
```

退出码 `0`；结构化清单 `raw/scan_after.json`。**扫描器未做任何改动，未加豁免名单。**

## 2. 三次计数（含一处基线时间差，需注意）

| 时点 | 命令 | 总数 | by module |
|---|---|---|---|
| `raw/scan_before.json`（主代理，10:52，**早于 07 提交** `550566e`/11:45） | `--report-before` | **110** | analysis 26, core 33, elements 13, metrics 16, model 21, pac2002_scope 1 |
| `raw/scan_before_head.json`（本步，HEAD 上删除前） | `--report-before` | **89** | analysis 11, core 33, elements 13, metrics 10, model 21, pac2002_scope 1 |
| `raw/scan_after.json`（本步，删除后） | `--report-after` | **31** | analysis 3, core 16, elements 11, pac2002_scope 1 |

`110 → 89` 的 21 条差是 07 提交切走 `api.py`/`metrics`/`analysis` 调用方造成的（27 行消失 + 6 行因行号位移重现），
不是本步动作。本步的真实删除基数是 **89**。按 `--report-after` 要求的口径，删除后为 **31**。

## 3. 89 → 31 的逐模块归因

| 模块 | 前 | 后 | 差 | 归因 |
|---|---|---|---|---|
| `model` | 21 | 0 | −21 | `model/` 整包删除（含 `__init__`/`front_axle`/`mass`/`vehicle` 内部互引、`analysis/vehicle_physics.py:10-11`、`tests/model/*`、`tests/cases/*`、`tests/results/*`、`tests/vehicle/test_native_vehicle.py` 的导入全部切到 `preparation.assembly`） |
| `metrics` | 10 | 0 | −10 | `metrics/` 整包删除（`analysis/metrics.py`、`analysis/vehicle_physics.py:83` 的转发、`tests/metrics/test_metrics.py` 全部切到 `report.metrics`） |
| `analysis` | 11 | 3 | −8 | 7 个 `analysis/*.py` 删除；`analysis/__init__.py` 只留 A2 导出；剩余 3 条见第 4 节（A2 保留） |
| `core` | 33 | 16 | −17 | `core/rank.py`、`core/reactions.py`、`core/__init__.py:18,19` 删除；剩余 16 条见第 4 节（A1 保留） |
| `elements` | 13 | 11 | −2 | `elements` 自身保留（A1）；减少的 2 条来自 `tests/model/test_force_assembly.py`（切到 `preparation.assembly`）与 `analysis/vehicle_kc_time_domain.py` 删除 |
| `pac2002_scope` | 1 | 1 | 0 | 见第 5 节（误报） |
| **合计** | **89** | **31** | **−58** | |

## 4. 删除后 31 条残留的逐条归因

全部 31 条都能在 `raw/step3_deletion_record.md` 找到保留登记；除第 5 节那 1 条误报外，
**没有一条指向已删除模块**（`model`、`metrics` 计数为 0；`core/rank`、`core/reactions`、
`pac2002_scope.py`、7 个 `analysis/*.py` 无任何引用）。

### 4.1 `elements`（11 条）→ `step3_deletion_record.md` 第 2.1 / 2.4 节

| 引用 | 定性 |
|---|---|
| `src/.../api.py:49` | A1 保留：生产调用者 |
| `src/.../preparation/assembly/front_axle.py:17` | A1 保留：生产调用者 |
| `src/.../preparation/assembly/vehicle.py:20` | A1 保留：生产调用者 |
| `src/.../preparation/vehicle_dynamic.py:37` | A1 保留：生产调用者 |
| `src/.../elements/__init__.py:3,4,5`、`elements/assembly.py:19`、`elements/elastic.py:17` | A1 保留：包内自身引用（包本体保留） |
| `tests/elements/test_elements.py:13`、`tests/model/test_force_assembly.py:5` | A1 保留：覆盖保留包本构的测试 |

### 4.2 `core`（16 条）→ 第 2.2 / 2.4 节

| 引用 | 定性 |
|---|---|
| `src/.../core/__init__.py:12,27,28` | A1 保留：facade 对保留子模块的引用 |
| `src/.../core/constraints.py:49,50` | A1 保留：残差/Jacobian 内核引用 `rigid_body`/`spatial` |
| `src/.../core/rigid_body.py:21` | A1 保留：包内自身引用 |
| `src/.../elements/assembly.py:17,18`、`elements/elastic.py:10,11` | A1 保留：`elements/` 本构依赖 `core.rigid_body`/`core.spatial`（这正是 `core/` 保留的原因） |
| `tests/core/test_constraints.py:5,20`、`tests/core/test_rigid_body.py:5,11`、`tests/core/test_spatial.py:5`、`tests/elements/test_elements.py:6` | A1 保留：覆盖保留 `core` 模块的测试 |

### 4.3 `analysis`（3 条）→ 第 2.3 节

| 引用 | 定性 |
|---|---|
| `src/.../analysis/__init__.py:11` | A2 保留：facade 对 `vehicle_physics` 的引用 |
| `src/.../vehicle/service.py:30` | A2 保留：`compute_static_wheel_loads` 的唯一生产调用者 |
| `tests/physics/test_vehicle_physics.py:5` | A2 保留：该文件按任务书保留不动 |

## 5. 残留的第 31 条：README 归档说明中的 `pac2002_scope.py`（主代理裁决后保留）

`packages/suspension_multibody/README.md:47 [text_reference/pac2002_scope.py]`。

**主代理裁决（2026-09-22）**：这不是误报，而是**真实且应当保留**的一条——该行是 08 新增的
迁移说明原文（「顶层 `pac2002_scope.py`（06 迁至 `schema/pac2002_scope.py` 等）」）。
它是对「旧归属 → 新归属」的**归档描述**，与 08 SPEC 目标 5「迁移说明」的要求一致，
删掉反而丢文档价值，因此**不为通过扫描而复写文档**。该条计入「保留项」而非「残留违规」。

**扫描器同步修正（主代理）**：初版对裸词 `pac2002_scope` 的例外规则会把活模块
`schema/pac2002_scope` 也匹配进去，产生真正的误报；现改为只匹配**包限定形式**或
**带 `.py` 扩展名的旧文件形态**（见 `legacy_reference_scan.py` 的 `TEXT_REFERENCE` 注释）。
修正前后：31 → 30 条（去掉误报）。

## 6. 打包内容检查

```bash
uv build --package suspension-kernel && uv build --package suspension-multibody
```

两个包均构建成功（退出码 `0`），产物：

| 产物 | 条目数 | 旧模块归属 |
|---|---|---|
| `dist/suspension_kernel-0.1.0-py3-none-any.whl` | 12 | 无 |
| `dist/suspension_kernel-0.1.0.tar.gz` | 150 | 无 |
| `dist/suspension_multibody-0.1.0-py3-none-any.whl` | 132 | 无 |
| `dist/suspension_multibody-0.1.0.tar.gz` | 283 | 无 |

按正则 `suspension_multibody/(core/rank|core/reactions|model/|metrics/|pac2002_scope\.py|analysis/(_geometry|compliance|metrics|benchmarks|time_domain_physics|vehicle_kc_time_domain|time_signals)\.py)`
检查四个产物的文件清单：**命中 0**。wheel 内 `suspension_multibody/` 只含
`analysis/{__init__,vehicle_physics}.py`（A2）、`core/{__init__,constraints,rigid_body,spatial}.py`（A1）
与 `elements/{__init__,assembly,base,elastic}.py`（A1）。

`Requires-Dist` 无旧包依赖，**不含任何已删模块的点分导入路径**（`core.rank`、`model.*`、
`metrics.*` 均无）。其 `Description` 是 README 原文，其中迁移说明按名字列出了已删目录。
**主代理裁决**：验收 7 的判据是「文件清单与元数据不含旧模块**归属**」，按名字列举迁移对照
不构成旧归属；改建 README 反而丢掉 SPEC 目标 5 要求的迁移说明，故不改。

## 7. 主代理对三处上报冲突的裁决（2026-09-22）

1. **`--check --final` 退 1 是 A1 下的正确结果，非缺陷**。其 finding 只有两类：
   `legacy package still present`（`analysis`/`core`/`elements`，A1/A2 保留）与 test-scope 的
   `legacy_module_import`（`tests/{core,elements,model,physics}` 对保留包的覆盖测试）。且门禁
   自身测试 `tests/architecture/test_legacy_surface_gate.py:114-119` **断言 final 必须在保留
   旧包时退 1**——即「`--final` 退 0」与「architecture 全绿」在 A1 下不可兼得。08 验收 3 与
   09 G2 的判据均为「**已无生产调用者**的部分不存在」，不是「所有旧包缺席」。故**不放宽门禁、
   不改测试**，final 保持退 1 并在此登记。
2. **`core/constraints.py` 的 `residual`/`jacobian` 保留是对的**。其消费者是
   `ConstraintSystem.residual/jacobian`（同文件），只删模块级函数会立即 `NameError`（那才是
   半删）。fixer 按「保留并说明理由」处置，主代理确认。
3. **`ty check .` 退 1 已由主代理修复**：唯一诊断来自本 Epic 目录下 05 的证据探针按设计引用
   已退役的 `analysis.benchmarks`。该文件是归档 provenance、非受检面，故在 `pyproject.toml`
   的 `[tool.ty.src] exclude` 增加 `.codex-tasks/`（附理由注释）。修复后 `ty check .` 退出 0。
