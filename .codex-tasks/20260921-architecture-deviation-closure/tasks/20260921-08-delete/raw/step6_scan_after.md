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

## 5. 唯一非保留项残留：`pac2002_scope` 文本引用（误报）

`packages/suspension_multibody/README.md:59 [text_reference/pac2002_scope]`。

该处文本是 `` `schema/pac2002_scope` ``（活模块，06 之后的归属），
扫描器 `legacy_reference_scan.py:107-109` 对裸词 `pac2002_scope` 的例外规则无法与已退役的顶层模块区分。
详见 `raw/step3_deletion_record.md` 第 5 节；本步未改扫描器、未加豁免、未为通过而复写文档。

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

元数据（METADATA）：`suspension-kernel` 描述为 0 命中；`suspension-multibody` 的
`Requires-Dist` 无旧包依赖，**不含任何已删模块的点分导入路径**（`core.rank`、`model.*`、`metrics.*` 均无）。
需注意：其 `Description` 是 README 的原文，而本步新增的「Python 模块结构（08 之后）」一节在叙述里
**按名字列出了已删除的目录**（`model/`、`metrics/`、`core/rank.py`、`core/reactions.py`、`pac2002_scope.py`）。
这是迁移说明的叙述文本，不是模块归属；若验收口径要求元数据正文完全不出现这些词，需要把该节从 README 移出
（属需主代理裁决的口径问题）。
