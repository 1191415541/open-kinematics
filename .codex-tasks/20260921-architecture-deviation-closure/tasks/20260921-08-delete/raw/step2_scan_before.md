# 08 步骤 2：全仓扫描基线（删除前）

工具：`tasks/20260921-08-delete/legacy_reference_scan.py`（本任务交付物）。
命令：`uv run python .codex-tasks/20260921-architecture-deviation-closure/tasks/20260921-08-delete/legacy_reference_scan.py --report-before --json raw/scan_before.json`
退出码：`0`。原始输出：`raw/scan_before.log`、结构化清单 `raw/scan_before.json`。

## 1. 删除前基线

```text
scan moment      : before
retired modules  : core, elements, model, analysis, metrics, pac2002_scope
references       : 110
by module        : {'analysis': 26, 'core': 33, 'elements': 13, 'metrics': 16, 'model': 21, 'pac2002_scope': 1}
by form          : {'absolute_import': 43, 'relative_import': 66, 'text_reference': 1}
```

未加任何豁免名单。排除项按**目录**定义（历史任务记录 `.codex-tasks/`；生成目录 `build`、`dist`、`__pycache__`、`.venv`、缓存、`artifacts`），这些不是仓库的现役面。

## 2. 覆盖的引用形式（实测）

| 形式 | 条数 | 说明 |
|---|---|---|
| `relative_import` | 66 | `from ..core import X`、`from ...elements import X`、`from . import core`（含包内自身引用） |
| `absolute_import` | 43 | `from suspension_multibody.core import X`、`import suspension_multibody.analysis`（含 `as` 别名） |
| `text_reference` | 1 | `packages/suspension_multibody/README.md:36` 的 `pac2002_scope` 提及（文档，属步骤 5 的文档同步范围） |
| `dynamic_import` | 0 | 当前代码无对旧模块的动态导入（扫描器具备该能力，见下） |

**扫描器的判别力**（本文件记录能力，便于 08 复跑时信任计数）：

- AST 解析 Python，**绝不导入被扫文件**——半删状态下仍可运行。
- 识别绝对/相对/别名/`importlib.import_module`/`__import__`（含模块级字符串常量参数）。
- 文本形式要求**包限定前缀**（`suspension_multibody.core`）才能算引用，因为 `analysis`/`model`/`core`/`metrics` 是普通英文词——否则 README 里的「validation analysis」会被算成依赖，使计数失去证据价值。`pac2002_scope` 无英文读法，故其裸形式也计入。

## 3. 引用分布（决定 08 的删除范围）

| 区域 | 条数 | 处置 |
|---|---|---|
| `src/**` 内**旧包自身** | 54 | 随旧包一起删除（A1 保留项除外） |
| `src/**` **旧包之外** | 12 | 见第 4 节，逐条定性 |
| `tests/**` | 40 | 随测试归属调整；08 步骤 4 改造 |
| `scripts/**` | 3 | 08 迁移 |
| 根/文档 | 1 | 08 步骤 5 同步 |

## 4. `src/**` 旧包之外 12 条的技术定性（08 的迁移对象）

| 引用 | 定性 | 08 处置 |
|---|---|---|
| `api.py:34` → `analysis.compliance` | ① 迁 `report/` | 切到 `report/compliance`，随后删旧体 |
| `api.py:35` → `analysis.vehicle_kc_time_domain` | ① 迁 `report/`+`simulation/replay` | 切到新入口 |
| `api.py:42` → `metrics` | ① 迁 `report/metrics` | 切到 `report/metrics` |
| `axle_dynamics/contract_run.py:100` → `metrics` | ① 迁 `report/metrics` | 同上 |
| `vehicle/service.py:34` → `metrics` | ① 迁 `report/metrics` | 同上 |
| `adams/reference.py:10` → `analysis._geometry` | ① 迁 `report/geometry` | 同上 |
| `cases/kc_quasi_static/workflow.py:7` → `analysis._geometry` | ① 迁 `report/geometry` | 同上 |
| `vehicle/service.py:30` → `analysis.vehicle_physics` | **③ 保留（A2）** | **不迁、不删**（静轮荷最小范数求解，见 `../20260921-05-native-facts/raw/step4_static_wheel_loads_registration.md`） |
| `api.py:39` → `elements` | **③ 保留（A1）** | **不迁、不删**（元件本构，见 `raw/step1_symbol_matrix_closure.md` 第 2.1 节） |
| `preparation/assembly/front_axle.py:17` → `elements` | **③ 保留（A1）** | 同上 |
| `preparation/assembly/vehicle.py:20` → `elements` | **③ 保留（A1）** | 同上 |
| `preparation/vehicle_dynamic.py:37` → `elements` | **③ 保留（A1）** | 同上 |

**结论（08 的目标删除集）**：

- **删**：`metrics/`（整包）、`analysis/{_geometry,compliance,metrics,benchmarks,time_domain_physics,vehicle_kc_time_domain,time_signals}.py`、`model/`（整包）、`core/{rank,reactions}.py`、`pac2002_scope.py`；以及 `core/`、`analysis/` 包内已无现役职责的其余文件（按步骤 3 逐项核对）。
- **保留**：`elements/`（4 文件，A1）、`analysis/vehicle_physics.py` 及其 `__init__` 支撑（A2）——因此 **`analysis/` 与 `elements/` 两个包目录本体不删**，其存在有逐项理由；这正符合 A1「不设必须删完」。
- **连带**：`core/constraints.py` 的 `residual`/`jacobian` 模块级函数在删 `core/reactions.py` 时须一并裁定——其唯一消费者是 `reactions.py`，但 `ConstraintSystem`（`:499,506`）仍持同名方法，不得留下半删状态。

## 5. 与 07 的接口

第 4 节标 ① 的 7 条引用，其调用方位于 `api.py`、`adams/`、`cases/`、`axle_dynamics/`、`vehicle/`——**均不在 07 SPEC 的可写范围内**（07 可写 `report/**`、`simulation/replay.py`、`results/timeseries.py`、`analysis/**` 残留、`scripts/*`、`tests/{metrics,analysis,results,cli,data,architecture}`）。因此这 7 条由 **08 在 07 交付 `report/` 之后切换**，与 08 SPEC 目标 1「迁移剩余调用方」一致。07 交付前不得删除 `analysis`/`metrics` 任何文件本体。
