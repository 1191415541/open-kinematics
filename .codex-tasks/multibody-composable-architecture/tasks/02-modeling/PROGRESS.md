# 02 基础模型、SI 与依赖解环

## Recovery

- 任务：`02 建立基础模型并消除导入闭环`。形态：single-full。
- 父任务：`.codex-tasks/multibody-composable-architecture`。依赖 01（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 3 行；输入规格 `../../TASKS.md` 第 02 节。
- 本目录：`SPEC.md`、`TODO.csv`、`PROGRESS.md`。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/modeling packages/suspension_multibody/tests/architecture/test_import_boundaries.py -q`

## 实测确认的缺陷

`preparation/assembly/types.py` 是叶子类型模块，但导入它必然执行 `preparation/assembly/__init__.py`，后者导入 `front_axle`，而 `front_axle` 又导入 `elements` 与 `subsystems`——于是单独导入一个声明类型会连带加载整条作者层链。实测证据：

```
import suspension_multibody.preparation.assembly.types
  -> sys.modules 含 elements, elements.elastic, preparation.assembly.front_axle, subsystems.assembly
```

更严重的是真实的**循环**：`elements/__init__ → elements.assembly → preparation.assembly.types → preparation/assembly/__init__ → front_axle → elements`（半初始化）。实测复现：

```
import suspension_multibody.elements
  ImportError: cannot import name 'AntiRollBarElement' from partially initialized
  module 'suspension_multibody.elements' (most likely due to a circular import)
```

这个循环此前被 `api.py:35-44` 的导入顺序补丁掩盖：包根 `__init__.py` 先 `from .api import ...`，而 `api.py` 又在字母序之外先导入 `preparation.assembly`，于是循环在「正确的进入顺序」下恰好不显现。代价是 `import suspension_multibody` 一次加载 111 个子模块，低层模块无法独立使用。

## 做了什么

**低层搬迁（`modeling/`）**

- `modeling/primitives/spatial.py`：原 `preparation/geometry.py`，355 行，`git mv` 原样搬迁（保留字节，含 12 个 numba 内核）。
- `modeling/primitives/joints.py`：原 `preparation/assembly/types.py`，278 行，同法搬迁。
- `modeling/identity.py`：稳定 `EntityId`（实例路径 + 局部 ID），分隔符不可出现在局部 ID 中，保证渲染可反解。
- `modeling/ports.py`：几何端口与通道端口分型；需求与端口按 role/capability/labels 匹配，label 精确相等（左不得绑定右）；`required=False` 显式标记可消失分支并记录随之消失的输出。
- `modeling/instance.py`：只读 `ModelFragment` + `FragmentProvenance`；重复实体拒绝（不 last-write-wins）；点必须引用已声明刚体。
- `modeling/assembly.py`：`Assembly`（被测体，保留嵌套不摊平，`NestedInstance`）与 `SimulationAssembly`（+试验台，生成连接，带结构指纹与绑定记录）。
- `modeling/units.py`：`METRES_PER_MILLIMETRE = 1e-3` 单点定义。

**解环**

- 19 处内部导入从 `preparation.assembly.types` / `preparation.geometry` 改指 `modeling.primitives.*`，绕过 `preparation/assembly/__init__`。涉及 13 个源文件。
- 包根 `__init__.py` 改为 PEP 562 惰性公开面（`__getattr__` + `_PUBLIC_NAMES` 表）：公开名不变，但不再 eager 执行 `api`。
- 删除 `api.py` 的导入顺序补丁及其长注释；循环消失后顺序不再 load-bearing。

**迁移转发**

- `preparation/geometry.py` 与 `preparation/assembly/types.py` 重建为纯转发模块，42 处既有引用零改动即可用；12 在调用方迁移完成后删除。

## 验证记录（实测退出码）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/modeling tests/architecture/test_import_boundaries.py -q` | 0 | 主验收 71 passed |
| `pytest tests/architecture tests/contract tests/core tests/joints tests/elements -q` | 0 | 169 passed |
| `python scripts/kc_parity_check.py --check` | 0 | 冻结快照容差内 |
| `python scripts/case_parity_check.py` | 0 | 8 families 全 PASS，axle/vehicle_dynamic 逐位一致 |
| `python scripts/dynamic_hash_sentinel.py --check` | 0 | combined sha256 与 01 冻结值一致（见下） |
| `ruff check .` | 0 | All checks passed |

## 门禁设计（`tests/architecture/test_import_boundaries.py`，30 项）

在**独立子进程**中验证，避免测试进程已被污染的假通过：

1. 9 个低层模块各自导入时不得连带 `templates/subsystems/rigs/connections/preparation/simulation/kernel/report`。
2. 9 个公开入口各自可单独导入。
3. 任一入口先导入后，低层仍可达。
4. **全序对扫描**：9×8 = 72 个「先 A 后 B」组合，任一失败即报出具体对。
5. 导入产品不得加载 `adams`。
6. 导入低层模块不得执行包根公开面。

第 6 项是本次改造成效的直接度量：改造前它报告 111 个模块被连带加载，改造后为空。

## 未覆盖与保留

- 本轮未改动 `templates/`、`subsystems/`、`rigs/` 的语义实现（归 03/04/05），只改了它们的导入路径。
- `preparation/geometry.py`、`preparation/assembly/types.py` 两个转发模块是过渡产物，归 12 删除。
- 低层的 `PortRequirement` 只做「能否接受」的判定；真正的绑定求解与歧义拒绝归 04。

## 后续补记：02 遗留的第二个循环（04 任务期间发现并修复）

02 的入口清单只列了 9 个「重要」子包，因此漏掉了一条与低层无关的真环：
`results/vehicle.py:19` 从 `preparation.vehicle_dynamic` 导入常量
`_PRESCRIBED_STEERING_TYPES`，而 `preparation.vehicle_dynamic` 又（经
`results` 包）导入 `results` —— 单独导入 `suspension_multibody.results` 或
`preparation.vehicle_dynamic` 会抛 `ImportError: partially initialized module`。

它在 02 期间未被发现，因为全量 pytest 总是先导入包根，顺序恰好掩盖了它；
只有在 04 把入口清单扩到**全部 24 个顶层子包**并做全序对扫描后才暴露。

修法：该常量是内核契约知识（哪些 steering actuator kind 会产生约束行），
移到两侧共同依赖的 `axle_dynamics/schema.py` 作为 `PRESCRIBED_STEERING_TYPES`；
`results/vehicle.py` 与 `preparation/vehicle_dynamic.py` 均改为从契约层导入，
后者保留同名转发以维持 `__all__` 契约。

实测：`python -c "import suspension_multibody.results"` 与
`... preparation.vehicle_dynamic` 均退出码 0；`case_parity` 8 families 全 PASS；
`tests/vehicle` + `tests/results` 86 passed / 1 xfailed。

门禁同时收紧：`PUBLIC_ENTRY_POINTS` 由 9 项扩为 24 项（全部子包），
扫描项从 30 增到 60。清单刻意不「精选」，因为一个环恰恰由两个各自正常的模块构成，
而精选清单必然漏掉没人想到要写下的那一对。
