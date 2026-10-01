# wheel 文件链现状 + `VerticalTireElement` 引用点全清单（p4-01 冻结证据 · F14/F15）

## F14：wheel 文件链现状

### 1. `subsystems/wheel.py` 的模板来源
- `wheel.py:38` `from ..templates.builtin import WHEEL`
- `wheel.py:57-62 template_instance`：
  ```python
  def template_instance(context: SubsystemContext) -> SubsystemInstance:
      """Return the wheel template this assembly reads."""
      requested = _requested(context)
      if isinstance(requested, SubsystemInstance):
          return requested
      return instantiate(WHEEL, mode=context.mode)
  ```
- `wheel.py:65-67 _requested`：`return getattr(context.request, "wheel_template", None)`
- 模块 docstring `wheel.py:22-30` 明说：`AssemblyRequest` 现在**已**带 `wheel_template` 字段（docstring 里写「carries no wheel field yet」是**过期注释**，实际字段已存在，见第 3 项）—— 记录为**文档过期**，非阻塞。
- F14 称 `wheel.py:44-59 _instance` / `:22` 内置模板 import：实测当前为 `template_instance`（`:57-62`）与 `:38` import——**锚点漂移**（阶段一 04/04b 已改），只记录不改父文件。

### 2. `authoring/solver.py` 的 `_FILE_ROLE_TEMPLATES` 现值与注释
- 现值 `solver.py:726`：
  ```python
  _FILE_ROLE_TEMPLATES: tuple[str, ...] = ("steering", "chassis", "wheel")
  ```
- 原注释 `solver.py:718-725`：
  ```python
  #: field cannot hold; the steering, chassis and wheel roles have one entry each.
  #: The wheel role used to be absent because its wheel centre is a per-side mount
  #: on a body the wheel template did not own -- which stopped being true when the
  #: wheel subsystem became the wheel end's producer: a file's wheel subsystem
  #: describes its own wheel body, so the mount has somewhere to hang and the
  #: template belongs on the request like any other role's.
  ```
- 使用点 `solver.py:705-710`：`if role in _FILE_ROLE_TEMPLATES: templates[f"{role}_template"] = ...`
- **F14 称 `_FILE_ROLE_TEMPLATES = ("steering","chassis")`（不含 wheel）——已过期**：实测**含 wheel**。这是本行实测与父 Epic 事实的**唯一实质漂移**，p4-04 的判据 (c)（「若阶段一未放开 wheel 角色，则由本行放开」）已无剩余项（阶段一 04b 已放开）。

### 3. `AssemblyRequest` 模板字段清单与映射表
- 字段 `packages/suspension_multibody/src/suspension_multibody/subsystems/types.py:148-166`：
  ```python
  suspension_template: object | None = None   # :148
  steering_template: object | None = None     # :164
  chassis_template: object | None = None      # :165
  wheel_template: object | None = None        # :166
  ```
  **`wheel_template` 已存在**（F14 称「无 wheel_template」——已过期）。
- 映射表 `types.py:275-280 _ROLE_TEMPLATE_FIELD`：
  ```python
  _ROLE_TEMPLATE_FIELD: dict[str, str] = {
      "suspension": "suspension_template",
      "steering": "steering_template",
      "chassis": "chassis_template",
      "wheel": "wheel_template",
  }
  ```
  （F14 称映射 `:249`——实测为 `:275`，**锚点漂移**。）

### 4. 仓库中是否存在 `*.subsystem.json` / `*.sub.json` / `*.tpl.json` 实体文件
命令：
```bash
git ls-files '*.subsystem.json' '*.sub.json' '*.tpl.json' '*.asy.json'
```
输出：**空**（退出码 0），计数 `0`。
补充（含未跟踪）：
```bash
find . -name '*.subsystem.json' -o -name '*.sub.json' -o -name '*.tpl.json' -o -name '*.asy.json'
```
输出：**空**。
→ **仓库中不存在任何此类实体文件**（`{name}.tpl.json` / `{name}.sub.json` / `{name}.asy.json` 只是 `authoring/security.py` 的命名约定）。路线图说的 `wheel.subsystem.json` 是**尚不存在的目标物**。
`git ls-files 'packages/**/*.json'` 显示已跟踪的 `.json` 全是 schema、baseline、测试数据，无子系统/模板/总成文档实例。

---

## F15：`VerticalTireElement` 全仓引用点清单

命令：
```bash
grep -rn "VerticalTireElement" packages/ docs/ | grep -v "\.pyc"
```
（排除 `*.pyc` 二进制命中）

### 定义
- `packages/suspension_multibody/src/suspension_multibody/modeling/primitives/elements.py:567 class VerticalTireElement`（F15 锚点未漂移；力律 `:575-589` 压缩-only，`force = [0,0,stiffness*compression]`）。
- `modeling/primitives/__init__.py:21`（import）、`:63`（`__all__`）。

### 构造（本 Epic 不改；p4-04 是「消除补丁」边界）
- `subsystems/element_build.py:31`（import）、`:136-144 _tire`（构造）、`:68-69`（分派 `if row.kind == "tire": return _tire(...)`）。**归属：p4-04（如必要）/ 其它**；F15 称 `:31/136/138` 与分派 `:68-69`，实测一致。

### 按类型过滤删除（历史补丁）— **全仓唯一 `isinstance` 过滤**
- F15 称 `subsystems/assembler.py:231`。**实测：`assembler.py` 现已无 `isinstance` 过滤**（`grep -n "isinstance" assembler.py` 零命中）。
  现原文在 `assembler.py:416-422`（注释）说明该过滤**已被删除**：
  ```python
  # The axle contributes its own elements as they are.  It used to have its
  # vertical tires deleted here, because the vehicle's wheel ends carry
  # tires too; the axle is now composed *without* the wheel role
  # (``_AXLE_ROLES_IN_A_VEHICLE``), so there is no second statement about a
  # wheel to delete and the assembly stage no longer decides the wheel's
  # lifecycle by a type test.
  elements.extend(_rename_dataclasses(axle.elements, body_map, entry.prefix))
  ```
  替代机制：`assembler.py:88-95 _AXLE_ROLES_IN_A_VEHICLE: frozenset[str] = DEFAULT_AXLE_SUBSYSTEMS - {"wheel"}`。
  → **F15 的 `assembler.py:231` 已过期**：那个 `isinstance` 过滤**不存在了**（**属阶段一 04 已交付**）。测试反向固化于 `tests/subsystems/test_wheel_lifecycle.py:225-226`（断言源码中不含 `isinstance(element, VerticalTireElement)`）。**归属：阶段一 04 已完成 → p4-04 只需独立复验。**

### 试验台期类型判定
- `subsystems/rig_link.py:297`：
  ```python
  if type(element).__name__ != "VerticalTireElement":
      continue
  ```
  （函数 `_unloaded_radius` 起点实测 `:294`，F15 称 `:308`；调用 `:202`）
- F15 称 `rig_link.py:278 _is_replaced_tire` / `:308 _unloaded_radius` / `:333 _reown_tires`：
  - `_unloaded_radius` 实测在 `:294`（`type(...).__name__` 判定在 `:297`）。
  - `_reown_tires` / `_is_replaced_tire` **全仓源码零命中**（grep 只在路线图文档 `:30/125/170` 命中）——**已删除**（阶段一 05 已交付）。
  → **归属：阶段一 05 已完成，本 Epic 不碰**（F15 亦如此标注）。

### native 侧拒绝
- `preparation/vehicle_dynamic.py:55`（import `VerticalTireElement`）。
- `preparation/vehicle_dynamic.py:940-944`：
  ```python
  elif isinstance(element, VerticalTireElement):
      raise ValueError(
          f"vertical tire element {element.name!r} must be represented by the native tire ABI"
      )
  ```
  （F15 称 `:55/900`；`:900` 实测为 `BushingElement` 分支，`VerticalTireElement` 拒绝在 `:940`——**锚点漂移**。）**归属：p4-04（轮胎拒绝段，EPIC.md:225）。**
- 同段 `:936-939` 还有 **ARB 拒绝**（见 `arb_two_physics.md`）。

### 其它按名分流
- `compilation/model_view.py:179`：`if type(element).__name__ == "VerticalTireElement": tires.append(element) else: elements.append(element)`。**归属：其它行（本 Epic 未点名改动）。**
- `studies/bridge.py:125`（`_READABLE_ELEMENTS` 含 `"VerticalTireElement"`）；`:334`（`tires = tuple(_vertical_tire(...) for element ... if type(element).__name__ == "VerticalTireElement")`）。**归属：其它行。**
- `cases/kc_quasi_static/contract.py:354`：`if type(element).__name__ != "VerticalTireElement": continue`。**归属：其它行。**

### 测试引用（只读，供判据）
- `tests/model/test_force_assembly.py:8/81`（`isinstance(item, VerticalTireElement) == 2`）。
- `tests/modeling/test_elements.py:17/133`。
- `tests/subsystems/test_rig_link.py:188/193`。
- `tests/subsystems/test_wheel_lifecycle.py:15/209/225/226`（`isinstance(element, VerticalTireElement)` 不在源码中断言）。

### 归属汇总
| 引用点 | 归属 |
|---|---|
| 定义 `modeling/primitives/elements.py:567` | 本行只读 |
| 构造 `element_build.py:31/68-69/136-138` | p4-04（如必要）/ 其它 |
| `assembler.py` 的 `isinstance` 过滤 | **已删除（阶段一 04）→ p4-04 只复验** |
| `rig_link.py:297`（及已删的 `_reown_tires`） | 阶段一 05，本 Epic 不碰 |
| `preparation/vehicle_dynamic.py:940` 拒绝 | p4-04 |
| `compilation/model_view.py:179` | 其它行 |
| `studies/bridge.py:125/334` | 其它行 |
| `cases/kc_quasi_static/contract.py:354` | 其它行 |
