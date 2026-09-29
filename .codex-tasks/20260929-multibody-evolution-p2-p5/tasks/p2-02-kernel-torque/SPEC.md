# SPEC：p2-02 内核旋转主动力矩元与 ABI 变更

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-02`。
> 形态：`single-full`。

## Goal

把 `SUBTASKS.csv` 中 `p2-02` 的 `acceptance_criteria` 拆成下面 5 条，逐条可判定：

1. **内核新增旋转主动力矩元素类型，五处打通**：`Model` 新增该元素类型；`ElementKind` 枚举、`element_wrench.hpp` 的 wrench 码、元素装配、`element_reader.cpp` 的读取、ABI 编组——全部打通。
2. **ABI 真源单点**：`mb_config/version.hpp`（ABI 真源，`EPIC.md` F3 行 116：`:27/34/37`，值为 `kAxleKernelAbiVersion = 16`、`kVehicleKernelAbiVersion = 31`、`kCoreKernelAbiVersion = 1`）与 Python 侧单一真源 `kernel/native.py:33-35` **同步**。D1 的口径（`EPIC.md` 行 60，2026-09-29 已修订）：新增元素类型**至少**要求轴与整车两个常量各 +1（`version.hpp:18` 的注释表明 `kVehicleKernelAbiVersion` 随轴结构联动）；**具体升哪个、升到多少由本行按代码实测确定并登记**——本行只受两条约束：必须**单点提交**（本行独占 `version.hpp` 与 `native.py`）、必须**两处真源同步**。`tests/architecture/test_kernel_abi_version_single_source.py` 必须保持绿。
3. **力元求值断言三条**：静止（ω ≈ 0）不产生反向加速；滑移饱和（抱死）时力矩饱和；倒车（ω < 0）符号正确——三条**各有断言**（`EPIC.md` 行 239 (c)、行 288）。
4. **分层与门禁**：`check_module_layering.py --strict --final` 保持 0 环；`legacy_surface_gate` 绿（`EPIC.md` 行 239 (d)）。
5. **零回归**：本行**只新增元素类型、不动既有元素的装配顺序**，故 13 个轴侧动态用例的 `dynamic_hash_sentinel.py --check` 仍**逐字节一致**（`EPIC.md` 行 239 (e)、行 311）。

## 写范围（允许改的路径）

照 `SUBTASKS.csv` 的 `p2-02` `notes`：本行**是**「本 Epic 唯一被授权改 `mb_config/version.hpp` 与 `kernel/native.py` 的行」。

- `packages/suspension_kernel/cpp/**`：`mb_input/types.hpp` 的 `enum ElementKind`（`EPIC.md` F2 行 114：今天只有 7 类）、`mb_model/types.hpp` 的 `Model` 元素向量（F2：已有 `bodies, constraints, coordinate_couplers, springs, ..., static_rotation_gauges` 等）、`element_wrench.hpp`、元素装配、`element_reader.cpp`、ABI 编组（`cpp/axle_dynamics/core_abi.hpp` 一系）。
- `packages/suspension_kernel/cpp/include/mb_config/version.hpp`（`:27/34/37`，F3 行 116）——**本行独占，行 225**。
- `packages/suspension_multibody/src/suspension_multibody/kernel/native.py`（`:33-35`，F3）——**本行独占，行 225**。
- `packages/suspension_kernel/tests/`（新增力元求值断言与元素类型用例）。
- 契约 schema 若需新增元素类型字段：`packages/suspension_contracts/src/suspension_contracts/contracts/assembly.schema.json` —— 但**必须与阶段一 03 的放置段改动串行**（`EPIC.md` 行 218，这是 `S1` 前置的第二个理由）。
- 本目录 `raw/`。

## 禁止触碰

- **`subsystems/element_build.py`**：构造分派段的力矩元分支归 **p2-03**（`EPIC.md` 行 216：「本节指定 `element_build.py` 的**构造分派段**归 p2-03、**轮胎段**归 p4-04」）。本行只交内核侧。
- **`subsystems/brake.py`、`subsystems/drive.py`、`templates/builtin.py`、`templates/roles.py`**：归 p2-04（`EPIC.md` 行 215、行 219）。
- **`templates/roles.py` 的角色表（含 `ROLES` 与 `:158-162` 的 import 期硬断言，F12 行 139）**：p2 的任何行**不得**改（`EPIC.md` 行 219）。
- **`preparation/vehicle_dynamic.py`**：力矩段归 p2-05、转向段归 p2-06、轮胎拒绝段归 p4-04（行 217）。
- **不动既有元素的装配顺序**（`EPIC.md` 行 239 (e) 与行 311）：不得重排、不得改既有元素的编组顺序。
- **不得新增内核模块以外的 C++ 能力**（行 103 Non-Goals）：除 D1 授权的一次 ABI 变更与 D2 可能的一次之外，本 Epic 不新增其它内核元素类型。**本行的新增只有这一个元素类型**。
- **不得重录任何基线**（行 228、D5 行 64）；**不得新增 skip/xfail**（行 229）。
- 其它子任务目录（p2-01、p2-03 ~ p2-06、p3/p4/p5 各行）、`EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`——不得改。
- `raw/` 中不得放未执行的内容（行 355）。

## 依赖与时机

- `depends_on = p2-01`（`SUBTASKS.csv` `p2-02`）。上游 S1（阶段一 Epic 01–07 全 `DONE`，`EPIC.md` 行 67–75）。**阶段一未完成之前不得置 `IN_PROGRESS`**（行 75）。
- 串行链：`p2-01 → p2-02 → p2-03 → p2-04 → p2-05 → p2-06`（行 203–204）。本行必须在 p2-01 的**路径快照与门禁起点值**落盘之后（`raw/gates_baseline.md` 与 `raw/baseline_notes.md` 是本行「13 个轴侧动态用例逐字节一致」的对照来源）。
- **本行必须在 D1 裁决之后开工**（`EPIC.md` 行 60、行 311：「D1 必须先裁决」；`SUBTASKS.csv` `notes`：「D1 必须先裁决」）。D1 未裁决前本行不得置 `IN_PROGRESS`。
- 顺序约束：p2-03 依赖本行的元素类型与 ABI 编组落地；p2-02/p2-03 若需新增契约 schema 的元素类型字段，**必须与阶段一 03 的放置段改动串行**（行 218）。
- 与阶段五的关系：本 Epic **最多两次 ABI 变更**——p2-02 的必要一次，p5-04 的一次视 D2 裁决（行 225、Done-When (j) 行 305）。**本行是其中第一次，且必须单独一行提交**（D1 行 60）。

## 判据与证据落点

逐条对应 `EPIC.md` 行 239 的 (a)(b)(c)(d)(e)。

1. **(a) 内核元素类型打通** → `raw/kernel_element_path.md`
   - 跑什么：在内核侧新增元素类型后，跑该类型的构造 + 求解用例（`packages/suspension_kernel/tests`）。逐项记录五处改动的 `file:line`：`mb_input/types.hpp` 的 `ElementKind`、`element_wrench.hpp` 的 wrench 码、元素装配、`element_reader.cpp`、ABI 编组。
   - 看什么：五处的文件与行号；新增类型能被读取并进入元素装配的证据（用例原文 + 退出码）。
   - 落点：`raw/kernel_element_path.md`。
2. **(b) ABI 真源单点** → `raw/abi_single_source.md`
   - 跑什么：`uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q`（含 `test_kernel_abi_version_single_source.py`）。
   - 看什么：`mb_config/version.hpp:27/34/37` 与 `kernel/native.py:33-35` 的**前后值成对记录**（改前 16/31/1，改后新值）；版本常量单一真源门绿；退出码。
   - 落点：`raw/abi_single_source.md`，含改前/改后对照表。
3. **(c) 力元求值三态断言** → `raw/torque_eval_assertions.md`
   - 跑什么：本行新增的三条求值用例（静止 ω ≈ 0 / 滑移饱和（抱死）/ 倒车 ω < 0）。
   - 看什么：每条断言的**测试文件:行 + 断言表达式原文 + 通过输出**；静止态断言必须证明「不产生反向加速」（例如净力矩与 ω 同号或为零，符号方向正确），倒车态断言必须证明符号正确。
   - 落点：`raw/torque_eval_assertions.md`。
4. **(d) 分层门禁** → `raw/layering_gate.md`
   - 跑什么：`uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final`；`uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check`。
   - 看什么：**0 环**的输出原文与退出码；`legacy_surface_gate` 绿。
   - 落点：`raw/layering_gate.md`。
5. **(e) 动态哈希零回归** → `raw/dynamic_hash_baseline.md`
   - 跑什么：`uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check`。
   - 看什么：13 个轴侧动态用例口径下的**逐字节一致性**结论；任一 bit 变化即**回退**（行 239 (e)、行 311、父行 `notes`：「若既有 13 个轴侧动态用例逐字节有变即回退」）。
   - 落点：`raw/dynamic_hash_baseline.md`，含改前（p2-01 起点值）与改后对照。

## Constraints（冻结约束）

与 `EPIC.md` 「冻结约束」（行 223–233）中与本行相关的条目：

- **内核 ABI 单点提交**（行 225）：**只有本行**可以改 `mb_config/version.hpp` 与 `kernel/native.py` 的版本常量；版本常量的单一真源由 `tests/architecture/test_kernel_abi_version_single_source.py` 守护，改动必须让它保持绿。**本 Epic 最多两次 ABI 变更**——本行是必要的一次，另一处视 D2 裁决（行 225、行 305）。
- **分层方向不可逆**（行 226）：`modeling -> templates -> subsystems -> preparation/studies -> cases`；`modeling/` 不得反向导入作者层；`report/` 不得 import/调用 native/kernel/solver，也不得自求力律。改动必须保证 `just check-fast` 秒级通过。
- **基线不得重录**（行 228、D5 行 64）：`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节不变是硬门；其余基线若确需变化，逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」（自由度与约束行数、惯量、轮心与接触点几何、轮胎力路径），质量与质心相同**不足以**证明等价。本行**不应**触发任何基线变化。
- **不得新增 skip/xfail**（行 229）。
- **每步落地后必须重跑**（行 230）：`just check-fast`；改结构后加跑 `tests/architecture`；触及求解路径加跑 `just gate-numeric`。**本行触及内核求解路径，故数值门三项必须全绿**。
- **`model_dump(mode="json")` 的产物不得被改变**（行 227）：`api.py:116`/`:284` 用它算 `model_hash`。本行是内核侧改动，不应触碰该形状，但若契约 schema 因新增元素类型字段而变化，必须证明 `model_hash` 口径未变。

## 风险与回退

- **ABI 变更（p2-02）是全 Epic 最大风险**（行 311）：新增内核元素类型会触及 ABI 编组、版本常量与内核模块 DAG。缓解：D1 必须先裁决；本行**只新增元素类型、不动既有元素的装配顺序**，故 13 个轴侧动态用例的 `dynamic_hash_sentinel.py --check` 应仍逐字节一致——**若变化即回退**；`check_module_layering.py --strict --final` 必须保持 0 环。
- **契约 schema 与阶段一 03 串行**（行 218）：若本行需新增元素类型字段，必须与阶段一 03 的放置段改动串行（`S1` 的第二个理由）；**不得并行改 `assembly.schema.json`**。
- **不新增其它内核元素类型**（行 103）：本行的新增仅此一个；任何额外的新增都越界。
- **既有失败**（行 320）：起点须以 p2-01 的 `raw/baseline_notes.md` 为准；任何新增失败阻断完成，不相关既有失败独立列明。

## Done-When

- [ ] 内核 `Model` 新增旋转主动力矩元素类型，`ElementKind`、`element_wrench.hpp` 的 wrench 码、元素装配、`element_reader.cpp` 读取、ABI 编组五处全部打通（`raw/kernel_element_path.md` 逐处给出 `file:line`）。
- [ ] `mb_config/version.hpp:27/34/37` 与 `kernel/native.py:33-35` 同步更新，`test_kernel_abi_version_single_source.py` 绿（`raw/abi_single_source.md` 含改前/改后值）。
- [ ] 力元求值三条断言齐备：静止（ω ≈ 0）不产生反向加速、滑移饱和（抱死）力矩饱和、倒车（ω < 0）符号正确（`raw/torque_eval_assertions.md`）。
- [ ] `check_module_layering.py --strict --final` 输出 0 环、退出码 0；`legacy_surface_gate.py --check` 绿（`raw/layering_gate.md`）。
- [ ] `dynamic_hash_sentinel.py --check` 在 13 个轴侧动态用例口径下**逐字节一致**（`raw/dynamic_hash_baseline.md` 含 p2-01 起点值与改后对照）；若变化已回退并记录。
- [ ] `just check-fast` 绿；本行触及求解路径，`just gate-numeric` 三项全绿；无新增 skip/xfail；未重录任何基线。
- [ ] 未触碰 `subsystems/element_build.py`、`subsystems/brake.py`、`subsystems/drive.py`、`templates/roles.py`、`templates/builtin.py`、`preparation/vehicle_dynamic.py`（用 `git diff --stat` 自证并记入 `raw/layering_gate.md`）。
- [ ] ABI 变更按 Done-When (j)（行 305）登记：本行的一次改动已记入 `raw/abi_single_source.md`，且 `test_kernel_abi_version_single_source` 绿。

## Final Validation Command

```bash
uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && uv run --no-sync pytest packages/suspension_kernel/tests packages/suspension_multibody/tests/architecture -q
```
