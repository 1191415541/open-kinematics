# PROGRESS：04 轮端生命周期统一

> 父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `04`

## Session Start

- **Date**: （未开工）
- **Task name**: 20260929-04-wheel-lifecycle
- **Task dir**: `.codex-tasks/20260929-assembly-layer-rework/tasks/20260929-04-wheel-lifecycle/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（6 步）
- **Environment**: Python 3.12 / uv / pytest

## Context Recovery Block

- **Current milestone**: #1 — 单轴与整车引用同一份 wheel 子系统文件
- **Current status**: NOT_STARTED
- **Last completed**: 无
- **Current artifact**: `SPEC.md`
- **Key context**: 本子任务**尚未开工**——`SPEC.md` / `TODO.csv` 是规划产物，本文件是它的恢复块，未执行任何步骤、未产生任何证据。开工前置：`03` 完成（`03` 与 `04` 都触及 `subsystems/vehicle_assembly.py` 与 `subsystems/vehicle_parts.py`，必须串行）；`01` 的 `#1` 完成（`snapshot.py --check` 才可用作"产物是否变化"的判据）。
- **Known issues**: 本行唯一硬门是 D2——凝结后 `kc_baseline` 必须逐位不变；**严禁重录**。`kc_parity_check.py` 不带 `--actual-dir` 时是拿冻结快照与自身比较（恒过），不构成证据。
- **Next action**: 先跑一次 `tasks/20260929-01-freeze/snapshot.py --check`（待 01 的 `#1` 完成后）与数值门三项记录起点，再按 `TODO.csv` 从 `#1` 开始。

## 待开工登记（规划产物，非实施记录）

- 本子任务尚未开工；`SPEC.md` 与 `TODO.csv` 为规划产物，所有 `TODO.csv` 行 `status` 均为 `TODO`、`completed_at` 为空。
- 本文件只记录开工所需的恢复信息与已确定的口径，**不得**用规划文本冒充实施记录；`raw/` 只存**已执行**的证据。

## 已确定的口径（来自父 Epic 裁决，不待实施确认）

- **D2（裁决）**：不反转既有裁决 **D9**（单轴侧车轮归悬架试验台），**严禁重录 `kc_baseline`**。走"文件层统一 + 装配期刚性凝结"：单轴与整车引用**同一份** `wheel.subsystem.json`；装配单轴 K/C 试验台（轮心驱动工况）时把车轮刚体与轮毂刚体在内存中刚性凝结。硬门是凝结后**实体集合 / 约束行数 / 自由度不变且 `kc_baseline` 逐位不变**。
- **凝结复用既有机制**：`subsystems/vehicle_parts.py` 的 `_merge_fixed_wheel`（`:572-610`；`:482-508` 是 `wheel.mount_joint_kind == "fixed"` 的分支，按复合质量属性把车轮质量/惯量并入 mount body 且不建独立车轮体）与同文件的 `_fuse_welded_bodies`（由 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 恢复，F5b）。不新造凝结机制。
- **与 20260921 Epic A3 的关系（必须点名，避免两处"凝聚"语义混淆）**：20260921 Epic 的 A3 裁决把**整车侧**生产路径改为"不凝聚、weld 交内核 `fixed` 关节"；那是**另一处**的取舍，与本 Epic **单轴 K/C 侧**要凝结不冲突。本行只动单轴 K/C 侧，不改整车侧的 A3 结论。
- **不得迁移轮胎质量所有权**：凝结只改求解拓扑的表示，质量 / 质心 / 惯量口径不变（`raw/mass_ownership.md` 为证据载体）。
- **K/C 对标口径**：先 `scripts/kc_native_probe.py` + `scripts/kc_native_c_probe.py` 生成 actual，再 `scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` 判定；**不得**用不带 `--actual-dir` 的自比较。

---

## Final Summary（未开工）

（`TODO.csv` 的 6 步全部为 `TODO`；本子任务尚未开工，未执行任何步骤、未产生任何证据）

---

## 2026-09-29 计划修订（复核后）

- **改动**：凝结作用域的判据由「车轮子系统本次不产出车轮刚体」（F5b）改为**试验台/请求声明**——`RigSpec.supplies_wheels=True` 的单轴侧（K/C 与轴动态）把 wheel 子系统产出的车轮体刚性凝结进轮毂，整车侧不凝结；硬门从「`kc_baseline` 逐位不变」扩为「`kc_baseline` **与** `dynamic_hash_baseline` 的 13 个轴侧用例逐字节不变」，Done-When 与 Final Validation Command 补 `dynamic_hash_sentinel.py --check`；写范围里 `preparation/vehicle_dynamic.py` 限定为**仅轮端内容段**（`:900-903`、`:1009-1025`），模型访问段（`:188`、`:207`、`:210`、`:341`、`:368-369`）与 `:373` 命名规则归 03 且必须在 03 之后；snapshot 判据写明 04 不得产生差异。**本节取代本文件「已确定的口径」中只提 `kc_baseline` 的旧表述（该段原文按约定保留）。**
- **为什么**：Epic 开工前独立审核（code-reviewer）的阻断项——原判据「本次不产出车轮刚体」在文件层统一后失效，用户裁决改按声明判定；`preparation/vehicle_dynamic.py` 上的两个写者需分段串行（03 在前、04 在后）。
- **影响**：`TODO.csv` 第 3 行与第 4 行；`SPEC.md` 的 Goals 引言与 Goal 3、Non-Goals、Constraints（写范围、产物判据）、Risk、Deliverables、Done-When、Final Validation Command、Demo Flow。

---

## Final Summary（2026-09-29 落地，`TODO.csv` 6 行全 DONE）

口径按**用户裁决 B**（2026-09-29，本行开工前）：04 的验收 (a)「单轴与整车都读入同一份 wheel 子系统**文件**」在本行声明的写范围内不可实现（实测：`subsystems/types.py` 的 `_ROLE_TEMPLATE_FIELD` 无 `wheel` 项、`authoring/solver.py` 的 `_FILE_ROLE_TEMPLATES` 排除 wheel、内置 `WHEEL` 模板 `parts=()`，三个文件均不在 04 写范围），故重读为「`subsystems/wheel.py` 是单轴与整车轮端（车轮刚体 + 轮胎声明）的**唯一生产者**」，跨文件读 wheel 模板另立 **04b**。证据见 `raw/wheel_file_unification.md`。

### 改了什么

- `subsystems/wheel.py`（重写）：`wheel.py` 成为轮端唯一生产者。`build` 按 wheel 模板的 `parts` 产出车轮体（内置 WHEEL 声明 0 个部件 → 单轴不产出任何体，D9 与既有产物零变化的直接原因；模板声明的体用 `geometry.body_from_part` 建，质量由模板自己的声明给出）；模板来源由 `template_instance` 一处决定，`_requested` 用 `getattr(request, "wheel_template", None)` 预留文件通道（字段落地即生效）；新增 `wheel_center_body`，`tires` 的归属改由它统一给出。
- `subsystems/si_assembly.py`：新增 `_wheel_end_is_supplied`（**判据 = `RigSpec.supplies_wheels`**；无试验台的单轴按 D9 同样凝结）、`_wheel_bodies`（轮端体取自 wheel 贡献，不猜名字）、`_wheel_end_mounts`（挂接体按「谁声明了该侧轮心点」判定）；在 `runtime_from_outputs` 之后按判据调用凝结。wheel 贡献块**前移**到悬架行构建之前（原因：模板自己拥有车轮体时，`element_rows` 必须已经能读到该体），移动后 01 快照仍零差异。
- `subsystems/vehicle_parts.py`（车轮与挂接段）：新增 `_condense_wheel_end` / `_reown_wheel_end_element` / `_names_a_condensed_body`，算术全部复用既有 `_merge_fixed_wheel`（未改一行、未新造机制）；删掉的车轮体只带走「点名它的约束行」，轮心点与轮胎归属按帧换算跟到挂接体。
- `subsystems/assembler.py`（轮胎过滤段）：删除 `isinstance(element, VerticalTireElement)` 过滤与其 import；改用**角色**表达同一件事——`_AXLE_ROLES_IN_A_VEHICLE = DEFAULT_AXLE_SUBSYSTEMS - {"wheel"}`，车辆侧各轴不再组装 wheel 角色（车辆的轮端就是它自己条目声明的那些）。
- `subsystems/element_build.py`（轮胎行段）：轮胎行只在 `context.request.carries("wheel")` 时产出，与上一条同源。
- 新增 `tests/subsystems/test_wheel_lifecycle.py`（6 条）：内置模板不产出车轮体且轮胎在轮毂上；**模板声明的车轮体被凝结**（实体集合 / 两列约束行数 / 轮胎归属 / 轮心点与内置读数一致，质量并入挂接体且惯量不减小）；轮端来自 wheel 贡献；判据是 `supplies_wheels` 而非「本次产出了什么」；车辆侧各轴不含 wheel 角色且过滤器已消失；轮胎行随角色存在。

### 验证（退出码全部 0）

- `just check-fast`：ruff / ty 全过、三个架构门全过、快速集 **1053 passed / 1 xfailed**、kernel 33、contracts 32（基线 1047 + 本行新增 6，skip/xfail 未增长）。
- 数值门：`kc_native_probe.py` 9 K states（worst ratio 1.65548e-05）、`kc_native_c_probe.py` 66 C states（worst ratio 0.000185873）、`kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` → OK。
- `dynamic_hash_sentinel.py --check`：26 个 artifact 逐字节一致，combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`（与 03 收尾相同，未重录）。
- `tasks/20260929-01-freeze/snapshot.py --check`：**零差异**（`raw/approved_deltas.json` 保持 `[]`，04 未登记任何差异）。
- `git status --short -- packages/suspension_multibody/tests/data/`：空（`kc_baseline` 与 `dynamic_hash_baseline` 未被写）。

### 偏差与未收口（已登记，供 07 审计）

1. **(a) 的跨文件部分未做**：`AssemblyRequest` 的 wheel 模板载体字段与其角色表项、`authoring/solver.py::_FILE_ROLE_TEMPLATES` 纳入 wheel，已另立 **04b**（`depends_on=04`）。`subsystems/wheel.py::_requested` 已预留读取通道。
2. **K/C 多轴轮心选择未收口**（03 登记的未收口项，原指给 04）：`cases/kc_quasi_static/contract.py::wheel_centre_body` 仍走「常规名优先 + 全表回退」，未改为读取装配自身的轮端表；随 04b 一并处理。
3. **与 20260921 Epic A3 的关系**（SPEC 要求点名）：A3 的裁决是**整车侧**不凝聚、weld 交内核 `fixed` 关节；本行是**单轴侧**（`supplies_wheels=True` 的读数）把车轮体折进轮毂。两者方向相反、作用域不相交，A3 的结论未被触碰（01 快照与 26 个 artifact 都未变）。
4. **全量 pytest 未跑**：按用户指示本轮不跑全量（约 33 分钟），留到 07 收尾。
