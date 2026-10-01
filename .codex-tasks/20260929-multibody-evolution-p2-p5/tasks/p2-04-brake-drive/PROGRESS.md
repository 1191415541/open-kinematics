# PROGRESS：p2-04 brake 与 drive 子系统改造为力矩元

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-04`

## Session Start

- **Date**: 2026-09-29
- **Task name**: p2-04-brake-drive
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-04-brake-drive/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（5 步）
- **Environment**: Python 3.12.13 / uv / pytest 8.3.4 / ty

## Context Recovery Block

- **Current milestone**: #5 — 未越界自检 + 数值门（全部 5 行已完成）
- **Current status**: DONE
- **Last completed**: 验收五条命令全绿（真实退出码见 `raw/run_log.md` §1）
- **Current artifact**: `raw/` 下五份证据文件
- **Key context**:
  - 依赖 p2-03（`SUBTASKS.csv` `p2-04` `depends_on`）：力矩元的构造分派与编译层接入已落地，本行复用了 `compilation/element_blocks.py` 的 `pair_torque_bodies` / `torque_element_row`。
  - **与 p4-02 强制串行**（`EPIC.md` 行 215）：`templates/roles.py` 与 `templates/builtin.py` 是共享注册文件。本行只改了 `"brake"`/`"drive"` 两个 `RoleSpec` 条目自身的内容与 `BRAKE`/`DRIVE` 两个模板段，角色名集合与 `roles.py:162` 的六角色硬断言未触碰（`git diff` 自证）。
  - **`templates/roles.py` 的分段归属**（`EPIC.md` 行 227）：已遵守，见上。
  - 本行唯一的越界候选是 `subsystems/types.py::ELEMENT_KINDS`（不在禁改清单，属本行写范围）——补登 `rotational_torque` 并附说明。
- **Known issues**（全部登记在 `raw/reaction_paths.md` §5，未绕过）:
  - 组合层（`composition.py`，p4-02）与准备层（`preparation/vehicle_dynamic.py`，p2-05）尚未调用本行新增的元素构造入口，所以没有生产路径跑到力矩元。
  - `template_document_from` 不写 `needs`，模板上的 `needs` 会被导出/读回丢掉（实测），因此本行未在模板上声明需求。
  - 内核文档路由不接受 `rotational_torque`（`contract_model.cpp:831`），包的内核侧缺口，本行不属修复范围。
  - 落地的内核力律是阻力律（`anti_roll.cpp:132-139`，demand 硬编码 1.0），`RotationalTorqueParameters` 拒绝负增益，所以驱动元素承载幅值而非带符号力矩；本行把它写成断言。

---

## Final Summary

**已交付**

1. **属性槽标准化**：BRAKE = `piston_area` / `effective_radius` / `friction_coeff` / `rotor_inertia`；DRIVE = `gear_ratio` / `efficiency` / `max_torque`。`templates/roles.py` 的 `required_slots` 与 `templates/builtin.py` 的 `property_slots` 两处一致。改前/改后逐条理由见 `raw/template_slots.md`；缺槽负例（七个槽逐个摘除）由既有的 `Template.check_role_contract()`（`templates/model.py:362-366`）点名，异常类型 `TemplateError`，消息全文与层 file:line 在 `raw/template_slots.md` §3。
2. **产出力矩元**：`brake.py` / `drive.py` 的 `wheel_torque_amplitudes()` 与 `bias_share()` 删除；改为纯幅值函数 + 参数构造 + 元素行构造三件套。改造前 13 处全量命中与逐条处置见 `raw/wheel_torque_amplitudes.md`。
3. **反力路径断言**：制动与驱动各有「反力体读自被匹配端口」的断言、按角色名拒绝的负例、以及构造路径零体名规则的文件级断言；`upright`/`chassis` 在 `brake.py`/`drive.py` 命中 0。见 `raw/reaction_paths.md`。
4. **契约测试更新与理由**：F6 的六个制动锚点、五个驱动锚点逐条给出改前→改后→理由；外部消费点（`test_native_vehicle.py:1607` / `:1721` 制动，`:1500` / `:1630` 驱动，`test_vehicle_dynamic_contract.py:185`）实测为不受影响。见 `raw/contract_test_updates.md`。

**验证**：`raw/run_log.md`（五条验收命令退出码全 0；sentinel combined sha256 与冻结值一致；`ruff`（本行文件）与 `ty` 退出码 0；快速测试集 `1153 passed, 1 xfailed`）。

**未达成的条目**：见上「Known issues」以及 `raw/run_log.md` §9（组合层未接线是其中最关键的一条——判据 3 的断言证明的是子系统构造路径，而不是一条端到端装配路径）。仓库级 `ruff check .` 退出码为 1，但 19 条全部落在并发 agent 与其他子任务的 `raw/` 脚本上（逐文件计数见 `raw/run_log.md` §2）。
