# SPEC：04b wheel 模板的文件载体与 K/C 多轴轮心收口

> 子任务规格。父 Epic：`.codex-tasks/20260929-assembly-layer-rework/EPIC.md`；父行：`SUBTASKS.csv` 的 `04b`。
> 来源：04 的证据 `tasks/20260929-04-wheel-lifecycle/raw/wheel_file_unification.md` —— 用户裁决 B 把「跨文件读 wheel 子系统文件」从 04 移出，另立本行。

## Task Shape

- **Shape**: `single-full`

## Goals

1. **文件读取链打通（04 未落下的那一半）**：`AssemblyRequest` 新增可选的 wheel 模板载体字段并登记其角色表项（`subsystems/types.py` 的 `_ROLE_TEMPLATE_FIELD`），使 `role_instance("wheel")` 不再抛错；`authoring/solver.py` 的 `_FILE_ROLE_TEMPLATES` 纳入 `wheel`，使装配文档的 wheel 子系统经 `role_instance_from` 真正抵达 composition，被 `subsystems/wheel.py::template_instance` 读取（该函数与 `_requested` 已在 04 落地并预留 `getattr(request, "wheel_template")` 通道，字段落地即生效，无需再改该模块）。
2. **两侧同源**：同一份 wheel 子系统文件在单轴与整车两侧都产出车轮刚体与轮胎声明；整车侧仍由 `_add_wheel` 处理自己的轮端，单轴侧仍按 04 的 `supplies_wheels` 判据凝结。产物与 01 快照逐项一致。
3. **K/C 多轴轮心收口**：`cases/kc_quasi_static/contract.py::wheel_centre_body` 在装配自带轮端表（`VehicleRuntime.wheel_centers` / `wheel_body_names`）时按表作答，闭合 03 登记的「K/C 多轴轮心选择」缺口；多轴读数下每侧轮心体唯一可判，歧义仍拒绝而不是猜。
4. **零回归**：`kc_baseline` 与 `dynamic_hash_baseline` 轴侧用例逐位/逐字节不变。

## Non-Goals

- **默认不改 `templates/builtin.py` 的内置 `WHEEL`**：`parts=()` 是既有产物零变化的原因；给它加车轮体会让每个既有模型多出两个刚体，必须先给出独立于结果字节的等价判据并登记差异 —— 而「改变既有产物并登记」只有 05 被允许，本行不得先做。
- 不重录 `kc_baseline`；不用不带 `--actual-dir` 的 `kc_parity_check`（自比较恒过，不构成证据）。
- 不做 05（试验台非侵入）、06（可选对称）范围内的事。

## Constraints

- **写范围**：`subsystems/types.py`（仅 wheel 模板字段与角色表项）、`authoring/solver.py`（仅 `_FILE_ROLE_TEMPLATES` 一行与其注释）、`cases/kc_quasi_static/contract.py`（仅 `wheel_centre_body` 段）、`packages/suspension_multibody/tests/` 中对应目录。
- `subsystems/types.py` 与 06 的写范围相交 → 两者必须**串行**，不得并发写同一文件（06 的 `depends_on` 是 05）。
- 每步落地后必须重跑 `just check-fast` 与数值门三项。
- 不得新增 skip/xfail；`raw/` 只存**已执行**的证据。

## Environment

- **Project root**: `E:\杂件\open-kinematics`
- **Language/runtime**: Python 3.12（`uv run --no-sync`）
- **Test framework**: `pytest`
- **Existing test count**: 快速集 1053 passed / 1 xfailed；kernel 33；contracts 32

## Risk Assessment

- [ ] **字段落入 `model_dump` 会改所有 model_hash**：`api.py` 用 `model.model_dump(mode="json")` 算 `Provenance.model_hash`。若该字段落在 `VehicleModel` / `FrontAxleModel` 上，既有算例与 7 组合的字节一致性会连锁失效（D1）。落点必须是 **`AssemblyRequest`**（装配请求，不进模型 dump），与 02 加 `pairings` 时同一个理由。
- [ ] **`role_instance("wheel")` 的默认解析**：`role_instance` 对未声明的角色返回 `None`（「该角色自己的默认」），wheel 的默认就是内置模板，因此不改变既有路径。
- [ ] **多轴轮心歧义**：多轴读数下同一侧可能有两个轮端；必须拒绝而不是猜，并且只有当装配自带轮端表时才改判据，否则既有单轴读数一位不动。
- [ ] **是否给内置 WHEEL 加车轮体**：默认**不加**（见 Non-Goals）。

## Deliverables

- 写范围内各路径的改造。
- 覆盖 Goals 1–4 的断言（`tests/subsystems/`、`tests/authoring/`、`tests/simulation/` 的既有对应目录）。
- `raw/wheel_file_chain.md` —— 文件读取链打通的证据：文档的 wheel 子系统经 `assembly_request_for` → `role_instance_from` → `subsystems/wheel.py::template_instance` 抵达装配的实测（两侧同源断言）与产物差异判定。
- `raw/kc_center_multi_axle.md` —— `wheel_centre_body` 改判据后的多轴/单轴读数证据与 01 快照差异判定。

## Done-When

- [ ] 装配文档里的 wheel 子系统真正生效（不是被丢掉、也不是退回内置模板），单轴与整车两侧的轮端清单可断言同源。
- [ ] `wheel_centre_body` 在多轴读数下按装配自身的轮端表作答，歧义被拒绝。
- [ ] `just check-fast` 与数值门三项全绿；无新增 skip/xfail。
- [ ] `kc_baseline` 逐位不变、`dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节一致（均未重录）。
- [ ] `snapshot.py --check` 通过且差异为空（只有 05 被允许改变既有产物并登记）。

## Final Validation Command

```bash
cd /e/杂件/open-kinematics && \
uv run --no-sync pytest packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/authoring packages/suspension_multibody/tests/simulation -q && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_probe.py && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_c_probe.py && \
uv run --no-sync python packages/suspension_multibody/scripts/kc_parity_check.py --check --actual-dir artifacts/kc-native-probe && \
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check && \
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
```
