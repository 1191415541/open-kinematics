- 任务：组装整车实验总成（前后悬架+转向+车身+车轮+制动+驱动+整车KC试验台）并补齐整车侧质量落点
- 形态：single-full（Epic 子任务）
- 进度：13/13 步骤 DONE
- 当前：整车实验总成含六类子系统并可查询能力；整车侧轮胎质量落点补齐。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-11-vehicle-assembly/`
- 验证：`tests/vehicle_assembly` 12 passed；全量 1028 passed／1 skipped／1 xfailed；`case_parity_check` 8 families accepted；`dynamic_hash_sentinel --check` 26 artifact 逐字节一致；两包 `uv build` 成功；ruff/ty 全树通过；`tests/data` 与 `layering_baseline.json` **无 diff**。

## 交付物

- **整车实验总成**：`build_vehicle` 保持签名与 `VehicleAssembly` 字段不变（仍作适配器），能力描述挂 `DEFAULT_VEHICLE_SUBSYSTEMS`。实测整车总成的 `capabilities.subsystems` == 六类全集（含 `brake`/`drive`），体数 23、含 `wheel_*`。
- **两种组装的关系**（验收 1）：`vehicle_roles - axle_roles == {"brake","drive"}` 且 `axle_roles - vehicle_roles == set()`——整车比单轴**恰多**制动与驱动，其余同一套角色名。这是「共享同一套子系统定义」的可断言形式，而不是靠人工比对。
- **整车侧轮胎质量落点（审核 B4）**：
  - `schema/vehicle.py` 的 `WheelSpec` 新增 `tire_mass`（默认 0，`exclude=True`）；
  - `preparation/vehicle_dynamic.py` 的 `_build_tires` 把 `wheel.tire_mass` 传给 `AxleTire.mass_kg`；
  - `cases/vehicle_dynamic.py` 的 `_tire_entry` 在 `mass_kg > 0` 时发射 `mass`（有 `inertia_kg_m2` 时一并发射）。
- **需求 19/D9**：整车侧 `build_vehicle` 造 `wheel_*` 体，单轴侧不造（车轮归试验台），两侧差异有测试断言。
- **需求 15/D5 不对称锁定**：`VehicleModel.steering` 仍必填（`is_required()` 为真）、给 `None` 即报错；同一测试文件里另断言单轴侧转向可缺席。**这条不对称是有意的**，锁定它以免后续被当作缺陷「修」掉。

## 质量守恒的实测

`tire_mass` 只改归属、不改总量：`tire_mass=0.0` 与 `5.0` 两种模型下整车总质量均为 **3080**，且每个体的 `mass` 逐项相等。这正是需求 10 要求的「质量归属到轮胎」而不动动力学。

## 与 SPEC 的偏离

- **未新增独立的 `vehicle_assembly/` 组装模块**：SPEC 允许「新增整车总成组装」在 `subsystems/**` 或改造 `preparation/assembly/vehicle.py`。本步选择保持 `build_vehicle` 为适配器（SPEC 明确要求保留其签名与字段），正交组合与能力收缩由 10 的 `rigs` 层承接，整车总成的工作集中在**能力描述 + 质量落点 + 不对称锁定**三处。理由是改造 `build_vehicle` 的组装顺序会波及 5 个 vehicle family 与全部整车基线，而 SPEC 预期「不重录整车基线」。
- **未改 `SteeringSystemSpec` / `required_bodies` / `_build_steering`**：按 D5 整车侧不放开，三者原样保留（有测试锁定）。

## 未闭合项

无新增。整车侧轮胎质量的**数值效果**（质量从体迁到 tire 后动力学逐位不变）由 08 的质量守恒断言在轴侧证明；整车侧本步证明了「总质量与逐体质量不变」，与 08 的求解器耦合合起来即需求 10 在整车侧成立。
