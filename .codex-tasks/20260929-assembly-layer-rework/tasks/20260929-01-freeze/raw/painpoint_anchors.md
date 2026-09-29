# 5 条痛点的代码锚点复核（01 第 2 行的证据）

- 复核时间：2026-09-29
- 权威清单：`EPIC.md` 的「事实与修正」F1–F9（本文件是它的复核记录，不重复其全文）
- 复核方式：两个只读 explorer 分别核 R1–R2 与 R3–R5 的锚点；主代理**亲自复核**了其中 6 个承重锚点（下表标「主代理」）

| 需求 | 结论 | 关键锚点 | 复核者 |
|---|---|---|---|
| R1 装配层经 `VehicleModel` 中转 | 属实 | `subsystems/vehicle_assembly.py:122`（入口签名）、`:184-187`（`("front", model.front_axle, "front_")` / `("rear", model.rear_axle, "rear_")`）、`schema/vehicle.py:244`（`front_axle`/`rear_axle` 字段）、`:262-265`（四角校验）、`authoring/solver.py:632 file_axles_from`（`:622` 被 `assembly_request_for` 使用）、`authoring/vehicle.py:82 vehicle_model_from` | explorer + 主代理（`file_axles_from` 定义点） |
| R1 后果"无法 3 轴/拖挂/单轮" | 属实，且**被拒在更早两层** | `connections/policy.py:202-217`（`full_vehicle` 的 `role_counts`/`required_placements`）、`:252-296 check_assembly_shape`、`authoring/documents.py:62-64 PLACEMENT_ROLES`、`assembly.schema.json` 的 `placement_role` 枚举与轮数限制；负例原文见 `triaxle_refusal.md` | 主代理（`policy.py` 两处 + `check_assembly_shape`） |
| R1 另一面：文件驱动整车不是生产路径 | 属实 | `authoring/vehicle.py:82 vehicle_model_from` 无生产调用者，只在 `tests/authoring/test_vehicle_assembly_documents.py` 闭环 | explorer |
| R2 隐式字符串替换 | 属实，且**散落 4 处** | `subsystems/vehicle_assembly.py:212-228`（`body_map`）、`preparation/vehicle_dynamic.py:373`、`subsystems/vehicle_parts.py:172-173`、`:414`；白名单 `authoring/documents.py:76`、`subsystems/suspension.py:255` | explorer |
| R2 匹配机制"缺少" | **需修正**：机制已存在，只是没用在跨子系统插接 | `connections/matcher.py:102 match_requirements`（explicit 优先 / 0 候选分 required-optional / 多候选抛 `AmbiguousBindingError`）、调用点 `subsystems/composition.py:258`、`:347`；`composition.py:302-305` 只产出 bindings 记录 | 主代理（`match_requirements` 定义与两个调用点） |
| R3 单轴建轮胎、整车删掉 | 属实 | `subsystems/wheel.py:33-41`（单轴不产 body）、`:87-111 tires()`、`subsystems/element_build.py:136-144 _tire`（`wheel_body`/`wheel_center_local`）、`subsystems/vehicle_assembly.py:233-240`（按类型过滤 `VerticalTireElement`）、`subsystems/vehicle_parts.py:458 _add_wheel`（车轮体 `:510-515`、挂接 `:525-541`） | explorer |
| R3 整车"再挂轮胎" | **需修正**：整车轮胎走 native tire ABI，不挂 `VerticalTireElement` | `preparation/vehicle_dynamic.py:1009-1025`（用 `assembly.wheel_body_names[...]`）、`:900-903`（显式拒绝 `VerticalTireElement`） | explorer |
| R4 `_SIDES` 写死 | 属实 | `subsystems/suspension.py`、`subsystems/si_assembly.py` 的 `_SIDES`；`subsystems/wheel.py:114-116 sides()`；`subsystems/element_build.py:205-210`（`for side in ("L","R")`） | explorer |
| R4 镜像"写死在底层" | 属实，且**不是文档开关** | `subsystem.schema.json`/`template.schema.json` 无 `mirror` 字段；镜像行为在 `mirror_hardpoints`/`side_hardpoints`/`context.mirror`；当前契约"文件只写一侧、装配建两侧"见 `tests/authoring/test_vehicle_assembly_documents.py:202-214`；对称测试仅 `tests/model/test_symmetry.py`（12 行） | explorer |
| R5 试验台篡改所有权 | 属实，且**既有测试断言相反** | `subsystems/rig_link.py:315-348 _reown_tires`（改 `wheel_body` 并把 `wheel_center_local` 清零）、`:243-268 merge_rig_link`、`:211-218`（carrier weld）；`rigs/rig.py:73-110 RigSpec.supplies_wheels`；`SUSPENSION_MULTIBODY_RIG_ENTITIES`（`si_assembly.py:493-518`）；测试 `tests/subsystems/test_rig_link.py:172-199` 断言"轮胎必须被改 owner" | explorer |

## 结论

5 条痛点**全部属实**；其中 3 处需要修正措辞（R2 的"缺少机制"、R3 的"整车再挂轮胎"、R4 的"镜像写死"），修正后的表述已写进 `EPIC.md` 的 F4/F5/F6，并据此确定 02/04/06 的落点。

## 与既有裁决的关系（实施前必须处理）

- R3 与 20260922 Epic 的 **D9**（单轴侧车轮归悬架试验台）冲突 → 已由本 Epic 的 **D2** 裁决：不反转 D9，改走"文件层统一 + 装配期刚性凝结"。
- R5 与既有测试契约冲突 → 已由本 Epic 的 **D3** 裁决：确认反转。
