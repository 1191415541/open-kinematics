# 04 证据：文件层统一做到了哪一步（用户裁决 B 的口径）

## 1. 用户裁决（2026-09-29，本行开工前）

04 的验收 (a)「单轴与整车都读入同一份 wheel 子系统**文件**」在 04 声明的写范围内**不可实现**，
实测依据：

* `subsystems/wheel.py` 要读文件/文档提供的 wheel 模板，唯一通道是 `AssemblyRequest.role_instance("wheel")`；
* `subsystems/types.py` 的 `_ROLE_TEMPLATE_FIELD` **没有 `wheel` 项**，`role_instance("wheel")` 直接抛错；
* `authoring/solver.py` 的 `_FILE_ROLE_TEMPLATES = ("steering", "chassis")` **显式排除 wheel**，
  且其注释写明原因（wheel 的轮心挂在模板自己不拥有的体上）；
* 这两个文件都不在 04 的写范围（`types.py` 归 06、`solver.py` 归 03）；
* 内置 `WHEEL` 模板是 `parts=()`，`templates/builtin.py` 也不在 04 写范围。

用户选择 **B**：(a) 重读为「`subsystems/wheel.py` 是单轴与整车轮端（车轮刚体 + 轮胎声明）的
**唯一生产者**」；文件层统一只做「文档提供的 wheel 模板经 wheel.py 生效」这一条能落的部分；
其余（跨文件读 wheel 模板）另立子任务 **04b**。

## 2. 本行落下的部分（已落地、已断言）

| 项 | 位置 | 断言 |
|---|---|---|
| wheel 角色是轮端的唯一生产者 | `subsystems/wheel.py::build` | `tests/subsystems/test_wheel_lifecycle.py::test_the_wheel_subsystem_is_the_producer_of_the_wheel_end` |
| 模板声明的车轮体被构建（内置声明 0 个） | `subsystems/wheel.py::build` + `geometry.body_from_part` | 同上 + `test_a_declared_wheel_body_is_condensed_into_the_body_carrying_the_wheel_centre` |
| 轮胎挂在**模板声明**的那个体上 | `subsystems/wheel.py::wheel_center_body` / `tires` | `test_the_built_in_wheel_subsystem_declares_no_body_and_still_places_the_tire` |
| 文档提供的 wheel 模板经 wheel.py 生效的**接口已就位** | `subsystems/wheel.py::template_instance` → `_requested`（`getattr(request, "wheel_template", None)`） | 字段一旦存在即生效，无需再改本模块 |
| 装配阶段不再用类型过滤决定轮端 | `subsystems/assembler.py`（`VerticalTireElement` 过滤删除） | `test_a_vehicle_composes_its_axles_without_the_wheel_role` |
| 轮端存在与否由**角色**决定 | `subsystems/element_build.py::element_rows` + `assembler._AXLE_ROLES_IN_A_VEHICLE` | `test_tire_rows_exist_only_when_the_assembly_carries_the_wheel_role` |

## 3. 未落下的部分（已另立 04b）

* `AssemblyRequest.wheel_template` 字段（`subsystems/types.py`）与其角色表项；
* `authoring/solver.py::_FILE_ROLE_TEMPLATES` 加入 `wheel`，使文档的 wheel 子系统真正抵达装配；
* 08 之后的 `templates/builtin.py` 是否给内置 WHEEL 加车轮体（本行未动，因此既有产物零变化）。

## 4. 产物差异判定（口径「未变化部分逐项相等 + 已登记差异」）

```
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
  -> OK: the seven combinations assemble exactly what the snapshot froze     (exit 0)
```

**04 不得产生差异，差异为空**（`raw/approved_deltas.json` 保持 `[]`，未登记任何项；
只有 05 被允许改变既有产物并登记）。

补充：把 wheel 贡献块从「steering 之后」前移到「悬架行构建之前」是一次**顺序**改动
（原因：模板自己拥有车轮体时，轮胎归属必须在 `element_rows` 时就能读到该体）。
改动后 01 快照再次 `--check` 仍为零差异，`dynamic_hash_sentinel --check` 的 26 个 artifact
逐字节未变。
