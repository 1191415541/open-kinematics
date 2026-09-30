# PROGRESS：04b wheel 模板的文件载体与 K/C 多轴轮心收口

## 状态

`TODO` —— 尚未开工（`TODO.csv` 5 行全部 `TODO`；未执行任何步骤、未产生任何证据）。

## 为什么会有这一行

04 的验收 (a) 要求「单轴与整车都读入同一份 wheel 子系统**文件**」，但实测：

- `subsystems/types.py::_ROLE_TEMPLATE_FIELD` 没有 `wheel` 项 → `role_instance("wheel")` 直接抛错；
- `authoring/solver.py::_FILE_ROLE_TEMPLATES = ("steering", "chassis")` 显式排除 wheel；
- 内置 `WHEEL` 模板是 `parts=()`（`templates/builtin.py`）；
- 这三个文件均不在 04 声明的写范围内。

用户于 2026-09-29 裁决按 **B** 处理：04 重读为「`subsystems/wheel.py` 是单轴与整车轮端（车轮刚体 + 轮胎声明）的唯一生产者」，跨文件读 wheel 模板另立本行。04 已在 `subsystems/wheel.py::_requested` 预留 `getattr(request, "wheel_template", None)` 通道，本行的字段落地即生效。

另：03 登记的「K/C 多轴轮心选择」缺口（`cases/kc_quasi_static/contract.py::wheel_centre_body` 仍走「常规名优先 + 全表回退」）原指给 04，04 未收口，随本行一并处理。

## 依赖与串行

- `depends_on = 04`（已 DONE）。
- 写范围与 06 在 `subsystems/types.py` 上相交，必须**串行**执行，不得并发写同一文件。

---

## 2026-09-29 落地（DONE）

三处使能全部落地：`subsystems/types.py` 的 `AssemblyRequest.wheel_template` + `_ROLE_TEMPLATE_FIELD["wheel"]`；`authoring/solver.py::_FILE_ROLE_TEMPLATES` 纳入 `wheel`。`subsystems/wheel.py::_requested` 的预留通道因此即刻生效（该模块未再改动）。

另收口 03 登记的 K/C 多轴轮心缺口：`cases/kc_quasi_static/contract.py::wheel_centre_body` 先读装配自身的轮端表（`VehicleRuntime.wheel_centers`），多轴时按表作答、歧义被拒并点名候选；`si_assembly._declared_wheel_mounts` 从悬架模板的 `wheel_center` 连接取挂接体（文件模板该点标签为 `center`，纯标签搜索对它无效——这是本轮发现并修掉的第二个缺口）。

新增测试：`tests/authoring/test_wheel_file_chain.py`（3 条，含端到端：文档的 wheel 子系统成为当前模板、其车轮体被产出并被凝结进挂接体）、`tests/simulation/test_wheel_centre_selection.py`（3 条）。03 的边界用例报错原文随新语义更新为 "more than one wheel end"。

未收口：三轮整车的「一个单侧悬架」需要按条目声明侧（`AxleEntry`），随 06 处理。