# 阶段 4 — 子系统文件与不可变拓扑实例

## 交付

- `suspension_multibody/authoring/documents.py` 中的 `SubsystemDocument`、`EffectiveSubsystem`。
- `suspension_multibody/authoring/errors.py` 的统一错误基类。

## 必须满足的契约

1. 子系统引用一个模板文件；模板先加载，再校验子系统的每个硬点名与每个属性绑定名都属于模板声明。
2. 模板声明的**每一个**硬点都必须在子系统里给出坐标：模板持拓扑，子系统持几何，缺一个就没有可解的模型。缺项报错要点名缺了哪些硬点。
3. 拓扑字段（`bodies`/`joints`/`elements`/`ports` 等）出现在子系统文件里时由 schema 的 `additionalProperties: false` 拒绝，报错必须说明是「unexpected fields」而不是笼统的「文档无效」。
4. 属性绑定在 `effective()` 时才解析：相对路径以**子系统文件所在目录**为基准，且每个文件按槽位声明的 `element_type` 与 `allowed_models` 校验。延迟解析是为了让「子系统的数值/引用」与「它引用了什么」在同一处失败。
5. `effective(overrides=...)` 返回只读实例，且不修改 `self.payload`；覆盖键只允许 `hardpoints` 与 `property_bindings`。
6. 哈希四分离：`topology_hash`（等于模板的）、`values_hash`、`property_bindings_hash`、`effective_values_hash`。把线性属性文件换成曲线文件必须移动后三者**并保持**第一个不变。

## 验证

```
uv run --no-sync pytest packages/suspension_multibody/tests/authoring -q -p no:cacheprovider
```

对应测试：`test_template_and_subsystem_preserve_topology_hash`、
`test_subsystem_cannot_change_topology`、`test_subsystem_must_place_every_declared_hardpoint`、
`test_property_file_type_must_match_the_slot`、`test_missing_required_property_binding_is_refused`、
`test_swapping_linear_for_curve_keeps_topology_and_changes_values`。
