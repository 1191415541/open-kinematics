# 阶段 8 — 兼容转换：文件模板进入既有装配与求解

## 交付

- `authoring/solver.py`：`runtime_template_from`（文件模板 → 运行期 `Template`）、
  `assembly_request_from`（→ `AssemblyRequest`）、`front_axle_model_for`。
- `authoring/bridge.py`：`EffectiveSubsystem` → `FrontAxleModel`，含硬点名规范化与元素构建。

## 必须满足的契约

1. **不新增求解路径**。转换只产出既有代码已经消费的对象（`Template`、`AssemblyRequest`、
   `FrontAxleModel`），装配、试验台绑定、契约文档与内核全部沿用原实现。
2. **硬点名经 `HARDPOINT_ALIASES` 规范化**。模板用角色名写点（`tie_inner`），模型用固定别名
   拼写查找（`TIE_ROD_INBOARD`）；直接透传会在三层之外报「missing required hardpoint」。
3. **单侧声明镜像到两侧**。文件模板写一侧（与 `FrontAxleModel` 只描述左侧一致），但装配按
   `owner.endswith("_L"/"_R")` 分别取值；不镜像则右侧为空。镜像必须同时作用于刚体、连接、
   远端与衬套名，且不带侧别的部件（chassis、rack）只声明一次。
4. **一个 joint 一个点**。joint 在其 `point_a` 处约束两个刚体，`point_b` 若存在且不同则报错——
   那等于悄悄加入「两点重合」这条作者没要求的约束，而它只会以秩亏雅可比的形式出现。
5. **本体数据来自属性文件**。模型里的弹簧/减振器/缓冲块取自模板声明 + 已解析属性绑定，
   所以换属性文件会改变求解器输入而不改变模板。
6. **固定支撑不加侧别**。`chassis` 这类只存在一次的部件保持原名；给所有名字加后缀会造出
   没有任何贡献产生的 `chassis_L`。

## 验证

```
uv run --no-sync pytest packages/suspension_multibody/tests/authoring -q -p no:cacheprovider
```

对应测试：`test_hardpoint_roles_map_onto_the_model_lookup`、`test_file_template_becomes_a_runtime_template`、
`test_file_subsystem_becomes_a_model`、`test_files_supply_the_values_and_laws_a_real_kc_run_solves`。
最后一项同时断言：文件模板装配出的 K 模型与内置模板**逐项一致**（13 约束、10 刚体），
且文件提供的硬点与属性文件能跑通一次真实 K 求解并收敛。

## 已知缺口

文件格式每个硬点只能声明一个运动副，没有「按模式激活的列」，因此内置 double wishbone 的
C 模式（内侧后点由球铰 + 衬套承担）无法用文件表达——文件的 C 装配仍等于 K 装配。
修复需要给 `template.schema.json` 的 joint 增加 `modes` / `kind_by_mode`，并让
`element_properties.schema.json` 支持 `bushing` 类型属性文件。
