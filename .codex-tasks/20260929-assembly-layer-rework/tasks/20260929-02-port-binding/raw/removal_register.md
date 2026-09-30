# 名字猜身份的清点与登记（02 第 5 行）

- 执行时间：2026-09-29
- 范围：本行自有路径 `connections/`、`modeling/ports.py`、`templates/ports.py`、`subsystems/composition.py`、`authoring/documents.py` 的配对读取段

## A. 本行自有路径里 `"chassis"` / `"ground"` 的全部命中（实测 grep）

```text
subsystems/composition.py:64:    "chassis",
subsystems/composition.py:313:            subsystems=frozenset(roles) & {"suspension", "steering", "wheel", "chassis", "brake", "drive"},
authoring/documents.py:60:    {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
authoring/documents.py:76:_ASSEMBLY_SUPPLIED_BODIES = frozenset({"chassis", "ground", "rack", "rack_housing"})
```

判定：
- `composition.py:64` 与 `:313`、`documents.py:60` 是**角色词表**（六个子系统角色的名字之一就叫 `chassis`），不是「名字等于 chassis 就改写」的规则：它们不改变任何实体的身份，只说明这个装配带了哪些角色。
- `documents.py:76` 是白名单，见 B。
- **结论：本行自有路径没有按名字猜身份/改写名字的规则**。真正的改写规则在 `subsystems/vehicle_assembly.py:212-228`（`body_map`）、`subsystems/vehicle_parts.py:172-173` 与 `:414`、`preparation/vehicle_dynamic.py:373`——按父表这三处**归 03**，由 03 收口、07 复验全路径 grep。本行不越界改它们。

## B. `_ASSEMBLY_SUPPLIED_BODIES` 的处置结论

使用点（实测 grep）：

```text
authoring/documents.py:76:_ASSEMBLY_SUPPLIED_BODIES = frozenset({"chassis", "ground", "rack", "rack_housing"})
authoring/documents.py:218:                if str(joint[key]) not in body_names | _ASSEMBLY_SUPPLIED_BODIES:
authoring/documents.py:238:                if str(element[key]) not in body_names | _ASSEMBLY_SUPPLIED_BODIES:
```

它做的是**校验放宽**：子系统文件里某条关节/力元引用的体，要么在该子系统自己声明的 `bodies` 里，要么是「总成提供的体」之一，否则在该行报错。它**不改写**任何名字，也不会把某个体换成另一个体。

处置（本行决定）：
- **本轮保留**，并在此登记它校验什么、为什么它不属于「按名字猜身份」：它约束的是「这条引用是否合法」，冲突时**报错**而不是悄悄替换。
- **替代物交给 03**：把「总成提供的体」从一个写死的名字集合，改为由总成条目声明的角色/端口推导（03 拥有 `PLACEMENT_ROLES` 与放置校验段，且它正要重做文档形状规则）。03 的 SPEC 已列该文件的放置与角色枚举段，B 的收口随其落地；07 复验终局 grep。
- 未收口项已写入 `PROGRESS.md` 的 Known issues，不允许静默放过。

## C. 兼容开关

```text
grep -rn "os.environ\|getenv\|environ\[" connections/ modeling/ports.py templates/ports.py subsystems/composition.py authoring/documents.py
→ 零命中
```

本行自有路径**没有**由环境变量控制的过渡兼容路径，因此没有需要删除的开关。装配路径上确实存在一个开关 `SUSPENSION_MULTIBODY_RIG_ENTITIES`（`subsystems/si_assembly.py:521` 定义），但它控制的是试验台实体是否注入，属 **05** 的行内范围（父表把 `si_assembly.py` 的 bench 接入段给了 05），本行不越界删除。
