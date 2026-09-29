# 3 轴/不对称总成的负例实测（01 第 3 行的证据）

- 执行时间：2026-09-29
- 执行脚本：会话 scratch `probe_triaxle.py`（未入库；本文记录其输出原文）
- 目的：确认"3 轴"与"左右独立悬架文件"今天**在哪一层、以什么报错**被拒绝——这是 03/06 的前置墙，不是装配函数内部的问题

## (a) 形状规则层：`connections/policy.py::check_assembly_shape`

命令（脚本内直接调用公开函数）：

```python
from suspension_multibody.connections.policy import check_assembly_shape
check_assembly_shape("full_vehicle", [
    ("suspension", "front"), ("suspension", "middle"), ("suspension", "rear"),
    ("chassis", "any"), ("steering", "front"), ("brake", "any"), ("drive", "any"),
])
```

实测输出（原文）：

```text
3 axles (front/middle/rear)      REFUSED: RuleViolation: an assembly of kind 'full_vehicle' requires exactly two suspension subsystem(s), found 3
3 axles (front/rear/third)       REFUSED: RuleViolation: an assembly of kind 'full_vehicle' requires exactly two suspension subsystem(s), found 3
2 suspensions, L/R split         REFUSED: RuleViolation: an assembly of kind 'full_vehicle' requires one front suspension; found [('brake', 'any'), ('chassis', 'any'), ('drive', 'any'), ('steering', 'front'), ('suspension', 'front_left'), ('suspension', 'front_right'), ('suspension', 'rear')]
```

结论：`role_counts={"suspension": 2}`（`policy.py:204-210`）先于一切装配逻辑拒绝第 3 个悬架；`required_placements={("suspension","front"),("suspension","rear")}`（`:212-214`）拒绝"左右各一个文件"的写法。

## (b) 契约 schema 层：`assembly.schema.json` 的 `placement_role` 枚举

命令：用测试夹具 `write_axle_project` 写出一份合法总成文件，改 `assembly_kind` 为 `full_vehicle` 并追加第三个 `placement_role="middle"` 的悬架子系统，再用 `AssemblyDocument.load` 读入。

实测输出（原文）：

```text
REFUSED: AuthoringError: <tmp>/triaxle.asy.json: $/subsystems[2]/placement_role: 'middle' is not one of ['any', 'front', 'rear', 'front_left', 'front_right', 'rear_left', 'rear_right']
```

结论：`middle` **在文档加载阶段就被 schema 拒绝**，早于 `check_assembly_shape`。所以 03 要动的是"文档形状规则 + 契约 schema"，不是只改装配函数（对应 `EPIC.md` 的 F2）。

## 复现方式

```bash
# 脚本内容见本文件上方两段；依赖测试夹具（tests/authoring/fixtures.py::write_axle_project）
uv run --no-sync python <scratch>/probe_triaxle.py
```
