# p4-02 判据 (a)：角色表同步清单（F12 全清单逐项）

`anti_roll_bar` 加入后，**每一处**同时记录角色名的位置都已在 diff 里改到。
清单来源 = `tasks/p4-01-freeze/raw/role_table_sync.md` 的实测列表。

| # | 位置 | 改动 | 实测 |
|---|---|---|---|
| 1 | `templates/roles.py::ROLES` | 新增 `"anti_roll_bar": RoleSpec(...)` 条目 | `grep -c anti_roll_bar` = 3 |
| 2 | `templates/roles.py::_check_roles()` 的 `expected` 集合 | 六元组改为七元集合（**import 期硬断言**） | 同上 |
| 3 | `templates/roles.py` 模块 docstring 与 `role_names()` docstring | "six" → "seven" | 同上 |
| 4 | `templates/builtin.py` | 新增 `ANTI_ROLL_BAR` 模板、`ANTI_ROLL_BAR_NAME`、`__all__` 两项、`BUILTINS` 追加 | `grep -c anti_roll_bar` = 3 |
| 5 | `authoring/documents.py::FUNCTIONAL_ROLES` | 集合加入 `"anti_roll_bar"` | `grep -c` = 2 |
| 6 | `subsystems/types.py::SUBSYSTEM_ROLES` | 集合加入 `"anti_roll_bar"` | `grep -c` = 2 |
| 7 | `subsystems/capabilities.py::ALL_SUBSYSTEMS` | 集合加入 `"anti_roll_bar"` | `grep -c` = 1 |
| 8 | `subsystems/composition.py::SUBSYSTEM_ROLES` | 七元组，**追加在末尾**（既有六角色顺序不变）；`capabilities_for` 的交集改读该表，不再抄字面量 | `grep -c` = 2 |
| 9 | `connections/policy.py::ROLES` | 元组加入 `"anti_roll_bar"` | `grep -c` = 1 |
| 10 | `contracts/template.schema.json` 的 `functional_role` enum | 加入 `"anti_roll_bar"` | `grep -c` = 2 |
| 11 | `contracts/subsystem.schema.json` 的 `functional_role` enum | 加入 `"anti_roll_bar"` | `grep -c` = 1 |
| 12 | `contracts/assembly.schema.json` 的 `functional_role` enum | 加入 `"anti_roll_bar"` | `grep -c` = 1 |
| 13 | `tests/templates/test_template_model.py` | `test_six_roles_are_declared` → `test_seven_roles_are_declared`，期望元组加 `"anti_roll_bar"` | `grep -c` = 2 |

## 两个**刻意不动**的同步点及其理由

`subsystems/types.py` 的 `DEFAULT_AXLE_SUBSYSTEMS` / `DEFAULT_VEHICLE_SUBSYSTEMS`
（`EPIC.md:139` 的清单把它们列为「默认集合」）**没有**加入新角色，理由：

- 这两个集合描述的是**某个装配实际建了什么子系统**，不是「可能出现哪些角色」。
  它们今天分别是三元 / 六元字面量，而 `brake`/`drive` **也不在其中**——按同一口径，
  新角色同样不该进。
- 更关键的是正确性：装配的能力报告（`capabilities_for`）被承诺是**真话**。
  把 `anti_roll_bar` 放进整车默认集合，会让每个没声明防倾杆的整车装配都声称
  自己有能力做一个它并不构建的子系统——那是**谎报**，正是 `EPIC.md:232` 禁止的
  「把简化/专用分支写进 role 接口」的镜像错误。
- `SUBTASKS.csv` 的 `p4-02` `Done-When` 逐项列举的同步点里也**没有**这两项。

## import 期断言实测

```
$ uv run --no-sync python -c "
from suspension_multibody.templates.roles import ROLES, role_names
print('roles:', role_names())
print('arb mounts:', ROLES['anti_roll_bar'].required_mounts)
"
roles: ('anti_roll_bar', 'brake', 'chassis', 'drive', 'steering', 'suspension', 'wheel')
arb mounts: ('chassis_mount_L', 'chassis_mount_R', 'droplink_mount_L', 'droplink_mount_R')
```

退出码 0，**无 `RoleSpecError`**。`_check_roles()` 的 `torque_roles != {"brake","drive"}`
断言也仍然成立（新角色 `has_torque_channel=False`）。
