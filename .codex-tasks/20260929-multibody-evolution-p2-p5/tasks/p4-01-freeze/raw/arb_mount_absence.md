# `arb_mount` / `droplink_mount` / `chassis_mount` 不存在的事实（p4-01 冻结证据 · F13）

## 全仓 grep（真实命令 + 输出）

命令：
```bash
grep -rn "arb_mount\|droplink_mount\|chassis_mount" . \
    --include=*.py --include=*.json --include=*.md --include=*.csv --include=*.cpp --include=*.hpp \
  | grep -v "\.codex-tasks"
```
输出（全部命中，无截断）：
```
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:93:  - 4 个标准接口端口：`chassis_mount_L/R`（安装到车身或副车架）与 `droplink_mount_L/R`（安装到悬架受动构件）。
./packages/suspension_multibody/docs/multibody_architecture_evolution.md:195:  2. 悬架子系统暴露出标准的 `arb_mount` 端口，通过总成配对段完成连接；
./packages/suspension_multibody/tests/authoring/test_assembly_pairings.py:48:    _state_pairings(path, [{"requirement_role": "mount", "port": "chassis_mount"}])
./packages/suspension_multibody/tests/authoring/test_assembly_pairings.py:50:    assert dict(document.entries[0].pairings) == {"mount": "chassis_mount"}
```

结论：
- **源码（`packages/**/src/`）零命中**。
- **测试**中只有 `tests/authoring/test_assembly_pairings.py:48/50` 命中，且 `chassis_mount` 在这里是**任意字符串示例**（配对段语法测试用的 port 名字面量），并非真实端口声明——不含 `arb_mount`/`droplink_mount`。
- 其余命中只在**路线图文档** `docs/multibody_architecture_evolution.md:93/195`（正是 F13 所说）。

## 今天端口是怎么在装配期合成的

端口不是模板声明的，而是装配期由 body 名单**运行期派生**：

- `packages/suspension_multibody/src/suspension_multibody/subsystems/si_assembly.py:87-117 _ports_for_bodies`
  ```python
  def _ports_for_bodies(instance, bodies):
      ports: dict[str, GeometryPort] = {}
      for name in bodies:
          side = name.rsplit("_", 1)[-1] if name.endswith(("_L", "_R")) else None
          ports[name] = _port(instance, name, "body", side)
          if name.startswith("upright_"):
              centre = f"wheel_centre_{side}"
              ports[centre] = GeometryPort(
                  id=EntityId(instance, centre),
                  owner=EntityId(instance, name),
                  role="wheel_centre",
                  capabilities=frozenset({"wheel", "load"}),
                  labels=frozenset({side}) if side else frozenset(),
              )
      return ports
  ```
  （每个 body 一个 `role="body"` 端口；`upright_*` 额外一个 `role="wheel_centre"` 端口。）
- `si_assembly.py:120-150 _wheel_centre_needs`：每侧声明一个 `role="wheel_centre"` 的**可选** `PortRequirement`（`required=False`），供试验台供轮。

- 端口构造函数：`si_assembly.py:75-84 _port`。

## 悬架模板 `DOUBLE_WISHBONE` 的 `ports`/`needs` 实际值

- 模板定义：`packages/suspension_multibody/src/suspension_multibody/templates/builtin.py:397-414 DOUBLE_WISHBONE`。实测该 `Template(...)` 调用里**既没有 `ports=` 也没有 `needs=` 参数**（`grep -n "ports=\|needs=" builtin.py` 全仓零命中）——即使用 `Template` 的默认值。
- `Template` 字段默认值：`templates/model.py:311 ports: tuple[PortDeclaration, ...] = ()`、`:313 needs: tuple[PortNeed, ...] = ()`。
  → **`DOUBLE_WISHBONE.ports == ()`、`DOUBLE_WISHBONE.needs == ()`，皆为空元组**（与 F13 一致）。
- 校验逻辑引用：`model.py:410 port_names = [port.name for port in self.ports]`、`:419 for port in self.ports:`。

## 结论

- 代码中**不存在** `arb_mount` / `droplink_mount` / `chassis_mount` 端口。
- 现行端口是装配期由 body 名单**运行期合成**（`si_assembly.py:71/87` 的 `_ports_for_bodies`，`F13` 写的 `:71` 实测为 `:87` 函数体起点，`:104` `_wheel_centre_needs` 实测为 `:120`——**锚点漂移，只记录不改父文件**）。
- 因此 **p4-03 不是「把已有端口接上」，而是「先造出语义化端口」**（F13 结论成立）。
