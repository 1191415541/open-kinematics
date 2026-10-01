# p4-02 判据 (d)：`upright_L` / `upright_R` 硬编码删除

## 1. 父行 validation_command 的第二段（实测）

```
$ bash -c '! grep -rn "upright_L" packages/suspension_multibody/src/suspension_multibody/subsystems/suspension.py'
grep-exit=0
```

退出码 0 = **零命中**。

## 2. 两个名字在防倾杆构造路径的命中（实测）

```
$ grep -n "\"upright_L\"\|\"upright_R\"" packages/suspension_multibody/src/suspension_multibody/subsystems/suspension.py
（零命中）
```

## 3. 删除前 → 删除后

**删除前**（`global_elements`，改造前原文）：

```python
body_a="upright_L",
point_a=context.local("upright_L", spec.left_link_point.as_array()),
body_b="upright_R",
point_b=context.local("upright_R", spec.right_link_point.as_array()),
```

**删除后**：

```python
left = _wheel_end_body(context, "L")
right = _wheel_end_body(context, "R")
...
    body_a=left,
    point_a=context.local(left, spec.left_link_point.as_array()),
    body_b=right,
    point_b=context.local(right, spec.right_link_point.as_array()),
```

## 4. 替代机制：读**声明**，不是换一种按名字的推断

新函数 `_wheel_end_body(context, side)` 遍历 `_suspension_template(context).connections`，
取角色为 `wheel_center`、`far_owner` 非空、且属于该侧的那条连接的 `far_owner`。

- 双叉臂模板声明的正是 `ConnectionDefinition("wheel_spin_joint_L", "wheel_center",
  owner="wheel_hub_L", far_owner="upright_L", ...)`，所以读出来就是 `upright_L`，
  **但这个名字是模板说的，不是模块里写死的**。
- 换一个把轮端挂在别的体上的模板，答案随之改变，本函数一字不改。
- 声明里根本没有轮端体的模板会被**点名拒绝**（`ValueError` 含模板名与侧别），
  而不是被猜一个名字。

这和 `EPIC.md:233` 的要求一致：跨边界取身份必须来自形式化声明，不得按名字猜。

## 5. 行为未变的实测

```
$ uv run --no-sync python .codex-tasks/.../p4-02-arb-subsystem/raw/arb_rows_probe.py
AntiRollBarElement rows: 1
  name='arb'
  left_body='upright_L'   left_point=[   0. -700.  100.]
  right_body='upright_R'  right_point=[  0. 700. 100.]
  stiffness=1000.0
```

端体、作用点、刚度**与改造前逐位相同**。
