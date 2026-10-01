# p2-09 侦察：力矩元接入装配面的现场事实

> 全部为 2026-10-01 本机实跑/实读结果。来源：`code-reviewer` 裁决 `84c8f264`。

## 1. 缺口的实测证据（改前）

```
$ # 整车装配完成后 elements 的长度
1. elements: 0
2. wheel_centers: {'front_left': 'front_wheel_hub_L', 'front_right': 'front_wheel_hub_R',
                   'rear_left': 'rear_wheel_hub_L', 'rear_right': 'rear_wheel_hub_R'}
3. wheel_body_names: {'front_left': 'wheel_front_left', 'front_right': 'wheel_front_right',
                      'rear_left': 'wheel_rear_left', 'rear_right': 'wheel_rear_right'}
4. runtime 有 ports? False | 有 bindings? False
5. 声明制动的轮端: ['front_left', 'front_right', 'rear_left', 'rear_right']
6. driveline.driven_wheels: () maximum_brake_torque: 10000.0
7. wheel.drive_torque_reaction_body: 四个轮端都是 None
```

即：**装配产物里一个力矩元都没有**。`compose_vehicle_runtime` 返回的 `VehicleRuntime` 既没有 `ports`
也没有 `bindings`，而 `pair_torque_bodies` 需要这两者。

```
$ grep -n "brake\.build\|drive\.build\|brake\.wheel_torque_element\|drive\.wheel_torque_element" -r packages/suspension_multibody/src
（零命中）
$ grep -rn "wheel_torque_element" packages/suspension_multibody/tests
tests/subsystems/test_brake_subsystem.py:105,245,306
tests/subsystems/test_drive_subsystem.py:113
```

即：p2-04 交付的模块函数**唯一调用者是它们自己的测试**。

`subsystems/element_build.py:212-242` 的 `element_rows()` 只从两个来源取行：

```python
    for side in context.request.sides:
        for kind in ("spring", "damper"):
            rows.extend(suspension_subsystem.elements(context, side, kind))
        if context.request.carries("wheel"):
            rows.extend(wheel_subsystem.tires(context, side))
        rows.extend(suspension_subsystem.elements(context, side, "bump_stop"))
    rows.extend(suspension_subsystem.global_elements(context))
```

——没有 brake，没有 drive。

## 2. 装配面已有的、可复用的事实

| 事实 | 出处 | 对 p2-09 的意义 |
|---|---|---|
| 轮端的**非旋转承载件**（upright）由 `wheel_centers[wheel][0]` 给出 | `subsystems/vehicle_assembly.py:76` + `vehicle_parts.py:508/553` 的 `wheel_centers[wheel.name] = (upright, center.copy())` | 制动反力体（「卡钳装在转向节上」）的**声明来源**，可据此建语义端口而不写体名字面量 |
| 轮体由 `wheel_body_names[wheel]` 给出 | 同上 `:77` | 施力体（被制动的轮）的声明来源 |
| `RoleSpec.required_mounts` 已含 `("wheel_center", "spin_axis")` | `templates/roles.py` 的 BRAKE/DRIVE 条目 | 两个角色声明的挂点与轮端事实一一对应 |
| `BRAKE`/`DRIVE` 模板的 `property_slots` 已标准化 | `templates/builtin.py` 的 `BRAKE`/`DRIVE` | `piston_area`/`effective_radius`/`friction_coeff`/`rotor_inertia` 与 `gear_ratio`/`efficiency`/`max_torque` 已是真源，默认值齐备 |
| `pair_torque_bodies` 读 `report.binding_for(role)` 与 `ports[port_id].owner.local` | `compilation/element_blocks.py:159-205` | 只要装配面给出这两个值，配对即可复用既有实现 |
| `match_requirements` 是纯函数 | `connections/matcher.py:102` | 装配层可自行调用，不需要新的配对引擎 |
| 装配层已能产语义端口 | `si_assembly.py:153` 的 `_declared_ports` + `templates/ports.py::declaration_to_port` | p4-03 已建立的机制，p2-09 沿用 |

## 3. 零回归的关键约束（决定实现形态）

`vehicle_dynamics_baseline/sha256.json` 的 8 个用例今日逐位通过，其中 `braking` 用例
（`fixture._case(base, brake=0.4)`）的 7 个 ledger 摘要与 `default` **完全相同**——也就是说
该夹具走的是**旧 N·m 直给路径**，任何让整车默认带上力矩元的改动都会改变这 8 个摘要。

因此 p2-09 必须让力矩元**按声明 opt-in**：只有当装配请求/模型声明了制动或驱动的**需求源**时，
才生成力矩元行；不声明时产物与今日逐位一致。

## 4. 尚未验证的部分（不声称）

- 尚未实现装配层接线，故 `elements` 仍为 0。
- 「旧 N·m 直给路径与新 demand 路径不重复施力」尚未实测——它取决于归一化 role 在内核
  `cases/vehicle_dynamic.cpp:26-34/113-119` 的 `tire_role_of` 表里如何被拒收。
