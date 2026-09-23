# 轮胎/轮端质量现状路径

任务：20260922-01-baseline
实测时间：本轮（2026-09-22 会话）

## 数据源

`packages/suspension_multibody/src/suspension_multibody/schema/vehicle.py:60-61`：

```python
mass: float = Field(default=0.0, ge=0)
axial_inertia: float = Field(default=1.0, gt=0)
```

属 `WheelSpec`（类定义在 `:39`）。**注意默认值为 0.0**。

## 合并路径（质量进轮端刚体）

### 分支一：非固定轮（有自转副）

`preparation/assembly/vehicle.py:650-655`：

```python
bodies[wheel.body] = RigidBody(
    name=wheel.body,
    pose=SE3(origin, quaternion),
    mass=wheel.mass,
    inertia=wheel_inertia,
)
```

### 分支二：固定轮（`mount_joint_kind == "fixed"`）

`preparation/assembly/vehicle.py:632-640` 调 `_merge_fixed_wheel(...)`，其内部（约 `:718-726`）：

```python
mount_mass = float(mount.mass)
total_mass = mount_mass + float(wheel_mass)
if total_mass <= 0.0:
    bodies[mount_body] = replace(mount, mass=0.0)
    return
mount_com = np.asarray(mount.center_of_mass, dtype=float)
```

即**固定轮的质量被并入所挂载的刚体**（`mount_body`），并重算质心与惯量。

## 内核侧：`Tire` 结构体无质量字段

`packages/suspension_kernel/cpp/include/mb_model/types.hpp:101-125` 的 `Tire` 字段实测：

```
body, frame_body, drive_torque_body, drive_torque_reaction_body,
center, frame_center, drive_torque_axis, spin_axis, forward_axis,
radius, maximum_compression, k, c,
mu_longitudinal, mu_lateral,
brush_k_longitudinal, brush_k_lateral,
relaxation_length_longitudinal, relaxation_length_lateral,
detached_relaxation,
model_kind, state_slot_width, ...
```

**确认无质量、无惯量字段。** 质量全部挂在刚体上。

## 总质量/质心聚合点

`preparation/assembly/vehicle.py:374-397` 是整车总质量与质心的聚合处（`_fuse_welded_bodies` 之外的聚合逻辑）。

## 契约 schema 对 tire 是闭合的

`packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json` 的 tire 定义 `additionalProperties: false`，`required: [name, model, body]`，`properties: [blob, body, model, name, parameters]`。

**因此 08 新增轮胎质量字段必须同时改契约 schema**，否则文档校验先失败。

## kc 侧完全不发轮胎

实测：`cases/kc_quasi_static/contract.py:133-150` 的 `model_document` 顶层键为 `contract`/`contract_version`/`kind`/`name`/`units`/`gravity`/`bodies`/`joints`/`elements`/`markers`/`body_wrench_markers`，**无 `tires`**。

kc 侧虽有 `VerticalTireElement`（`preparation/assembly/front_axle.py:561-574`），但不进文档。

## 08 的落点说明

- **本任务（01）只记录现状，不改任何文件。**
- 08 的写范围是 C++ 内核 + 契约 schema + 契约文档读取 + native 镜像；Python 侧发射落点（`cases/vehicle_dynamic.py:327` 的 `_tire_entry`、`cases/axle_dynamic.py:342`、`schema/vehicle.py:60`）属 02 与 04-06 写范围。
