# 04 证据：质量/质心/惯量口径不变

## 1. 轮胎不带质量（本仓 schema 的事实）

`schema/elements.py:215 class VerticalTire` 的字段是
`stiffness` / `unloaded_radius` / `contact_point` / `local_axis` —— **没有质量、没有惯量**。
单轴侧的轮胎是 `modeling/primitives/elements.py:567 VerticalTireElement`（垂向力元），
同样只有 `wheel_body` / `wheel_center_local` / `stiffness` / `unloaded_radius`。

所以「轮胎质量所有权」在本仓单轴侧**不存在可迁移的量**：不存在把轮胎质量搬到别的体上的路径，
本行也没有新增这样一条。质量只出现在**车轮刚体**上，而那只在车轮模板声明车轮体时才存在。

## 2. 凝结搬的是车轮体的质量，且只是并入挂接体

`subsystems/vehicle_parts.py::_merge_fixed_wheel` 的复合质量口径（本行原样复用，未改一行）：

* `total_mass = mount_mass + wheel_mass`；
* `composite_com = (mount_mass * mount_com + wheel_mass * wheel_center_local) / total_mass`；
* `composite_inertia = mount.inertia + I_parallel(mount_mass) + wheel_inertia_local + I_parallel(wheel_mass)`；
* `total_mass <= 0` 时只把 mount 的质量置 0（既有的边界分支，未改）。

即：车轮的质量/质心/惯量**全部进入挂接体**，没有任何一部分被丢掉或被挪到第二个体上；
实体集合里也不再有独立车轮体（`_condense_wheel_end` 会删掉它）。
这就是「冻结基线逐位不变」的物理原因：内置模板不声明车轮体时，复合口径退化为恒等
（`wheel_mass = 0` 且 `wheel_inertia = 0` → `total_mass = mount_mass`、
`composite_com = mount_com`、`composite_inertia = mount.inertia`）。

## 3. 断言

`packages/suspension_multibody/tests/subsystems/test_wheel_lifecycle.py`：

* `test_a_declared_wheel_body_is_condensed_into_the_body_carrying_the_wheel_centre`
  * `hub_condensed.mass == approx(hub_plain.mass + 12.0)`（质量守恒进挂接体，误差 1e-12 相对）；
  * `hub_condensed.inertia >= hub_plain.inertia`（惯量随之增大，不是点质量）；
  * 实体集合、`constraints` / `ideal_constraints` 行数、轮胎归属、轮心点与内置读数一致。
* `test_the_built_in_wheel_subsystem_declares_no_body_and_still_places_the_tire`
  * 内置读数下没有 `wheel_L` 体，轮毂质量仍是 `WHEEL_HUB_MASS = 1.102840393`。

## 4. 结果侧的一致性

* `kc_baseline`：`git status --short -- packages/suspension_multibody/tests/data/` 为空；
  `kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` 退出 0。
* 轴侧动态：`dynamic_hash_sentinel.py --check` 26 个 artifact 逐字节一致，
  combined sha256 `fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9`。
* 产物：01 快照 `--check` 零差异。
