# ABI 未变证据（08 前半）

## 七符号与版本常量

导出符号（`nm` 读重建后的 dll）：

```
```

版本常量实测（从构建产物 `native_build.json` 读回，非手抄）：

```
abi_version = 15
vehicle_abi_version = 30
core_abi_version = 1
```

## Tire 字段偏移（append-only 断言）

`mb_config/version.hpp` 常量：`kAxleKernelAbiVersion = 15`、`kVehicleKernelAbiVersion = 30`、`kCoreKernelAbiVersion = 1`，本步未改。

`AxleInput` / `VehicleInput` 字段数（本步未追加任何字段）：106 / 97。

`Tire` 既有 32 个字段偏移全部不变；新增 `mass` / `inertia` 追加在末尾：

```
offset mass 2096
offset inertia 2104
sizeof(Tire) 2176    # 修改前 2096，即 +80 = 8 (mass) + 72 (Mat3)
```

断言位置：`packages/suspension_kernel/tests/test_tire_mass.py` 的
`test_the_appended_fields_left_every_earlier_tire_offset_alone`、
`test_the_abi_version_constants_are_unchanged`、
`test_the_frozen_input_structures_are_untouched`，
数据由 `packages/suspension_kernel/tests/fixtures/tire_mass_probe.cpp` 用 `offsetof` 实测打印。
