# p3-03 判据 (d)：报表侧滚转中心通道命名

## 结论：**本行未在报表侧暴露滚转中心通道。**

## 依据

### 1. 检查方法

```
$ grep -rn "roll_center\|roll_centre\|rollcenter" --include=*.py \
    packages/suspension_multibody/src/suspension_multibody/report \
    packages/suspension_multibody/src/suspension_multibody/outputs \
    packages/suspension_multibody/src/suspension_multibody/studies
（零命中；退出码 1）
```

全包范围内（排除 `vehicle/roll_centers.py` 自身）：

```
$ grep -rn "roll_center\|roll_centre" --include=*.py packages/suspension_multibody/src/
packages/suspension_multibody/src/suspension_multibody/vehicle/roll_centers.py:...
（仅此一个文件）
```

`report/` 与 `outputs/` 目录下**没有任何**滚转中心相关的通道声明、派生量或指标。

### 2. 本行的写范围本来就到不了那里

`SUBTASKS.csv` 的 `p3-03` `notes` 与 `SPEC.md`「写范围」把本行限制在
`vehicle/roll_centers.py` 与两个测试目录；`outputs/builtin.py` 的派生输出声明
**明确归 p3-05**（`EPIC.md` 行 221）。本行**没有**改 `report/`、`outputs/`、`studies/`
下任何文件。

自证：

```
$ git status --short -- packages/suspension_multibody/src/suspension_multibody/report \
                       packages/suspension_multibody/src/suspension_multibody/outputs \
                       packages/suspension_multibody/src/suspension_multibody/studies
（空）
```

### 3. 因此判据 (d) 的第二种情形适用

`SPEC.md` 判据 4 的原文：「若本行**不**暴露滚转中心通道，须在 `raw/channel_naming.md` 中记
『未暴露』+ 该决定的依据」。本文件即该记录。

**不存在「硬编码 `front`/`rear` 字面量」的风险**：本行没有新增任何通道名，
`placement`（`roll_centers.py:130` 的 `runtime.axle_assemblies.items()` 的键），
**不是**写死的 `"front"`/`"rear"`；四构型/多轴场景下它是装配说了算的值。
**不是**写死的 `"front"`/`"rear"`；四构型/多轴场景下它是装配说了算的值。
`SPEC.md` 判据 4 的这一层意图（通道名由安装角色驱动）在本行范围内已满足。

### 4. 分层方向未被触碰

本行未改 `report/`，故 `legacy_surface_gate.py` 的两条规则
（`report_native_import` / `report_constitutive_call`）**不受影响**。实测：

```
$ uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
mode      : migration
findings  : 0
OK: no unregistered Python boundary violation
exit 0
```

### 5. 若后续行要暴露（留给 p3-05 / p5-03 的提示，非本行动作）

`RollCenterResult` 已经带 `axle`（= `placement`）与完整的几何读数，
`compute_vehicle_roll_centers` 返回的是 `dict[placement, RollCenterResult]`。
任何后续要暴露通道的行都应从这个 dict 的键生成通道名，而不是另起字面量。
**本行不做这件事，也不为此预留任何代码。**
