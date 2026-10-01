# p3-01 / 子任务 4（Goal 4）：基线影响面复核（F10，`EPIC.md:140`）

## 0. 基线文件位置与只读自证

`EPIC.md:132` / SPEC 写的 `tests/data/kc_baseline/...` 是**简写**；实测真实路径在包内：

```bash
find . -type d -name kc_baseline
# ./packages/suspension_multibody/tests/data/kc_baseline
find . -name "dynamic_hash_baseline.json" -not -path "*/node_modules/*"
# ./packages/suspension_multibody/tests/data/dynamic_hash_baseline.json
```

四个文件实测大小与 sha256（`cd packages/suspension_multibody/tests/data && sha256sum ...`）：

```
9b80a05afff4fe37102ac46a075aeadb1d0c45240a93d87c98b0ab4b7fb9c010 *kc_baseline/c_states.json
24b138be866f91e7b230cf863472f98bb991ea094edad74e18ed512f015c68a8 *kc_baseline/k_states.json
cd4eb1a447e3735b7352916f3c5f279fc0ce0de6b3aa1e399675c292cbb309cf *kc_baseline/manifest.json
7f81076b011cb006a6f23fed9b0e75dc6ec6f34806725359214a3b483a49898b *dynamic_hash_baseline.json
```

文件清单（`ls -la kc_baseline/`）：`c_states.json`（94375 B）、`k_states.json`（5058 B）、`manifest.json`（417 B）；`dynamic_hash_baseline.json`（9507 B）。**本行只读，未重录任何一份。** 上面的 sha256 是本行写文件的**起点记录**，供 p3-06 零回归对照。

## 1. roll center 类字段命中数（实测）

命令 1：

```bash
cd packages/suspension_multibody/tests/data
grep -rniE 'roll_center|rollcentre|roll centre' kc_baseline/ dynamic_hash_baseline.json
```

真实输出：**空**（无任何行），`exit code = 1`（grep 无命中）。→ **命中数 = 0。**

命令 2（放宽到 F10 点名的两个量）：

```bash
grep -rniE 'roll_stiffness|track_change' kc_baseline/ dynamic_hash_baseline.json
```

真实输出：**空**，`exit code = 1`。→ **命中数 = 0。**

命令 3（确认搜索范围不是空的）：

```bash
grep -rl '' kc_baseline/
# kc_baseline/c_states.json
# kc_baseline/k_states.json
# kc_baseline/manifest.json
```

三个文件都可读且非空，所以命令 1/2 的 0 命中是**真的 0 命中**，不是搜错了目录。

**结论：`kc_baseline/{k_states.json,c_states.json,manifest.json}` 与 `dynamic_hash_baseline.json` 都不含 roll center 字段，也不含 `roll_stiffness` / `track_change`。F10（`EPIC.md:140`）判定成立。**

## 2. 两个基线的实际字段清单（完整落盘）

### 2.1 `kc_baseline/k_states.json` — 顶层是 **list**，9 个条目

叶子键名集合（实测）：

```
['case_id', 'left_camber_deg', 'left_toe_deg', 'left_wheel_center_x_mm',
 'left_wheel_center_y_mm', 'left_wheel_center_z_mm', 'rack_displacement_mm',
 'right_camber_deg', 'right_toe_deg', 'right_wheel_center_x_mm',
 'right_wheel_center_y_mm', 'right_wheel_center_z_mm', 'wheel_travel_mm']
```

→ 与 F10 一致：含 `left_wheel_center_y_mm`、camber、toe；**无 roll center**。

### 2.2 `kc_baseline/c_states.json` — 顶层是 **list**，66 个条目

叶子键名集合（实测）：

```
['camber_deg_difference', 'case_id', 'deformation_left', 'deformation_right',
 'left_camber_deg', 'left_toe_deg', 'left_wheel_center_x', 'left_wheel_center_y',
 'left_wheel_center_z', 'level', 'load_left', 'load_right', 'path',
 'right_camber_deg', 'right_toe_deg', 'right_wheel_center_x',
 'right_wheel_center_y', 'right_wheel_center_z', 'side_mode',
 'toe_deg_difference', 'track_mm', 'wheel_center_z_difference',
 'wheel_center_z_mean']
```

→ 与 F10 一致：含 `track_mm`、camber、toe；**无 roll center**。

### 2.3 `kc_baseline/manifest.json` — dict，全文（417 B）

```json
{
  "c_grid": { "levels": 11, "paths": ["fx","fy","fz","mx","my","mz"] },
  "c_state_count": 66,
  "contract": "kc-parity-v1",
  "k_grid": { "rack_mm": [-5.0,0.0,5.0], "wheel_mm": [-10.0,0.0,10.0] },
  "k_state_count": 9,
  "model": "benchmark_front_double_wishbone"
}
```

叶子键名：`['c_state_count', 'contract', 'k_state_count', 'levels', 'model', 'paths', 'rack_mm', 'wheel_mm']`。**无 roll center 字段。**

### 2.4 `dynamic_hash_baseline.json` — dict，6 个顶层键

顶层键（实测）：`['acceptance_exit_code', 'acceptance_statuses', 'artifact_count', 'combined_sha256', 'entries', 'failed_cases']`

`entries` 长度 = **26**（与 F10 的「26 条目」一致）。每个 entry 的键集合（全部 26 条并集，实测）：

```
['arrays_npz_sha256', 'artifact', 'completed_samples', 'manifest_sha256', 'status']
```

即 **F10 写的「只有 `arrays_npz_sha256` / `manifest_sha256` / `status`」漏了另外两个键**：`artifact`（条目名，如 `"braking/native/native_result"`）与 `completed_samples`（实测 26 条全为 `null`）。F10 的说法**方向正确、枚举不全**，本行按实测补全。全部 26 条 `status` 都是 `"success"`（`distinct statuses: ['success']`）。

**无 roll center 字段。** 首条原文样例：

```json
{"arrays_npz_sha256": "599e0d1292baa148eaa09ebdc159e86987bc369317aa10654bd6bde36f219298",
 "artifact": "braking/native/native_refined_result",
 "completed_samples": null,
 "manifest_sha256": "73b818872d2f33f6e4c3282e57695f830576f95e3a10d774bbdd044a03e0b549",
 "status": "success"}
```

（另记：顶层 `acceptance_exit_code` = 1；`acceptance_statuses` 实测共 **13** 个工况，其中 `PASSED` 4 个、`FAILED` 9 个。这是**既有基线内容**，与阶段三无关，本行只记录不改动。）

## 3. `roll_stiffness`（只有常量，无计算实现）

命令：

```bash
grep -rn "roll_stiffness" --include=*.py --include=*.json --include=*.md . | grep -v "^./.codex-tasks"
```

产品代码命中 **1 处**：

```
packages/suspension_multibody/src/suspension_multibody/adams/vehicle_parameters.py:40:    roll_stiffness: float = 55_000.0
```

上下文（`:36-43`）：它是 `suspension_stiffness` / `suspension_damping` / `tire_vertical_stiffness` 等一串 Adams 车辆参数常量中的一个，纯数据类字段。**F10 给出的 `adams/vehicle_parameters.py:40` 未过期。**

其余命中全部落在 `artifacts/**/adams_reference_bundle.json`（实测 24 个文件、`"roll_stiffness": 55000.0` 的 JSON 记录，属 Adams 参考产物，不是产品代码；`grep -rln "roll_stiffness" artifacts/ | wc -l` → 24）。

→ **`roll_stiffness` 没有计算实现，只有常量声明。** 全仓无任何函数用这个字段算出滚转刚度。

## 4. `track_change`（只有模板声明，无计算实现）

命令：

```bash
grep -rn "track_change" --include=*.py --include=*.json --include=*.md . | grep -v "^./.codex-tasks"
```

**multibody 包内**命中 **2 处**，F10 给出的两处**均未过期**：

```
packages/suspension_multibody/src/suspension_multibody/templates/builtin.py:393:    OutputDeclaration("track_change", "mm", "derived"),
packages/suspension_multibody/src/suspension_multibody/templates/roles.py:85:        outputs=("wheel_travel", "camber", "toe", "track_change"),
```

上下文原文：
- `builtin.py:389-395` `_OUTPUTS` 元组，含 `OutputDeclaration("track_change", "mm", "derived")`（`:393`）。注意 `kind` 写的是 `"derived"`，但**没有任何代码把这个声明解算出来**——`grep track_change` 在 `suspension_multibody/src/` 的其余部分零命中。
- `roles.py:71-90` 的 `suspension` `RoleSpec`，`outputs=("wheel_travel", "camber", "toe", "track_change")`（`:85`）。同一份四个输出也在 `builtin.py:390-393` 重复声明。

**其余命中与 multibody 无关**（属另一个解算器产品 `suspension_kinematics`，两产品不得互相导入）：
```
packages/suspension_kinematics/src/suspension_kinematics/metrics/vehicle_geometry.py:92:def calculate_track_change(ctx: MetricContext) -> float:
packages/suspension_kinematics/src/suspension_kinematics/metrics/catalog.py:81:        MetricDefinition("track_change_mm", calculate_track_change),
```
以及 `suspension_kinematics` 的 GUI 标签（`gui/suspension/reporting.py:36`、`workbench.py:99`）与测试。**`suspension_kinematics` 里有实现，`suspension_multibody` 里没有**；`EPIC.md:138`（F9）明说两解算器产品不得互相导入，所以那份实现不可复用。

`artifacts/real-adams-car/**` 里还有一批 `ltrack_change` / `rtrack_change`（Adams 输出通道名），同样是产物不是本包实现。

→ **`track_change` 在 multibody 里只有两处声明（模板输出声明 + 角色契约），无计算实现。** F10 判定成立。

## 5. 对阶段三的结论（F10 复核）

- 通用滚转中心引擎**不会被既有基线保护**：`kc_baseline` 与 `dynamic_hash_baseline` 里 roll center 类字段命中 0，新引擎的正确性只能靠**新增断言**。
- 但**「不改变已有 K/C 读数」仍是硬门**（`EPIC.md:228`、D5 `:64`）：`roll_centers.py` 今天 `vehicle → subsystems.geometry` 的依赖（`roll_centers.py:22`）已经存在；p3-03 引入新引擎若加重该方向，须过 `tests/architecture/test_import_boundaries.py`（`vehicle` 在该文件的被检查包列表中，实测 `:92`）。
- 本行未改动上述任何文件；四个基线文件的 sha256 见 §0，供 p3-06 复核。
