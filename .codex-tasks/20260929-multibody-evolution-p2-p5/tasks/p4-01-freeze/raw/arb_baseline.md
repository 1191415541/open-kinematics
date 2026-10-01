# ARB 冻结产物复核（p4-01 冻结证据 · F17）

本行**只读**，未重录任何基线。

## 1. `tests/data/axle_dynamics_baseline/sha256.json` — `anti_roll_output`

命令：
```bash
grep -n "anti_roll_output" packages/suspension_multibody/tests/data/axle_dynamics_baseline/sha256.json
```
命中 **13 次**（13 个 case 各一行），值全部相同，原样贴出：
```
4:      "anti_roll_output": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
13:     ... 同值 ...
22/31/40/49/58/67/76/85/94/103/112:  同值
```
值 = `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` —— 这正是**空字节的 sha256**（`sha256(b"")`），即基线记录的是「**无 ARB**」。
文件顶层：`"contract": "axle-dynamics-snapshot-v1"`，`"recorded_from": "the ctypes route (axle_run) ..."`。

## 2. `tests/data/vehicle_dynamics_baseline/sha256.json` — 同字段

命令：
```bash
grep -n "anti_roll_output" packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/sha256.json
```
命中 **8 次**（8 个 case），值同样全部为：
```
"anti_roll_output": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
```
（同为**空字节 sha256**。）
注：同文件 `spring_output` / `bushing_output` 在某些 case 也是空字节值（例如 `braking`、`default`），属该基线既有口径。

## 3. `tests/data/dynamic_hash_baseline.json` — 是否有 ARB 字段

命令：
```bash
grep -in "anti_roll\|arb" packages/suspension_multibody/tests/data/dynamic_hash_baseline.json
```
输出：**空（零命中，退出码 0）**。
文件顶层键为 `acceptance_exit_code` / `acceptance_statuses` 等；每条目字段只有 `arrays_npz_sha256` / `manifest_sha256` / `status` —— **无 ARB 字段**。

## 4. `tests/data/kc_baseline/` — `anti_roll|arb` 命中数

命令：
```bash
grep -rin "anti_roll\|arb" packages/suspension_multibody/tests/data/kc_baseline/
```
输出：**空（零命中，退出码 1 = 无匹配）**。
该目录 3 个文件：`c_states.json`、`k_states.json`、`manifest.json`（`git ls-files` 确认）。

## 结论与 D5 登记口径

- 冻结产物记录的是「**无 ARB**」（空字节 sha256）→ ARB 独立化**不太可能**扰动既有数值基线。
- **不得据此放松**：任何产物变化仍按 D5（`EPIC.md:64`）逐项登记「文件 + 步骤 + 前后值 + 独立于结果字节的物理等价判据」。
- 本行未跑数值门、未重录基线；`kc_baseline/` 与 `dynamic_hash_baseline.json` 逐字节未动。
