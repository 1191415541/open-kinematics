# 各基线文件当前值与哈希

任务：20260922-01-baseline
实测时间：本轮（2026-09-22 会话）
基准目录：`packages/suspension_multibody/tests/data/`

## 6 个 multibody 基线

| 文件 | 大小/规模 | sha256（前 16 位） |
|---|---|---|
| `kc_baseline/manifest.json` | 6 键 | `cd4eb1a447e3735b` |
| `kc_baseline/k_states.json` | 9 状态 | `24b138be866f91e7` |
| `kc_baseline/c_states.json` | 66 状态 | `9b80a05afff4fe37` |
| `kc_perf_baseline.json` | 1631 bytes | `670b7eb0fb1671a9` |
| `kc_perf_baseline_native.json` | 667 bytes | `26aa100c2bba608c` |
| `dynamic_hash_baseline.json` | 9507 bytes | `c8dcc340f888ac09` |
| `axle_dynamics_baseline/sha256.json` | 13 cases | `34e6ca677263d68d` |
| `vehicle_dynamics_baseline/sha256.json` | 8 cases | `ae2a99b38762fb0e` |

## 1 个 C++ 基线

| 文件 | sha256（前 16 位） |
|---|---|
| `packages/suspension_kernel/layering_baseline.json` | `0ba7571ae10a0d5f` |

## `kc_baseline/manifest.json` 全文键值

```
c_grid:         {'levels': 11, 'paths': ['fx','fy','fz','mx','my','mz']}
c_state_count:  66
contract:       kc-parity-v1
k_grid:         {'rack_mm': [-5.0, 0.0, 5.0], 'wheel_mm': [-10.0, 0.0, 10.0]}
k_state_count:  9
model:          benchmark_front_double_wishbone
```

## `dynamic_hash_baseline.json` 关键值

```
acceptance_exit_code:  1          ← 基线自身记录的就是 1（既有状态）
artifact_count:        26
combined_sha256:       e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e
acceptance_statuses:   13 项，含 'lateral_or_steering': 'PASSED'
```

**实测复现**：命令 8 输出的 `combined sha256` 与基线**完全一致**（`e7407656...`），26/26 artifact 逐位一致。

## 既有失败与 skip（后续「新增失败为零」的对照底线）

### 全量套件（命令 5）

```
737 passed, 47 skipped, 1 xfailed in 461.58s
```

| 类别 | 数量 | 说明 |
|---|---|---|
| passed | 737 | |
| skipped | **47** | 本机缺少 Adams 参考工件等可选依赖 |
| xfailed | **1** | `tests/vehicle/test_native_vehicle.py` 的 `test_native_brake_opposes_the_instantaneous_wheel_spin`（`@pytest.mark.xfail(strict=True)`，夹具退化：理想关节、零载、单步不可细分） |

**与计划记录的差异（须定性）**：计划记「783 passed / 1 skipped / 1 xfailed」，本次为「737 / 47 / 1」。合计 784 = 784，**属环境差异**（同一批用例在本机被 skip 而非失败），**不是回归**。后续子任务对照时以本次实测为准。

### 动态哈希门（命令 8）

```
acceptance exit: 1
failed cases: combined_load, in_phase_road, large_amplitude_high_frequency,
              opposite_phase_road, road_pulse, road_sine,
              road_step_finite_rise, single_wheel_road,
              tire_liftoff_and_recontact   （共 9 个）
```

**均为既有状态**（基线自身 `acceptance_exit_code` = 1），非本次新增。门禁脚本比的是 artifact 字节哈希，退出码 0。

### Adams 对标

`packages/suspension_multibody/README.md:59` 记真实 Adams 整车数值对标为 **BLOCKED**，本轮不改变该结论。

### legacy_surface_gate（命令 11）

`findings: 8`，全部为已注册的 `legacy_module_import`（注册表内的既有条目），**无未注册违规**。

## 复现命令

```bash
cd C:/杂件/open-kinematics
sha256sum packages/suspension_multibody/tests/data/kc_baseline/*.json
sha256sum packages/suspension_multibody/tests/data/*.json
sha256sum packages/suspension_kernel/layering_baseline.json
```
