# 04 证据：单轴侧凝结的等价性（D2 硬门）

本文件记录的是**已执行**的命令与退出码，不是预期值。所有命令在 `E:\杂件\open-kinematics` 下执行。

## 1. K/C 逐位等价（用 probe 生成 actual，再带 `--actual-dir` 比对）

```
uv run --no-sync python packages/suspension_multibody/scripts/kc_native_probe.py
  -> checked 9 K states, worst error / tolerance ratio 1.65548e-05
  -> worst component: ('k-w-10-r+5', 'left_wheel_center_x_mm')
  -> wrote candidate K states -> artifacts/kc-native-probe/k_states.json
  -> exit 0

uv run --no-sync python packages/suspension_multibody/scripts/kc_native_c_probe.py
  -> checked 66 C states, worst error / tolerance ratio 0.000185873
  -> worst component: ('c-fy--0.60', 'deformation_left', 2)
  -> wrote candidate C states -> artifacts/kc-native-probe/c_states.json
  -> exit 0

uv run --no-sync python packages/suspension_multibody/scripts/kc_parity_check.py \
    --check --actual-dir artifacts/kc-native-probe
  -> OK: candidate matches the frozen K/C snapshot within tolerance
  -> exit 0
```

判据说明：不带 `--actual-dir` 的自比较恒过、不构成证据（AGENTS.md 第 2 节），本行用的是带 actual 的形式。

## 2. `kc_baseline` 未被重录

```
git status --short -- packages/suspension_multibody/tests/data/
  -> （空输出，exit 0）
```

即 `tests/data/kc_baseline/` 与 `tests/data/dynamic_hash_baseline.json` 都没有被本行写过。

## 3. 轴侧动态逐字节不变（cyber 门）

```
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
  -> artifacts hashed : 26
  -> combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
  -> OK: dynamic output matches the frozen baseline byte-for-byte
  -> exit 0
```

`fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` 与 03 收尾时记录的
combined sha256 完全一致（26 个 artifact，含 13 个轴侧用例），未重录。

## 4. 产物未变化（01 快照）

```
uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check
  -> OK: the seven combinations assemble exactly what the snapshot froze
  -> exit 0
```

（在把 wheel 贡献块前移到悬架行之前**又跑了一次**，仍是同一结果。）

## 5. 凝结的等价性怎么被证明的

`kc_baseline` 逐位不变是**结果**；本行给出的、独立于结果字节的等价判据是：

| 量 | 凝结前（内置模板，无车轮体） | 凝结后（模板声明 `wheel_L/R`，各 12 kg） |
|---|---|---|
| 实体集合 | 13 个 body | 同 13 个（`wheel_L/R` 被折进 `wheel_hub_L/R`） |
| `constraints` 行数 | 16（K，无试验台） | 16 |
| `ideal_constraints` 行数 | 16 | 16 |
| 轮胎行 | `tire_L -> wheel_hub_L` | `tire_L -> wheel_hub_L` |
| 轮心点 | `(wheel_hub_L, wheel_center) = [0, -700, 300]` | 同值 |
| 轮毂质量 | `WHEEL_HUB_MASS` | `WHEEL_HUB_MASS + 12.0`（复合质量，平行轴惯量） |

以上逐条由 `packages/suspension_multibody/tests/subsystems/test_wheel_lifecycle.py`
的 `test_a_declared_wheel_body_is_condensed_into_the_body_carrying_the_wheel_centre`
断言（该模板只让出**质量**；内置模板不声明车轮体，所以既有产物一位不动）。

## 6. 复用的既有机制（未新造）

* `subsystems/vehicle_parts.py::_merge_fixed_wheel`（复合质量 + 平行轴惯量）——凝结的算术全部走它；
* `subsystems/vehicle_parts.py::_fuse_welded_bodies` / `_condense_welded_bodies`——未被本行调用，
  也未改动；本行没有新造第二套凝结。

与 **20260921 Epic 的 A3** 的关系（避免两处"凝聚"语义混淆）：
A3 的裁决是**整车侧**把 weld 交给内核的 `fixed` 关节，**不凝聚**（`_condense_welded_bodies`
默认返回原值，只有 `SUSPENSION_MULTIBODY_CONDENSE_WELDS=1` 才融合）。本行做的是**单轴侧**
（`RigSpec.supplies_wheels=True` 的读数）把车轮体折进轮毂，方向相反、作用域不相交：
整车侧不凝结的结论未被触碰（01 快照与 `dynamic_hash_sentinel` 的 26 个 artifact 都逐项/逐字节未变）。
