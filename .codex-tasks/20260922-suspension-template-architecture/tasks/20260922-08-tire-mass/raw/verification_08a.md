# 08 前半（契约字段 + Tire 携带质量）验证记录

仓库：`E:/杂件/open-kinematics`。以下退出码均为本机实测。

## 主验证命令

| 命令 | 退出码 | 输出摘要 |
|---|---|---|
| `uv run python packages/suspension_multibody/scripts/build_axle_native.py` | 0 | 重建 CMake+Ninja 全量（99 目标），镜像 dll 落在 `packages/suspension_multibody/src/suspension_multibody/native/suspension_kernel.dll` |
| `uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q` | 0 | `27 passed` |
| `uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q` | 0 | `21 passed`（含新增 6 例） |
| `uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict --final` | 0 | `OK: layering matches the recorded baseline`；cycles 0、跨聚合 include 0 |
| `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/architecture -q` | 0 | `91 passed` |
| `git diff --check` | 0 | 无空白错误 |

## 额外门（自行加跑，属本步变更范围相关的数值门）

| 命令 | 退出码 | 输出摘要 |
|---|---|---|
| `uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check` | 0 | `combined sha256 e7407656731ed556efc28fb89d8fc69881725b3bfb2f39066eb898986389d48e`，`OK: dynamic output matches the frozen baseline byte-for-byte` |
| `uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check` | 0 | `OK: candidate matches the frozen K/C snapshot within tolerance` |
| `uv run python packages/suspension_multibody/scripts/case_parity_check.py` | 0 | `OK: 8 families accepted` |
| `uv run --package suspension-multibody pytest packages/suspension_multibody/tests/axle_dynamics/test_native_mirror_freshness.py -q` | 0 | `7 passed` |

动态字节门**未重录**：本步只让字段到达 `Tire`，求解器未读该字段（见 `no_behavior_change_08a.*`），
所以三个门原样通过，无 D4 登记事项。

## 行为不变的独立实测（动态解）

`no_behavior_change_08a.py`：取作者层真实发射的 `axle_dynamic` 模型文档（`road_pulse` 工况），
一份原样、一份给每个 tire 追加 `mass=12.5` 与对角 `inertia`，各跑一次 `suspension_kernel_run`：

```
body_state      (401, 6, 19)  bit-identical: True   max|diff| 0.0
energy          (401, 21)     bit-identical: True   max|diff| 0.0
diagnostics     (403, 16)     逐位相等（含 NaN 位） nanmax diff 0.0
```

即本步不改变任何数值输出，符合「只让字段到达 Tire」的范围界定。
