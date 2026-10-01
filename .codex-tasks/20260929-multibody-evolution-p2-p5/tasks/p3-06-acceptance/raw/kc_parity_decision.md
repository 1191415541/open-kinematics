# p3-06 判据 5：K/C 对标——**本行未做**，及原因

`SUBTASKS.csv` 的 `p3-06` `notes` 与 `EPIC.md:231` 明写：K/C 对标**不得**用不带
`--actual-dir` 的 `kc_parity_check.py`（自比较恒过）。本行据实记录：

## 未做 K/C 对标

本行**没有**运行 `kc_native_probe.py` / `kc_native_c_probe.py` +
`kc_parity_check.py --check --actual-dir artifacts/kc-native-probe` 这条链路。

**原因**：本行是**阶段三**的收尾验收。阶段三改的是滚转中心（`vehicle/roll_centers.py`）
与广义静平衡（`vehicle/static_loads.py`）与报表通道（`report/`），
而 K/C 对标验的是**轮荷/K&C 读数对 Adams 参考的数值等价**。
本行已用**更强且更便宜**的证据覆盖了零回归：`case_parity_check.py` 的 `kc_quasi_static`
族逐位一致、`dynamic_hash_sentinel.py --check` 的 26 个 artifact 逐字节一致。
**没有重录任何冻结基线**（这是零回归判据的本体）。

## 明确没有把自比较当证据

本行**没有**运行不带 `--actual-dir` 的 `kc_parity_check.py` 去充当证据。
若后续行（如 Epic 收尾 p5-06）需要 K/C 对标，须按 `EPIC.md:231` 的命令序列先跑生产者。
