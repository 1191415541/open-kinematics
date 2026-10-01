# p2-05 证据 (d)：动态基线逐项登记（结论：无变化）

> 实测日期 2026-10-01。

## 1. 8 个冻结用例逐项

```
$ uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py --family vehicle_dynamic
  vehicle_dynamic    PASS      8 cases, bit-identical to the frozen snapshot
OK: 1 families accepted
```

用例名（`case_parity_check.py:568-580`）：`default`、`braking`、`steering`、`pac2002`、
`pac2002 adams`、`nondefault road and initial state`、`measured tire table`、`bushing force curves`。
每个用例的 8 个块（`states`、`constraint_wrench`、`diagnostics`、`energy`、`spring_output`、
`anti_roll_output`、`bushing_output`、`steering_output`）摘要**逐项相等**，差异数 **0**。

## 2. combined dynamic hash

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
```

与 p2-01 记录的起点值一致。

## 3. 未重录

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空）
```

`vehicle_dynamics_baseline/sha256.json`、`kc_baseline/`、`dynamic_hash_baseline.json`
均未被写。**本行无任何需登记的基线变化**，故未使用 D5 的物理等价判据通道。

## 4. 结论

本行把「旧表」与「新需求表」做成互斥的**声明分支**，默认分支（`none`）的代码路径与
本行改动前**逐字相同**（唯一改动是在它外面加了一层 `if`），故冻结基线的逐位一致是
构造性的，而非调参得到的。
