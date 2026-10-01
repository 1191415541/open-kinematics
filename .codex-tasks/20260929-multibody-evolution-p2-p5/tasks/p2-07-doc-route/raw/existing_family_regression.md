# p2-07 判据 (d)：既有族文档往返不变、未重录任何基线

## 1. 结论

改造只**新增**一条读取分支与一个追加点，既有族的读取路径逐字未动；既有族的文档往返、
冻结的动态哈希基线与 `tests/data/` 全部不变。

## 2. 既有族文档往返（同一入口，实测）

最小文档：`ground`（固定）+ `slider`（自由），一个**棱柱副**，元素覆盖
**spring / damper / bump_stop / bushing / aerodynamic_drag** 五族（含 bushing 的 6x6 刚度与阻尼、
bump_stop 的 clearance/direction、aero 的 `application_point`/`forward_axis`/`coefficient`）。
文档声明 `mm`（例：`stiffness 300 N/mm`、bump_stop `clearance 20 mm`），
case 为 `axle_dynamic`，`0 → 2 ms`，3 个采样点。

探针：`raw/existing_family_probe.py`（可复跑）。

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-07-doc-route/raw/existing_family_probe.py
status: success
blocks: ['body_state', 'bump_stop_output', 'bushing_output', 'constraint_wrench',
         'damper_output', 'diagnostics', 'energy', 'spring_output']
body_state shape: (3, 2, 19)
spring_output shape: (3, 1, 4)
bushing_output shape: (3, 1, 12)
first sample state row: [0. 0. 0. 1. 0. 0. 0. 0. 0. 0. 0. 0.]
last  sample state row: [0. 0. 0. 1. 0. 0. 0. 0. 0. 0. 0. 0.]
exit=0
```

块名集合与改造前一致（五族各自的 ledger 都在，没有多出也没有少掉），
`spring_output` / `bushing_output` 的每元素宽度（4 / 12）未变。

判据 (a) 记录的 `raw/document_route_probe.py` 也复核过：改造后它不再在**模型**阶段被拒，
而是在下一层按新语义报出 case 文档的问题（那是最小探针的 case 缺 `time`，与本行无关）：

```
$ uv run --no-sync python .codex-tasks/.../raw/document_route_probe.py
KernelContractError: case document: case document has no time object
exit=0
```

即「按类型名拒绝该元素」这条路已经不存在了；换成一份完整的 case 后即为判据 (c) 的 `success`。

## 3. diff 层面的零回归自证

```
$ git diff -U0 -- packages/suspension_kernel/cpp/src/cases/contract_model.cpp \
                 packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp \
                 packages/suspension_kernel/cpp/include/mb_cases/functions.hpp \
  | grep -E '^-' | grep -v '^---'
（空输出）
```

三个文件的新增行之外**没有任何删除行**，也就是说既有六个元素族的解析、保存与 `fill` 路径
一行未动；`contract_registry.cpp` 与 `contract_selftest.cpp` 的删除行只是名字表重排与
同一条计数断言的更新（见 `document_route_implementation.md` 第 3 节）。

## 4. 冻结基线未动

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
acceptance exit  : 1
failed cases     : ['combined_load', 'in_phase_road', 'large_amplitude_high_frequency',
                    'opposite_phase_road', 'road_pulse', 'road_sine', 'road_step_finite_rise',
                    'single_wheel_road', 'tire_liftoff_and_recontact']

OK: dynamic output matches the frozen baseline byte-for-byte
exit=0
```

26 个 artifact 逐字节一致，`combined sha256` 为冻结值；失败用例清单与基线记录一致
（那九个是冻结基线里已记录的 `time_convergence` 失败，不是本行引入的）。

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空输出，退出码 0）
```

`tests/data/` 干净，`kc_baseline/**`、`sha256.json`、`dynamic_hash_baseline.json`、
`kc_perf_baseline_native.json` 都未被写入——本行没有执行任何 `--record`。

## 5. 既有测试套（回归面）

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases -q -p no:cacheprovider
107 passed in 45.04s
exit=0

$ uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider
32 passed in 0.10s
exit=0

$ uv run --no-sync pytest packages/suspension_kernel/tests -q -p no:cacheprovider
41 passed in 14.66s
exit=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/cases/kc_quasi_static -q -p no:cacheprovider
34 passed in 3.14s
exit=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/axle_dynamics \
                       packages/suspension_multibody/tests/subsystems \
                       packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
376 passed in 15.19s
exit=0
```

其中 `tests/cases` 覆盖 `axle_dynamic`、`vehicle_kc`、`vehicle_dynamic`、`handling`、
`ride_random_road`、`ride_four_post` 与 `kc_quasi_static` 各族的契约往返，
`tests/axle_dynamics` 里有冻结字节级哈希的验收矩阵。

## 6. 未被覆盖的部分

- `cpp/tests` 下除 `mb_contract_selftest` 外的其它 C++ 自测（`mb_cases_selftest` 等）
  经 `packages/suspension_kernel/tests/test_registry_consistency.py` 一并跑过（12 passed）；
  剩余未跑的 C++ 自测不在本行写范围内，未触碰。
- 车辆级（`vehicle_dynamic`）真实案例的端到端数值未单独重跑，由第 5 节的两个套件覆盖其契约往返。
