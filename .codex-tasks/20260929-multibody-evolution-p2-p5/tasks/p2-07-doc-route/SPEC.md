# SPEC：p2-07 内核文档路由接受 rotational_torque 元素族

> 子任务规格。父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p2-07`。
> 形态：`single-full`。**本行由 `code-reviewer` 裁决 `fb245729` 后新增**（来源见 `notes`）。

## Goal

把下面 4 条拆开，逐条可判定。

1. **生产文档路径能读 `rotational_torque` 族**：`ContractModel::read` 解析并保存该族，
   文档入口在既有逐族 `build_model` 之后对这些块调用既有块读取器追加。
   **不改公开 ABI**（不新增 `AxleInput` 逐族字段、不动 `mb_config/version.hpp` 与
   `kernel/native.py` 的版本常量），**不放宽** `build_model.cpp` 的两种输入互斥守卫。
2. **`contract_registry.cpp` 的 `kElements[]` 与契约 schema 的 `element.type` enum 同步加入该族**：
   否则 `validate_model` 与文档解析都会先拒绝。
3. **可达性用「同一模型两次运行」证明**：同模型同工况同初值，一次含该族、一次移除该族，
   两次 `run_contract` 都 `status == "success"`、`body_state` 有限，且**受力体末态角速度差
   大于明确容差**——只证明「不报错」不算，必须证明该族**被实际求值**。反力体响应同时核对。
4. **不改任何既有族的输入形式**，既有族的文档往返逐项不变。

## 写范围（允许改的路径）

- `packages/suspension_kernel/cpp/include/mb_cases/functions.hpp`
- `packages/suspension_kernel/cpp/src/cases/contract_model.cpp`
- `packages/suspension_kernel/cpp/src/abi/kernel_contract_run.cpp`
- `packages/suspension_kernel/cpp/src/contract/contract_registry.cpp`
- `packages/suspension_kernel/cpp/tests/contract_selftest.cpp`（`contract_registry_size(1)` 的计数断言 10 → 11）
- `packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json`（`element.type` enum）
- 新增测试 `packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`
- 本目录 `raw/`（证据）、临时脚本或会话 scratch

## 禁止触碰

- `mb_config/version.hpp`、`mb_input/types.hpp` 的版本常量与 `kernel/native.py`（ABI 单点提交归 p2-02；
  本行**不需要**改 ABI，若实测确需改则停止并回报）
- `cpp/src/assembly/build_model.cpp` 的互斥守卫（放宽会改变所有 C ABI 调用者的混用语义）
- `cpp/src/element/anti_roll.cpp` 的力矩求值本体（缺口见「已知边界」）
- `packages/suspension_multibody/src/suspension_multibody/cases/vehicle_dynamic.py`
  （把车辆力矩元写进文档归 p2-05）
- 任何冻结基线：`tests/data/kc_baseline/**`、`tests/data/**/sha256.json`、
  `dynamic_hash_baseline.json`、`kc_perf_baseline_native.json`——**禁止重录**
- `EPIC.md`、`SUBTASKS.csv`、父 `PROGRESS.md`，以及其它子任务目录
- `raw/` 中不得放未执行的内容

## 依赖与时机

- `depends_on = p2-03`（力矩元已进 `element_blocks.py`、槽位已定）
- `p2-05` 依赖本行（它删除预采样后必须走文档路径才能把力矩元送进内核）
- 前置 S1（阶段一 01–07 全 `DONE`）已满足

## 判据与证据落点

1. **(a) 拒绝复现（改造前）** → `raw/document_route_refusal.md`
   - 跑：最小两体模型文档（一转动副 + 一个 `rotational_torque` 元素）经 `run_contract`
   - 看：改造前的拒绝原文与退出路径（`file:line`）
2. **(b) 文档解析落地** → `raw/document_route_implementation.md`
   - 看：`ContractModel::read` 新增分支与其保存的块；`kernel_contract_run.cpp` 追加点；
     `kElements[]` 与 schema enum 的同步行；**未改 ABI** 的自证（`git diff --stat` 不含
     `mb_config/version.hpp`、`mb_input/types.hpp`、`kernel/native.py`）
3. **(c) 两次运行对照** → pytest 输出 + `raw/document_route_evidence.md`
   - 看：含族与移除族两次 `run_contract` 的 `status`、`body_state` 有限性、
     受力体末态角速度实测值与差、容差取值与理由、反力体响应
4. **(d) 既有族往返不变** → `raw/existing_family_regression.md`
   - 看：bushing / spring / aerodynamic_drag 等既有族的文档往返结果与改造前逐项一致

## 已知边界（不在本行范围）

`cpp/src/element/anti_roll.cpp:128` 把驾驶员需求硬编码为 `1.0`、且不读轮胎滑移。
本行只证明**该族经文档可达且影响轨迹**；「按实时打滑状态与驾驶员信号求值」是 G1 的另一层，
归 p2-05 结项前处置。本行不得声称已解决那一层。

## Done-When

- [ ] 最小文档含 `rotational_torque` 时 `validate_model` 通过且 `run_contract` `status == "success"`
- [ ] 移除该族重跑仍 `success`，两次受力体末态角速度差 > 容差（证明被求值）
- [ ] `contract_registry_size(1) == 11`，`contract_selftest` 通过
- [ ] 未改 ABI 版本常量、未动 `build_model.cpp` 守卫、未重录任何基线
- [ ] 既有族文档往返逐项不变

## Final Validation Command

```bash
uv run --no-sync pytest packages/suspension_multibody/tests/cases/test_rotational_torque_document.py -q -p no:cacheprovider && uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider && uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
