# PROGRESS：p2-10 内核 normalized torque demand 独立缓冲与旧 N·m 路由隔离

- 父 Epic：`20260929-multibody-evolution-p2-p5`
- 形态：`single-full`
- 状态：**DONE**
- 来源：`code-reviewer` 裁决新增（delegation `5e01b75d`）

## 恢复块

1. `任务:` 内核需求通道改走独立缓冲，旧 N·m 路由与它彻底隔离
2. `形态:` single-full
3. `进度:` 7/7 步骤 DONE
4. `当前:` 已关闭。`SampleInput`/`Model`/`ContractCase` 各有一套独立 demand 存储；两条力律改读新缓冲；
   `vehicle_dynamic` 家族接上两个归一化 role 并按名拒绝混用与越界
5. `文件:` `packages/suspension_kernel/cpp/{include,src}/**` 共 6 处 + 探针与 `tests/test_rotational_torque.py`
   + 本目录 `raw/demand_plumbing.md`
6. `下一步:` 无。p2-09 已把本行登记进自己的 `depends_on`

## 交付

见 `raw/demand_plumbing.md`：缺口实测证据、改动清单逐文件、门禁逐项、两条新断言的逐字读数、
文档路由端到端四场景、诚实登记（未做/未声称）。

**核心结果**（实跑）：

- `packages/suspension_kernel/tests` → **47 passed**（起点 45；既有 45 条未删未弱化）
- 采样层隔离读数：`isolate none torque 15 brake_torque 35 wheel_demand 0 brake_demand 0` 与
  `isolate both torque 15 brake_torque 35 wheel_demand 0.5 brake_demand 0.75`
- 需求通道不读旧表：`eval decoupled tau_a 400 tau_b -400`（读旧表会得 −1000）
- 文档路由：`brake_torque` → `brake_pressure` 改名提交 **success**；同通道混用与越界均按名拒绝
- `dynamic_hash_sentinel --check` → 逐字节一致（`fdfd5a6b…eebc9`）；`tests/data` 未被写
- ABI 常量 **未动**（仍 17/32/1）

## 边界

未改任何 Python 生产代码（装配侧归 p2-09）；未改 ABI 结构与版本常量；未改 `drive_brake.cpp` 的旧施加逻辑；
未改 `axle_dynamic` 家族 role 表；未重录基线；未新增 skip/xfail。
