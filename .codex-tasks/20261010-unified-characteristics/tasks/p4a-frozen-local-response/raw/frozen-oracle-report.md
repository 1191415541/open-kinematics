# P4a 步骤 5 证据：独立有限扰动 oracle 与直接/转置核对

> 验收条件（TODO.csv 行 5）：**同冻结 oracle 步长序列、非对称/单位/frame 等价，原状态无修改**。
> 证据落点：本文件（SPEC.md §51）。命令通过/文件存在只作门禁，不作为 DONE 依据；
> 下面记录实际行为断言、退出码、误差量级与缺口。

## 1. 结论

步骤 5 **完成并通过行为验收**。交付物是**独立 oracle 自测**
`cpp/tests/frozen_oracle_selftest.cpp`（P4a 唯一一个不与算子本身同源的自测）
+ 其注册（`CMakeLists.txt` 目标、`tests/test_registry_consistency.py` 门禁）。

**本步交付的是"证明前三步的算子没有联合出错"，不是新算子。**
步骤 1–3 各自对照"由构造已知的答案"验证冻结协议、坐标驱动切线与载荷列；
它们的共同盲区是**三者可能互相自洽却联合错误**（切线的整体尺度、载荷列的符号约定、
装配漏掉预载项）。本步用一条**不信任解析切线**的路径补上这个盲区：
perturb 冻结状态、重新求值 Newton 迭代真正使用的 `static_residual`（一个**与被测对象无关**
的计算），要求解析切线 `A dx` 预测实际的残量变化，且误差随步长**按二阶速率下降**。

## 2. 交付物

| 文件 | 性质 | 说明 |
|---|---|---|
| `cpp/tests/frozen_oracle_selftest.cpp` | **新增** | 6 个测试（本步在既有 5 个之上新增测试 4b） |
| `CMakeLists.txt` | 改 | `mb_frozen_oracle_selftest` 目标（链接 `static_residual` 的完整力链） |
| `tests/test_registry_consistency.py` | 改 | 注册并**解析**自测输出（非 grep） |

## 3. 本步新增覆盖（测试 4b）与两个复审缺陷

### 3.1 覆盖缺口（本步最重要的产出）

交付时 oracle 已有 5 个测试，其中测试 3（非对称转置）与测试 4（伴随）用的是
**手工构造的矩阵**。这不满足 SPEC §51 / 设计 §439 的原文要求——
"**同一物理问题中** ... 直接模式响应、转置求解和矩阵乘模式的数值等价，**含非对称切线**"。

探针实测（`probe_sym`）：**oracle 的物理夹具（`rows == 0`）装配切线完全对称**，
`worst|A - A^T| = 0.000000e+00`（scale 4.24e3）——转置核对在其上**真空**。

于是新增**测试 4b** 在**真实非对称冻结物理切线**上核对，覆盖 SPEC §51 的三项，
并**判别 `A` 与其转置**：

1. **非对称**：夹具装配出非对称切线，并断言其确实非对称（否则路由验收是空转）。
2. **直接/转置等价**：`A X = B_w P`（直接）与 `A^T Z = G^T`（伴随）给出同一物理测量 `Y`，
   且断言**前向解与转置解确实不同**（否则等价是对称矩阵的巧合）。
3. **单位等价**：坐标单位对角缩放 `x = S x'` 下 `A' = A S`、`G' = G S`，
   物理量 `Y = G X` 必须不变（设计 §342"尺度变换还原后才输出"），并配**未补偿 `G` 的反证**。

### 3.2 独立复审发现的两处缺陷（均已修）

交付后经**独立只读复审**（`code-reviewer`），发现两处**真实缺陷**：

**缺陷 1（夹具不可达，已修）**：初版夹具用**非对称刚度的衬套**制造切线非对称，
但两个生产模型入口**都拒绝非对称 bushing 刚度**——
`build_model.cpp:326` 与 `element_reader.cpp:311` 均要求 `bushing matrices must be symmetric`。
故该夹具会在**任何受支持模型都无法产生**的矩阵上检验装配器，等于没验证物理。
**已改**为**受支持**的非对称来源：**对称衬套 + 一条 `AXLE_INPLANE` 约束行**
（`AXLE_INPLANE` 恰好贡献 1 行，故 `dim = ndof + rows = 7`；约束行的 KKT 块
`A[pose][mult] = -J`、`A[mult][pose] = +J` 天然非对称，与真实受载连杆一致）。
实测：`worst|A - A^T| = 1.44`（scale 6.0e3）、可分解。

**缺陷 2（切线只与自身比较，已修）**：初版只比较**同一装配矩阵**上的两条代数路线；
若切线导数被误写成 `K^T` 而残量仍用 `K`，所有断言都会通过。
**已加**"切线必须是**残量**的导数"判别块：要求 `A dx` 预测 `static_residual` 的变化，
并要求 `A^T dx` 在该同一检查上**失败**。实测：

| 步长 | `|A dx - dR|` | `|A^T dx - dR|` |
|---|---|---|
| h=1e-4 | **1.809e-13** | 1.011e-03 |
| h=1e-5 | **3.314e-14** | 1.011e-04 |

判别力约 **1e10 倍**，足以抓住"切线写成了另一个系统的导数"。

**缺陷 2 的连带发现（测量必须触及乘子坐标）**：修缺陷 2 的过程中，突变
"伴随路由误用前向"**再次 SURVIVED**。探针实测根因是**结构性的**：
当右端只有位姿分量、测量也只读位姿坐标时，`A^{-1}` 的位姿块是**对称**的
（对称因子的 Schur 补），故 `G^T A^{-1} b == (A^{-T} G)^T b` **恒等**，
两条路线**必然**给出同一个数。**已改**为测量 `G` 同时读取**乘子（约束反力）坐标**——
一个真实的物理输出，也是 `A` 的 KKT 块**唯一非对称**之处。实测修正后：
正确伴随与直接一致 **1.110e-16**，而误用前向差 **1.34**（突变随即被 CAUGHT）。

## 4. 修正后测试 4b 的实测断言

| 断言 | 实测 |
|---|---|
| 夹具切线真非对称 | `worst|A - A^T| = 1.44`（scale 6.0e3） |
| 切线可分解 | `factorize_numeric` 成功 |
| `A` 预测残量变化 | `|A dx - dR| = 1.809e-13` / `3.314e-14` |
| `A^T` 不预测（判别） | `|A^T dx - dR| = 1.011e-03` / `1.011e-04` |
| 直接/转置一致 | `-6.722891983528e-01` vs 同值，**diff 1.110e-16** |
| 前向/转置解确实不同 | 1.49 |
| 单位缩放等价 | 相对 **0.000e+00** |
| 未补偿 `G` 反证（断言非空转） | 相对 1.448 |

## 5. 自设破坏（mutation testing）

命令：`uv run --no-sync python <scratch>/sabotage_dt.py`
结果：**7 条中 6 条 CAUGHT、0 编译失败、0 跳过**，控制组（无操作编辑）通过。

| # | 突变 | 结果 |
|---|---|---|
| 1 | 伴随路由改用前向求解 | **CAUGHT**（缺陷 2 连带发现修复后） |
| 2 | 残量判别改用 `A^T` 预测 | CAUGHT |
| 3 | 约束法向置零（抹掉夹具非对称） | CAUGHT |
| 4 | 前向/转置差异阈值放宽到 0 | **SURVIVED（预期，见下）** |
| 5 | 载荷列被替换为全零 | CAUGHT |
| 6 | 单位缩放丢掉补偿的 `G' = G S` | CAUGHT |
| 7 | 单位反证被中和 | CAUGHT |

**突变 4 的 SURVIVED 是预期的、不是缺陷**：它改的是**测试自身的阈值**，
不改产品行为；真实的前向/转置差异（1.49）远高于原阈值 1e-6 与放宽后的 0，
故断言在两种阈值下都通过。作为等价变异登记，不假装能抓住。

### 5.1 oracle 自身的分辨率（实测）

敏感性扫描（`tmp_oracle_sensitivity.py`）：

| 注入的加性误差 dA | 二阶速率比 r1=e1/e2 | 判定（阈值 50） |
|---|---|---|
| 0（基线） | 100.0 | — |
| 1e-3 | ~102 | survived |
| 1e-2 | ~128 | survived |
| 1e-1 | 53.5 | survived（仅超阈值 7%） |
| 5e-1 | 5.8 | **CAUGHT** |
| 1.0 | 9.6 | **CAUGHT** |
| 5e1 | 9.9 | **CAUGHT** |

即该 oracle 的**分辨率下界约为 1e-4 相对切线误差**：更小的加性误差被**真实的**
`O(h^2)` 截断项淹没。**这是有界 oracle，边界如实记录**，不是"切线精确到机器精度"。

## 6. 门禁与退出码

| 命令 | 结果 |
|---|---|
| `mb_frozen_oracle_selftest.exe`（6 测试） | `OK: frozen oracle self-test passed`，exit 0 |
| `mb_linear_batch_selftest.exe` / `mb_frozen_tangent_selftest.exe` / `mb_frozen_load_selftest.exe` | 各 exit 0 |
| `just build-kernel` / `just build-axle-native` | exit 0 |
| `pytest packages/suspension_kernel/tests/test_registry_consistency.py` | **16 passed**，exit 0 |
| `just check-fast` | **2004 passed / 1 xfailed**（skip 0，xfail 未增长）；kernel 116 / contracts 64 passed |
| `just gate-architecture` | 0 环、0 新边、`layering_baseline.json` 未改 |
| `just gate-numeric` | sentinel **逐位不变** exit 0；parity **8 族**；perf k-100 ×1.049 / c-66 ×1.088，预算内 |
| `ruff check .` / `ty check .` | All checks passed |
| `git diff --check` | 干净 |

### 6.1 sentinel 判读

`dynamic_hash_sentinel` 输出中的 `combined_load` 等 `FAILED` 行与
`solver self-convergence: FAILED`、`adams accuracy: BLOCKED`，
是 **Adams 2024.1 未安装导致的既有报告**（本机仅 `G:\MSC.Software\Adams\2025_1_1`），
与本次改动无关；失败清单与冻结基线 `failed_cases` **逐项相同**，
判据是末行 `OK` 与 **exit 0**（逐位一致）。**本步未重录任何冻结基线**。

## 7. 一致性检查与诚实局限

- **新增分层边 0**（改动全在 `cpp/tests/` 与 Python 测试，未触内核分层）；
  `layering_baseline.json` 未变；模块循环 0。
- **诚实局限 1**：本步的物理夹具是"受支持的非对称冻结系统（衬套 + 单约束行）"，
  不是整车；它验证"真实物理切线上直接/转置/单位等价 + 切线确为残量导数"这一**结构性质**，
  真实构型覆盖由步骤 6 的 `case_parity_check.py` 8 族承担。
- **诚实局限 2**：oracle **有界**（§5.1），加性误差 ≲1e-1 藏于截断项下，是**分辨率**非漏检。
- **诚实局限 3**：本步不交付 ABI 导出、`characteristics_response_kinds` 声明、
  `local_response_v1` 结果通道、具名轮端→`body_wrench` 映射层、frame 换算
  ——均仍为登记缺口（见 PROGRESS.md「遗留缺口」）。

## 8. 清理

| 文件 | 处置 |
|---|---|
| `$PI_SCRATCH_DIR/probe_*.{cpp,exe}`（sym/kkt/dbg/asym/dt/units/supported/sup_full） | 探针，用后回收 |
| `$PI_SCRATCH_DIR/sabotage_dt.py` | 突变脚本，回收 |
| `packages/suspension_kernel/cpp/tests/probe_minimal.cpp` | **上一轮遗留的调试探针**（无引用、未注册进 CMake）→ 移出工作区 |
| `backups/*.p4a-step5.bak` | **保留**（改动前备份） |
