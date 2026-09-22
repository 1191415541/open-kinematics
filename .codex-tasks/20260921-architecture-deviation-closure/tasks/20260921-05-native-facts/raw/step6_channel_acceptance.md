# 05 步骤 6：逐通道容差验收与凝聚实体对照

本步骤按父级 `VALIDATION.md`（01 冻结）的容差与门禁，对 native 元件力旋量通道做逐通道验收，并复核凝聚后实体身份。**验收结论：native 通道未通过等价性验收，据此阻断切换**（与步骤 7 的差异登记互为证据）。

## 1. 验收照用的冻结容差与门禁（原文引用）

| 项目 | 冻结口径（`VALIDATION.md`） | 本步执行结果 |
|---|---|---|
| K/C 平移字段 | `0.1 + 0.002*abs(reference)` mm（§9.2） | `kc_parity_check.py --check` 退出 0（`OK: candidate matches the frozen K/C snapshot within tolerance`） |
| K/C 转角字段 | `0.02 + 0.005*abs(reference)` deg（§9.2） | 同上 |
| C 形变比较 | 平移 `1e-6 + 1e-4*abs(ref)`、转动 `1e-8 + 1e-4*abs(ref)`（§9.2） | 同上 |
| 动态数组逐位门 | `arrays.npz` bytes SHA-256，26 个 artifact（§9.1） | `dynamic_hash_sentinel.py --check` 退出 0，26/26 逐位一致，组合哈希 `e7407656…8d48e` 未变 |
| 八 family parity | 全 family 接受（§9.3） | `case_parity_check.py` 退出 0（`OK: 8 families accepted`） |
| ABI 七符号 | 七符号在、`axle=15 / vehicle=30 / core=1`、无新增（§4） | 未新增导出；步骤 3 已实测 15/30/1/1；本步未触碰 ABI |
| 契约版本兼容 | 允许 1→2（05 SPEC 约束） | 开关关闭 `contract_version=1`、打开 `=2`（步骤 5 实测） |
| 通道级容差 | **未冻结逐通道等价容差**（EPIC 修订记录明确：Python 报告来源切换单独使用冻结的通道级容差验收，而该容差在 01 未冻结） | 见第 2 节：本步用冻结的 K/C 容差作为**下界**对照，差异量级远超该下界 |

## 2. 逐通道验收结果（元件力旋量通道）

对照方式与数据见 `raw/step7_law_difference_registration.md` 第 1-2 节（同一状态、单位折算、**力矩参考点对齐**、可复现探针）。逐通道判定：

| 通道 / 元件族 | 通道存在性 | 与 Python 事实的等价性 | 判定 |
|---|---|---|---|
| spring（type 1） | 通道支持（2 行/元素） | K 模式下模型不声明力元件（`cases/kc_quasi_static/contract.py:118-120`），C 模式下本探针模型无弹簧 | **未验收通过**（无可比事实） |
| bushing（type 2） | 通道存在，C 模式 32 行/样本 | **对齐参考点后**力差 `7.6e-10 N`、力矩差 `2.2e-10 N·m`（力律等价）；但固定体端 native 全 NaN 而 Python 给出有限值（结构性缺失） | **不通过**（结构性缺失，非数值差异） |
| anti_roll / bump stop / steering / drive_brake（type 3/4/5） | 通道支持 | 本探针模型不含这些元件，未产出可比行 | **未验收通过**（无可比事实） |
| tire（type 6） | 通道支持（1 行/元素） | K/C 的 `kc_quasi_static` 模型未装配垂向轮胎元件（`cases/kc_quasi_static/contract.py:118-120` 仅发 bushing） | **未验收通过**（无可比事实） |
| external / gravity（type 7） | 通道存在，K/C 各 10 行/样本 | K 模式力与力矩恰为 `0.0`（无外力）；两侧均无实体载荷可比 | **未验收通过**（无可比事实；K 模式无实体元件事实） |

**结论**：五类中无一类通过等价性验收——一类显式不通过（bushing，原因为固定体端结构性缺失），其余四类因该通道在 `kc_quasi_static` 下无独立可比事实而**不能宣称通过**（不得以「无差异」代替「已验收」）。另：bushing 的力律本身经参考点对齐后**已证等价**（差 1e-10 量级），不通过的原因只剩结构性缺失一条，见 `raw/step7_law_difference_registration.md` 第 2 节。

## 3. 凝聚后实体身份对照

- 凝聚的等价性契约与 body ID→凝聚体 ID 映射登记在步骤 2（`raw/step2_condensation_equivalence.md`，含 2 个新测试），本步沿用，未改动。
- native 通道的身份列按 **body 索引**输出（列 10/11/12），而 Python 报告侧按 **body 名**（`ComponentLoad.endpoint`）；映射登记见 `VALIDATION.md:298`（`VehicleAssembly.body_aliases` → `preparation/vehicle_dynamic.py:338,405-411`）。本步实测确认：native 未输出别名身份映射（manifest 只带 body names，`kernel_contract_run.cpp:813`），因此**凝聚后实体身份**在 native 通路中仍需报告侧解析，与步骤 1 对照表的判定一致。
- 因整体切换已被阻断（第 2 节），凝聚实体的 native 通路对照**不构成已通过项**；**A3 之后**凝聚等价性转为「生产路径不凝聚 + weld 送 native fixed」的实测等价（世界系质量性质，见 `a3_condensation_switchover.md` §4），本步当时的判定已被 A3 取代，不再是未闭合项。

## 4. 门禁实测（本步全集）

| 命令 | 退出码 | 证据 |
|---|---|---|
| `build_axle_native.py` | 0 | `raw/step5_build.log` |
| `dynamic_hash_sentinel.py --check` | 0（26/26 逐位一致） | `raw/step5_dynamic_hash.log` |
| `kc_parity_check.py --check` | 0 | `raw/step5_kc_parity.log` |
| `case_parity_check.py` | 0（8 families） | `raw/step5_case_parity.log` |
| kernel tests | 0（15 passed） | `raw/step5_kernel_tests.log` |
| contracts tests | 0（22 passed） | `raw/step5_contracts_tests.log` |
| results+physics+vehicle+cases+architecture | 0（263 passed, 1 xfailed） | `raw/step5_suite.log` |
| `ruff check .` / `ty check .` | 0 / 0 | `raw/step5_ruff.log`、`raw/step5_ty.log` |

**未改任何容差**：本步当时 `dynamic_hash_baseline.json`、`kc_baseline/`、`kc_perf_baseline*.json` 与 `vehicle_dynamics_baseline/` 均未修改（`git status` 无这些文件）。**A3 之后**（2026-09-22 用户裁决）`vehicle_dynamics_baseline/sha256.json` 经明确授权重录（8 个 case，按不凝聚路径的 native 结果）；其余三项仍未改动。

## 5. 本步骤结论

1. 门禁全集通过（第 4 节），**默认路径未被本子任务影响**。
2. 元件力旋量通道**未通过逐通道等价性验收**（第 2 节），因此步骤 5 的解码面只能作为**显式启用的只读事实面**，不得接管 `api.py` 的生产取值来源。
3. 该结论与步骤 7 的差异登记一致，二者共同构成对 `elements/` 删除的阻断证据（EPIC G3 / A1 修订）。
