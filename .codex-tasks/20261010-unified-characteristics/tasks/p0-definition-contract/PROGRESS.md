# P0 实施进度

## 恢复入口

- 日期：2026-10-13（Asia/Shanghai）。
- 任务：冻结核心四组合、指标/目标力律、边界及冻结协议；让特性请求穿过唯一编译/native reader/结果契约，未知内容逐层拒绝。
- 形态：single-full；状态：**DONE**；产品实施已完成并行为验收。
- 进度：**6/6**；步骤 1–4 于 2026-10-10 DONE；步骤 5/6 于 2026-10-12 重开，
  2026-10-13 补齐后重新收口（retry_count=1）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。

## 重开与收口（2026-10-12 → 2026-10-13）

**重开原因**：native 只读取 `profile`/`version`，`held`/`released` 与
`wheel_end_response` 未被完整消费——「接受请求但不消费」正是契约禁止的空壳形态。

**本轮补齐**（详见 [raw/validation.md](raw/validation.md) 第 9 节）：

- `cpp/include/mb_cases/functions.hpp`：`ContractCharacteristics` 新增
  `boundary_released` / `boundary_held` 与 `has_wheel_end_response` / `wheel_end_response`。
- `cpp/src/cases/case_characteristics.cpp`：新增 `read_boundary_names(...)`，拒绝
  非字符串/空串/列表内重复，并**拒绝同时出现在 `held` 与 `released` 的名字**；
  `wheel_end_response` 由「只校验对象」改为**原样存回**。
- `cpp/src/abi/kernel_contract_run.cpp`：请求存在时把解码后的请求**回显到结果 manifest
  的 `characteristics` 字段**（兑现头文件原有承诺）。
- 测试 `packages/suspension_kernel/tests/test_characteristics_request.py`：
  **15 → 25 passed**。
- **反证（判别力）**：把 held/released 冲突检查置 `if (false)` → 重建 → 测试
  **FAILED**；还原后 25 passed。证明该测试能识别接线断开，而非只断言部署。

## 2026-10-13 的 Adams 审计修订

原 `raw/adams-rule-audit.csv` / `.md` 的前提「本机没有安装 Adams Car 2024.1」**已被实测证伪**，
且原版存在**版本一致性缺陷**（版本列写 `Adams Car 2024.1`，`source_page` 却是
`G:\MSC.Software\Adams\2025_1_1\...`）。经用户裁定（2026-10-13），按 P5 权威 2024.1 审计重写：

- 实测：`C:\Program Files\MSC.Software\Adams\2024_1\` 存在；
  `probe_profile('adams-car-2024.1')` → `available=True` / `version="2024.1"` /
  `license_probe="passed"`；`validate-adams --smoke --require-installed` **exit 0**。
  原版失败根因是**路径核对范围不足**（漏掉 `C:\Program Files\MSC.Software\Adams\2024_1`）。
- 修订结论：**G6 由 `pending_definition` 升为 `stated`**；**G3 改为 `partial`**
  （`adams_solver/gtcmat.html` 明确给出 C++ Solver 动态柔度算法，旧「动态段无完整算法」
  对 Solver 手册不成立）；G1/G2/G5 为 `partial`；G4 为 `not_stated`。
- `source_sha256` **非手写**：由写入脚本
  `backups/20261013-p0-adams-2024-1-audit/write_p0_audit.py` 在写入时对本机 2024.1 help
  实时计算（页面缺失即中止）。实测复核与 P5 权威版一致（`svc_d_5.html`→`0915d926…`、
  `gtcmat.html`→`e9308542…`、`example_o_wheelenv.html`→`f5f0596d…`）。
- **口径未放宽**：Adams 兼容的完整动态冻结等价声称仍被阻断；未知项不得默认使用本项目规则。
  P5 的 `raw/adams-rule-audit.csv` 为逐项对标真源；P0 版是可自足的阻断清单。
- 原版保留于 `backups/20261013-p0-adams-2024-1-audit/`（未删除）。

## 门禁（2026-10-13 实际执行）

| 门 | 结果 |
|---|---|
| build-kernel / build-axle-native | exit 0 |
| `test_characteristics_request.py` | **25 passed** |
| kernel 套件 | **129 passed** |
| contracts 套件 | **78 passed** |
| `test_no_legacy_vehicle_models.py` | **5 passed** |
| 快速集 | **2051 passed, 1 xfailed**（skip 0，xfail 未增长） |
| characteristics 定向 | **489 passed** |
| ruff / ty | `All checks passed!`（exit 0） |
| 三条架构门 | exit 0（findings 0 / 0 cycles / 3 checks） |
| `dynamic_hash_sentinel.py --check` | exit 0（逐位冻结不变） |
| `case_parity_check.py` | exit 0（8 families） |
| `kc_perf_gate.py` | **FAIL**（高负载环境抖动，非本次回归；见下） |
| `git diff --check` | exit 0（仅 LF→CRLF 提示） |

**`kc_perf_gate.py` 判定**：两次执行均 FAIL（k-100 ×1.288→×1.467、c-66 ×1.275→×1.427），
但判定为「高负载下的负载敏感抖动 + 该路径不经本次改动」，依据两条独立证据：
①**比值自身在抖动**（基线固定），门禁脚本自述「安静机器 ×0.92、繁忙桌面 ×1.5」；
②实测 perf 的两个用例经 `validate()` 后**不含 `characteristics` 键**，本次新增分支根本不执行。
**不重录基线、不放宽 `BUDGET_FACTOR`**，如实记为环境抖动而非回归，也不写成 `PASS`。
未重录任何数值/性能/架构基线；`layering_baseline.json` 未再改动。

## 阻断项（不影响 P0 DONE）

- **独立参考数据缺失**：`raw/reference-manifest.json` 保持 `pending_reference`。
  按 EPIC §36 与 SPEC §41–42 **不阻断 P0 DONE**，但**严格阻断 P5 与 P6 的 Adams 兼容发布**。
- **native 其余路径仍宽松**：`contract_read_characteristics` 之外的 case 文档字段仍走旧
  `JsonValue::find` 宽松路径（P0 既定范围，非遗漏）。
- **指标数值实现未做**：属 P1–P4 范围。

## 证据与同步规则

实施在 raw/ 保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不 DONE；
同类 3 次失败暂停重评，保留 FAILED/retry_count。每步同步本恢复块；子任务完成后回填父
SUBTASKS，completed_at 记实际时间。
