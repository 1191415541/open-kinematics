- 任务：执行独立终局验收并逐条核对 G1-G8
- 形态：single-full（Epic 子任务）
- 进度：0/11 步骤 TODO，尚未实施
- 当前：未开工。前置为全部 01-11 完成。
- 文件：`.codex-tasks/20260922-suspension-template-architecture/tasks/20260922-12-acceptance/`
- 验证：未运行。

## 恢复信息

**本轮交付为规划，未写任何生产代码。** 开工前必须核验：

- 01-11 全部 DONE，且各自的门禁与验收通过。
- 01 的 `raw/baseline_values.md` 可用（既有失败对照的底线）。
- 各子任务的 PROGRESS 中**基线重录台账**已填写（本步要核对它）。

## 本任务的性质

本步是 EPIC 的**终局门禁**，只读生产代码与既有交付物，不修代码、不改测试、不重录基线。判定口径：

- **`DONE` 行不等于 Goal 达成**。所有子任务 DONE 之后，仍需独立执行的终局判据是**端到端四件事 (a)–(d)**。
- 每条 Goal 结论必须指向独立证据（命令退出码或文件），不得以子任务自证代替。
- 既有失败与 01 基线对照；新增失败即阻断完成。

## 端到端四件事（本 Epic 的 Goal 判据，来自 EPIC Done-When）

```text
(a) 同一个轴的模板，K 模式跑一次、C 模式跑一次 → 只差激活列
(b) 同一个轴模板 + 同一属性文件，准静态 study 跑一次、动态 study 跑一次
(c) 换一份属性文件重跑 (b)，刚度/阻尼按属性变化而模板与几何不变
(d) 总成 × 试验台矩阵任取两个组合跑通，其中一个此前不存在；
    并用最小单位输出 + 自定义表达式算出一个新指标
```

## 需要核对的基线重录范围（来自各子任务的预期）

| 子任务 | 预期重录的基线 | 预期不动的基线 |
|---|---|---|
| 05 | `kc_baseline/`（C 模式部分） | `kc_baseline/k_states.json`、`dynamic_hash_baseline.json` |
| 06 | 若属性替代模板默认值则涉及 C 模式 | — |
| 08 | 视轮胎质量改动而定 | `layering_baseline.json`（C++ 模块不动） |
| 09 | `kc_baseline/`、`kc_perf_baseline*.json` | `dynamic_hash_baseline.json`、`axle_dynamics_baseline/`、`vehicle_dynamics_baseline/` |
| 10 | 预期不重录 | 全部（若变须单独裁决） |

核对要求：每个重录都有「文件 + 导致重录的步骤 + 重录前后值 + 等价性判定依据」四项；**授权重录不等于免登记**。

## 三个易犯错误（自检项）

1. **以 DONE 代替达成**：端到端四件事必须实际跑，不得以"子任务已测过"推断。
2. **把 skip 减少当成进展**：本机 `artifacts/` 下 Adams 参考工件就绪会让 46 项 skip 转为实跑（实测：01 基线为 47 skipped，主代理收口时为 1 skipped），这是环境差异，不是本 Epic 的成果。
3. **把重录当验收通过**：若某个门是靠重录基线才变绿，必须额外提供等价性论证，否则对应 Goal 不判达成。

## 主代理收口时实测的全量套件基线（供对照）

`uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q` → `783 passed, 1 skipped, 1 xfailed`（退出 0）。
其余：`--strict --final` 0、kernel 15、contracts 22、动态哈希 26/26 逐位一致、K/C parity 0、8 family accepted、ruff/ty 0、两包 build 0、`git diff --check` 0。

## 下一步

等 01-11 全部完成后，从 `TODO.csv` 第 1 行开始。父 `SUBTASKS.csv` 第 12 行状态由主代理回填。
