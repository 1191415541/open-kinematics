# p5-06 独立复核的处置（code-reviewer `8ab15196`）

复核裁定：**「9 条满足 + (i) 未完全满足」偏宽，不能支撑 Epic 完全关闭。**
本节逐条记录复核发现、主代理的独立实证、以及处置。

---

## 复核发现 1：(a) 应判**不满足** —— 主代理 AST 实证**成立**

**复核意见**：`vehicle_dynamic.py:342` 无条件调用旧 helper，opt-in 路径调用次数不为 0；
`done_when_a_to_j.md:13` 把事后丢弃判成「声明分支隔离」，放宽了冻结判据。

**主代理独立实证（AST 扫描 + 全仓 grep）**：

```
call sites of _build_wheel_torque_signals:
  (342, ['prepare_vehicle_run'])          # 唯一调用点，无任何 enclosing 条件分支
comparisons mentioning torque_demand in vehicle_dynamic.py: (none)
grep -c "declared_demand" vehicle_dynamic.py: 0
```

- 该 helper 在 `prepare_vehicle_run` 内**无条件**调用（`:342`），随后 `:352-356`
  对有 demand 信号的轮做 `pop`。
- 全仓**不存在** `torque_demand == "none"` 的比较，`declared_demand` 这个变量**从未在任何
  提交中出现**（`git log --all -S "declared_demand"` 为空）。

**证据不符（严重）**：`tasks/p2-05-retire-signals/raw/retirement_grep.md` 的 §1 以
「`vehicle_dynamic.py:298-315` 改后原文（要点）」为标题，引用了一段：

```
declared_demand = getattr(model.driveline, "torque_demand", "none")
if declared_demand == "none":
    wheel_torque, brake_torque = _build_wheel_torque_signals(
```

**这段代码在仓库任何提交、任何文件中都不存在**。该行号（`:298-315`）在当前文件里是别的
内容。即：p2-05 的证据文件**把「调用隔离」写成了一个从未实现的分支形态**。

**实际成立的隔离**只有**数据流侧**：`pop` 使旧表对有 demand 信号的轮为空表，不进内核。
运行时计数（p2-05 §1 表格：opt-in 三档旧表行数 0）可信，但那是 **pop 的效果**，
不是**调用隔离**。

**处置**：Done-When (a) 的判据原文是「`_build_wheel_torque_signals` 只在 none 分支被调用、
**opt-in 路径调用次数为 0**」。字面看，当前实现**不满足**「调用次数为 0」。
故 (a) **改判为不满足**（判据未达成，实现与判据不一致），并在
`done_when_a_to_j.md` 与 `acceptance.md` 中更正；p2-05 证据文件中那段不存在的代码必须
更正为实际形态。

## 复核发现 2：(f) 应判**未证实** —— 复核意见成立

**复核意见**：`done_when_a_to_j.md:83` 比对的是 Python 实现模块 `subsystems/wheel.py`，
不是单轴与整车**实际读取的** wheel 子系统文档；同一实现模块可以消费不同模板。
复核引 `wheel.py:57` 的 `_requested()` 证明模板由 `context.request.wheel_template` 决定。

**主代理实证**：`subsystems/wheel.py:57-67` 确实有
`_requested(context)` → `getattr(context.request, "wheel_template", None)`。
判据原文（`EPIC.md:93` G6）要求「比对**读到的文件路径与内容指纹**」。
我此前用「全仓只有一个 `role = "wheel"` 的实现文件」作结构性论证——这证明不了
**两侧读到的是同一份 template/subsystem 文档**，因为同一实现可以实例化不同模板。

**处置**：(f) **改判为未证实**（证据对象与判据要求不符）。

## 复核发现 3：(h) 两处证据不足 —— 复核意见成立

**(3a) 总线写入未验证「改变同次仿真轨迹」**：复核指出
`tests/api/test_signal_bus.py:128/143` 只断言字典被改，没有写入后**再求解**的轨迹对照。

**主代理实证**：`grep -c "run_request\|simulate\|run_case" tests/api/test_signal_bus.py` = **0**。
该文件 13 个用例全是「读结果文档的数字」「写文档返回新对象」这类断言，**没有一次求解**。
故 `done_when_a_to_j.md:108` 所称「前后两次运行轨迹对照」**不被该测试支持**。

**(3b) ABS 收敛断言与陈述相反**：复核指出
`test_abs_closed_loop.py:327` 目标递增，却在 `:337` 要求尾段滑移**递减**

**主代理实测**（带 `SUSPENSION_KERNEL_CONTROLLER_OUTPUT=1` 重跑该夹具）：

```
target=0.002  |slip|_tail_mean=1.328477e-02   err_vs_target=1.128e-02
target=0.006  |slip|_tail_mean=1.241716e-02   err_vs_target=6.417e-03
target=0.010  |slip|_tail_mean=1.148207e-02   err_vs_target=1.482e-03
```

目标从 0.002 升到 0.010，尾段 |slip| 从 1.328e-02 **降到** 1.148e-02；误差从 1.128e-02
单调收缩到 1.482e-03。**误差确实在收缩**（这支持控制律有效），但
`done_when_a_to_j.md` 与 `acceptance.md` 里我写的「使被测量**收敛到目标**」
在措辞上不准确：该测试断言的是**方向与变化量**，不是「进入目标容差带」。
测试的 docstring 自己也说明「one proportional law has a steady-state error」。
故应改为：**误差随目标升高单调收缩**（有实测支撑），而非「收敛到目标」。

## 复核发现 4：sentinel 自足**成立**（我的结论正确）

复核确认：`dynamic_hash_sentinel.py:220` 默认执行 acceptance runner，
`run_axle_dynamics_acceptance.py:530` 自建模型与工况，只写/读它自己那一个输出目录。
**但复核同时指出**：`numeric_sentinel.txt:1` 同时记录 self-convergence `FAILED`、
Adams `BLOCKED`、acceptance exit `1`。故哈希一致证明的是**冻结输出未漂移**，
**不是**「所有动力学验收成功」。我的记录里没有把这两件事区分开——须更正。

## 复核发现 5：Goal 覆盖有缺口 —— 复核意见成立

- **G4（`EPIC.md:89`）**含「报表通道按安装角色动态注册（`normal_load_axle_{placement}`）」，
  而我的 `done_when_a_to_j.md` (d) 只写了静平衡，**没有该子项的实跑证据**。
  主代理实证：`outputs/builtin.py:401`、`report/wheel_loads.py:156` 确有
  `channels[f"normal_load_axle_{placement}"]`，但**我没有在 (d) 里给出它的断言或实测**。
- **p5-06 独立重跑**：我的 `acceptance.md` 写了 ruff/ty 退出码，但 `raw/` 下**没有 ruff/ty 的
  输出原文**；快速集（`--ignore` 那一条）与 p5-06 **自己重跑**仓库外 FMU 校验的命令与输出
  也未落盘。须补齐。
- **基线逐字节**：干净 `git status` 不能替代与 p5-01 冻结起点的**逐字节**对照，尤其仓库曾重建。
  主代理实证：`git log -- packages/suspension_multibody/tests/data/` 的最后改动是 `db22f9e`，
  在 Epic 起点之前——**该结论成立**，但必须以「最后改动提交落在 Epic 起点之前」作为证据，
  而不是以 `git status` 干净。

## 复核发现 6：FMU 两处 —— 需进一步核实

**(6a) 外部校验把 `road_velocity` 当作「制动输入」**：复核指出
`fmu_validation.md:323` 选的第一个输入实际是 `road_velocity`（`:34`），
且 `:367` 无条件返回 0。

**主代理实证**：`$PI_SCRATCH_DIR/p505b/outside/validate_fmu.py` 末尾只有 `return 0`，
且 `brake = brake_inputs[0]`（取第一个 input，不按名筛选）。
虽然它移动的是 `road_velocity`（仍是**真实输入**，轨迹断言本身有效），
但文档称其为「制动输入」**不准确**；且脚本无失败路径的**判别性返回码**
（`low/(high)` 的比较结果没有转成退出码）。

**(6b) FMU 步进语义**：复核指出 `fmu_wrapper.c:389` 把新输入写入**整个时间域**、
`:688` 用原始模型**重新批量求解**、`:750` 仅推进时钟，故「加载并步进」不足以证明
离线 Co-Simulation 的**接续语义**。

**主代理核实**：`:389` 的 `for (size_t sample = 0; sample < variable->count; ++sample)`
确实把新值写入**全部样本**；`solve` 对整份 case 重新提交一次内核调用。
内核 ABI 是**批式**的（一次提交跑完整个时间网格），这是既有事实。
故「接续先前状态」在批式 ABI 下**不可能**实现——这不是实现 bug，而是**D4 范围内的能力边界**，
但**必须如实写入验收记录**，不能声称「加载并步进」等价于标准的逐步 Co-Simulation。

---

## 处置汇总

| 复核发现 | 主代理实证 | 处置 |
|---|---|---|
| 1. (a) 不满足 | **成立**（AST：无分支；证据引用不存在的代码） | (a) 改判**不满足**；更正 p2-05 证据文件的伪代码 |
| 2. (f) 未证实 | **成立**（比对对象是实现而非文档；模板可不同） | (f) 改判**未证实** |
| 3a. 总线无轨迹对照 | **成立**（该文件 0 次求解） | (h) 该子项改判**未证实** |
| 3b. 收敛措辞 | **成立**（实测误差单调收缩，但非「进入容差带」） | 更正措辞为「误差单调收缩」 |
| 4. sentinel 自足 | **成立**，但须区分「未漂移」与「全部验收成功」 | 更正记录，区分两件事 |
| 5. Goal 覆盖缺口 | **成立**（G4 动态注册无证据；ruff/ty 等输出未落盘） | 补证据或改判 |
| 6a. FMU 外部校验 | **成立**（选的是 road_velocity；无判别性返回码） | 更正文档，补返回码 |
| 6b. FMU 步进语义 | **成立**（批式 ABI 下无法接续） | 如实写入能力边界，不声称等价 |

**结论**：Epic **不能关闭**。Done-When 的满足数为 **(a)(f)(h) 亦不成立**，
实际为 **6 满足 / 4 不成立或未证实**：满足 (b)(c)(d)(e)(g)(j)；
不成立 (a)（判据未达成）、(i)（skip 增长）；未证实 (f)（证据对象不符）、(h)（两处证据不足）。
